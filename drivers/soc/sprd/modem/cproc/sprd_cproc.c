// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018 Spreadtrum Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/mm.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/iopoll.h>
#include <linux/kref.h>
#include <linux/list.h>
#include <linux/overflow.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/unaligned.h>
#include <linux/wait.h>

#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/sipc.h>
#include <linux/debugfs.h>
#include <linux/sprd_cp.h>

#include "../include/sprd_cproc.h"

#ifdef CONFIG_DEBUG_FS
static const struct file_operations sprd_cproc_debug_fops;
static struct dentry *cproc_root;

static void sprd_cproc_init_debugfs(void);
#endif

#ifndef CONFIG_SPRD_SIPC_V2
#define SOC_MODEM 0
#define modem_ram_vmap_nocache(type, start, size) \
	shmem_ram_vmap_nocache((start), (size))
#define modem_ram_unmap(type, mem) shmem_ram_unmap((mem))
#endif

#define CPROC_WDT_TRUE true
#define CPROC_WDT_FLASE false
/*
 * Bound temporary uncached mappings while matching the 64 KiB loader writes.
 * Page-sized mappings left thousands of lazy vmap descriptors after boot.
 */
#define CPROC_VMALLOC_SIZE_LIMIT (64 * 1024)
#define MAX_CPROC_ENTRY_NUM 0x10

enum {
	CP_NORMAL_STATUS = 0,
	CP_STOP_STATUS,
	CP_WDTIRQ_STATUS,
	CP_UNKNOWN_STATUS,
	CP_MAX_STATUS,
};

enum {
	BE_SEGMFG = BIT(4),
	BE_RDONLY = BIT(5),
	BE_WRONLY = BIT(6),
	BE_CPDUMP = BIT(7),
	BE_MNDUMP = BIT(8),
	BE_RDWDT = BIT(9),
	BE_RDWDTS = BIT(10),
	BE_RDLDIF = BIT(11),
	BE_LD = BIT(12),
	BE_CTRL_ON = BIT(13),
	BE_CTRL_OFF = BIT(14),
	BE_IOCTL = BIT(15),
};

enum {
	ARM_CTRL = 0x0, /* arm control */
	AGDSP_CTRL, /* agdsp control */
	CTRL_NR,
};

static int cp_boot_mode;

const char *cp_status_info[] = {
	"started\n",
	"stopped\n",
	"wdtirq\n",
	"unknown\n",
};

const char *cproc_dts_args[CPROC_CTRL_NR] = {
	"shutdown",  "deepsleep", "corereset", "sysreset",
	"getstatus", "dspreset",  "fshutdown",
};

struct cproc_proc_fs;

struct cproc_proc_entry {
	char *name;
	struct proc_dir_entry *entry;
	struct cproc_device *cproc;
	unsigned int flag; /*if set bit4 => bit0~bit3 for segs index */
};

struct cproc_proc_fs {
	struct proc_dir_entry *procdir;
	struct cproc_proc_entry entries[MAX_CPROC_ENTRY_NUM];
};

struct cproc_device {
	struct list_head node;
	struct kref ref;
	struct mutex control_lock;
	bool misc_registered;
	bool removing;
	struct dentry *debugfs;
	struct miscdevice miscdev;
	struct cproc_init_data *initdata;
	void *vbase;
	int wdtirq;
	int wdtcnt;
	wait_queue_head_t wdtwait;
	char *name;
	int status;
	int type;
	struct cproc_proc_fs procfs;
};

struct cproc_parent {
	struct list_head children;
};

static void sprd_cproc_release_ref(struct kref *ref);

struct cproc_dump_info {
	char parent_name[20];
	char name[20];
	u32 start_addr;
	u32 size;
};

static int sprd_cproc_regmap_update_bit(struct cproc_ctrl *ctrl, u32 index,
					u32 mask, u32 val);
static int sprd_cproc_regmap_read(struct cproc_ctrl *ctrl, u32 index,
				  unsigned int *val);

static int __init early_mode(char *str)
{
	if (!strcmp("shutdown", str))
		cp_boot_mode = 1;

	return 0;
}

early_param("modem", early_mode);

static int sprd_cproc_open(struct inode *inode, struct file *filp)
{
	struct cproc_device *cproc =
		container_of(filp->private_data, struct cproc_device, miscdev);

	kref_get(&cproc->ref);
	filp->private_data = cproc;

	pr_debug("%s: %s!\n", __func__, cproc->initdata->devname);

	return 0;
}

static int sprd_cproc_release(struct inode *inode, struct file *filp)
{
	struct cproc_device *cproc = filp->private_data;

	pr_debug("%s: %s !\n", __func__, cproc->initdata->devname);

	kref_put(&cproc->ref, sprd_cproc_release_ref);
	return 0;
}

static long sprd_cproc_ioctl(struct file *filp, unsigned int cmd,
			     unsigned long arg)
{
	return -ENOTTY;
}

static int sprd_cproc_mmap(struct file *filp, struct vm_area_struct *vma)
{
	return -EOPNOTSUPP;
}

static const struct file_operations sprd_cproc_fops = {
	.owner = THIS_MODULE,
	.open = sprd_cproc_open,
	.release = sprd_cproc_release,
	.unlocked_ioctl = sprd_cproc_ioctl,
	.mmap = sprd_cproc_mmap,
};

static int cproc_proc_open(struct inode *inode, struct file *filp)
{
	struct cproc_proc_entry *entry =
		(struct cproc_proc_entry *)pde_data(inode);

	pr_debug("%s: %s\n!", __func__, entry->name);

	filp->private_data = entry;

	return 0;
}

static int cproc_proc_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static ssize_t cproc_proc_read(struct file *filp, char __user *buf,
			       size_t count, loff_t *ppos)
{
	struct cproc_proc_entry *entry =
		(struct cproc_proc_entry *)filp->private_data;
	struct cproc_device *cproc = entry->cproc;
	char *type = entry->name;
	unsigned int len;
	unsigned int flag;
	void *vmem;
	int rval;
	int i;
	unsigned long r;
	u32 base, size;

	if (*ppos < 0)
		return -EINVAL;
	if (!count)
		return 0;
	flag = entry->flag;
	pr_debug("%s: type = %s, flag = 0x%x\n", __func__, type, flag);

	if ((flag & BE_RDONLY) == 0)
		return -EPERM;

	if (flag & (BE_CPDUMP | BE_MNDUMP))
		return -EOPNOTSUPP;
	if ((flag & (BE_CPDUMP | BE_SEGMFG)) != 0) {
		if (flag & BE_CPDUMP) {
			base = cproc->initdata->base;
			size = cproc->initdata->maxsz;
		} else {
			for (i = 0; i < cproc->initdata->segnr; i++) {
				if (strcmp(cproc->initdata->segs[i].name,
					   entry->name) == 0)
					break;
			}

			if (i == cproc->initdata->segnr)
				return -EINVAL;

			base = cproc->initdata->segs[i].base;
			size = cproc->initdata->segs[i].maxsz;
		}

		if (*ppos >= size)
			return 0;

		if (count > size - *ppos)
			count = size - *ppos;

		r = count;
		i = 0;
		do {
			u32 copy_size = min_t(unsigned long, r,
					      CPROC_VMALLOC_SIZE_LIMIT);

			vmem = modem_ram_vmap_nocache(
				SOC_MODEM,
				base + *ppos + CPROC_VMALLOC_SIZE_LIMIT * i,
				copy_size);
			if (!vmem) {
				unsigned long addr =
					base + *ppos +
					CPROC_VMALLOC_SIZE_LIMIT * i;
				pr_err("Unable to map cproc base: 0x%lx\n",
				       addr);
				if (i > 0) {
					*ppos += CPROC_VMALLOC_SIZE_LIMIT * i;
					return CPROC_VMALLOC_SIZE_LIMIT * i;
				} else {
					return -ENOMEM;
				}
			}
			if (r < CPROC_VMALLOC_SIZE_LIMIT)
				copy_size = r;
			if (unalign_copy_to_user(buf, vmem, copy_size)) {
				pr_err("cproc read copy data to user err!\n");
				modem_ram_unmap(SOC_MODEM, vmem);
				return -EFAULT;
			}
			modem_ram_unmap(SOC_MODEM, vmem);
			r -= copy_size;
			buf += copy_size;
			i++;
		} while (r > 0);
		/*remap and unmap in each read operation, shi yunlong, end*/
	} else if ((flag & BE_RDWDTS) != 0) {
		int status = READ_ONCE(cproc->status);
		const char *info;

		if (status < 0 || status >= CP_MAX_STATUS)
			return -EINVAL;
		info = cp_status_info[status];
		len = strlen(info);
		if (*ppos >= len)
			return 0;

		count = min_t(size_t, count, len - *ppos);
		if (unalign_copy_to_user(buf, info + *ppos, count)) {
			pr_err("cproc: read, copy data to user error!\n");
			return -EFAULT;
		}
	} else if ((flag & BE_RDWDT) != 0) {
		/* wait forever */
		rval = wait_event_interruptible(
			cproc->wdtwait, cproc->wdtcnt != CPROC_WDT_FLASE ||
						READ_ONCE(cproc->removing));
		if (rval < 0)
			return rval;
		if (READ_ONCE(cproc->removing))
			return -ENODEV;

		len = strlen(cp_status_info[CP_WDTIRQ_STATUS]);
		if (*ppos >= len)
			return 0;
		count = min_t(size_t, count, len - *ppos);
		if (unalign_copy_to_user(
			    buf, cp_status_info[CP_WDTIRQ_STATUS] + *ppos,
			    count)) {
			pr_err("cproc: read, copy data to user error!\n");
			return -EFAULT;
		}
	} else if ((flag & BE_RDLDIF) != 0) {
		struct cproc_init_data *data = cproc->initdata;
		struct cproc_segments *segm = NULL;
		struct load_node lnode;
		char __user *p = buf;

		/* lseek i to *ppos, equivalent to i = *ppos /
		 * sizeof(struct load_node)
		 */
		if (*ppos >= (loff_t)data->segnr * sizeof(lnode))
			return 0;
		i = DIV_ROUND_UP_ULL(*ppos, sizeof(lnode));

		for (len = 0, segm = &data->segs[i]; i < data->segnr;
		     i++, segm++) {
			if (segm->maxsz == 0)
				break;

			memset(&lnode, 0, sizeof(lnode));
			pr_debug("%s: segm[%d] name=%s base=0x%x,size=0x%0x\n",
				 __func__, i, segm->name, segm->base,
				 segm->maxsz);
			strscpy(lnode.name, segm->name, sizeof(lnode.name));
			lnode.base = segm->base;
			lnode.size = segm->maxsz;
			len += sizeof(lnode);

			if (count < len) {
				len -= sizeof(lnode);
				break;
			}

			if (unalign_copy_to_user(p, &lnode, sizeof(lnode))) {
				pr_err("%s: copy ldif to user err!\n",
				       __func__);
				return -EFAULT;
			}

			p += sizeof(lnode);
		}

		count = len;

	} else if ((flag & BE_MNDUMP) != 0) {
		return -EOPNOTSUPP;
	} else {
		return -EINVAL;
	}

	*ppos += count;
	return count;
}

static int cproc_proc_copy_iram(struct cproc_device *cproc)
{
	struct cproc_ctrl *ctrl = cproc->initdata->ctrl;
	void __iomem *iram;
	u8 bytes[sizeof(u32)];
	size_t i, j;

	if (ctrl->iram_loaded)
		return 0;
	if (!ctrl->iram_size || ctrl->iram_size > sizeof(ctrl->iram_data) ||
	    ctrl->iram_size % sizeof(u32))
		return -EINVAL;

	iram = ioremap(ctrl->iram_addr, ctrl->iram_size);
	if (!iram)
		return -ENOMEM;
	/* OF has already converted big-endian DT cells to CPU u32 values.
	 * The ARM target consumes little-endian words, independent of AP
	 * kernel endianness. Byte I/O avoids unaligned/burst SRAM accesses.
	 */
	for (i = 0; i < ctrl->iram_size / sizeof(u32); i++) {
		put_unaligned_le32(ctrl->iram_data[i], bytes);
		for (j = 0; j < sizeof(bytes); j++)
			writeb(bytes[j], iram + i * sizeof(u32) + j);
	}
	/* Commit IRAM boot code before marking it loaded and releasing CP reset. */
	wmb();
	iounmap(iram);
	ctrl->iram_loaded = 1;
	return 0;
}

static ssize_t cproc_proc_copy_from_user(void *to, const void __user *from,
					 unsigned long n)
{
	unsigned long i, ret, size;
	char *buf, *src;
	u32 *word_dst, *word_src;
	u8 *byte_dst, *byte_src;

	buf = kzalloc(n, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	/* when the dst address is on aon sp iram, we use the function
	 * copy_from_user or the function memcpy to copy data to the dst
	 * in sharkl5(arm32), the instuction stm may cause aon sp iram
	 * repose unknown error to cpu. A ways to get around this
	 * probem is below, Firstly,  copy the user data to a ddr buf,
	 * than,  copy it to iram one byte by one byte
	 */
	ret = unalign_copy_from_user(buf, from, n);

	/* Returns number of bytes that could not be copied.
	 * On success, this will be zero.
	 */
	src = buf;

	/*Firstly, cpoy all word align address one word by one word*/
	size = (n - ret) / sizeof(u32);
	word_dst = (u32 *)to;
	word_src = (u32 *)src;
	for (i = 0; i < size; i++)
		*word_dst++ = *word_src++;

	/*than, cpoy all left context one byte by one byte */
	size = (n - ret) % sizeof(u32);
	byte_dst = (u8 *)word_dst;
	byte_src = (u8 *)word_src;
	for (i = 0; i < size; i++)
		*byte_dst++ = *byte_src++;

	kfree(buf);

	return ret;
}

static ssize_t cproc_proc_write_locked(struct file *filp,
				       const char __user *buf, size_t count,
				       loff_t *ppos)
{
	struct cproc_proc_entry *entry =
		(struct cproc_proc_entry *)filp->private_data;
	struct cproc_device *cproc = entry->cproc;
	char *type = entry->name;
	u32 base, size, offset;
	void *vmem;
	int i, ret;
	unsigned long r = count;
	unsigned int flag;

	if (*ppos < 0)
		return -EINVAL;
	if (!count)
		return 0;
	flag = entry->flag;
	pr_debug("%s: type = %s flag = 0x%x\n", __func__, type, flag);

	if ((flag & BE_WRONLY) == 0) {
		return -EPERM;
	} else if ((flag & BE_CTRL_ON) != 0) {
		if (READ_ONCE(cproc->status) != CP_STOP_STATUS)
			return -EBUSY;
		/* Every failure after hardware side effects remains unverified. */
		WRITE_ONCE(cproc->status, CP_UNKNOWN_STATUS);
		WRITE_ONCE(cproc->wdtcnt, CPROC_WDT_FLASE);
		ret = cproc->initdata->start(cproc);
		if (ret)
			return ret;
		/* Do not overwrite a watchdog IRQ observed during the command. */
		cmpxchg(&cproc->status, CP_UNKNOWN_STATUS, CP_NORMAL_STATUS);
		return count;
	} else if ((flag & BE_CTRL_OFF) != 0) {
		pr_debug("%s: stop!\n", __func__);
		WRITE_ONCE(cproc->status, CP_UNKNOWN_STATUS);
		ret = cproc->initdata->stop(cproc);
		if (ret)
			return ret;
		cmpxchg(&cproc->status, CP_UNKNOWN_STATUS, CP_STOP_STATUS);
		return count;
	} else if ((flag & BE_LD) != 0) {
		/* A failed start/stop cannot authorize overwriting a live domain. */
		if (READ_ONCE(cproc->status) != CP_STOP_STATUS)
			return -EBUSY;
		i = ~((~0) << 4) & flag;
		base = cproc->initdata->segs[i].base;
		size = cproc->initdata->segs[i].maxsz;
		if (*ppos >= size || count > size - *ppos)
			return -EFBIG;
		offset = *ppos;

		pr_debug("%s: write %s, offse = %d!\n", __func__, type, offset);

		if (!cproc->initdata->ctrl->iram_loaded &&
		    cproc->initdata->ctrl->iram_size > 0) {
			ret = cproc_proc_copy_iram(cproc);
			if (ret)
				return ret;
		}
	} else {
		pr_err("%s: flag erro!\n", __func__);
		return -EPERM;
	}

	if (size <= offset) {
		pr_debug(
			"%s: max bin size <= offset write over! size = 0x%x, offset = 0x%x\n",
			__func__, size, offset);
		return 0;
	}

	pr_debug("%s: write addr = 0x%08x, size = %lu\n!", __func__,
		 base + offset, r);
	count = min((size_t)(size - offset), count);
	r = count;
	i = 0;
	do {
		unsigned long ret;
		const char __user *pos;
		u32 copy_size =
			min_t(unsigned long, r, CPROC_VMALLOC_SIZE_LIMIT);

		vmem = modem_ram_vmap_nocache(
			SOC_MODEM, base + offset + CPROC_VMALLOC_SIZE_LIMIT * i,
			copy_size);
		if (!vmem) {
			unsigned long addr =
				base + offset + CPROC_VMALLOC_SIZE_LIMIT * i;
			pr_err("Unable to map cproc base: 0x%lx\n", addr);
			if (i > 0) {
				*ppos += CPROC_VMALLOC_SIZE_LIMIT * i;
				return CPROC_VMALLOC_SIZE_LIMIT * i;
			} else {
				return -ENOMEM;
			}
		}
		if (r < CPROC_VMALLOC_SIZE_LIMIT)
			copy_size = r;
		pos = buf + CPROC_VMALLOC_SIZE_LIMIT * i;
		/* dst is aon sp iram, we must used cproc_proc_copy_from_user */
		if (strstr(cproc->initdata->devname, "pm"))
			ret = cproc_proc_copy_from_user(vmem, pos, copy_size);
		else
			ret = unalign_copy_from_user(vmem, pos, copy_size);
		if (ret) {
			pr_err("%s: copy data from user err = %lu!\n", __func__,
			       ret);
			modem_ram_unmap(SOC_MODEM, vmem);
			return -EFAULT;
		}
		modem_ram_unmap(SOC_MODEM, vmem);
		r -= copy_size;
		i++;
	} while (r > 0);

	*ppos += count;
	return count;
}

static ssize_t cproc_proc_write(struct file *filp, const char __user *buf,
				size_t count, loff_t *ppos)
{
	struct cproc_proc_entry *entry = filp->private_data;
	struct cproc_device *cproc = entry->cproc;
	ssize_t ret;

	if (mutex_lock_interruptible(&cproc->control_lock))
		return -ERESTARTSYS;
	ret = READ_ONCE(cproc->removing) ?
		      -ENODEV :
		      cproc_proc_write_locked(filp, buf, count, ppos);
	mutex_unlock(&cproc->control_lock);
	return ret;
}

static loff_t cproc_proc_lseek(struct file *filp, loff_t off, int whence)
{
	struct cproc_proc_entry *entry =
		(struct cproc_proc_entry *)filp->private_data;
	struct cproc_device *cproc = entry->cproc;
	char *type = entry->name;
	loff_t new;

	switch (whence) {
	case SEEK_SET:
		new = off;
		break;
	case SEEK_CUR:
		if (check_add_overflow(filp->f_pos, off, &new))
			return -EINVAL;
		break;
	case SEEK_END:
		if (strcmp(type, "mem") == 0) {
			if (check_add_overflow((loff_t)cproc->initdata->maxsz,
					       off, &new))
				return -EINVAL;
		} else {
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}
	return vfs_setpos(filp, new, MAX_LFS_FILESIZE);
}

static unsigned int cproc_proc_poll(struct file *filp, poll_table *wait)
{
	struct cproc_proc_entry *entry =
		(struct cproc_proc_entry *)filp->private_data;
	struct cproc_device *cproc = entry->cproc;
	char *type = entry->name;
	unsigned int mask = 0;

	pr_debug("%s: type = %s\n", __func__, type);

	if (strcmp(type, "wdtirq") == 0) {
		poll_wait(filp, &cproc->wdtwait, wait);
		if (cproc->wdtcnt != CPROC_WDT_FLASE)
			mask |= POLLIN | POLLRDNORM;
	} else {
		pr_err("%s: file don't support poll !\n", __func__);
	}
	return mask;
}

static long cproc_proc_ioctl_locked(struct file *filp, unsigned int cmd,
				    unsigned long arg)
{
	struct cproc_proc_entry *entry =
		(struct cproc_proc_entry *)filp->private_data;
	struct cproc_device *cproc = entry->cproc;
	struct cproc_ctrl *ctrl;
	long ret = -ENOTTY;
	u32 status, mask;
	int done;

	if (!(entry->flag & BE_IOCTL)) {
		pr_err("ioctl into non-supported file %s!\n", entry->name);

		return -EOPNOTSUPP;
	}

	ctrl = cproc->initdata->ctrl;
	mask = ctrl->ctrl_mask[CPROC_CTRL_LOAD_STATUS];

	switch (cmd) {
	case SPRD_CP_IOCGLDSTAT:
		if (!access_ok((void __user *)arg, _IOC_SIZE(cmd)))
			return -EFAULT;

		ret = sprd_cproc_regmap_read(ctrl, CPROC_CTRL_LOAD_STATUS,
					     &status);
		if (ret)
			return ret;
		done = (status & mask) ? 1 : 0;
		ret = __put_user(done, (int __user *)arg);
		break;

	case SPRD_CP_IOCCLDSTAT:
		ret = sprd_cproc_regmap_update_bit(ctrl, CPROC_CTRL_LOAD_STATUS,
						   mask, ~mask);
		break;

	default:
		break;
	}

	return ret;
}

static long cproc_proc_ioctl(struct file *filp, unsigned int cmd,
			     unsigned long arg)
{
	struct cproc_proc_entry *entry = filp->private_data;
	struct cproc_device *cproc = entry->cproc;
	long ret;

	if (mutex_lock_interruptible(&cproc->control_lock))
		return -ERESTARTSYS;
	ret = READ_ONCE(cproc->removing) ?
		      -ENODEV :
		      cproc_proc_ioctl_locked(filp, cmd, arg);
	mutex_unlock(&cproc->control_lock);
	return ret;
}

static const struct proc_ops cpproc_fs_fops = {
	.proc_open = cproc_proc_open,
	.proc_release = cproc_proc_release,
	.proc_lseek = cproc_proc_lseek,
	.proc_read = cproc_proc_read,
	.proc_write = cproc_proc_write,
	.proc_poll = cproc_proc_poll,
	.proc_ioctl = cproc_proc_ioctl,
#ifdef CONFIG_COMPAT
	.proc_compat_ioctl = cproc_proc_ioctl,
#endif
};

static int sprd_cproc_fs_init(struct cproc_device *cproc)
{
	u8 i, ucnt;
	unsigned int flag;
	umode_t mode;
	struct cproc_ctrl *ctrl;

	cproc->procfs.procdir = proc_mkdir(cproc->name, NULL);
	if (!cproc->procfs.procdir)
		return -ENOMEM;

	memset(cproc->procfs.entries, 0, sizeof(cproc->procfs.entries));

	for (flag = 0, ucnt = 0, mode = 0, i = 0; i < MAX_CPROC_ENTRY_NUM;
	     i++, flag = 0, mode = 0) {
		switch (i) {
		case 0:
			cproc->procfs.entries[i].name = "start";
			flag |= (BE_WRONLY | BE_CTRL_ON);
			ucnt++;
			break;

		case 1:
			cproc->procfs.entries[i].name = "stop";
			flag |= (BE_WRONLY | BE_CTRL_OFF);
			ucnt++;
			break;

		case 2:
			cproc->procfs.entries[i].name = "wdtirq";
			flag |= (BE_RDONLY | BE_RDWDT);
			ucnt++;
			break;

		case 3:
			cproc->procfs.entries[i].name = "status";
			flag |= (BE_RDONLY | BE_RDWDTS);
			ctrl = cproc->initdata->ctrl;
			if (ctrl->ctrl_reg[CPROC_CTRL_LOAD_STATUS] !=
			    INVALID_REG)
				flag |= BE_IOCTL;
			ucnt++;
			break;

		case 4:
			cproc->procfs.entries[i].name = "ldinfo";
			flag |= (BE_RDONLY | BE_RDLDIF);
			ucnt++;
			break;

		default:
			if (cproc->initdata->segnr + ucnt >
			    MAX_CPROC_ENTRY_NUM) {
				pr_debug("%s: entries size to small\n",
					 __func__);
				return -E2BIG;
			}

			if (i - ucnt >= cproc->initdata->segnr)
				return 0;

			cproc->procfs.entries[i].name =
				cproc->initdata->segs[i - ucnt].name;
			flag |= (BE_WRONLY | BE_RDONLY | BE_LD | BE_SEGMFG |
				 (i - ucnt));
			break;
		}

		cproc->procfs.entries[i].flag = flag;
		cproc->procfs.entries[i].cproc = cproc;

		mode |= 0600;
		if (flag & (BE_CPDUMP | BE_MNDUMP))
			mode |= 0004;

		pr_debug("%s: entry name is %s type 0x%x addr: 0x%p\n",
			 __func__, cproc->procfs.entries[i].name,
			 cproc->procfs.entries[i].flag,
			 &cproc->procfs.entries[i]);
		cproc->procfs.entries[i].entry =
			proc_create_data(cproc->procfs.entries[i].name, mode,
					 cproc->procfs.procdir, &cpproc_fs_fops,
					 &cproc->procfs.entries[i]);
		if (!cproc->procfs.entries[i].entry)
			return -ENOMEM;
	}
	return 0;
}

static void sprd_cproc_fs_exit(struct cproc_device *cproc)
{
	WRITE_ONCE(cproc->removing, true);
	wake_up_interruptible_all(&cproc->wdtwait);
	proc_remove(cproc->procfs.procdir);
	cproc->procfs.procdir = NULL;
}

static int sprd_cproc_regmap_update_bit(struct cproc_ctrl *ctrl, u32 index,
					u32 mask, u32 val)
{
	u32 reg = ctrl->ctrl_reg[index];
	u32 orig, tmp, set, clr;
	int ret;

	if (reg == INVALID_REG || IS_ERR_OR_NULL(ctrl->rmap[index]))
		return -EOPNOTSUPP;
	/* BIT(31) denotes the vendor's non-bit-operation register encoding. */
	if (reg & BIT(31)) {
		reg &= ~BIT(31);
		ret = regmap_read(ctrl->rmap[index], reg, &orig);
		if (ret)
			return ret;
		tmp = (orig & ~mask) | (val & mask);
		if (tmp == orig)
			return 0;
		return regmap_write(ctrl->rmap[index], reg, tmp);
	}
	/* Vendor syscons use separate SET/CLR aliases. Upstream regmap-mmio
	 * has no Spreadtrum update_bits callback, so issue those writes here.
	 */
	set = val & mask;
	clr = ~set & mask;
	if (set) {
		ret = regmap_write(ctrl->rmap[index], reg + 0x1000, set);
		if (ret)
			return ret;
	}
	if (clr)
		return regmap_write(ctrl->rmap[index], reg + 0x2000, clr);
	return 0;
}

static int sprd_cproc_regmap_read(struct cproc_ctrl *ctrl, u32 index,
				  unsigned int *val)
{
	if (ctrl->ctrl_reg[index] == INVALID_REG ||
	    IS_ERR_OR_NULL(ctrl->rmap[index]))
		return -EOPNOTSUPP;
	return regmap_read(ctrl->rmap[index], ctrl->ctrl_reg[index] & ~BIT(31),
			   val);
}

static irqreturn_t sprd_cproc_irq_handler(int irq, void *dev_id)
{
	struct cproc_device *cproc = (struct cproc_device *)dev_id;

	pr_debug("%s: cp watchdog enable !\n", __func__);
	WRITE_ONCE(cproc->wdtcnt, CPROC_WDT_TRUE);
	WRITE_ONCE(cproc->status, CP_WDTIRQ_STATUS);
	wake_up_interruptible_all(&cproc->wdtwait);
	return IRQ_HANDLED;
}

static int sprd_cproc_native_arm_start(void *arg)
{
	struct cproc_device *cproc = (struct cproc_device *)arg;
	struct cproc_init_data *pdata = cproc->initdata;
	struct cproc_ctrl *ctrl;
	int ret;
	u32 state;

	if (!pdata)
		return -ENODEV;
	ctrl = pdata->ctrl;

	pr_debug("%s: %s, type = 0x%x, status = 0x%x\n", __func__,
		 pdata->devname, cproc->type, cproc->status);

	if (strstr(pdata->devname, "cp") || strstr(pdata->devname, "nrphy") ||
	    strstr(pdata->devname, "v3phy")) {
		ret = cproc_proc_copy_iram(cproc);
		if (ret)
			return ret;
	}

	/* clear force shutdown */
	pr_debug("%s: force shutdown reg =0x%x, mask =0x%x\n", __func__,
		 ctrl->ctrl_reg[CPROC_CTRL_SHUT_DOWN],
		 ctrl->ctrl_mask[CPROC_CTRL_SHUT_DOWN]);
	if (ctrl->ctrl_reg[CPROC_CTRL_SHUT_DOWN] != INVALID_REG) {
		ret = sprd_cproc_regmap_update_bit(
			ctrl, CPROC_CTRL_SHUT_DOWN,
			ctrl->ctrl_mask[CPROC_CTRL_SHUT_DOWN],
			~ctrl->ctrl_mask[CPROC_CTRL_SHUT_DOWN]);
		if (ret)
			return ret;
	}

	/* Clear CPROC_CTRL_EXT0 for some special modem core */
	if (ctrl->ctrl_reg[CPROC_CTRL_EXT0] != INVALID_REG) {
		pr_debug("%s: CPROC_CTRL_EXT0 reg =0x%x, mask =0x%x\n",
			 __func__, ctrl->ctrl_reg[CPROC_CTRL_EXT0],
			 ctrl->ctrl_mask[CPROC_CTRL_EXT0]);
		ret = sprd_cproc_regmap_update_bit(
			ctrl, CPROC_CTRL_EXT0, ctrl->ctrl_mask[CPROC_CTRL_EXT0],
			~ctrl->ctrl_mask[CPROC_CTRL_EXT0]);
		if (ret)
			return ret;
	}

	/* clear force deep sleep */
	pr_debug("%s: deep sleep reg =0x%x, mask =0x%x\n", __func__,
		 ctrl->ctrl_reg[CPROC_CTRL_DEEP_SLEEP],
		 ctrl->ctrl_mask[CPROC_CTRL_DEEP_SLEEP]);
	if (ctrl->ctrl_reg[CPROC_CTRL_DEEP_SLEEP] != INVALID_REG) {
		ret = sprd_cproc_regmap_update_bit(
			ctrl, CPROC_CTRL_DEEP_SLEEP,
			ctrl->ctrl_mask[CPROC_CTRL_DEEP_SLEEP],
			~ctrl->ctrl_mask[CPROC_CTRL_DEEP_SLEEP]);
		if (ret)
			return ret;
	}

	/* clear core reset */
	pr_debug("%s: core reset reg =0x%x, mask =0x%x\n", __func__,
		 ctrl->ctrl_reg[CPROC_CTRL_CORE_RESET],
		 ctrl->ctrl_mask[CPROC_CTRL_CORE_RESET]);
	if (ctrl->ctrl_reg[CPROC_CTRL_CORE_RESET] != INVALID_REG) {
		msleep(50);
		ret = sprd_cproc_regmap_update_bit(
			ctrl, CPROC_CTRL_CORE_RESET,
			ctrl->ctrl_mask[CPROC_CTRL_CORE_RESET],
			~ctrl->ctrl_mask[CPROC_CTRL_CORE_RESET]);
		if (ret)
			return ret;

		{
			int err;

			err = read_poll_timeout(
				sprd_cproc_regmap_read, ret,
				ret || !(state &
					 ctrl->ctrl_mask[CPROC_CTRL_CORE_RESET]),
				10, 1000000, false, ctrl, CPROC_CTRL_CORE_RESET,
				&state);
			if (ret || err)
				return ret ? ret : err;
		}
	}

	/* clear sys reset */
	pr_debug("%s: sys reset reg =0x%x, mask =0x%x\n", __func__,
		 ctrl->ctrl_reg[CPROC_CTRL_SYS_RESET],
		 ctrl->ctrl_mask[CPROC_CTRL_SYS_RESET]);
	if (ctrl->ctrl_reg[CPROC_CTRL_SYS_RESET] != INVALID_REG) {
		msleep(50);
		ret = sprd_cproc_regmap_update_bit(
			ctrl, CPROC_CTRL_SYS_RESET,
			ctrl->ctrl_mask[CPROC_CTRL_SYS_RESET],
			~ctrl->ctrl_mask[CPROC_CTRL_SYS_RESET]);
		if (ret)
			return ret;

		{
			int err;

			err = read_poll_timeout(
				sprd_cproc_regmap_read, ret,
				ret || !(state &
					 ctrl->ctrl_mask[CPROC_CTRL_SYS_RESET]),
				10, 1000000, false, ctrl, CPROC_CTRL_SYS_RESET,
				&state);
			if (ret || err)
				return ret ? ret : err;
		}
	}

	return 0;
}

static int sprd_cproc_native_arm_stop(void *arg)
{
	struct cproc_device *cproc = (struct cproc_device *)arg;
	struct cproc_init_data *pdata = cproc->initdata;
	struct cproc_ctrl *ctrl;
	int ret;

	if (!pdata)
		return -ENODEV;
	ctrl = pdata->ctrl;
	ctrl->iram_loaded = 0;
	pr_debug("%s: stop %s\n", __func__, cproc->name);

	/* hold tgdsp and ldsp */
	if ((ctrl->ctrl_reg[CPROC_CTRL_DSPCORE_RESET] & INVALID_REG) !=
	    INVALID_REG) {
		ret = sprd_cproc_regmap_update_bit(
			ctrl, CPROC_CTRL_DSPCORE_RESET,
			ctrl->ctrl_mask[CPROC_CTRL_DSPCORE_RESET],
			ctrl->ctrl_mask[CPROC_CTRL_DSPCORE_RESET]);
		if (ret)
			return ret;
	}

	/* reset core */
	if ((ctrl->ctrl_reg[CPROC_CTRL_CORE_RESET] & INVALID_REG) !=
	    INVALID_REG) {
		ret = sprd_cproc_regmap_update_bit(
			ctrl, CPROC_CTRL_CORE_RESET,
			ctrl->ctrl_mask[CPROC_CTRL_CORE_RESET],
			ctrl->ctrl_mask[CPROC_CTRL_CORE_RESET]);
		if (ret)
			return ret;
		msleep(50);
	}
	/* reset sys */
	if ((ctrl->ctrl_reg[CPROC_CTRL_SYS_RESET] & INVALID_REG) !=
	    INVALID_REG) {
		ret = sprd_cproc_regmap_update_bit(
			ctrl, CPROC_CTRL_SYS_RESET,
			ctrl->ctrl_mask[CPROC_CTRL_SYS_RESET],
			ctrl->ctrl_mask[CPROC_CTRL_SYS_RESET]);
		if (ret)
			return ret;
		msleep(50);
	}
	if ((ctrl->ctrl_reg[CPROC_CTRL_DEEP_SLEEP] & INVALID_REG) !=
	    INVALID_REG) {
		/* force deep sleep */
		ret = sprd_cproc_regmap_update_bit(
			ctrl, CPROC_CTRL_DEEP_SLEEP,
			ctrl->ctrl_mask[CPROC_CTRL_DEEP_SLEEP],
			ctrl->ctrl_mask[CPROC_CTRL_DEEP_SLEEP]);
		if (ret)
			return ret;
		msleep(50);
	}

	/* set CPROC_CTRL_EXT0 for some special modem core */
	if ((ctrl->ctrl_reg[CPROC_CTRL_EXT0] & INVALID_REG) != INVALID_REG) {
		/* CPROC_CTRL_EXT0 */
		ret = sprd_cproc_regmap_update_bit(
			ctrl, CPROC_CTRL_EXT0, ctrl->ctrl_mask[CPROC_CTRL_EXT0],
			ctrl->ctrl_mask[CPROC_CTRL_EXT0]);
		if (ret)
			return ret;
	}

	if ((ctrl->ctrl_reg[CPROC_CTRL_SHUT_DOWN] & INVALID_REG) !=
	    INVALID_REG) {
		/* force shutdown */
		ret = sprd_cproc_regmap_update_bit(
			ctrl, CPROC_CTRL_SHUT_DOWN,
			ctrl->ctrl_mask[CPROC_CTRL_SHUT_DOWN],
			ctrl->ctrl_mask[CPROC_CTRL_SHUT_DOWN]);
		if (ret)
			return ret;
	}
	return 0;
}

static int sprd_cproc_native_agdsp_start(void *arg)
{
	struct cproc_device *cproc = (struct cproc_device *)arg;
	struct cproc_init_data *pdata = cproc->initdata;
	struct cproc_ctrl *ctrl;
	int ret;

	if (!pdata)
		return -ENODEV;

	pr_debug("sprd_cproc: %s\n", __func__);

	ctrl = pdata->ctrl;

	/* clear agdsp sys shutdown */
	if ((ctrl->ctrl_reg[CPROC_CTRL_SHUT_DOWN] & INVALID_REG) !=
	    INVALID_REG) {
		ret = sprd_cproc_regmap_update_bit(
			ctrl, CPROC_CTRL_SHUT_DOWN,
			ctrl->ctrl_mask[CPROC_CTRL_SHUT_DOWN],
			~ctrl->ctrl_mask[CPROC_CTRL_SHUT_DOWN]);
		if (ret)
			return ret;
	}

	/* clear agdsp core shutdown */
	if ((ctrl->ctrl_reg[CPROC_CTRL_EXT0] & INVALID_REG) != INVALID_REG) {
		ret = sprd_cproc_regmap_update_bit(
			ctrl, CPROC_CTRL_EXT0, ctrl->ctrl_mask[CPROC_CTRL_EXT0],
			~ctrl->ctrl_mask[CPROC_CTRL_EXT0]);
		if (ret)
			return ret;
	}

	/* clear agdsp core reset */
	if ((ctrl->ctrl_reg[CPROC_CTRL_CORE_RESET] & INVALID_REG) !=
	    INVALID_REG) {
		ret = sprd_cproc_regmap_update_bit(
			ctrl, CPROC_CTRL_CORE_RESET,
			ctrl->ctrl_mask[CPROC_CTRL_CORE_RESET],
			~ctrl->ctrl_mask[CPROC_CTRL_CORE_RESET]);
		if (ret)
			return ret;
	}

	/* clear agdsp core deep sleep */
	if ((ctrl->ctrl_reg[CPROC_CTRL_DEEP_SLEEP] & INVALID_REG) !=
	    INVALID_REG) {
		ret = sprd_cproc_regmap_update_bit(
			ctrl, CPROC_CTRL_DEEP_SLEEP,
			ctrl->ctrl_mask[CPROC_CTRL_DEEP_SLEEP],
			~ctrl->ctrl_mask[CPROC_CTRL_DEEP_SLEEP]);
		if (ret)
			return ret;
	}

	/* config reg protection */
	if ((ctrl->ctrl_reg[CPROC_CTRL_EXT2] & INVALID_REG) != INVALID_REG) {
		ret = sprd_cproc_regmap_update_bit(ctrl, CPROC_CTRL_EXT2,
						   0xffff, ~0xffff);
		if (ret)
			return ret;
		ret = sprd_cproc_regmap_update_bit(
			ctrl, CPROC_CTRL_EXT2, ctrl->ctrl_mask[CPROC_CTRL_EXT2],
			ctrl->ctrl_mask[CPROC_CTRL_EXT2]);
		if (ret)
			return ret;
	}

	/* config boot addr */
	if ((ctrl->ctrl_reg[CPROC_CTRL_EXT3] & INVALID_REG) != INVALID_REG) {
		ret = sprd_cproc_regmap_update_bit(ctrl, CPROC_CTRL_EXT3,
						   0xffffffff, ~0xffffffff);
		if (ret)
			return ret;
		ret = sprd_cproc_regmap_update_bit(
			ctrl, CPROC_CTRL_EXT3, ctrl->ctrl_mask[CPROC_CTRL_EXT3],
			ctrl->ctrl_mask[CPROC_CTRL_EXT3]);
		if (ret)
			return ret;
	}

	/* config agdsp ctrl1 */
	if ((ctrl->ctrl_reg[CPROC_CTRL_EXT4] & INVALID_REG) != INVALID_REG) {
		ret = sprd_cproc_regmap_update_bit(
			ctrl, CPROC_CTRL_EXT4, ctrl->ctrl_mask[CPROC_CTRL_EXT4],
			ctrl->ctrl_mask[CPROC_CTRL_EXT4]);
		if (ret)
			return ret;
	}

	/* config agdsp ctrl */
	if ((ctrl->ctrl_reg[CPROC_CTRL_EXT5] & INVALID_REG) != INVALID_REG) {
		ret = sprd_cproc_regmap_update_bit(
			ctrl, CPROC_CTRL_EXT5, ctrl->ctrl_mask[CPROC_CTRL_EXT5],
			ctrl->ctrl_mask[CPROC_CTRL_EXT5]);
		if (ret)
			return ret;
	}

	/* agdsp reset */
	if ((ctrl->ctrl_reg[CPROC_CTRL_EXT1] & INVALID_REG) != INVALID_REG) {
		ret = sprd_cproc_regmap_update_bit(
			ctrl, CPROC_CTRL_EXT1, ctrl->ctrl_mask[CPROC_CTRL_EXT1],
			~ctrl->ctrl_mask[CPROC_CTRL_EXT1]);
		if (ret)
			return ret;
	}

	return 0;
}

static int sprd_cproc_native_agdsp_stop(void *arg)
{
	struct cproc_device *cproc = (struct cproc_device *)arg;
	struct cproc_init_data *pdata = cproc->initdata;
	struct cproc_ctrl *ctrl;
	int ret;

	if (!pdata)
		return -ENODEV;

	pr_debug("sprd_cproc: %s\n", __func__);

	ctrl = pdata->ctrl;

	/* agdsp core reset */
	if ((ctrl->ctrl_reg[CPROC_CTRL_EXT1] & INVALID_REG) != INVALID_REG) {
		ret = sprd_cproc_regmap_update_bit(
			ctrl, CPROC_CTRL_EXT1, ctrl->ctrl_mask[CPROC_CTRL_EXT1],
			ctrl->ctrl_mask[CPROC_CTRL_EXT1]);
		if (ret)
			return ret;
		msleep(50);
	}

	/* agdsp sys reset */
	if ((ctrl->ctrl_reg[CPROC_CTRL_CORE_RESET] & INVALID_REG) !=
	    INVALID_REG) {
		ret = sprd_cproc_regmap_update_bit(
			ctrl, CPROC_CTRL_CORE_RESET,
			ctrl->ctrl_mask[CPROC_CTRL_CORE_RESET],
			ctrl->ctrl_mask[CPROC_CTRL_CORE_RESET]);
		if (ret)
			return ret;
		msleep(50);
	}

	/* agdsp force deep sleep */
	if ((ctrl->ctrl_reg[CPROC_CTRL_DEEP_SLEEP] & INVALID_REG) !=
	    INVALID_REG) {
		ret = sprd_cproc_regmap_update_bit(
			ctrl, CPROC_CTRL_DEEP_SLEEP,
			ctrl->ctrl_mask[CPROC_CTRL_DEEP_SLEEP],
			ctrl->ctrl_mask[CPROC_CTRL_DEEP_SLEEP]);
		if (ret)
			return ret;
		msleep(50);
	}

	/* agdsp core force shutdown */
	if ((ctrl->ctrl_reg[CPROC_CTRL_EXT0] & INVALID_REG) != INVALID_REG) {
		ret = sprd_cproc_regmap_update_bit(
			ctrl, CPROC_CTRL_EXT0, ctrl->ctrl_mask[CPROC_CTRL_EXT0],
			ctrl->ctrl_mask[CPROC_CTRL_EXT0]);
		if (ret)
			return ret;
	}

	/* agdsp sys force shutdown */
	if ((ctrl->ctrl_reg[CPROC_CTRL_SHUT_DOWN] & INVALID_REG) !=
	    INVALID_REG) {
		ret = sprd_cproc_regmap_update_bit(
			ctrl, CPROC_CTRL_SHUT_DOWN,
			ctrl->ctrl_mask[CPROC_CTRL_SHUT_DOWN],
			ctrl->ctrl_mask[CPROC_CTRL_SHUT_DOWN]);
		if (ret)
			return ret;
	}

	return 0;
}

static int sprd_cproc_native_start(void *arg)
{
	struct cproc_device *cproc = (struct cproc_device *)arg;

	if (!cproc)
		return -ENODEV;

	pr_debug("sprd_cproc: native start type = 0x%x\n", cproc->type);
	if (cproc->type == ARM_CTRL)
		return sprd_cproc_native_arm_start(cproc);
	else if (cproc->type == AGDSP_CTRL)
		return sprd_cproc_native_agdsp_start(cproc);

	return -EINVAL;
}

static int sprd_cproc_native_stop(void *arg)
{
	struct cproc_device *cproc = (struct cproc_device *)arg;

	if (!cproc)
		return -ENODEV;

	pr_debug("sprd_cproc: native stop type = 0x%x\n", cproc->type);
	if (cproc->type == ARM_CTRL)
		return sprd_cproc_native_arm_stop(cproc);
	else if (cproc->type == AGDSP_CTRL)
		return sprd_cproc_native_agdsp_stop(cproc);

	return -EINVAL;
}

static u32 g_common_loader[] = { 0xe59f0000, 0xe12fff10, 0x8ae00000 };

static u32 g_r5_loader[] = {
	0xee110f10, 0xe3c00005, 0xe3c00a01, 0xf57ff04f, 0xee010f10, 0xf57ff06f,
	0xee110f30, 0xe3800802, 0xe3800801, 0xe3c00902, 0xee010f30, 0xf57ff04f,
	0xe3a00000, 0xee070f15, 0xee0f0f15, 0xe3a01000, 0xee061f12, 0xf57ff06f,
	0xe3a01000, 0xee061f11, 0xe3a0103f, 0xee061f51, 0xe3a01f42, 0xee061f91,
	0xee110f30, 0xe3c00802, 0xe3c00801, 0xe3c00902, 0xee010f30, 0xee110f10,
	0xe3800001, 0xf57ff04f, 0xee010f10, 0xf57ff06f, 0xe51ff004, 0x8b800600
};

struct sprd_cproc_data {
	int (*start)(void *arg);
	int (*stop)(void *arg);
	u32 iram_size;
	u32 *iramdata;
};

static struct sprd_cproc_data g_cproc_data = {
	.start = sprd_cproc_native_start,
	.stop = sprd_cproc_native_stop,
	.iram_size = sizeof(g_common_loader),
	.iramdata = &g_common_loader[0],
};

static struct sprd_cproc_data g_cproc_data_pubcp = {
	.start = sprd_cproc_native_start,
	.stop = sprd_cproc_native_stop,
	.iram_size = sizeof(g_r5_loader),
	.iramdata = &g_r5_loader[0],
};

static const struct of_device_id sprd_cproc_match_table[] = {
	{ .compatible = "sprd,scproc", .data = &g_cproc_data },
	{ .compatible = "sprd,scproc_pubcp", .data = &g_cproc_data_pubcp },
	{},
};
MODULE_DEVICE_TABLE(of, sprd_cproc_match_table);

static int sprd_cproc_parse_dt(struct cproc_init_data **init,
			       struct device_node *node)
{
	struct cproc_init_data *pdata;
	struct cproc_ctrl *ctrl;
	struct resource res;
	struct device_node *np = node, *chd = NULL;
	int ret, i, segnr;
	u32 iram_dsize;
	u32 cr_num;
	struct reserved_mem *rmem;
	struct device_node *mem_np;
	phys_addr_t allowed_end;
	int index;
	struct device_node *parent_np = of_get_parent(node);

	struct sprd_cproc_data *pcproc_data;
	const struct of_device_id *of_id =
		of_match_node(sprd_cproc_match_table, parent_np);
	of_node_put(parent_np);
	if (of_id) {
		pcproc_data = (struct sprd_cproc_data *)of_id->data;
	} else {
		pr_err("%s: not find matched id!\n", __func__);
		return -ENODEV;
	}

	/* Only an actual reserved-memory owner can authorize AP load accesses.
	 * In particular the 336 MiB cptl reg window is larger than its 272 MiB
	 * reserved carveout; reg/ranges alone must never authorize RAM access.
	 */
	mem_np = of_parse_phandle(np, "memory-region", 0);
	if (!mem_np)
		return -EINVAL;
	rmem = of_reserved_mem_lookup(mem_np);
	of_node_put(mem_np);
	if (!rmem || !rmem->size || !IS_ALIGNED(rmem->base, PAGE_SIZE) ||
	    !IS_ALIGNED(rmem->size, PAGE_SIZE) ||
	    check_add_overflow(rmem->base, rmem->size - 1, &allowed_end))
		return -EINVAL;

	segnr = of_get_available_child_count(np);
	if (!segnr || segnr > MAX_CPROC_ENTRY_NUM - 7)
		return -EINVAL;
	pr_debug("%s: segnr = %d\n", __func__, segnr);
	pdata = kzalloc(sizeof(*pdata) + segnr * sizeof(struct cproc_segments),
			GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	ctrl = kzalloc_obj(*ctrl, GFP_KERNEL);
	if (!ctrl) {
		kfree(pdata);
		return -ENOMEM;
	}

	ret = of_property_read_string(np, "sprd,name",
				      (const char **)&pdata->devname);
	if (ret) {
		pr_err("cproc: parse name error!\n");
		goto error;
	}

	pr_debug("cproc: parse devname = %s\n", pdata->devname);
	/* get ic type */
	ret = of_property_read_u32(np, "sprd,type", (u32 *)&pdata->type);
	if (ret) {
		if (strstr(pdata->devname, "pm"))
			pdata->type = ARM_CTRL;
		else if (strstr(pdata->devname, "agdsp"))
			pdata->type = AGDSP_CTRL;
		else
			pdata->type = ARM_CTRL;
	}

	for (i = 0; i < CPROC_CTRL_NR; i++)
		ctrl->ctrl_reg[i] = INVALID_REG;

	for (cr_num = 0;
	     cr_num < ARRAY_SIZE(cproc_dts_args) && cproc_dts_args[cr_num];
	     cr_num++) {
		struct of_phandle_args args;
		int named_index;

		named_index = of_property_match_string(np, "syscon-names",
						       cproc_dts_args[cr_num]);
		if (named_index < 0) {
			/* The five base controls are part of the boot contract. */
			if (cr_num <= CPROC_CTRL_LOAD_STATUS) {
				ret = named_index;
				goto error;
			}
			continue;
		}
		ret = of_parse_phandle_with_args(np, "syscons", "#syscon-cells",
						 named_index, &args);
		if (ret)
			goto error;
		if (args.args_count != 2) {
			of_node_put(args.np);
			ret = -EINVAL;
			goto error;
		}
		ctrl->rmap[cr_num] = syscon_node_to_regmap(args.np);
		of_node_put(args.np);
		if (IS_ERR(ctrl->rmap[cr_num])) {
			ret = PTR_ERR(ctrl->rmap[cr_num]);
			goto error;
		}
		ctrl->ctrl_reg[cr_num] = args.args[0];
		ctrl->ctrl_mask[cr_num] = args.args[1];
		ctrl->reg_nr = cr_num + 1;
	}

	/* get iram size */
	ret = of_property_read_u32(np, "sprd,iram-dsize", &iram_dsize);
	if (ret)
		iram_dsize = MAX_IRAM_DATA_NUM;
	/* The DT count is in words and the destination is a fixed array.
	 * Reject, rather than truncate, an invalid boot handoff.
	 */
	if ((of_id->data == &g_cproc_data_pubcp && !iram_dsize) ||
	    iram_dsize > ARRAY_SIZE(ctrl->iram_data)) {
		pr_err("cproc: IRAM word count %u exceeds %zu\n", iram_dsize,
		       ARRAY_SIZE(ctrl->iram_data));
		ret = -EOVERFLOW;
		goto error;
	}

	/* get iram data */
	ret = of_property_read_u32_array(np, "sprd,iram-data", ctrl->iram_data,
					 iram_dsize);
	if (ret) {
		if (of_id->data == &g_cproc_data_pubcp)
			goto error;
		ctrl->iram_size = 0;
	} else {
		ctrl->iram_size = iram_dsize * sizeof(u32);
	}

	for (i = 0; i < iram_dsize; i++) {
		pr_debug("%s: iram-data[%d] = 0x%08x\n", __func__, i,
			 ctrl->iram_data[i]);
	}

	index = 0;
	if (ctrl->iram_size > 0) {
		/* get iram_base+offset */
		ret = of_address_to_resource(np, index, &res);
		if (ret) {
			pr_err("cproc: parse iram base error!");
			goto error;
		}
		if (ctrl->iram_size > resource_size(&res)) {
			ret = -EINVAL;
			goto error;
		}
		ctrl->iram_addr = res.start;
		pr_debug("%s: iram_addr=0x%p\n", __func__,
			 (void *)ctrl->iram_addr);
		index++;
	}

	/* get cp base */
	ret = of_address_to_resource(np, index, &res);
	if (ret) {
		pr_err("cproc: parse cp base error!\n");
		goto error;
	}

	if (res.start > U32_MAX || !resource_size(&res) ||
	    resource_size(&res) > U32_MAX || res.end > U32_MAX) {
		ret = -EOVERFLOW;
		goto error;
	}
	pdata->base = res.start;
	pdata->maxsz = res.end - res.start + 1;
	pr_debug("%s: cp base = 0x%p, size = 0x%x\n", __func__,
		 (void *)pdata->base, pdata->maxsz);
	index++;

	/* Preserve no peer-controlled dump mapping. The dump interfaces stay
	 * absent until every descriptor is bounded and per-open state exists.
	 */

	/* get irq */
	if (of_find_property(np, "interrupts", NULL) ||
	    of_find_property(np, "interrupts-extended", NULL)) {
		pdata->wdtirq = of_irq_get(np, 0);
		if (pdata->wdtirq < 0) {
			ret = pdata->wdtirq;
			goto error;
		}
	}
	pr_debug("%s: wdt irq %u\n", __func__, pdata->wdtirq);

	i = 0;
	for_each_available_child_of_node(np, chd) {
		struct cproc_segments *seg;

		seg = &pdata->segs[i];
		ret = of_property_read_string(chd, "cproc,name",
					      (const char **)&seg->name);
		if (ret) {
			pr_err("cproc: parse chil[%d] name error!\n", i);
			goto error;
		}
		pr_debug("%s: child node [%d] name=%s\n", __func__, i,
			 seg->name);
		/* get child base addr */
		ret = of_address_to_resource(chd, 0, &res);
		if (ret) {
			pr_err("cproc: parse chil[%d] base error!\n", i);
			goto error;
		}
		if (res.start > U32_MAX || !resource_size(&res) ||
		    resource_size(&res) > U32_MAX || res.end > U32_MAX) {
			ret = -EOVERFLOW;
			goto error;
		}
		if (res.start < rmem->base || res.end > allowed_end ||
		    res.start < pdata->base ||
		    res.end >= (u64)pdata->base + pdata->maxsz) {
			pr_err("cproc: load segment outside reserved owner/window\n");
			ret = -ERANGE;
			goto error;
		}
		seg->base = res.start;
		seg->maxsz = res.end - res.start + 1;
		pr_debug("%s: child node [%d] base=0x%x, size=0x%0x\n",
			 __func__, i, seg->base, seg->maxsz);
		i++;
	}

	pdata->segnr = segnr;

	pdata->start = pcproc_data->start;
	pdata->stop = pcproc_data->stop;

	pdata->ctrl = ctrl;
	*init = pdata;
	return 0;
error:
	of_node_put(chd);
	if (pdata->wdtirq > 0)
		irq_dispose_mapping(pdata->wdtirq);
	if (pdata->mini_mem)
		modem_ram_unmap(SOC_MODEM, pdata->mini_mem);
	kfree(ctrl);
	kfree(pdata);
	return ret;
}

static void sprd_cproc_destroy_pdata(struct cproc_init_data **init)
{
	struct cproc_init_data *pdata = *init;

	if (pdata) {
		if (pdata->mini_mem)
			modem_ram_unmap(SOC_MODEM, pdata->mini_mem);
		if (pdata->wdtirq > 0)
			irq_dispose_mapping(pdata->wdtirq);
		kfree(pdata->ctrl);
		kfree(pdata);
	}
	*init = NULL;
}

static void sprd_cproc_release_ref(struct kref *ref)
{
	struct cproc_device *cproc =
		container_of(ref, struct cproc_device, ref);

	sprd_cproc_destroy_pdata(&cproc->initdata);
	kfree(cproc);
}

static void sprd_cproc_remove_child(struct cproc_device *cproc)
{
	sprd_cproc_fs_exit(cproc);
	debugfs_remove(cproc->debugfs);
	if (cproc->wdtirq > 0)
		free_irq(cproc->wdtirq, cproc);
	if (cproc->misc_registered)
		misc_deregister(&cproc->miscdev);
	kref_put(&cproc->ref, sprd_cproc_release_ref);
}

static void sprd_cproc_remove(struct platform_device *pdev)
{
	struct cproc_parent *parent = platform_get_drvdata(pdev);
	struct cproc_device *cproc, *tmp;

	list_for_each_entry_safe(cproc, tmp, &parent->children, node) {
		list_del(&cproc->node);
		sprd_cproc_remove_child(cproc);
	}
}

static int sprd_cproc_probe(struct platform_device *pdev)
{
	struct cproc_parent *parent;
	struct device_node *child;
	int ret;

	if (!pdev->dev.of_node)
		return -ENODEV;
	parent = devm_kzalloc(&pdev->dev, sizeof(*parent), GFP_KERNEL);
	if (!parent)
		return -ENOMEM;
	INIT_LIST_HEAD(&parent->children);
	platform_set_drvdata(pdev, parent);

	for_each_available_child_of_node(pdev->dev.of_node, child) {
		struct cproc_device *cproc;

		cproc = kzalloc_obj(*cproc, GFP_KERNEL);
		if (!cproc) {
			ret = -ENOMEM;
			goto fail;
		}
		kref_init(&cproc->ref);
		mutex_init(&cproc->control_lock);
		ret = sprd_cproc_parse_dt(&cproc->initdata, child);
		if (ret) {
			kfree(cproc);
			goto fail;
		}
		list_add_tail(&cproc->node, &parent->children);
		cproc->name = cproc->initdata->devname;
		cproc->type = cproc->initdata->type;
		/* Boot arguments express intent, not a verified hardware state. */
		cproc->status = CP_UNKNOWN_STATUS;
		dev_dbg(&pdev->dev,
			"CP boot shutdown requested: %d; state unknown\n",
			cp_boot_mode);
		init_waitqueue_head(&cproc->wdtwait);
		cproc->miscdev.minor = MISC_DYNAMIC_MINOR;
		cproc->miscdev.name = cproc->name;
		cproc->miscdev.fops = &sprd_cproc_fops;
		cproc->miscdev.parent = &pdev->dev;
		ret = misc_register(&cproc->miscdev);
		if (ret)
			goto fail;
		cproc->misc_registered = true;

		if (cproc->initdata->wdtirq > 0) {
			ret = request_irq(cproc->initdata->wdtirq,
					  sprd_cproc_irq_handler, 0,
					  cproc->name, cproc);
			if (ret)
				goto fail;
			cproc->wdtirq = cproc->initdata->wdtirq;
		}
		ret = sprd_cproc_fs_init(cproc);
		if (ret)
			goto fail;
#ifdef CONFIG_DEBUG_FS
		sprd_cproc_init_debugfs();
		cproc->debugfs = debugfs_create_file(cproc->name, 0400,
						     cproc_root,
						     cproc->initdata,
						     &sprd_cproc_debug_fops);
#endif
	}
	if (list_empty(&parent->children))
		return -ENODEV;
	return 0;
fail:
	of_node_put(child);
	sprd_cproc_remove(pdev);
	return dev_err_probe(&pdev->dev, ret,
			     "cannot register CPROC children\n");
}

static struct platform_driver sprd_cproc_driver = {
	.probe    = sprd_cproc_probe,
	.remove   = sprd_cproc_remove,
	.driver   = {
		.owner = THIS_MODULE,
		.name = "sprd_cproc",
		.suppress_bind_attrs = true,
		.of_match_table = sprd_cproc_match_table,
	},
};

static int __init sprd_cproc_init(void)
{
	if (platform_driver_register(&sprd_cproc_driver) != 0) {
		pr_err("%s: platform drv register Failed\n", __func__);
		return -1;
	}

	return 0;
}

static void __exit sprd_cproc_exit(void)
{
	platform_driver_unregister(&sprd_cproc_driver);
}

#if defined(CONFIG_DEBUG_FS)
static int sprd_cproc_debug_show(struct seq_file *m, void *private)
{
	struct cproc_init_data *pdata = (struct cproc_init_data *)m->private;
	struct cproc_ctrl *ctrl;
	struct cproc_segments *segs;
	int i;

	sipc_debug_putline(m, '*', 100);

	if (!pdata)
		return 0;

	seq_printf(m, "cproc %s info:\n", pdata->devname);
	seq_printf(m, "wdtirq: %d, type: %d\n", pdata->wdtirq, pdata->type);
	seq_printf(m, "base: 0x%08x, size: 0x%08x\n", (u32)pdata->base,
		   pdata->maxsz);

	if (pdata->mini_mem)
		seq_printf(m, "mini dump mem: 0x%08x, size: 0x%08x\n",
			   (u32)pdata->mini_mem_paddr, pdata->mini_mem_size);

	if (pdata->ctrl) {
		ctrl = pdata->ctrl;
		if (ctrl->iram_size > 0) {
			seq_printf(m, "iram_addr: 0x%08x, iram_size: 0x%08x\n",
				   (u32)ctrl->iram_addr, ctrl->iram_size);

			sipc_debug_putline(m, '-', 80);
			seq_puts(m, "iram data list:\n");

			for (i = 0; i < ctrl->iram_size / sizeof(u32); i++) {
				if ((i + 1) % 4 == 0)
					seq_printf(m, "0x%08x\n",
						   ctrl->iram_data[i]);
				else
					seq_printf(m, "0x%08x, ",
						   ctrl->iram_data[i]);
			}
			seq_puts(m, "\n");
			sipc_debug_putline(m, '-', 80);
		}

		sipc_debug_putline(m, '-', 80);
		seq_puts(m, "ctrl reg list:\n");

		for (i = 0; i < ctrl->reg_nr; i++)
			seq_printf(m, "reg[%2d]: reg=0x%08x, mask=0x%08x\n", i,
				   ctrl->ctrl_reg[i], ctrl->ctrl_mask[i]);
		sipc_debug_putline(m, '-', 80);
	}

	segs = pdata->segs;
	sipc_debug_putline(m, '-', 80);
	seq_puts(m, "segment list:\n");

	for (i = 0; i < pdata->segnr; i++)
		seq_printf(m,
			   "segment[%2d]:base=0x%08x, size=0x%08x, name=%s\n",
			   i, segs[i].base, segs[i].maxsz, segs[i].name);

	sipc_debug_putline(m, '-', 80);

	return 0;
}

static int sprd_cproc_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, sprd_cproc_debug_show, inode->i_private);
}

static const struct file_operations sprd_cproc_debug_fops = {
	.open = sprd_cproc_debug_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static void sprd_cproc_init_debugfs(void)
{
	if (!cproc_root)
		cproc_root = debugfs_create_dir("cproc", NULL);
}
#endif /* CONFIG_DEBUG_FS */

module_init(sprd_cproc_init);
module_exit(sprd_cproc_exit);

MODULE_DESCRIPTION("SPRD Communication Processor Driver");
MODULE_LICENSE("GPL");
