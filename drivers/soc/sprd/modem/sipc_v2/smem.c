// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2019 Spreadtrum Communications Inc.
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

#include <linux/debugfs.h>
#include <linux/genalloc.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/overflow.h>
#include <linux/mutex.h>
#include <linux/io.h>
#include <linux/vmalloc.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/sipc.h>

#include "sipc_priv.h"
#include "sipc_debugfs.h"

/*
 * workround: Due to orca ipa hardware limitations
 * the sipc share memory must map from
 * 0x280000000(orca side) to 0x80000000(roc1
 * side), and the size must be 256M
 */
#ifdef CONFIG_SPRD_IPA_PCIE_WORKROUND
#define IPA_SRC_BASE 0x280000000
#define IPA_DST_BASE 0x80000000
#define IPA_SIZE 0x10000000
#endif

struct smem_phead {
	struct list_head smem_phead;
	spinlock_t lock;
	u32 poolnum;
};

struct smem_pool {
	struct list_head smem_head;
	struct list_head smem_plist;
	spinlock_t lock;

	void *pcie_base;
	u32 addr;
	u32 size;
	u32 dst;
	u32 mem_type;

	atomic_t used;
	struct gen_pool *gen;
};

struct smem_record {
	struct list_head smem_list;
	char comm[TASK_COMM_LEN];
	pid_t pid;
	u32 size;
	u32 addr;
};

struct smem_map_list {
	struct list_head map_head;
	spinlock_t lock;
	u32 inited;
};

struct smem_map {
	struct list_head map_list;
	const void *mem;
	unsigned int count;
	bool iomem;
};

static struct smem_phead sipc_smem_phead = {
	.smem_phead = LIST_HEAD_INIT(sipc_smem_phead.smem_phead),
	.lock = __SPIN_LOCK_UNLOCKED(sipc_smem_phead.lock),
};
static struct smem_map_list mem_mp = {
	.map_head = LIST_HEAD_INIT(mem_mp.map_head),
	.lock = __SPIN_LOCK_UNLOCKED(mem_mp.lock),
	.inited = 1,
};
static DEFINE_MUTEX(smem_init_lock);

static struct smem_pool *shmem_find_pool(u8 dst)
{
	struct smem_phead *phead = &sipc_smem_phead;
	struct smem_pool *spool = NULL;
	struct smem_pool *pos;
	unsigned long flags;

	/* The num of one pool is 0, means the poll is not ready */
	if (!phead->poolnum)
		return NULL;

	spin_lock_irqsave(&phead->lock, flags);
	list_for_each_entry(pos, &phead->smem_phead, smem_plist) {
		if (pos->dst == dst) {
			spool = pos;
			break;
		}
	}
	spin_unlock_irqrestore(&phead->lock, flags);
	return spool;
}

static void *soc_modem_ram_vmap(phys_addr_t start, size_t size, int noncached)
{
	struct page **pages;
	phys_addr_t page_start;
	unsigned int page_count;
	pgprot_t prot;
	unsigned int i;
	void *vaddr;
	phys_addr_t addr;
	unsigned long flags;
	struct smem_map *map;
	struct smem_map_list *smem = &mem_mp;

	if (!size ||
	    size > SIZE_MAX - offset_in_page(start) - (PAGE_SIZE - 1) ||
	    start > PHYS_ADDR_MAX - size)
		return NULL;
	map = kzalloc_obj(struct smem_map, GFP_KERNEL);
	if (!map)
		return NULL;

	page_start = start - offset_in_page(start);
	page_count = DIV_ROUND_UP(size + offset_in_page(start), PAGE_SIZE);
	if (noncached)
		prot = pgprot_noncached(PAGE_KERNEL);
	else
		prot = PAGE_KERNEL;

	pages = kmalloc_array(page_count, sizeof(struct page *), GFP_KERNEL);
	if (!pages) {
		kfree(map);
		return NULL;
	}

	for (i = 0; i < page_count; i++) {
		addr = page_start + i * PAGE_SIZE;
		if (!pfn_valid(PHYS_PFN(addr))) {
			kfree(pages);
			kfree(map);
			return NULL;
		}
		pages[i] = pfn_to_page(PHYS_PFN(addr));
	}
	/* vmap retains the requested cache attributes; vm_map_ram no longer does. */
	vaddr = vmap(pages, page_count, VM_MAP, prot);
	kfree(pages);
	if (!vaddr) {
		kfree(map);
		return NULL;
	}
	vaddr += offset_in_page(start);

	map->count = page_count;
	map->mem = vaddr;

	if (smem->inited) {
		spin_lock_irqsave(&smem->lock, flags);
		list_add_tail(&map->map_list, &smem->map_head);
		spin_unlock_irqrestore(&smem->lock, flags);
	}
	return vaddr;
}

/* PM_SYS shares the captured AON SRAM window; it has no struct page map. */
static void *soc_modem_sram_iomap(phys_addr_t start, size_t size)
{
	struct smem_map *map;
	void __iomem *mem;
	unsigned long flags;

	if (!size || start < 0x8400 || size > 0x2000 ||
	    start - 0x8400 > 0x2000 - size)
		return NULL;
	map = kzalloc_obj(*map, GFP_KERNEL);
	if (!map)
		return NULL;
	mem = ioremap(start, size);
	if (!mem) {
		kfree(map);
		return NULL;
	}
	map->mem = (void __force *)mem;
	map->iomem = true;
	spin_lock_irqsave(&mem_mp.lock, flags);
	list_add_tail(&map->map_list, &mem_mp.map_head);
	spin_unlock_irqrestore(&mem_mp.lock, flags);
	return (void __force *)mem;
}

static void *pcie_modem_ram_vmap(phys_addr_t start, size_t size, int noncached)
{
	if (noncached == 0) {
		pr_err("%s: cache not support!\n", __func__);
		return NULL;
	}

#ifdef CONFIG_SPRD_PCIE_EP_DEVICE
	return sprd_ep_map_memory(PCIE_EP_MODEM, start, size);
#endif

#ifdef CONFIG_PCIE_EPF_SPRD
	return sprd_pci_epf_map_memory(SPRD_FUNCTION_0, start, size);
#endif

	return NULL;
}

static void pcie_modem_ram_unmap(const void *mem)
{
#ifdef CONFIG_SPRD_PCIE_EP_DEVICE
	return sprd_ep_unmap_memory(PCIE_EP_MODEM, mem);
#endif

#ifdef CONFIG_PCIE_EPF_SPRD
	return sprd_pci_epf_unmap_memory(SPRD_FUNCTION_0, mem);
#endif
}

static void soc_modem_ram_unmap(const void *mem)
{
	struct smem_map *map, *next;
	unsigned long flags;
	struct smem_map_list *smem = &mem_mp;
	bool found = false;

	if (smem->inited) {
		spin_lock_irqsave(&smem->lock, flags);
		list_for_each_entry_safe(map, next, &smem->map_head, map_list) {
			if (map->mem == mem) {
				list_del(&map->map_list);
				found = true;
				break;
			}
		}
		spin_unlock_irqrestore(&smem->lock, flags);

		if (found) {
			if (map->iomem)
				iounmap((void __iomem *)map->mem);
			else
				vunmap((void *)((unsigned long)mem &
						PAGE_MASK));
			kfree(map);
		}
	}
}

static void *shmem_ram_vmap(u8 dst, phys_addr_t start, size_t size,
			    int noncached)
{
	struct smem_pool *spool;

	spool = shmem_find_pool(dst);
	if (spool == NULL) {
		pr_err("%s: pool dst %d is not existed!\n", __func__, dst);
		return NULL;
	}

	if (spool->mem_type == SMEM_LOCAL) {
		struct smsg_ipc *ipc = dst < SIPC_ID_NR ? smsg_ipcs[dst] : NULL;
		phys_addr_t base;

		if (!ipc)
			return NULL;
		base = (phys_addr_t)spool->addr +
		       ((phys_addr_t)ipc->high_offset << 32);
		if (!size || start < base || size > spool->size ||
		    start - base > spool->size - size)
			return NULL;
	}

	if (spool->mem_type == SMEM_PCIE) {
		if (start < spool->addr ||
		    start + size > spool->addr + spool->size) {
			pr_info("%s: error, start = 0x%llx, size = 0x%lx.\n",
				__func__, start, size);
			return NULL;
		}

		pr_info("%s: succ, start = 0x%llx, size = 0x%lx.\n", __func__,
			start, size);
		return (spool->pcie_base + start - spool->addr);
	}

	if (dst == SIPC_ID_PM_SYS)
		return noncached ? soc_modem_sram_iomap(start, size) : NULL;
	return soc_modem_ram_vmap(start, size, noncached);
}

int smem_init(u32 addr, u32 size, u32 dst, u32 mem_type)
{
	struct smem_phead *phead = &sipc_smem_phead;
	struct smem_pool *spool, *other;
	unsigned long flags;
	u32 alignment;
	int ret = 0;

	/* Only evidenced local host peers are supported. */
	if (!sipc_local_peer_supported(dst) || mem_type != SMEM_LOCAL)
		return -EOPNOTSUPP;
	if (!addr || !size || size > U32_MAX - addr)
		return -EINVAL;
	alignment = size >= SMEM_ALIGN_POOLSZ ? PAGE_SIZE : SMEM_ALIGN_BYTES;
	/* Never round outside the reserved region advertised by the DT. */
	if (!IS_ALIGNED(addr, alignment) || !IS_ALIGNED(size, alignment))
		return -EINVAL;

	mutex_lock(&smem_init_lock);
	spool = shmem_find_pool(dst);
	if (spool) {
		ret = (spool->addr == addr && spool->size == size &&
		       spool->mem_type == mem_type) ?
			      0 :
			      -EBUSY;
		goto out;
	}
	list_for_each_entry(other, &phead->smem_phead, smem_plist) {
		if ((u64)addr < (u64)other->addr + other->size &&
		    (u64)other->addr < (u64)addr + size) {
			ret = -EBUSY;
			goto out;
		}
	}
	spool = kzalloc_obj(*spool, GFP_KERNEL);
	if (!spool) {
		ret = -ENOMEM;
		goto out;
	}
	spool->addr = addr;
	spool->size = size;
	spool->dst = dst;
	spool->mem_type = mem_type;
	atomic_set(&spool->used, 0);
	spin_lock_init(&spool->lock);
	INIT_LIST_HEAD(&spool->smem_head);
	spool->gen = gen_pool_create(
		size >= SMEM_ALIGN_POOLSZ ? PAGE_SHIFT : SMEM_MIN_ORDER, -1);
	if (!spool->gen) {
		ret = -ENOMEM;
		goto free_pool;
	}
	ret = gen_pool_add(spool->gen, addr, size, -1);
	if (ret) {
		gen_pool_destroy(spool->gen);
		goto free_pool;
	}
	/* Publish only the fully initialized allocator. */
	spin_lock_irqsave(&phead->lock, flags);
	list_add_tail(&spool->smem_plist, &phead->smem_phead);
	phead->poolnum++;
	spin_unlock_irqrestore(&phead->lock, flags);
	goto out;
free_pool:
	kfree(spool);
out:
	mutex_unlock(&smem_init_lock);
	return ret;
}

/* ****************************************************************** */
u32 smem_alloc(u8 dst, u32 size)
{
	struct smem_pool *spool;
	struct smem_record *recd;
	unsigned long flags;
	u32 addr = 0;

	spool = shmem_find_pool(dst);
	if (spool == NULL) {
		pr_err("%s: pool dst %d is not existed!\n", __func__, dst);
		return 0;
	}

	if (!size || size > U32_MAX - PAGE_SIZE)
		return 0;
	recd = kzalloc_obj(struct smem_record, GFP_KERNEL);
	if (!recd)
		return 0;

	if (spool->size >= SMEM_ALIGN_POOLSZ)
		size = PAGE_ALIGN(size);
	else
		size = ALIGN(size, SMEM_ALIGN_BYTES);

	addr = gen_pool_alloc(spool->gen, size);
	if (!addr) {
		pr_err("%s:pool dst=%d, size=0x%x failed to alloc smem!\n",
		       __func__, dst, size);
		kfree(recd);
		return 0;
	}

	/* record smem alloc info */
	atomic_add(size, &spool->used);
	recd->size = size;
	get_task_comm(recd->comm, current);
	recd->pid = task_pid_nr(current);
	recd->addr = addr;
	spin_lock_irqsave(&spool->lock, flags);
	list_add_tail(&recd->smem_list, &spool->smem_head);
	spin_unlock_irqrestore(&spool->lock, flags);

	return addr;
}
EXPORT_SYMBOL_GPL(smem_alloc);

void smem_free(u8 dst, u32 addr, u32 size)
{
	struct smem_pool *spool = shmem_find_pool(dst);
	struct smem_record *recd, *found = NULL;
	unsigned long flags;

	if (!spool || !addr)
		return;
	spin_lock_irqsave(&spool->lock, flags);
	list_for_each_entry(recd, &spool->smem_head, smem_list) {
		if (recd->addr == addr) {
			found = recd;
			list_del(&recd->smem_list);
			break;
		}
	}
	spin_unlock_irqrestore(&spool->lock, flags);
	if (!found) {
		pr_warn("smem: untracked free dst=%u addr=%#x\n", dst, addr);
		return;
	}
	atomic_sub(found->size, &spool->used);
	gen_pool_free(spool->gen, addr, found->size);
	kfree(found);
}
EXPORT_SYMBOL_GPL(smem_free);

void *shmem_ram_vmap_nocache(u8 dst, phys_addr_t start, size_t size)
{
	return shmem_ram_vmap(dst, start, size, 1);
}
EXPORT_SYMBOL_GPL(shmem_ram_vmap_nocache);

void *shmem_ram_vmap_cache(u8 dst, phys_addr_t start, size_t size)
{
	return shmem_ram_vmap(dst, start, size, 0);
}
EXPORT_SYMBOL_GPL(shmem_ram_vmap_cache);

void shmem_ram_unmap(u8 dst, const void *mem)
{
	struct smem_pool *spool;

	spool = shmem_find_pool(dst);
	if (spool == NULL) {
		pr_err("%s: pool dst %d is not existed!\n", __func__, dst);
		return;
	}

	if (spool->mem_type == SMEM_PCIE)
		/* do nothing,  because it also do nothing in shmem_ram_vmap */
		return;
	else
		return soc_modem_ram_unmap(mem);
}
EXPORT_SYMBOL_GPL(shmem_ram_unmap);

void *modem_ram_vmap_nocache(u32 modem_type, phys_addr_t start, size_t size)
{
	if (modem_type == PCIE_MODEM)
		return pcie_modem_ram_vmap(start, size, 1);
	else
		return soc_modem_ram_vmap(start, size, 1);
}
EXPORT_SYMBOL_GPL(modem_ram_vmap_nocache);

void *modem_ram_vmap_cache(u32 modem_type, phys_addr_t start, size_t size)
{
	if (modem_type == PCIE_MODEM)
		return pcie_modem_ram_vmap(start, size, 0);
	else
		return soc_modem_ram_vmap(start, size, 0);
}
EXPORT_SYMBOL_GPL(modem_ram_vmap_cache);

void modem_ram_unmap(u32 modem_type, const void *mem)
{
	if (modem_type == PCIE_MODEM)
		return pcie_modem_ram_unmap(mem);
	else
		return soc_modem_ram_unmap(mem);
}
EXPORT_SYMBOL_GPL(modem_ram_unmap);

#ifdef CONFIG_DEBUG_FS
static int smem_debug_show(struct seq_file *m, void *private)
{
	struct smem_phead *phead = &sipc_smem_phead;
	struct smem_pool *spool, *pos;
	struct smem_record *recd;
	u32 fsize;
	unsigned long flags;
	u32 cnt = 1;

	spin_lock_irqsave(&phead->lock, flags);
	list_for_each_entry(pos, &phead->smem_phead, smem_plist) {
		spool = pos;
		fsize = gen_pool_avail(spool->gen);

		sipc_debug_putline(m, '*', 80);
		seq_printf(m, "%d, dst:%d, name: %s, smem pool info:\n", cnt++,
			   spool->dst,
			   smsg_ipcs[spool->dst] ? smsg_ipcs[spool->dst]->name :
						   "unbound");
		seq_printf(m,
			   "phys_addr=0x%x, total=0x%x, used=0x%x, free=0x%x\n",
			   spool->addr, spool->size, spool->used.counter,
			   fsize);
		seq_puts(m, "smem record list:\n");

		spin_lock(&spool->lock);
		list_for_each_entry(recd, &spool->smem_head, smem_list) {
			seq_printf(m, "task %s: pid=%u, addr=0x%x, size=0x%x\n",
				   recd->comm, recd->pid, recd->addr,
				   recd->size);
		}
		spin_unlock(&spool->lock);
	}
	spin_unlock_irqrestore(&phead->lock, flags);
	return 0;
}

static int smem_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, smem_debug_show, inode->i_private);
}

static const struct file_operations smem_debug_fops = {
	.open = smem_debug_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

int smem_init_debugfs(void *root)
{
	if (!root)
		return -ENXIO;
	debugfs_create_file("smem", 0444, (struct dentry *)root, NULL,
			    &smem_debug_fops);
	return 0;
}
EXPORT_SYMBOL_GPL(smem_init_debugfs);

#endif /* endof CONFIG_DEBUG_FS */

MODULE_AUTHOR("Chen Gaopeng");
MODULE_DESCRIPTION("SIPC/SMEM driver");
MODULE_LICENSE("GPL");
