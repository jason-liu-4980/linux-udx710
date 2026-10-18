// SPDX-License-Identifier: GPL-2.0-only
/*
 * Spreadtrum vendor serial function, AP mainline adaptation.
 * Wire descriptors and class 0x22 port-open handshake derive from the GPL
 * BSP f_vserial.c at d834a127aa46356c63bd5a59bc6d86c66ea371be.
 * The misc-device lifetime and request ownership are independently managed.
 */
#include <linux/completion.h>
#include <linux/configfs.h>
#include <linux/kref.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/rwsem.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/usb/composite.h>
#include <linux/uaccess.h>

#include "u_serial.h"

#define VSER_SIZE SZ_16K
#define VSER_TX_COUNT 4

struct vser {
	struct usb_function_instance instance;
	struct usb_function function;
	struct miscdevice misc;
	struct kref ref;
	struct rw_semaphore io_sem;
	struct mutex read_lock, write_lock;
	spinlock_t lock;
	wait_queue_head_t wait;
	struct completion rx_done;
	struct usb_ep *in, *out;
	struct usb_request *rx;
	struct list_head tx_idle;
	unsigned int tx_available;
	atomic_t opens, allocated;
	unsigned int interface;
	u32 generation;
	u32 rx_generation;
	unsigned int rx_offset, rx_remaining;
	bool bound, enabled, online, dead, tx_error, faulted;
};

static atomic_t vser_active = ATOMIC_INIT(0);

static struct usb_interface_descriptor vser_intf = {
	.bLength = sizeof(vser_intf),
	.bDescriptorType = USB_DT_INTERFACE,
	.bNumEndpoints = 2,
	.bInterfaceClass = USB_CLASS_VENDOR_SPEC,
};
#define VSER_EP(name, direction, packet)                \
	static struct usb_endpoint_descriptor name = {  \
		.bLength = USB_DT_ENDPOINT_SIZE,        \
		.bDescriptorType = USB_DT_ENDPOINT,     \
		.bEndpointAddress = direction,          \
		.bmAttributes = USB_ENDPOINT_XFER_BULK, \
		.wMaxPacketSize = cpu_to_le16(packet),  \
	}
VSER_EP(vser_fs_in, USB_DIR_IN, 64);
VSER_EP(vser_fs_out, USB_DIR_OUT, 64);
VSER_EP(vser_hs_in, USB_DIR_IN, 512);
VSER_EP(vser_hs_out, USB_DIR_OUT, 512);
VSER_EP(vser_ss_in, USB_DIR_IN, 1024);
VSER_EP(vser_ss_out, USB_DIR_OUT, 1024);
static struct usb_ss_ep_comp_descriptor vser_ss_comp = {
	.bLength = sizeof(vser_ss_comp),
	.bDescriptorType = USB_DT_SS_ENDPOINT_COMP,
};
static struct usb_descriptor_header *vser_fs[] = {
	(void *)&vser_intf,
	(void *)&vser_fs_in,
	(void *)&vser_fs_out,
	NULL,
};
static struct usb_descriptor_header *vser_hs[] = {
	(void *)&vser_intf,
	(void *)&vser_hs_in,
	(void *)&vser_hs_out,
	NULL,
};
static struct usb_descriptor_header *vser_ss[] = {
	(void *)&vser_intf,   (void *)&vser_ss_in,   (void *)&vser_ss_comp,
	(void *)&vser_ss_out, (void *)&vser_ss_comp, NULL,
};

static struct vser *func_to_vser(struct usb_function *f)
{
	return container_of(f, struct vser, function);
}

static void vser_release_ref(struct kref *ref)
{
	struct vser *v = container_of(ref, struct vser, ref);

	atomic_set(&vser_active, 0);
	kfree(v);
}

static bool vser_ready(struct vser *v)
{
	return READ_ONCE(v->bound) && READ_ONCE(v->online) &&
	       READ_ONCE(v->enabled) && !READ_ONCE(v->dead) &&
	       !READ_ONCE(v->faulted);
}

static void vser_tx_put(struct vser *v, struct usb_request *req)
{
	unsigned long flags;

	spin_lock_irqsave(&v->lock, flags);
	list_add_tail(&req->list, &v->tx_idle);
	v->tx_available++;
	spin_unlock_irqrestore(&v->lock, flags);
	wake_up_all(&v->wait);
}

static struct usb_request *vser_tx_get(struct vser *v)
{
	struct usb_request *req = NULL;
	unsigned long flags;

	spin_lock_irqsave(&v->lock, flags);
	if (!list_empty(&v->tx_idle)) {
		req = list_first_entry(&v->tx_idle, struct usb_request, list);
		list_del_init(&req->list);
		v->tx_available--;
	}
	spin_unlock_irqrestore(&v->lock, flags);
	return req;
}

static void vser_tx_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct vser *v = req->context;

	if (req->status)
		WRITE_ONCE(v->tx_error, true);
	vser_tx_put(v, req);
}

static void vser_rx_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct vser *v = req->context;

	complete(&v->rx_done);
}

/* Serialize queue publication against the transition to disabled. Completion
 * is not invoked synchronously from usb_ep_queue(), as required by gadget API.
 */
static int vser_queue(struct vser *v, struct usb_ep *ep,
		      struct usb_request *req, u32 generation)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&v->lock, flags);
	ret = vser_ready(v) && v->generation == generation ?
		      usb_ep_queue(ep, req, GFP_ATOMIC) :
		      -ENOTCONN;
	spin_unlock_irqrestore(&v->lock, flags);
	return ret;
}

static int vser_wait_online(struct vser *v, struct file *file)
{
	int ret;

	if (file->f_flags & O_NONBLOCK)
		return vser_ready(v) ? 0 : -EAGAIN;
	ret = wait_event_interruptible(v->wait, vser_ready(v) ||
							READ_ONCE(v->dead) ||
							READ_ONCE(v->faulted));
	if (ret)
		return ret;
	return READ_ONCE(v->dead) ? -ENODEV : READ_ONCE(v->faulted) ? -EIO : 0;
}

static ssize_t vser_read(struct file *file, char __user *buf, size_t count,
			 loff_t *pos)
{
	struct vser *v = file->private_data;
	struct usb_request *req;
	u32 generation;
	int ret;

	if (!count)
		return 0;
	if (count > VSER_SIZE)
		return -EINVAL;
	if (file->f_flags & O_NONBLOCK) {
		if (!mutex_trylock(&v->read_lock))
			return -EAGAIN;
	} else {
		ret = mutex_lock_interruptible(&v->read_lock);
		if (ret)
			return ret;
	}
	/* A completed USB packet may span many userspace stream reads. */
	if (!v->rx_remaining) {
		if (file->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			goto unlock;
		}
		ret = vser_wait_online(v, file);
		if (ret)
			goto unlock;
	}
	down_read(&v->io_sem);
	generation = READ_ONCE(v->generation);
	if (v->rx_remaining) {
		if (v->rx_generation != generation || !v->rx) {
			v->rx_remaining = 0;
			v->rx_offset = 0;
			ret = -ENOTCONN;
			goto release;
		}
		goto copy;
	}
	if (!vser_ready(v)) {
		ret = -ENOTCONN;
		goto release;
	}
	req = v->rx;
retry:
	reinit_completion(&v->rx_done);
	req->length = ALIGN(count, v->out->maxpacket);
	ret = vser_queue(v, v->out, req, generation);
	if (ret)
		goto release;
	ret = wait_for_completion_interruptible(&v->rx_done);
	if (ret) {
		/* Do not reuse/free a request until its UDC ownership is returned. */
		usb_ep_dequeue(v->out, req);
		wait_for_completion(&v->rx_done);
		/* Completion can win the signal/dequeue race. Deliver its bytes,
		 * rather than silently dropping a successful USB transfer.
		 */
		if (req->status || !req->actual)
			goto release;
	}
	if (req->status) {
		ret = req->status;
		goto release;
	}
	if (req->actual > req->length || req->actual > VSER_SIZE) {
		ret = -EOVERFLOW;
		goto release;
	}
	if (!req->actual)
		goto retry;
	v->rx_offset = 0;
	v->rx_remaining = req->actual;
	v->rx_generation = generation;
copy:
	if (READ_ONCE(v->generation) != v->rx_generation ||
	    READ_ONCE(v->dead) || READ_ONCE(v->faulted)) {
		v->rx_remaining = 0;
		v->rx_offset = 0;
		ret = -ENOTCONN;
		goto release;
	}
	ret = min_t(size_t, count, v->rx_remaining);
	if (copy_to_user(buf, v->rx->buf + v->rx_offset, ret)) {
		ret = -EFAULT;
	} else {
		v->rx_offset += ret;
		v->rx_remaining -= ret;
	}
release:
	up_read(&v->io_sem);
	if (v->rx_remaining)
		wake_up_all(&v->wait);
unlock:
	mutex_unlock(&v->read_lock);
	return ret;
}

static ssize_t vser_write(struct file *file, const char __user *buf,
			  size_t count, loff_t *pos)
{
	struct vser *v = file->private_data;
	struct usb_request *req;
	size_t done = 0, amount;
	u32 generation;
	int ret;

	if (!count)
		return 0;
	ret = mutex_lock_interruptible(&v->write_lock);
	if (ret)
		return ret;
	ret = vser_wait_online(v, file);
	if (ret)
		goto unlock;
	down_read(&v->io_sem);
	generation = READ_ONCE(v->generation);
	while (done < count) {
		if (!vser_ready(v) || READ_ONCE(v->tx_error) ||
		    generation != READ_ONCE(v->generation)) {
			ret = -EIO;
			break;
		}
		req = vser_tx_get(v);
		if (!req) {
			if (file->f_flags & O_NONBLOCK) {
				ret = -EAGAIN;
				break;
			}
			ret = wait_event_interruptible(
				v->wait,
				READ_ONCE(v->tx_available) || !vser_ready(v) ||
					READ_ONCE(v->tx_error) ||
					generation != READ_ONCE(v->generation));
			if (ret)
				break;
			continue;
		}
		amount = min_t(size_t, count - done, VSER_SIZE);
		if (copy_from_user(req->buf, buf + done, amount)) {
			vser_tx_put(v, req);
			ret = -EFAULT;
			break;
		}
		req->length = amount;
		ret = vser_queue(v, v->in, req, generation);
		if (ret) {
			vser_tx_put(v, req);
			break;
		}
		done += amount;
	}
	up_read(&v->io_sem);
unlock:
	mutex_unlock(&v->write_lock);
	return done ? done : ret;
}

static int vser_open(struct inode *inode, struct file *file)
{
	struct miscdevice *misc = file->private_data;
	struct vser *v = container_of(misc, struct vser, misc);

	if (READ_ONCE(v->dead))
		return -ENODEV;
	if (atomic_inc_return(&v->opens) > 2) {
		atomic_dec(&v->opens);
		return -EBUSY;
	}
	kref_get(&v->ref);
	file->private_data = v;
	return stream_open(inode, file);
}

static int vser_close(struct inode *inode, struct file *file)
{
	struct vser *v = file->private_data;

	atomic_dec(&v->opens);
	kref_put(&v->ref, vser_release_ref);
	return 0;
}

static __poll_t vser_poll(struct file *file, poll_table *wait)
{
	struct vser *v = file->private_data;

	poll_wait(file, &v->wait, wait);
	if (READ_ONCE(v->dead))
		return EPOLLHUP | EPOLLERR;
	if (READ_ONCE(v->tx_error) || READ_ONCE(v->faulted))
		return EPOLLERR;
	return (vser_ready(v) && READ_ONCE(v->tx_available) ?
			EPOLLOUT | EPOLLWRNORM :
			0) |
	       (READ_ONCE(v->rx_remaining) && READ_ONCE(v->rx_generation) ==
						      READ_ONCE(v->generation) ?
			EPOLLIN | EPOLLRDNORM :
			0);
}

static const struct file_operations vser_fops = {
	.owner = THIS_MODULE,
	.open = vser_open,
	.release = vser_close,
	.read = vser_read,
	.write = vser_write,
	.poll = vser_poll,
};

static struct usb_request *vser_alloc_req(struct vser *v, struct usb_ep *ep,
					  bool tx)
{
	struct usb_request *req = gs_alloc_req(ep, VSER_SIZE, GFP_KERNEL);

	if (!req)
		return NULL;
	req->context = v;
	req->complete = tx ? vser_tx_complete : vser_rx_complete;
	INIT_LIST_HEAD(&req->list);
	return req;
}

static void vser_free_req(struct usb_ep *ep, struct usb_request *req)
{
	if (req)
		gs_free_req(ep, req);
}

static void vser_free_requests(struct vser *v)
{
	struct usb_request *req;

	vser_free_req(v->out, v->rx);
	v->rx = NULL;
	while ((req = vser_tx_get(v)))
		vser_free_req(v->in, req);
}

static void vser_disable(struct usb_function *f)
{
	struct vser *v = func_to_vser(f);
	unsigned long flags;
	bool enabled;

	spin_lock_irqsave(&v->lock, flags);
	enabled = v->enabled;
	v->enabled = false;
	v->online = false;
	v->generation++;
	spin_unlock_irqrestore(&v->lock, flags);
	wake_up_all(&v->wait);
	if (enabled) {
		int out_ret = usb_ep_disable(v->out);
		int in_ret = usb_ep_disable(v->in);

		if (out_ret || in_ret) {
			WRITE_ONCE(v->faulted, true);
			pr_err("vser: endpoint drain failed (%d, %d); resources pinned\n",
			       out_ret, in_ret);
			wake_up_all(&v->wait);
		}
	}
}

static int vser_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct vser *v = func_to_vser(f);
	struct usb_request *req;
	int ret, i;

	if (READ_ONCE(v->faulted))
		return -EIO;
	ret = usb_interface_id(c, f);
	if (ret < 0)
		return ret;
	v->interface = ret;
	vser_intf.bInterfaceNumber = ret;
	vser_fs_in.bEndpointAddress = USB_DIR_IN;
	vser_fs_out.bEndpointAddress = USB_DIR_OUT;
	v->in = usb_ep_autoconfig(c->cdev->gadget, &vser_fs_in);
	v->out = usb_ep_autoconfig(c->cdev->gadget, &vser_fs_out);
	if (!v->in || !v->out)
		return -ENODEV;
	vser_hs_in.bEndpointAddress = vser_ss_in.bEndpointAddress =
		vser_fs_in.bEndpointAddress;
	vser_hs_out.bEndpointAddress = vser_ss_out.bEndpointAddress =
		vser_fs_out.bEndpointAddress;
	ret = usb_assign_descriptors(f, vser_fs, vser_hs, vser_ss, vser_ss);
	if (ret)
		return ret;
	v->rx = vser_alloc_req(v, v->out, false);
	if (!v->rx)
		goto nomem;
	for (i = 0; i < VSER_TX_COUNT; i++) {
		req = vser_alloc_req(v, v->in, true);
		if (!req)
			goto nomem;
		vser_tx_put(v, req);
	}
	WRITE_ONCE(v->bound, true);
	return 0;
nomem:
	vser_free_requests(v);
	usb_free_all_descriptors(f);
	return -ENOMEM;
}

static void vser_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct vser *v = func_to_vser(f);

	WRITE_ONCE(v->bound, false);
	vser_disable(f);
	if (READ_ONCE(v->faulted)) {
		/* No successful drain means no safe request retirement. */
		get_device(&c->cdev->gadget->dev);
		return;
	}
	/* disable returns queued requests; wait out existing user buffer accesses. */
	down_write(&v->io_sem);
	vser_free_requests(v);
	usb_free_all_descriptors(f);
	up_write(&v->io_sem);
}

static int vser_set_alt(struct usb_function *f, unsigned int intf,
			unsigned int alt)
{
	struct vser *v = func_to_vser(f);
	int ret;

	if (intf != v->interface || alt)
		return -EINVAL;
	vser_disable(f);
	if (READ_ONCE(v->faulted))
		return -EIO;
	ret = config_ep_by_speed(f->config->cdev->gadget, f, v->in);
	if (ret)
		return ret;
	ret = config_ep_by_speed(f->config->cdev->gadget, f, v->out);
	if (ret)
		return ret;
	ret = usb_ep_enable(v->in);
	if (ret)
		return ret;
	ret = usb_ep_enable(v->out);
	if (ret) {
		int cleanup = usb_ep_disable(v->in);

		if (cleanup) {
			WRITE_ONCE(v->faulted, true);
			pr_err("vser: endpoint-enable rollback failed: %d\n",
			       cleanup);
		}
		return ret;
	}
	WRITE_ONCE(v->enabled, true);
	return 0;
}

static int vser_setup(struct usb_function *f,
		      const struct usb_ctrlrequest *ctrl)
{
	struct vser *v = func_to_vser(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	unsigned long flags;
	int ret;

	if (ctrl->bRequestType !=
		    (USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE) ||
	    ctrl->bRequest != 0x22 ||
	    le16_to_cpu(ctrl->wIndex) != v->interface ||
	    le16_to_cpu(ctrl->wLength) || !READ_ONCE(v->enabled))
		return -EOPNOTSUPP;
	/* Vendor resize/test/IQ controls are unsupported, never successful no-ops. */
	cdev->req->length = 0;
	cdev->req->zero = 0;
	ret = usb_ep_queue(cdev->gadget->ep0, cdev->req, GFP_ATOMIC);
	if (ret)
		return ret;
	spin_lock_irqsave(&v->lock, flags);
	v->online = (le16_to_cpu(ctrl->wValue) & 0xff) == 1;
	if (v->online)
		v->tx_error = false;
	spin_unlock_irqrestore(&v->lock, flags);
	wake_up_all(&v->wait);
	return 0;
}

static void vser_free_func(struct usb_function *f)
{
	struct vser *v = func_to_vser(f);

	if (READ_ONCE(v->faulted))
		return; /* Explicit bounded quarantine; never release an uncertain DMA owner. */
	atomic_set(&v->allocated, 0);
	kref_put(&v->ref, vser_release_ref);
}

static void vser_free_instance(struct usb_function_instance *instance)
{
	struct vser *v = container_of(instance, struct vser, instance);

	WRITE_ONCE(v->dead, true);
	wake_up_all(&v->wait);
	misc_deregister(&v->misc);
	kref_put(&v->ref, vser_release_ref);
}

static void vser_item_release(struct config_item *item)
{
	struct vser *v = container_of(to_config_group(item), struct vser,
				      instance.group);

	usb_put_function_instance(&v->instance);
}

static struct configfs_item_operations vser_item_ops = {
	.release = vser_item_release
};
static const struct config_item_type vser_type = {
	.ct_item_ops = &vser_item_ops,
	.ct_owner = THIS_MODULE,
};

static struct usb_function_instance *vser_alloc_instance(void)
{
	struct vser *v;
	int ret;

	if (atomic_cmpxchg(&vser_active, 0, 1))
		return ERR_PTR(-EBUSY);
	v = kzalloc_obj(*v, GFP_KERNEL);
	if (!v) {
		atomic_set(&vser_active, 0);
		return ERR_PTR(-ENOMEM);
	}
	kref_init(&v->ref);
	init_rwsem(&v->io_sem);
	mutex_init(&v->read_lock);
	mutex_init(&v->write_lock);
	spin_lock_init(&v->lock);
	init_waitqueue_head(&v->wait);
	init_completion(&v->rx_done);
	INIT_LIST_HEAD(&v->tx_idle);
	atomic_set(&v->opens, 0);
	atomic_set(&v->allocated, 0);
	v->instance.free_func_inst = vser_free_instance;
	v->misc.minor = MISC_DYNAMIC_MINOR;
	v->misc.name = "vser";
	v->misc.fops = &vser_fops;
	v->misc.mode = 0600;
	/* One captured vser.gs0; open files and fault quarantine retain singleton ownership. */
	ret = misc_register(&v->misc);
	if (ret) {
		kref_put(&v->ref, vser_release_ref);
		return ERR_PTR(ret);
	}
	config_group_init_type_name(&v->instance.group, "", &vser_type);
	return &v->instance;
}

static struct usb_function *
vser_alloc_function(struct usb_function_instance *instance)
{
	struct vser *v = container_of(instance, struct vser, instance);

	if (atomic_cmpxchg(&v->allocated, 0, 1))
		return ERR_PTR(-EBUSY);
	kref_get(&v->ref);
	v->function.name = "vser";
	v->function.bind = vser_bind;
	v->function.unbind = vser_unbind;
	v->function.set_alt = vser_set_alt;
	v->function.disable = vser_disable;
	v->function.setup = vser_setup;
	v->function.free_func = vser_free_func;
	return &v->function;
}
DECLARE_USB_FUNCTION_INIT(vser, vser_alloc_instance, vser_alloc_function);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Spreadtrum vendor serial AP misc-device USB function");
