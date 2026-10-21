// SPDX-License-Identifier: GPL-2.0-only
/* UDX710 native SBUF AT streams: ring 0 URC, ring 1 commands. */
#include <linux/device.h>
#include <linux/kref.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/sipc.h>
#include <linux/skbuff.h>
#include <linux/wwan.h>

#include "sbuf.h"

#define SIPC_AT_DST 5
#define SIPC_AT_CHANNEL 6
#define SIPC_AT_RINGS 32
#define SIPC_AT_MTU 2048
#define SIPC_AT_RX_SLOTS 4

struct sipc_at_session;

struct sipc_at_port {
	struct sipc_at_session *session;
	struct wwan_port *port;
	struct work_struct io;
	spinlock_t lock;
	u32 ring;
	unsigned int rx_budget;
	bool started;
	struct sk_buff *tx;
	unsigned int tx_offset;
};

struct sipc_at_session {
	struct kref ref;
	struct sbuf_mgr *sbuf;
	struct device *parent;
	struct workqueue_struct *control_wq;
	struct work_struct fault_work;
	u32 generation;
	bool dead;
	int fault;
	struct sipc_at_port ports[2];
};

struct sipc_at {
	struct device *dev;
	struct device *parent;
	struct sbuf_mgr *sbuf;
	struct notifier_block notifier;
	struct workqueue_struct *control_wq;
	struct work_struct work;
	struct sipc_at_session *session;
	u32 generation;
	bool attempted;
	bool removing;
};

static void sipc_at_session_release(struct kref *ref)
{
	struct sipc_at_session *s =
		container_of(ref, struct sipc_at_session, ref);

	put_device(s->parent);
	sbuf_put_manager(s->sbuf);
	kfree(s);
}

static void sipc_at_dead(struct sipc_at_session *s)
{
	/* One flag revokes both ports at the same ring-commit boundary. */
	sbuf_session_cancel(s->sbuf, 0, &s->dead);
	sbuf_session_cancel(s->sbuf, 1, &s->dead);
}

static void sipc_at_fault(struct sipc_at_session *s, int error)
{
	if (cmpxchg(&s->fault, 0, error))
		return;
	sipc_at_dead(s);
	queue_work(s->control_wq, &s->fault_work);
}

static void sipc_at_rx_release(struct sk_buff *skb)
{
	struct sipc_at_port *p = skb_shinfo(skb)->destructor_arg;
	struct sipc_at_session *s = p->session;

	spin_lock_bh(&p->lock);
	p->rx_budget++;
	if (p->started && !READ_ONCE(s->dead))
		schedule_work(&p->io);
	spin_unlock_bh(&p->lock);
	/* The last reference can free the lock itself. */
	kref_put(&s->ref, sipc_at_session_release);
}

static bool sipc_at_rx_reserve(struct sipc_at_port *p)
{
	bool reserved = false;

	spin_lock_bh(&p->lock);
	if (p->started && !READ_ONCE(p->session->dead) && p->rx_budget) {
		p->rx_budget--;
		reserved = true;
	}
	spin_unlock_bh(&p->lock);
	return reserved;
}

static void sipc_at_rx_unused(struct sipc_at_port *p)
{
	/* No bytes were consumed. Do not reschedule an empty ring by freeing the
	 * speculative buffer: its next trigger is WRPTR, not a polling loop.
	 */
	spin_lock_bh(&p->lock);
	p->rx_budget++;
	spin_unlock_bh(&p->lock);
}

static void sipc_at_receive(struct sipc_at_port *p)
{
	struct sipc_at_session *s = p->session;
	struct sk_buff *skb;
	int ret;

	while (sipc_at_rx_reserve(p)) {
		skb = alloc_skb(SIPC_AT_MTU, GFP_KERNEL);
		if (!skb) {
			sipc_at_rx_unused(p);
			sipc_at_fault(s, -ENOMEM);
			return;
		}
		ret = sbuf_session_read(s->sbuf, s->generation, &s->dead,
					p->ring, skb->data, SIPC_AT_MTU, 0,
					false);
		if (ret <= 0) {
			kfree_skb(skb);
			sipc_at_rx_unused(p);
			if (ret != -ENODATA && ret != -EBUSY && ret != -EAGAIN)
				sipc_at_fault(s, ret ? ret : -EIO);
			return;
		}
		skb_put(skb, ret);
		kref_get(&s->ref);
		skb->destructor = sipc_at_rx_release;
		skb_shinfo(skb)->destructor_arg = p;
		wwan_port_rx(READ_ONCE(p->port), skb);
	}
}

static void sipc_at_transmit(struct sipc_at_port *p)
{
	struct sipc_at_session *s = p->session;
	struct sk_buff *skb;
	unsigned int offset;
	int ret;

	for (;;) {
		spin_lock_bh(&p->lock);
		skb = p->started && !READ_ONCE(s->dead) ? p->tx : NULL;
		offset = p->tx_offset;
		spin_unlock_bh(&p->lock);
		if (!skb)
			return;
		/* stop joins this worker before freeing the sole pending SKB. */
		ret = sbuf_session_write(s->sbuf, s->generation, &s->dead,
					 p->ring, skb->data + offset,
					 skb->len - offset, 0, false);
		if (ret <= 0) {
			if (ret != -EBUSY && ret != -EAGAIN)
				sipc_at_fault(s, ret ? ret : -EIO);
			return;
		}
		spin_lock_bh(&p->lock);
		p->tx_offset += ret;
		if (p->tx_offset == skb->len) {
			p->tx = NULL;
			p->tx_offset = 0;
			if (p->started && !READ_ONCE(s->dead))
				wwan_port_txon(p->port);
			spin_unlock_bh(&p->lock);
			consume_skb(skb);
			return;
		}
		spin_unlock_bh(&p->lock);
	}
}

static void sipc_at_io(struct work_struct *work)
{
	struct sipc_at_port *p = container_of(work, struct sipc_at_port, io);

	sipc_at_transmit(p);
	sipc_at_receive(p);
}

static int sipc_at_start(struct wwan_port *port)
{
	struct sipc_at_port *p = wwan_port_get_drvdata(port);
	struct sipc_at_session *s = p->session;
	u32 generation;
	int ret;

	ret = sbuf_session_state(s->sbuf, &generation);
	if (ret)
		return ret;
	if (generation != s->generation)
		return -EPIPE;
	spin_lock_bh(&p->lock);
	if (READ_ONCE(s->dead)) {
		spin_unlock_bh(&p->lock);
		return -EPIPE;
	}
	/* device_add can make open possible before wwan_create_port returns. */
	WRITE_ONCE(p->port, port);
	p->started = true;
	wwan_port_txon(port);
	schedule_work(&p->io);
	spin_unlock_bh(&p->lock);
	return 0;
}

static void sipc_at_quiesce(struct sipc_at_port *p)
{
	struct sk_buff *skb;

	spin_lock_bh(&p->lock);
	p->started = false;
	if (p->port)
		wwan_port_txoff(p->port);
	spin_unlock_bh(&p->lock);
	cancel_work_sync(&p->io);
	spin_lock_bh(&p->lock);
	skb = p->tx;
	p->tx = NULL;
	p->tx_offset = 0;
	spin_unlock_bh(&p->lock);
	kfree_skb(skb);
}

static void sipc_at_stop(struct wwan_port *port)
{
	struct sipc_at_port *p = wwan_port_get_drvdata(port);

	/* Local close drops an unpublished tail, not the CP communication session.
	 * RX budget is restored only by SKB destruction, never reset here/start.
	 */
	sipc_at_quiesce(p);
}

static int sipc_at_tx(struct wwan_port *port, struct sk_buff *skb)
{
	struct sipc_at_port *p = wwan_port_get_drvdata(port);
	struct sipc_at_session *s = p->session;
	u32 generation;
	int ret;

	if (skb->len > SIPC_AT_MTU)
		return -EMSGSIZE;
	if (!skb->len) {
		consume_skb(skb);
		return 0;
	}
	ret = sbuf_session_state(s->sbuf, &generation);
	if (ret)
		return ret;
	if (generation != s->generation)
		return -EPIPE;
	spin_lock_bh(&p->lock);
	if (!p->started || READ_ONCE(s->dead))
		ret = -EPIPE;
	else if (p->tx)
		ret = -EAGAIN;
	else {
		p->tx = skb;
		p->tx_offset = 0;
		wwan_port_txoff(port);
		schedule_work(&p->io);
		ret = 0;
	}
	spin_unlock_bh(&p->lock);
	return ret;
}

static const struct wwan_port_ops sipc_at_ops = {
	.start = sipc_at_start,
	.stop = sipc_at_stop,
	.tx = sipc_at_tx,
};

static void sipc_at_remove_ports(struct sipc_at_session *s)
{
	unsigned int i;

	sipc_at_dead(s);
	for (i = 0; i < ARRAY_SIZE(s->ports); i++) {
		struct sipc_at_port *p = &s->ports[i];

		if (p->port) {
			wwan_remove_port(p->port);
			WRITE_ONCE(p->port, NULL);
		}
	}
}

static void sipc_at_fault_work(struct work_struct *work)
{
	struct sipc_at_session *s =
		container_of(work, struct sipc_at_session, fault_work);

	dev_err(s->parent, "AT frontend stopped: %d\n", s->fault);
	sipc_at_remove_ports(s);
}

/* Called on the ordered control queue, never from fault_work itself. */
static void sipc_at_drop_session(struct sipc_at_session *s)
{
	unsigned int i;

	sipc_at_dead(s);
	for (i = 0; i < ARRAY_SIZE(s->ports); i++)
		sipc_at_quiesce(&s->ports[i]);
	/* No I/O worker can enqueue another fault after the joins above. Control
	 * and fault work share an ordered queue, so fault_work cannot run here.
	 */
	cancel_work_sync(&s->fault_work);
	sipc_at_remove_ports(s);
	kref_put(&s->ref, sipc_at_session_release);
}

static struct sipc_at_session *sipc_at_create_session(struct sipc_at *at,
						      u32 generation)
{
	struct sipc_at_session *s;
	unsigned int i;
	int ret;

	s = kzalloc(sizeof(*s), GFP_KERNEL);
	if (!s)
		return ERR_PTR(-ENOMEM);
	kref_init(&s->ref);
	s->sbuf = at->sbuf;
	kref_get(&s->sbuf->ref);
	s->parent = get_device(at->parent);
	s->control_wq = at->control_wq;
	s->generation = generation;
	INIT_WORK(&s->fault_work, sipc_at_fault_work);
	for (i = 0; i < ARRAY_SIZE(s->ports); i++) {
		struct sipc_at_port *p = &s->ports[i];

		p->session = s;
		p->ring = i;
		p->rx_budget = SIPC_AT_RX_SLOTS;
		spin_lock_init(&p->lock);
		INIT_WORK(&p->io, sipc_at_io);
	}
	/* This board has one AT provider. Register URC then command so the core
	 * assigns at0/at1 within their shared modem WWAN device, including reuse.
	 */
	for (i = 0; i < ARRAY_SIZE(s->ports); i++) {
		struct sipc_at_port *p = &s->ports[i];
		struct wwan_port *port;

		port = wwan_create_port(at->parent, WWAN_PORT_AT, &sipc_at_ops,
					NULL, p);
		if (IS_ERR(port)) {
			ret = PTR_ERR(port);
			sipc_at_drop_session(s);
			return ERR_PTR(ret);
		}
		WRITE_ONCE(p->port, port);
	}
	return s;
}

static void sipc_at_work(struct work_struct *work)
{
	struct sipc_at *at = container_of(work, struct sipc_at, work);
	u32 generation;
	unsigned int i;
	int ret;

	ret = sbuf_session_state(at->sbuf, &generation);
	if (at->session &&
	    (ret || generation != at->generation || at->removing)) {
		sipc_at_drop_session(at->session);
		at->session = NULL;
	}
	if (ret || at->removing) {
		at->attempted = false;
		return;
	}
	if (!at->session) {
		if (at->attempted && generation == at->generation)
			return;
		at->generation = generation;
		at->attempted = true;
		at->session = sipc_at_create_session(at, generation);
		if (IS_ERR(at->session)) {
			dev_err(at->dev, "AT port registration: %ld\n",
				PTR_ERR(at->session));
			at->session = NULL;
			return;
		}
	}
	for (i = 0; i < ARRAY_SIZE(at->session->ports); i++) {
		struct sipc_at_port *p = &at->session->ports[i];

		spin_lock_bh(&p->lock);
		if (p->started && !READ_ONCE(at->session->dead))
			schedule_work(&p->io);
		spin_unlock_bh(&p->lock);
	}
}

static int sipc_at_event(struct notifier_block *nb, unsigned long event,
			 void *data)
{
	struct sipc_at *at = container_of(nb, struct sipc_at, notifier);

	/* The work reads actual state/rings. Coalesced RESET/READY is caught by
	 * generation; a kick racing a running work schedules another execution.
	 */
	queue_work(at->control_wq, &at->work);
	return NOTIFY_OK;
}

static void sipc_at_put_device(void *data)
{
	put_device(data);
}

static int sipc_at_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct fwnode_handle *node;
	struct sipc_at *at;
	int ret;

	at = devm_kzalloc(dev, sizeof(*at), GFP_KERNEL);
	if (!at)
		return -ENOMEM;
	at->dev = dev;
	node = fwnode_find_reference(dev_fwnode(dev), "sprd,modem", 0);
	if (IS_ERR(node))
		return PTR_ERR(node);
	at->parent = bus_find_device_by_fwnode(&platform_bus_type, node);
	fwnode_handle_put(node);
	if (!at->parent)
		return -EPROBE_DEFER;
	ret = devm_add_action_or_reset(dev, sipc_at_put_device, at->parent);
	if (ret)
		return ret;
	if (!device_link_add(dev, at->parent, DL_FLAG_AUTOREMOVE_CONSUMER))
		return -EINVAL;
	at->control_wq = alloc_ordered_workqueue("sipc-at", 0);
	if (!at->control_wq)
		return -ENOMEM;
	INIT_WORK(&at->work, sipc_at_work);
	at->notifier.notifier_call = sipc_at_event;
	ret = sbuf_create(SIPC_AT_DST, SIPC_AT_CHANNEL, SIPC_AT_RINGS,
			  SIPC_AT_MTU, SIPC_AT_MTU);
	if (ret)
		goto destroy_queue;
	at->sbuf = sbuf_get_manager(SIPC_AT_DST, SIPC_AT_CHANNEL);
	ret = blocking_notifier_chain_register(&at->sbuf->events,
					       &at->notifier);
	if (ret)
		goto destroy_sbuf;
	platform_set_drvdata(pdev, at);
	queue_work(at->control_wq, &at->work);
	return 0;
destroy_sbuf:
	sbuf_destroy(SIPC_AT_DST, SIPC_AT_CHANNEL);
	sbuf_put_manager(at->sbuf);
destroy_queue:
	destroy_workqueue(at->control_wq);
	return ret;
}

static void sipc_at_remove(struct platform_device *pdev)
{
	struct sipc_at *at = platform_get_drvdata(pdev);

	blocking_notifier_chain_unregister(&at->sbuf->events, &at->notifier);
	WRITE_ONCE(at->removing, true);
	queue_work(at->control_wq, &at->work);
	flush_workqueue(at->control_wq);
	destroy_workqueue(at->control_wq);
	sbuf_destroy(SIPC_AT_DST, SIPC_AT_CHANNEL);
	sbuf_put_manager(at->sbuf);
}

static const struct of_device_id sipc_at_of_match[] = {
	{ .compatible = "sprd,udx710-wwan-at" },
	{}
};
MODULE_DEVICE_TABLE(of, sipc_at_of_match);

static struct platform_driver sipc_at_driver = {
	.probe = sipc_at_probe,
	.remove = sipc_at_remove,
	.driver = {
		.name = "sipc-wwan-at",
		.of_match_table = sipc_at_of_match,
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(sipc_at_driver);
MODULE_DESCRIPTION("UDX710 native SBUF WWAN AT ports");
MODULE_LICENSE("GPL");
