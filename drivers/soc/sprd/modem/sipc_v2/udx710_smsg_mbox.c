// SPDX-License-Identifier: GPL-2.0-only
/* AP-side SMSG transport boundary. No CP firmware or private register ABI. */
#include <linux/atomic.h>
#include <linux/err.h>
#include <linux/mailbox_client.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/wait.h>

#include "udx710_smsg_mbox.h"

#define UDX710_SMSG_TX_SLOTS 32

struct udx710_smsg_tx {
	u32 words[2];
	bool busy;
};

struct udx710_smsg_link {
	struct mbox_client client;
	struct mbox_chan *channel;
	udx710_smsg_rx_t receive;
	void *context;
	spinlock_t lock;
	bool stopping;
	atomic_t active;
	wait_queue_head_t idle;
	wait_queue_head_t txwait;
	atomic_t tx_done_seq;
	struct udx710_smsg_tx tx[UDX710_SMSG_TX_SLOTS];
};

static void udx710_smsg_idle(struct udx710_smsg_link *link)
{
	unsigned long flags;

	spin_lock_irqsave(&link->lock, flags);
	if (atomic_dec_and_test(&link->active))
		wake_up(&link->idle);
	spin_unlock_irqrestore(&link->lock, flags);
}

static void udx710_smsg_receive(struct mbox_client *client, void *data)
{
	struct udx710_smsg_link *link =
		container_of(client, struct udx710_smsg_link, client);
	struct udx710_smsg message;
	const u32 *words = data;
	unsigned long flags;

	spin_lock_irqsave(&link->lock, flags);
	if (link->stopping) {
		spin_unlock_irqrestore(&link->lock, flags);
		return;
	}
	atomic_inc(&link->active);
	spin_unlock_irqrestore(&link->lock, flags);

	/* The SPRD provider passes two CPU-endian words from readl(). */
	message.channel = words[0];
	message.type = words[0] >> 8;
	message.flag = words[0] >> 16;
	message.value = words[1];
	link->receive(link->context, &message);
	udx710_smsg_idle(link);
}

static void udx710_smsg_sent(struct mbox_client *client, void *data, int result)
{
	struct udx710_smsg_link *link =
		container_of(client, struct udx710_smsg_link, client);
	struct udx710_smsg_tx *tx =
		container_of(data, struct udx710_smsg_tx, words);
	unsigned long flags;

	/* Completion never claims the CP processed this message. */
	if (result)
		dev_err_ratelimited(client->dev, "SMSG mailbox TX failed: %d\n",
				    result);
	spin_lock_irqsave(&link->lock, flags);
	tx->busy = false;
	atomic_inc(&link->tx_done_seq);
	wake_up_interruptible_all(&link->txwait);
	spin_unlock_irqrestore(&link->lock, flags);
}

struct udx710_smsg_link *udx710_smsg_link_open(struct device *dev,
					       const char *mbox_name,
					       udx710_smsg_rx_t receive,
					       void *context)
{
	struct udx710_smsg_link *link;
	int ret;

	if (!dev || !receive || !mbox_name)
		return ERR_PTR(-EINVAL);
	link = kzalloc_obj(*link, GFP_KERNEL);
	if (!link)
		return ERR_PTR(-ENOMEM);

	spin_lock_init(&link->lock);
	atomic_set(&link->active, 0);
	init_waitqueue_head(&link->idle);
	init_waitqueue_head(&link->txwait);
	atomic_set(&link->tx_done_seq, 0);
	link->receive = receive;
	link->context = context;
	link->client.dev = dev;
	link->client.tx_block = false;
	link->client.knows_txdone = false;
	link->client.rx_callback = udx710_smsg_receive;
	link->client.tx_done = udx710_smsg_sent;
	link->channel = mbox_request_channel_byname(&link->client, mbox_name);
	if (IS_ERR(link->channel)) {
		ret = PTR_ERR(link->channel);
		kfree(link);
		return ERR_PTR(ret);
	}
	return link;
}
EXPORT_SYMBOL_GPL(udx710_smsg_link_open);

int udx710_smsg_link_send(struct udx710_smsg_link *link,
			  const struct udx710_smsg *message)
{
	struct udx710_smsg_tx *tx = NULL;
	unsigned long flags;
	int i, ret;

	if (!link || !message)
		return -EINVAL;
	spin_lock_irqsave(&link->lock, flags);
	if (link->stopping) {
		spin_unlock_irqrestore(&link->lock, flags);
		return -ESHUTDOWN;
	}
	for (i = 0; i < UDX710_SMSG_TX_SLOTS; i++) {
		if (!link->tx[i].busy) {
			tx = &link->tx[i];
			tx->busy = true;
			break;
		}
	}
	if (!tx) {
		spin_unlock_irqrestore(&link->lock, flags);
		return -ENOBUFS;
	}
	tx->words[0] = message->channel | ((u32)message->type << 8) |
		       ((u32)message->flag << 16);
	tx->words[1] = message->value;
	atomic_inc(&link->active);
	spin_unlock_irqrestore(&link->lock, flags);

	/* Never hold our lock across the provider call: tx_done may be prompt. */
	ret = mbox_send_message(link->channel, tx->words);
	if (ret < 0) {
		/* Nonblocking mailbox failure means this request was not queued. */
		spin_lock_irqsave(&link->lock, flags);
		tx->busy = false;
		spin_unlock_irqrestore(&link->lock, flags);
	}
	udx710_smsg_idle(link);
	return ret < 0 ? ret : 0;
}
EXPORT_SYMBOL_GPL(udx710_smsg_link_send);

/* Blocking applies only to local queue backpressure, never to CP readiness.
 * Hold an active reference across sleep so close can cancel/wake the caller.
 * Wait for an actual tx_done sequence, not merely a free private slot: the
 * mailbox framework queue may be full before all private slots are occupied.
 */
int udx710_smsg_link_send_timeout(struct udx710_smsg_link *link,
				  const struct udx710_smsg *message,
				  int timeout)
{
	unsigned long flags;
	long left = timeout < 0 ? MAX_SCHEDULE_TIMEOUT : timeout;
	int ret, sequence;

	if (!link || !message)
		return -EINVAL;
	if (!timeout)
		return udx710_smsg_link_send(link, message);
	might_sleep();
	spin_lock_irqsave(&link->lock, flags);
	if (link->stopping) {
		spin_unlock_irqrestore(&link->lock, flags);
		return -ESHUTDOWN;
	}
	atomic_inc(&link->active);
	spin_unlock_irqrestore(&link->lock, flags);
	for (;;) {
		sequence = atomic_read(&link->tx_done_seq);
		ret = udx710_smsg_link_send(link, message);
		if (ret != -ENOBUFS && ret != -EAGAIN && ret != -EBUSY)
			break;
		left = wait_event_interruptible_timeout(
			link->txwait,
			READ_ONCE(link->stopping) ||
				atomic_read(&link->tx_done_seq) != sequence,
			left);
		if (left < 0) {
			ret = left;
			break;
		}
		if (!left) {
			ret = -ETIME;
			break;
		}
		if (READ_ONCE(link->stopping)) {
			ret = -ESHUTDOWN;
			break;
		}
	}
	udx710_smsg_idle(link);
	return ret;
}
EXPORT_SYMBOL_GPL(udx710_smsg_link_send_timeout);

void udx710_smsg_link_close(struct udx710_smsg_link *link)
{
	unsigned long flags;

	if (!link)
		return;
	spin_lock_irqsave(&link->lock, flags);
	link->stopping = true;
	wake_up_interruptible_all(&link->txwait);
	spin_unlock_irqrestore(&link->lock, flags);
	wait_event(link->idle, !atomic_read(&link->active));
	/* Pair with the last sender's wakeup before releasing link storage. */
	spin_lock_irqsave(&link->lock, flags);
	spin_unlock_irqrestore(&link->lock, flags);
	/* Queued requests are abandoned by the framework without callbacks.
	 * Their storage belongs to this link and is released only after shutdown.
	 */
	mbox_free_channel(link->channel);
	kfree(link);
}
EXPORT_SYMBOL_GPL(udx710_smsg_link_close);

MODULE_DESCRIPTION("UDX710 AP SMSG standard mailbox transport boundary");
MODULE_LICENSE("GPL");
