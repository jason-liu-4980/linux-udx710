// SPDX-License-Identifier: GPL-2.0-only
/* SY108 board MCU over its fixed 115200/8N1 UART.
 * ADC3..5 expose raw counts.
 * USB requests are acknowledged by the driver and announced as device events.
 * Power requests are acknowledged by the blocking reboot notifier, before
 * device shutdown closes the UART.
 */
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/reboot.h>
#include <linux/serdev.h>
#include <linux/spinlock.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#define SY108_FRAME_MAX 71
#define SY108_EVENT_MAX 16
#define SY108_TIMEOUT msecs_to_jiffies(2000)

struct sy108_event {
	u64 sequence;
	u8 minor;
};

struct sy108_mcu {
	struct serdev_device *serdev;
	struct mutex command_lock;
	spinlock_t rx_lock;
	struct completion response;
	struct work_struct event_work;
	struct notifier_block reboot;
	u8 stream[SY108_FRAME_MAX];
	unsigned int used;
	struct sy108_event events[SY108_EVENT_MAX];
	struct sy108_event last_event, poweroff;
	u64 next_event;
	unsigned int event_head, event_count;
	bool waiting, stopping, orphan_reply;
	u8 expected, expected_len, expected_channel;
	u8 reply[64];
	int response_error, link_error;
	u64 rx_bytes, valid_frames, bad_frames, requests, timeouts;
	u64 poweroff_events, usb_reset_events;
	u64 ack_attempts, ack_failures, last_ack_seq;
	int last_ack_error;
};

static size_t sy108_encode(u8 minor, const u8 *payload, unsigned int len,
			   u8 *frame)
{
	unsigned int i;
	u8 sum = 0;

	frame[0] = 0xaa;
	frame[1] = 0x60;
	put_unaligned_le16(len + 7, frame + 2);
	frame[4] = minor;
	frame[5] = len;
	if (len)
		memcpy(frame + 6, payload, len);
	for (i = 0; i < len + 6; i++)
		sum += frame[i];
	frame[len + 6] = sum;
	return len + 7;
}

static void sy108_handle_frame(struct sy108_mcu *mcu, const u8 *frame)
{
	u8 minor = frame[4], len = frame[5];

	lockdep_assert_held(&mcu->rx_lock);
	mcu->valid_frames++;
	if ((minor == 0x04 && !len) || (minor == 0x0a && len == 1)) {
		if (mcu->event_count == SY108_EVENT_MAX) {
			mcu->link_error = -EOVERFLOW;
		} else {
			struct sy108_event *event =
				&mcu->events[(mcu->event_head +
					      mcu->event_count++) %
					     SY108_EVENT_MAX];

			*event = (struct sy108_event){
				.sequence = ++mcu->next_event,
				.minor = minor,
			};
			if (minor == 0x04)
				mcu->poweroff_events++;
			else
				mcu->usb_reset_events++;
			schedule_work(&mcu->event_work);
		}
		/* The worker owns USB ACKs; power ACK waits for orderly poweroff. */
		return;
	}
	/* SY108_MCU_V07 echoes the ADC channel in payload[0]; match it to
	 * the pending request before accepting the sample.
	 */
	if (minor == 0x10 && len == 4 && frame[6] != mcu->expected_channel)
		return;
	if (mcu->orphan_reply && minor == mcu->expected &&
	    len == mcu->expected_len) {
		/* Exactly one timed-out request remains outstanding; no subsequent
		 * request was admitted. Drain its late reply without publishing it.
		 */
		mcu->orphan_reply = false;
		if (mcu->link_error == -ETIMEDOUT)
			mcu->link_error = 0;
		return;
	}
	if (mcu->waiting && minor == mcu->expected &&
	    len == mcu->expected_len) {
		memcpy(mcu->reply, frame + 6, len);
		mcu->response_error = 0;
		mcu->waiting = false;
		complete(&mcu->response);
	}
}

static size_t sy108_receive(struct serdev_device *serdev, const u8 *data,
			    size_t count)
{
	struct sy108_mcu *mcu = serdev_device_get_drvdata(serdev);
	unsigned long flags;
	size_t n;

	spin_lock_irqsave(&mcu->rx_lock, flags);
	if (mcu->stopping)
		goto out;
	mcu->rx_bytes += count;
	for (n = 0; n < count; n++) {
		mcu->stream[mcu->used++] = data[n];
		for (;;) {
			unsigned int total, i;
			u8 sum = 0;

			if (!mcu->used)
				break;
			if (mcu->stream[0] != 0xaa ||
			    (mcu->used > 1 && mcu->stream[1] != 0x60))
				goto discard;
			if (mcu->used < 4)
				break;
			total = get_unaligned_le16(mcu->stream + 2);
			if (total < 7 || total > SY108_FRAME_MAX ||
			    (mcu->used >= 6 && total != mcu->stream[5] + 7))
				goto invalid;
			if (mcu->used < total)
				break;
			for (i = 0; i + 1 < total; i++)
				sum += mcu->stream[i];
			if (sum != mcu->stream[total - 1])
				goto invalid;
			sy108_handle_frame(mcu, mcu->stream);
			mcu->used -= total;
			memmove(mcu->stream, mcu->stream + total, mcu->used);
			continue;
invalid:
			mcu->bad_frames++;
discard:
			mcu->used--;
			memmove(mcu->stream, mcu->stream + 1, mcu->used);
		}
	}
out:
	spin_unlock_irqrestore(&mcu->rx_lock, flags);
	return count;
}

static const struct serdev_device_ops sy108_serdev_ops = {
	.receive_buf = sy108_receive,
	.write_wakeup = serdev_device_write_wakeup,
};

static int sy108_request(struct sy108_mcu *mcu, u8 minor, const u8 *payload,
			 unsigned int len, u8 expected, unsigned int reply_len,
			 u8 *reply)
{
	u8 frame[11];
	unsigned long flags;
	size_t size;
	ssize_t written;
	int ret;

	if (len > 4 || reply_len > sizeof(mcu->reply))
		return -EINVAL;
	mutex_lock(&mcu->command_lock);
	spin_lock_irqsave(&mcu->rx_lock, flags);
	ret = mcu->stopping ? -ESHUTDOWN : mcu->link_error;
	if (!ret && (mcu->event_count || mcu->poweroff.sequence))
		ret = -EAGAIN;
	if (ret) {
		spin_unlock_irqrestore(&mcu->rx_lock, flags);
		goto unlock;
	}
	reinit_completion(&mcu->response);
	mcu->expected = expected;
	mcu->expected_len = reply_len;
	mcu->expected_channel = minor == 0x01 && len ? payload[0] : 0;
	mcu->response_error = -EINPROGRESS;
	mcu->waiting = true;
	mcu->requests++;
	spin_unlock_irqrestore(&mcu->rx_lock, flags);

	size = sy108_encode(minor, payload, len, frame);
	written = serdev_device_write(mcu->serdev, frame, size, SY108_TIMEOUT);
	if (written != (ssize_t)size) {
		ret = written < 0 ? written : -EIO;
		goto failed;
	}
	if (!wait_for_completion_timeout(&mcu->response, SY108_TIMEOUT)) {
		ret = -ETIMEDOUT;
		goto failed;
	}
	spin_lock_irqsave(&mcu->rx_lock, flags);
	ret = mcu->response_error;
	if (!ret)
		memcpy(reply, mcu->reply, reply_len);
	spin_unlock_irqrestore(&mcu->rx_lock, flags);
	goto unlock;
failed:
	spin_lock_irqsave(&mcu->rx_lock, flags);
	if (!mcu->waiting && !mcu->response_error && ret == -ETIMEDOUT) {
		/* Completion won the race with the timeout boundary. */
		memcpy(reply, mcu->reply, reply_len);
		ret = 0;
	} else {
		mcu->waiting = false;
		/* AA60 has no transaction ID. Block new requests until this
		 * fully transmitted request's late reply has been drained.
		 */
		mcu->link_error = ret;
		mcu->orphan_reply = ret == -ETIMEDOUT &&
				    written == (ssize_t)size;
		if (ret == -ETIMEDOUT)
			mcu->timeouts++;
	}
	spin_unlock_irqrestore(&mcu->rx_lock, flags);
unlock:
	mutex_unlock(&mcu->command_lock);
	return ret;
}

static void sy108_ack(struct sy108_mcu *mcu, const struct sy108_event *event)
{
	unsigned long flags;
	u8 zero = 0, frame[8];
	size_t size;
	ssize_t written;
	int ret;

	/* serdev serializes complete frames, including against ADC requests. */
	size = sy108_encode(event->minor == 4 ? 0x40 : 0xa0, &zero,
			    event->minor == 4 ? 0 : 1, frame);
	written = serdev_device_write(mcu->serdev, frame, size, SY108_TIMEOUT);
	ret = written == (ssize_t)size ? 0 : written < 0 ? written : -EIO;
	if (!ret)
		serdev_device_wait_until_sent(mcu->serdev, SY108_TIMEOUT);
	spin_lock_irqsave(&mcu->rx_lock, flags);
	mcu->ack_attempts++;
	if (ret)
		mcu->ack_failures++;
	mcu->last_ack_seq = event->sequence;
	mcu->last_ack_error = ret;
	spin_unlock_irqrestore(&mcu->rx_lock, flags);
	if (ret)
		dev_warn(&mcu->serdev->dev, "MCU ACK seq=%llu failed: %d\n",
			 event->sequence, ret);
}

static void sy108_events(struct work_struct *work)
{
	struct sy108_mcu *mcu = container_of(work, struct sy108_mcu, event_work);
	unsigned long flags;

	for (;;) {
		struct sy108_event event;
		char sequence[48];
		char *env[3];

		spin_lock_irqsave(&mcu->rx_lock, flags);
		if (mcu->stopping || !mcu->event_count) {
			spin_unlock_irqrestore(&mcu->rx_lock, flags);
			return;
		}
		event = mcu->events[mcu->event_head];
		mcu->event_head = (mcu->event_head + 1) % SY108_EVENT_MAX;
		mcu->event_count--;
		mcu->last_event = event;
		if (event.minor == 4 && !mcu->poweroff.sequence)
			mcu->poweroff = event;
		spin_unlock_irqrestore(&mcu->rx_lock, flags);

		if (event.minor == 0x0a)
			sy108_ack(mcu, &event);
		snprintf(sequence, sizeof(sequence), "MCU_SEQ=%llu", event.sequence);
		env[0] = event.minor == 4 ? "MCU_EVENT=poweroff" : "MCU_EVENT=usb-reset";
		env[1] = sequence;
		env[2] = NULL;
		kobject_uevent_env(&mcu->serdev->dev.kobj, KOBJ_CHANGE, env);
	}
}

static int sy108_poweroff_notify(struct notifier_block *nb,
				 unsigned long action, void *unused)
{
	struct sy108_mcu *mcu = container_of(nb, struct sy108_mcu, reboot);
	struct sy108_event event;
	unsigned long flags;

	if (action != SYS_POWER_OFF)
		return NOTIFY_DONE;
	/* Drain received requests while the UART is still available. This
	 * blocking notifier runs before device_shutdown(), unlike sys-off prepare.
	 */
	flush_work(&mcu->event_work);
	spin_lock_irqsave(&mcu->rx_lock, flags);
	event = mcu->poweroff;
	mcu->poweroff.sequence = 0;
	spin_unlock_irqrestore(&mcu->rx_lock, flags);
	if (event.sequence)
		sy108_ack(mcu, &event);
	/* The stock protocol attempts an ACK once; an I/O error does not veto
	 * the already requested system poweroff.
	 */
	return NOTIFY_DONE;
}

static ssize_t event_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct sy108_mcu *mcu = dev_get_drvdata(dev);
	struct sy108_event event;
	unsigned long flags;

	spin_lock_irqsave(&mcu->rx_lock, flags);
	/* Retain an early power request until the boot-policy adapter is ready. */
	event = mcu->poweroff.sequence ? mcu->poweroff : mcu->last_event;
	spin_unlock_irqrestore(&mcu->rx_lock, flags);
	if (!event.sequence)
		return sysfs_emit(buf, "none\n");
	return sysfs_emit(buf, "seq=%llu kind=%s\n", event.sequence,
			  event.minor == 4 ? "poweroff" : "usb-reset");
}
static DEVICE_ATTR_RO(event);

static int sy108_read_raw(struct iio_dev *indio,
			  const struct iio_chan_spec *chan, int *val, int *val2,
			  long mask)
{
	struct sy108_mcu *mcu = iio_priv(indio);
	u8 payload[4] = { chan->channel, 0, 0, 0 };
	u8 reply[4];
	int ret;

	if (mask != IIO_CHAN_INFO_RAW)
		return -EINVAL;
	ret = sy108_request(mcu, 0x01, payload, sizeof(payload), 0x10,
			    sizeof(reply), reply);
	if (ret)
		return ret;
	*val = get_unaligned_le16(reply + 2);
	return IIO_VAL_INT;
}

static int sy108_fwnode_xlate(struct iio_dev *indio,
			      const struct fwnode_reference_args *args)
{
	unsigned int i;

	if (args->nargs != 1)
		return -EINVAL;
	for (i = 0; i < indio->num_channels; i++)
		if (indio->channels[i].channel == args->args[0])
			return i;
	return -EINVAL;
}
static const struct iio_info sy108_iio_info = {
	.read_raw = sy108_read_raw,
	.fwnode_xlate = sy108_fwnode_xlate,
};
#define SY108_ADC(n)           \
	{ .type = IIO_VOLTAGE, \
	  .indexed = 1,        \
	  .channel = n,        \
	  .info_mask_separate = BIT(IIO_CHAN_INFO_RAW) }
static const struct iio_chan_spec sy108_channels[] = {
	SY108_ADC(3),
	SY108_ADC(4),
	SY108_ADC(5),
};

static ssize_t firmware_version_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct sy108_mcu *mcu = dev_get_drvdata(dev);
	char version[65];
	unsigned int i;
	int ret;

	ret = sy108_request(mcu, 0x06, NULL, 0, 0x60, 64, (u8 *)version);
	if (ret)
		return ret;
	version[64] = '\0';
	for (i = 0; version[i]; i++)
		if (version[i] < 32 || version[i] > 126)
			version[i] = '.';
	return sysfs_emit(buf, "%s\n", version);
}
static DEVICE_ATTR_RO(firmware_version);
static ssize_t status_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct sy108_mcu *mcu = dev_get_drvdata(dev);
	unsigned long flags;
	ssize_t count;

	spin_lock_irqsave(&mcu->rx_lock, flags);
	count = sysfs_emit(
		buf,
		"rx_bytes=%llu valid_frames=%llu bad_frames=%llu requests=%llu timeouts=%llu usb_reset=%llu poweroff=%llu link_error=%d ack_attempts=%llu ack_failures=%llu last_ack_seq=%llu last_ack_error=%d\n",
		mcu->rx_bytes, mcu->valid_frames, mcu->bad_frames,
		mcu->requests, mcu->timeouts, mcu->usb_reset_events,
		mcu->poweroff_events, mcu->link_error, mcu->ack_attempts,
		mcu->ack_failures, mcu->last_ack_seq, mcu->last_ack_error);
	spin_unlock_irqrestore(&mcu->rx_lock, flags);
	return count;
}
static DEVICE_ATTR_RO(status);
static struct attribute *sy108_attrs[] = {
	&dev_attr_firmware_version.attr,
	&dev_attr_status.attr,
	&dev_attr_event.attr,
	NULL,
};
ATTRIBUTE_GROUPS(sy108);

static void sy108_stop(void *data)
{
	struct sy108_mcu *mcu = data;
	unsigned long flags;

	spin_lock_irqsave(&mcu->rx_lock, flags);
	mcu->stopping = true;
	if (mcu->waiting) {
		mcu->response_error = -ESHUTDOWN;
		mcu->waiting = false;
		complete(&mcu->response);
	}
	spin_unlock_irqrestore(&mcu->rx_lock, flags);
	cancel_work_sync(&mcu->event_work);
}

static int sy108_probe(struct serdev_device *serdev)
{
	struct device *dev = &serdev->dev;
	struct gpio_desc *reset, *wake;
	struct iio_dev *indio;
	struct sy108_mcu *mcu;
	int ret;

	indio = devm_iio_device_alloc(dev, sizeof(*mcu));
	if (!indio)
		return -ENOMEM;
	mcu = iio_priv(indio);
	mcu->serdev = serdev;
	mutex_init(&mcu->command_lock);
	spin_lock_init(&mcu->rx_lock);
	init_completion(&mcu->response);
	INIT_WORK(&mcu->event_work, sy108_events);
	serdev_device_set_drvdata(serdev, mcu);
	serdev_device_set_client_ops(serdev, &sy108_serdev_ops);

	reset = devm_gpiod_get(dev, "rst-mcu", GPIOD_ASIS);
	if (IS_ERR(reset))
		return dev_err_probe(dev, PTR_ERR(reset),
				     "MCU reset GPIO unavailable\n");
	wake = devm_gpiod_get(dev, "wkup-mcu", GPIOD_ASIS);
	if (IS_ERR(wake))
		return dev_err_probe(dev, PTR_ERR(wake),
				     "MCU wake GPIO unavailable\n");
	/* Use the original board driver's raw reset=0 and wake=1 levels.
	 */
	ret = gpiod_direction_output_raw(reset, 0);
	if (ret)
		return ret;
	ret = gpiod_direction_output_raw(wake, 1);
	if (ret)
		return ret;
	ret = devm_serdev_device_open(dev, serdev);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(dev, sy108_stop, mcu);
	if (ret)
		return ret;
	if (serdev_device_set_baudrate(serdev, 115200) != 115200)
		return -EINVAL;
	serdev_device_set_flow_control(serdev, false);
	ret = serdev_device_set_parity(serdev, SERDEV_PARITY_NONE);
	if (ret)
		return ret;
	/* Publish the event queue before userspace issues read requests. An
	 * early power/USB event must remain observable even without policy yet.
	 */

	indio->name = "sy108-mcu";
	indio->info = &sy108_iio_info;
	indio->modes = INDIO_DIRECT_MODE;
	indio->channels = sy108_channels;
	indio->num_channels = ARRAY_SIZE(sy108_channels);
	ret = devm_iio_device_register(dev, indio);
	if (ret)
		return ret;
	mcu->reboot.notifier_call = sy108_poweroff_notify;
	ret = devm_register_reboot_notifier(dev, &mcu->reboot);
	if (ret)
		return ret;
	dev_info(dev, "MCU ADC3..5 and driver-owned USB/power ACKs registered\n");
	return 0;
}

static void sy108_shutdown(struct serdev_device *serdev)
{
	sy108_stop(serdev_device_get_drvdata(serdev));
}
static const struct of_device_id sy108_of_match[] = {
	{ .compatible = "meig,sy108-mcu" },
	{}
};
MODULE_DEVICE_TABLE(of, sy108_of_match);
static struct serdev_device_driver sy108_driver = {
	.probe = sy108_probe,
	.shutdown = sy108_shutdown,
	.driver = {
		.name = "sy108-mcu",
		.of_match_table = sy108_of_match,
		.dev_groups = sy108_groups,
		.suppress_bind_attrs = true,
	},
};
module_serdev_device_driver(sy108_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("SY108 board MCU serdev protocol, raw ADC and notifications");
