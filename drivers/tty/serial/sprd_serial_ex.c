// SPDX-License-Identifier: GPL-2.0-only
/*
 * UDX710 SP UART: AP owns the FIFO; PM_SYS proxies its receive interrupt.
 * PM_SYS masks IRQ2, writes a 128-byte SBUF notice and waits for one byte
 * from the AP before unmasking. Keep that session alive across TTY closes.
 * This is a fixed 115200/8N1 board link, not the ordinary AP IRQ UART.
 */
#include <linux/clk.h>
#include <linux/jiffies.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/serial_core.h>
#include <linux/sipc.h>
#include <linux/tty_flip.h>
#include <linux/workqueue.h>

#include "sprd_serial_common.h"
struct clk;
struct sprd_ex_clocks {
	struct clk *enable;
	struct clk *uart;
	struct clk *source;
};

#define EX_OPEN_MS 5000

/* Fixed original PM_SYS firmware protocol (nr_pmsys SHA256
 * 2b4c03f7f991871699a2a2dbe3621cd3d34b520251d9736c59c82fce10ede6b7):
 * UART IRQ2: mask IRQ, test IMSR & 0x2001, SBUF-write 128 bytes of 0x0f.
 * SBUF READ callback: consume one AP byte, unmask IRQ2 (value ignored).
 * AP owns RXD/TXD. Keep COMM/ring0 and clocks alive across tty open/close.
 * Firmware has no READY-state rebind or UART quiesce command. New Linux
 * boots require firmware OPEN/INIT; retained sessions cannot be recreated
 * by assuming that an AP-only reboot reset the peer.
 */

#define EX_BASE 0x620e0000
#define EX_LINE 1
#define EX_EVENTS 1
#define EX_NOTIFY_BYTES 128
#define EX_RX_BUDGET 64
#define EX_RX_IRQS 0x2081
#define EX_RETRY 1 /* jiffy; scheduling policy, not a protocol timeout */

enum ex_rx_phase { EX_NOTICE, EX_READ, EX_DRAIN, EX_ACK };

struct ex_event {
	u32 generation;
	u32 status;
	enum ex_rx_phase phase;
};

struct sprd_ex {
	struct uart_port port;
	struct sprd_ex_clocks clocks;
	struct device *endpoint;
	bool clocks_enabled, source_pinned, uart_pinned;
	resource_size_t region_base, region_size;
	bool region_claimed;
	struct workqueue_struct *queue;
	struct delayed_work work;
	struct sbuf_uart_notifier *notifier;
	struct sbuf_uart_backend *backend;
	/* All state below is protected by port.lock. Worker alone advances phase. */
	struct ex_event events[EX_EVENTS];
	unsigned int head, count;
	u32 generation;
	u64 config_seq, applied_seq, notice_seq;
	struct ktermios config;
	bool accepting, armed, opened, notice_wait;
	bool tx_requested, rx_enabled, rx_stopped, tx_empty;
	int fault;
};

static struct sprd_ex *to_ex(struct uart_port *port)
{
	return container_of(port, struct sprd_ex, port);
}

/* Called with port.lock, including every enqueue/rearm and admission change. */
static void ex_schedule(struct sprd_ex *ex, unsigned long delay)
{
	lockdep_assert_held(&ex->port.lock);
	if (ex->armed)
		mod_delayed_work(ex->queue, &ex->work, delay);
}

static void ex_fault(struct sprd_ex *ex, int error)
{
	lockdep_assert_held(&ex->port.lock);
	if (!ex->fault)
		ex->fault = error ?: -EIO;
	ex->accepting = false;
	ex->tx_requested = false;
	ex->tx_empty = false;
}

/* No MMIO, lease, SBUF read/ACK or callback-side synchronization. */
static void ex_notify(int event, u32 generation, void *data)
{
	struct sprd_ex *ex = data;
	struct uart_port *port = &ex->port;
	struct ex_event *ticket;
	unsigned long flags;

	uart_port_lock_irqsave(port, &flags);
	if (!ex->accepting)
		goto out;
	if (generation != ex->generation) {
		ex_fault(ex, -EPIPE);
		goto schedule;
	}
	if (event == SBUF_UART_NOTIFY_STOP) {
		ex_fault(ex, -ESHUTDOWN);
		goto schedule;
	}
	if (event == SBUF_NOTIFY_READ) {
		if (!++ex->notice_seq) {
			ex_fault(ex, -EOVERFLOW);
			goto schedule;
		}
		ex->notice_wait = false;
		/* READ is a wakeup, not one packet. Register also emits READ when
		 * the ring may be empty. Coalesce wakeups and drain until empty.
		 */
		if (!ex->count) {
			ticket = &ex->events[ex->head];
			*ticket = (struct ex_event){
				.generation = generation,
				.phase = EX_NOTICE,
			};
			ex->count = 1;
		}
	}
	/* READY never impersonates READ; WRITE can expedite a pending ACK. */
schedule:
	ex_schedule(ex, 0);
out:
	uart_port_unlock_irqrestore(port, flags);
}

static bool ex_live(struct sprd_ex *ex)
{
	lockdep_assert_held(&ex->port.lock);
	return ex->armed && !ex->fault;
}

static void ex_pop(struct sprd_ex *ex)
{
	lockdep_assert_held(&ex->port.lock);
	ex->head = (ex->head + 1) % EX_EVENTS;
	ex->count--;
}

/* One FIFO batch / one notification read / one ACK at most per invocation.
 * The worker owns a lease. Drop port.lock around all backend try-I/O.
 */
static int ex_rx_step(struct sprd_ex *ex)
{
	struct uart_port *port = &ex->port;
	struct ex_event *ticket;
	unsigned long flags;
	u8 notice[EX_NOTIFY_BYTES];
	enum ex_rx_phase phase;
	u64 read_seq;
	int ret;

	uart_port_lock_irqsave(port, &flags);
	if (!ex_live(ex) || !ex->count || ex->notice_wait ||
	    ex->applied_seq != ex->config_seq) {
		uart_port_unlock_irqrestore(port, flags);
		return 0;
	}
	ticket = &ex->events[ex->head];
	if (ticket->generation != ex->generation) {
		uart_port_unlock_irqrestore(port, flags);
		return -EPIPE;
	}
	if (ticket->phase == EX_NOTICE)
		ticket->phase = EX_READ;
	phase = ticket->phase;
	read_seq = ex->notice_seq;
	if (phase == EX_DRAIN) {
		/* CREAD/stop_rx suppress delivery, not protocol FIFO consumption. */
		sprd_rx_pio_budget(port, EX_RX_BUDGET,
				   ex->opened && ex->rx_enabled &&
					   !ex->rx_stopped);
		if (!(serial_in(port, SPRD_STS1) & SPRD_RX_FIFO_CNT_MASK))
			ticket->phase = EX_ACK;
		uart_port_unlock_irqrestore(port, flags);
		return 0;
	}
	uart_port_unlock_irqrestore(port, flags);

	if (phase == EX_READ) {
		ret = sbuf_uart_try_read(ex->notifier, ex->generation, notice,
					 sizeof(notice));
		if (ret == -EBUSY || ret == -EAGAIN)
			return 0;
		uart_port_lock_irqsave(port, &flags);
		if (ret == -ENODATA) {
			ticket->phase = EX_NOTICE;
			/* Do not discard a wakeup that raced this empty observation. */
			if (read_seq == ex->notice_seq)
				ex_pop(ex);
			ret = 0;
		} else if (ret > 0) {
			/* Only a real firmware notice means PM_SYS has masked IRQ2.
			 * Clearing ICLR on an empty/coalesced READ wakeup can erase a
			 * new timeout before PM_SYS observes it, stranding RX tail bytes.
			 */
			ticket->status |= serial_in(port, SPRD_IMSR);
			serial_out(port, SPRD_ICLR, ~0U);
			/* The notice is metadata. Never put it in the TTY buffer. */
			ticket->phase = EX_DRAIN;
			ret = 0;
		} else if (!ret) {
			ret = -EPROTO;
		}
		uart_port_unlock_irqrestore(port, flags);
		return ret;
	}

	ret = sbuf_uart_try_ack(ex->notifier, ex->generation);
	if (ret == -EBUSY || ret == -EAGAIN)
		return 0;
	if (ret != 1)
		return ret < 0 ? ret : -EPROTO;
	uart_port_lock_irqsave(port, &flags);
	/* The ACK unmasks PM_SYS IRQ2. A new notice may already be in the
	 * ring, with its wakeup coalesced into this work item. Try read again;
	 * only a successful read can advance to another ACK.
	 */
	ticket->phase = EX_NOTICE;
	ticket->status = 0;
	uart_port_unlock_irqrestore(port, flags);
	return 0;
}

/* The same PM_SYS UART implementation computes space as 128 - STS1[15:8].
 * Cap each batch at 64; do not impose a one-byte-per-jiffy baud-rate limit.
 */
static void ex_tx_step(struct sprd_ex *ex)
{
	struct uart_port *port = &ex->port;
	unsigned long flags;
	u32 status, space;

	uart_port_lock_irqsave(port, &flags);
	if (!ex_live(ex) || ex->applied_seq != ex->config_seq)
		goto out;
	status = (serial_in(port, SPRD_STS1) & SPRD_TX_FIFO_CNT_MASK) >> 8;
	space = status < SPRD_FIFO_SIZE ? SPRD_FIFO_SIZE - status : 0;
	if (ex->opened && ex->tx_requested && space) {
		sprd_tx_pio(port, min_t(u32, space, 64), UART_TX_NOSTOP);
		ex->tx_empty = false;
		if (!port->x_char &&
		    kfifo_is_empty(&port->state->port.xmit_fifo))
			ex->tx_requested = false;
	}
	/* Cached for tx_empty(), which cannot obtain a process-context lease. */
	ex->tx_empty = !(serial_in(port, SPRD_STS1) & SPRD_TX_FIFO_CNT_MASK);
out:
	uart_port_unlock_irqrestore(port, flags);
}

static void ex_report_fault(struct sprd_ex *ex)
{
	struct tty_struct *tty;

	if (!ex->port.state || !READ_ONCE(ex->opened))
		return;
	tty = tty_port_tty_get(&ex->port.state->port);

	if (tty) {
		set_bit(TTY_IO_ERROR, &tty->flags);
		wake_up_interruptible(&tty->read_wait);
		wake_up_interruptible(&tty->write_wait);
		tty_kref_put(tty);
	}
}

static void ex_work(struct work_struct *work)
{
	struct sprd_ex *ex =
		container_of(to_delayed_work(work), struct sprd_ex, work);
	struct uart_port *port = &ex->port;
	unsigned long flags;
	bool again;
	int ret, step;

	uart_port_lock_irqsave(port, &flags);
	if (!ex_live(ex) || !port->state) {
		ret = ex->fault;
		uart_port_unlock_irqrestore(port, flags);
		if (ret)
			ex_report_fault(ex);
		return;
	}
	if (ex->applied_seq != ex->config_seq) {
		sprd_rx_status_mask(port, &ex->config);
		ex->rx_enabled = ex->config.c_cflag & CREAD;
		ex->applied_seq = ex->config_seq;
	}
	uart_port_unlock_irqrestore(port, flags);

	ret = sbuf_uart_lease_try_begin(ex->notifier, ex->generation);
	if (ret == -EAGAIN || ret == -EBUSY)
		goto retry;
	if (ret)
		goto fail;
	/* Complete notice/read, up to two FIFO64 drains, and ACK in one work
	 * dispatch. Sleeping a jiffy between every phase needlessly approaches
	 * the 128-byte RX FIFO limit at 115200 baud.
	 */
	for (step = 0; step < 4; step++) {
		ret = ex_rx_step(ex);
		if (ret)
			break;
	}
	if (!ret)
		ex_tx_step(ex);
	if (sbuf_uart_lease_end(ex->notifier) && !ret)
		ret = -EIO;
	if (ret)
		goto fail;
	uart_port_lock_irqsave(port, &flags);
	again = ex_live(ex) &&
		((ex->count && !ex->notice_wait) || ex->tx_requested ||
		 !ex->tx_empty || ex->applied_seq != ex->config_seq);
	if (again)
		ex_schedule(ex, EX_RETRY);
	uart_port_unlock_irqrestore(port, flags);
	return;
retry:
	uart_port_lock_irqsave(port, &flags);
	if (ex_live(ex))
		ex_schedule(ex, EX_RETRY);
	uart_port_unlock_irqrestore(port, flags);
	return;
fail:
	uart_port_lock_irqsave(port, &flags);
	ex_fault(ex, ret);
	uart_port_unlock_irqrestore(port, flags);
	dev_err_ratelimited(port->dev, "PM_SYS UART session failed: %d\n", ret);
	ex_report_fault(ex);
}

/* Atomically callable serial-core methods only touch local state. */
static unsigned int ex_tx_empty(struct uart_port *port)
{
	return READ_ONCE(to_ex(port)->tx_empty) ? TIOCSER_TEMT : 0;
}

static unsigned int ex_get_mctrl(struct uart_port *port)
{
	return TIOCM_CTS | TIOCM_DSR | TIOCM_CAR;
}

static void ex_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
}
static void ex_break_ctl(struct uart_port *port, int state)
{
}

static void ex_start_tx(struct uart_port *port)
{
	struct sprd_ex *ex = to_ex(port);

	lockdep_assert_held(&port->lock);
	if (ex_live(ex) && ex->opened) {
		ex->tx_requested = true;
		ex->tx_empty = false;
		ex_schedule(ex, 0);
	}
}

static void ex_stop_tx(struct uart_port *port)
{
	lockdep_assert_held(&port->lock);
	to_ex(port)->tx_requested = false;
}

static void ex_stop_rx(struct uart_port *port)
{
	lockdep_assert_held(&port->lock);
	to_ex(port)->rx_stopped = true;
}

static void ex_set_termios(struct uart_port *port, struct ktermios *new,
			   const struct ktermios *old)
{
	struct sprd_ex *ex = to_ex(port);
	unsigned long flags;

	/* The evidenced MCU service uses 115200 8N1, with no flow control.
	 * Restrict wire format: no unsynchronized remote baud/format transition.
	 * Read/ignore-status policy may change, but data work waits for apply.
	 * No lease/mutex or hardware here: serial-core forbids sleeping.
	 */
	new->c_cflag &= ~(CSIZE | CSTOPB | PARENB | PARODD | CMSPAR | CRTSCTS);
	new->c_cflag |= CS8 | CLOCAL;
	tty_termios_encode_baud_rate(new, 115200, 115200);
	uart_port_lock_irqsave(port, &flags);
	uart_update_timeout(port, new->c_cflag, 115200);
	ex->config = *new;
	if (!++ex->config_seq)
		ex_fault(ex, -EOVERFLOW);
	ex_schedule(ex, 0);
	uart_port_unlock_irqrestore(port, flags);
}

/* Serial-core's port mutex serializes startup/shutdown. All callbacks and
 * worker rearm paths use port.lock. No worker needs the serial-core mutex.
 */
static int ex_detach_notifier(struct sprd_ex *ex)
{
	unsigned long flags;
	int ret = 0;

	uart_port_lock_irqsave(&ex->port, &flags);
	ex->accepting = false;
	ex->opened = false;
	ex->armed = false;
	ex->tx_requested = false;
	ex->tx_empty = false;
	uart_port_unlock_irqrestore(&ex->port, flags);
	if (ex->notifier) {
		sbuf_uart_cancel(ex->notifier);
		/* Wait for admitted callbacks before joining work they could queue. */
		ret = sbuf_uart_synchronize(ex->notifier);
	}
	cancel_delayed_work_sync(&ex->work);
	if (!ret && ex->notifier) {
		ret = sbuf_uart_unregister(ex->notifier);
		if (!ret)
			ex->notifier = NULL;
	}
	if (ret) {
		uart_port_lock_irqsave(&ex->port, &flags);
		ex_fault(ex, ret);
		uart_port_unlock_irqrestore(&ex->port, flags);
		/* Callbacks can still hold ex on an invariant failure. Never free it. */
		dev_crit(ex->port.dev,
			 "UART-ex teardown failed: %d; retaining resources\n",
			 ret);
	}
	return ret;
}

static int ex_startup(struct uart_port *port)
{
	struct sprd_ex *ex = to_ex(port);
	unsigned long flags;
	int ret;

	uart_port_lock_irqsave(port, &flags);
	ret = ex->fault;
	if (!ret) {
		ex->opened = true;
		ex->rx_stopped = false;
		ex->armed = true;
		ex_schedule(ex, 0);
	}
	uart_port_unlock_irqrestore(port, flags);
	return ret;
}

static void ex_shutdown(struct uart_port *port)
{
	struct sprd_ex *ex = to_ex(port);
	unsigned long flags;

	/* Closing a TTY does not close the firmware's SBUF session. Continue
	 * consuming notices and FIFO bytes, then ACK, without delivering data.
	 */
	uart_port_lock_irqsave(port, &flags);
	ex->opened = false;
	ex->tx_requested = false;
	ex_schedule(ex, 0);
	uart_port_unlock_irqrestore(port, flags);
}

static const char *ex_type(struct uart_port *port)
{
	return "SPRD UART-ex";
}
static void ex_release_port(struct uart_port *port)
{
}
static int ex_request_port(struct uart_port *port)
{
	return 0;
}
static void ex_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE)
		port->type = PORT_SPRD;
}

static int ex_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	/* No TIOCSSERIAL resource/clock/FIFO/flag changes on this fixed profile. */
	return -EOPNOTSUPP;
}

static const struct uart_ops ex_uart_ops = {
	.tx_empty = ex_tx_empty,
	.set_mctrl = ex_set_mctrl,
	.get_mctrl = ex_get_mctrl,
	.stop_tx = ex_stop_tx,
	.start_tx = ex_start_tx,
	.stop_rx = ex_stop_rx,
	.break_ctl = ex_break_ctl,
	.startup = ex_startup,
	.shutdown = ex_shutdown,
	.set_termios = ex_set_termios,
	.type = ex_type,
	.release_port = ex_release_port,
	.request_port = ex_request_port,
	.config_port = ex_config_port,
	.verify_port = ex_verify_port,
};

/* PM_SYS has no SBUF rebind operation. Both the endpoint and this serial
 * adapter are built-in, with manual unbind disabled, for a whole boot.
 */
static struct sprd_ex *ex_owner;

static struct device *ex_get_endpoint(void)
{
	struct device_node *np;
	struct platform_device *pdev;
	u32 dst;

	for_each_compatible_node(np, NULL, "sprd,sipc") {
		if (!of_device_is_available(np) ||
		    of_property_read_u32(np, "sprd,dst", &dst) ||
		    dst != SIPC_ID_PM_SYS)
			continue;
		pdev = of_find_device_by_node(np);
		of_node_put(np);
		if (!pdev)
			return ERR_PTR(-EPROBE_DEFER);
		device_lock(&pdev->dev);
		if (!device_is_bound(&pdev->dev) ||
		    !dev_get_drvdata(&pdev->dev)) {
			device_unlock(&pdev->dev);
			put_device(&pdev->dev);
			return ERR_PTR(-EPROBE_DEFER);
		}
		if (pdev->dev.driver->owner ||
		    !pdev->dev.driver->suppress_bind_attrs) {
			device_unlock(&pdev->dev);
			put_device(&pdev->dev);
			return ERR_PTR(-EOPNOTSUPP);
		}
		device_unlock(&pdev->dev);
		return &pdev->dev;
	}
	return ERR_PTR(-ENODEV);
}

static void ex_free(struct sprd_ex *ex)
{
	if (ex->queue)
		destroy_workqueue(ex->queue);
	if (ex->clocks_enabled) {
		clk_disable_unprepare(ex->clocks.enable);
		clk_disable_unprepare(ex->clocks.uart);
	}
	if (ex->uart_pinned)
		clk_rate_exclusive_put(ex->clocks.uart);
	if (ex->source_pinned)
		clk_rate_exclusive_put(ex->clocks.source);
	if (!IS_ERR_OR_NULL(ex->clocks.enable))
		clk_put(ex->clocks.enable);
	if (!IS_ERR_OR_NULL(ex->clocks.uart))
		clk_put(ex->clocks.uart);
	if (!IS_ERR_OR_NULL(ex->clocks.source))
		clk_put(ex->clocks.source);
	if (ex->port.membase)
		iounmap(ex->port.membase);
	if (ex->region_claimed)
		release_mem_region(ex->region_base, ex->region_size);
	if (!IS_ERR_OR_NULL(ex->endpoint))
		put_device(ex->endpoint);
	put_device(ex->port.dev);
	kfree(ex);
}

static int ex_enable_clocks(struct sprd_ex *ex)
{
	struct clk *parent;
	int ret;

	ex->clocks.enable = clk_get(ex->port.dev, "enable");
	if (IS_ERR(ex->clocks.enable))
		return PTR_ERR(ex->clocks.enable);
	ex->clocks.uart = clk_get(ex->port.dev, "uart");
	if (IS_ERR(ex->clocks.uart))
		return PTR_ERR(ex->clocks.uart);
	ex->clocks.source = clk_get(ex->port.dev, "source");
	if (IS_ERR(ex->clocks.source))
		return PTR_ERR(ex->clocks.source);
	ret = clk_rate_exclusive_get(ex->clocks.source);
	if (ret)
		return ret;
	ex->source_pinned = true;
	ret = clk_rate_exclusive_get(ex->clocks.uart);
	if (ret)
		return ret;
	ex->uart_pinned = true;
	parent = clk_get_parent(ex->clocks.uart);
	/* PM_SYS initialized this mux before registering its UART proxy. */
	if (IS_ERR_OR_NULL(parent) ||
	    !clk_is_match(parent, ex->clocks.source) ||
	    clk_get_rate(ex->clocks.uart) != SPRD_DEF_RATE ||
	    clk_get_rate(ex->clocks.source) != SPRD_DEF_RATE)
		return -EINVAL;
	ret = clk_prepare_enable(ex->clocks.uart);
	if (ret)
		return ret;
	ret = clk_prepare_enable(ex->clocks.enable);
	if (ret) {
		clk_disable_unprepare(ex->clocks.uart);
		return ret;
	}
	ex->clocks_enabled = true;
	ex->port.uartclk = clk_get_rate(ex->clocks.uart);
	return 0;
}

static int ex_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct sprd_ex *ex;
	struct ktermios config = { .c_cflag = B115200 | CS8 | CREAD | CLOCAL };
	unsigned long flags;
	int ret;

	if (ex_owner)
		return -EBUSY;
	ex = kzalloc_obj(*ex, GFP_KERNEL);
	if (!ex)
		return -ENOMEM;
	ex->port.dev = get_device(&pdev->dev);
	ex->endpoint = ex_get_endpoint();
	if (IS_ERR(ex->endpoint)) {
		ret = PTR_ERR(ex->endpoint);
		goto free;
	}
	ex->port.line = EX_LINE;
	ex->port.type = PORT_SPRD;
	ex->port.iotype = UPIO_MEM;
	ex->port.fifosize = SPRD_FIFO_SIZE;
	ex->port.ops = &ex_uart_ops;
	ex->port.flags = UPF_BOOT_AUTOCONF | UPF_FIXED_PORT | UPF_FIXED_TYPE;
	spin_lock_init(&ex->port.lock);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res || res->start != EX_BASE ||
	    resource_size(res) < SPRD_IMSR + sizeof(u32)) {
		ret = -EINVAL;
		goto free;
	}
	ex->port.mapbase = ex->region_base = res->start;
	ex->region_size = resource_size(res);
	if (!request_mem_region(res->start, ex->region_size,
				dev_name(&pdev->dev))) {
		ret = -EBUSY;
		goto free;
	}
	ex->region_claimed = true;
	ex->port.membase = ioremap(res->start, ex->region_size);
	if (!ex->port.membase) {
		ret = -ENOMEM;
		goto free;
	}
	ex->queue = alloc_ordered_workqueue("sprd-uart-ex", WQ_MEM_RECLAIM);
	if (!ex->queue) {
		ret = -ENOMEM;
		goto free;
	}
	INIT_DELAYED_WORK(&ex->work, ex_work);
	ret = sbuf_uart_backend_create(&ex->backend);
	if (ret)
		goto free;
	/* From here PM_SYS may learn the ring address: keep its storage and
	 * clock references even if negotiation fails. Never repeat OPEN in place.
	 */
	ex_owner = ex;
	ret = sbuf_uart_backend_start(ex->backend,
				      msecs_to_jiffies(EX_OPEN_MS));
	if (!ret)
		ret = sbuf_uart_backend_wait_ready(
			ex->backend, msecs_to_jiffies(EX_OPEN_MS));
	if (ret)
		goto retained;
	ret = sbuf_uart_backend_get_generation(ex->backend, &ex->generation);
	if (ret)
		goto retained;
	/* Firmware initializes the UART clock before opening COMM. Wait for
	 * that handshake before checking/pinning the functional clock parent.
	 */
	ret = ex_enable_clocks(ex);
	if (ret)
		goto retained;
	ex->accepting = true;
	ret = sbuf_uart_backend_register(ex->backend, ex_notify, ex,
					 &ex->notifier);
	if (ret)
		goto retained;
	/* No remote quiesce command exists. IEN masks this UART's interrupt
	 * source while AP programs the fixed board link. PM_SYS only reads IMSR.
	 */
	serial_out(&ex->port, SPRD_IEN, 0);
	serial_out(&ex->port, SPRD_CTL2, 0x4040);
	serial_out(&ex->port, SPRD_ICLR, ~0U);
	if (serial_in(&ex->port, SPRD_CTL1) &
	    (SPRD_DMA_EN | SPRD_LOOPBACK_EN)) {
		ret = -EOPNOTSUPP;
		goto retained;
	}
	tty_termios_encode_baud_rate(&config, 115200, 115200);
	sprd_set_termios(&ex->port, &config, NULL);
	ex->config = config;
	ex->config_seq = ex->applied_seq = 1;
	ex->rx_enabled = true;
	ex->tx_empty = true;
	serial_out(&ex->port, SPRD_IEN, EX_RX_IRQS);
	ret = uart_add_one_port(&sprd_uart_driver, &ex->port);
	if (ret)
		goto retained;
	platform_set_drvdata(pdev, ex);
	uart_port_lock_irqsave(&ex->port, &flags);
	ex->armed = true;
	ex_schedule(ex, 0);
	uart_port_unlock_irqrestore(&ex->port, flags);
	dev_info(&pdev->dev, "PM_SYS UART proxy ready at %u Hz, 115200 8N1\n",
		 ex->port.uartclk);
	return 0;
retained:
	if (ex->clocks_enabled)
		serial_out(&ex->port, SPRD_IEN, 0);
	ex_detach_notifier(ex);
	sbuf_uart_backend_cancel(ex->backend);
	sbuf_uart_backend_join(ex->backend, msecs_to_jiffies(EX_OPEN_MS));
	return dev_err_probe(&pdev->dev, ret,
			     "PM_SYS UART session failed; reboot to retry\n");
free:
	ex_free(ex);
	return ret;
}

static void ex_system_shutdown(struct platform_device *pdev)
{
	struct sprd_ex *ex = platform_get_drvdata(pdev);

	serial_out(&ex->port, SPRD_IEN, 0);
	ex_detach_notifier(ex);
	sbuf_uart_backend_cancel(ex->backend);
}

static int ex_suspend(struct device *dev)
{
	/* PM_SYS also owns suspend-time UART gating; retain the active proxy
	 * until that firmware suspend handshake is integrated.
	 */
	return -EOPNOTSUPP;
}

static const struct dev_pm_ops ex_pm_ops = {
	.suspend = ex_suspend,
	.freeze = ex_suspend,
	.poweroff = ex_suspend,
};

static const struct of_device_id ex_match[] = {
	{ .compatible = "sprd,sc9836-uart-ex" },
	{}
};
MODULE_DEVICE_TABLE(of, ex_match);

static struct platform_driver ex_platform_driver = {
	.probe = ex_probe,
	.shutdown = ex_system_shutdown,
	.driver = {
		.name = "sprd-serial-ex",
		.suppress_bind_attrs = true,
		.of_match_table = ex_match,
		.pm = &ex_pm_ops,
	},
};

int __init sprd_serial_ex_register(void)
{
	return platform_driver_register(&ex_platform_driver);
}

MODULE_DESCRIPTION("UDX710 PM_SYS interrupt-proxy UART");
MODULE_LICENSE("GPL");
