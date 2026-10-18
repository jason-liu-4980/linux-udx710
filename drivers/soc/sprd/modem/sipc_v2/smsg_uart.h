/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _SPRD_SMSG_UART_H
#define _SPRD_SMSG_UART_H

#include <linux/types.h>

struct smsg;
struct smsg_uart_channel;

/* Fixed PM_SYS/COMM (6/1), local host mailbox only. Caller must serialize
 * endpoint lookup/binding/removal and keep IPC/link alive through release.
 * This interface does not acquire a device/driver lifetime reference.
 *
 * One successful reservation consumes this tuple for this kernel boot.
 * Cancel and release permanently quarantine it; neither sends CLOSE nor
 * establishes remote quiescence. No reset/reopen/reclaim interface exists.
 * Initialize caller state before reserve/start. All external users must be
 * joined before the caller frees their own state or releases the endpoint.
 *
 * The private slot/cache are boot-lifetime. IRQ ingress, admission, cancellation
 * and cache access share one short spinlock. No channel pointer escapes IRQ
 * while unreferenced. try_send holds that lock through the verified atomic-safe
 * mailbox enqueue; zero means queued, not delivered or processed remotely.
 */
int smsg_uart_reserve(struct smsg_uart_channel **out);
int smsg_uart_try_send(struct smsg_uart_channel *channel,
		       const struct smsg *msg);
int smsg_uart_try_recv(struct smsg_uart_channel *channel, struct smsg *msg);

/* Process context; absolute jiffies deadline, at most MAX_JIFFY_OFFSET ahead.
 * recv_until returns 0 with a message, or a negative error. wait_retry sleeps
 * at most interval ticks and returns 0 to retry, or a negative error. Each call
 * checks the same caller-owned absolute deadline; messages never renew it.
 */
int smsg_uart_recv_until(struct smsg_uart_channel *channel, struct smsg *msg,
			 unsigned long deadline);
int smsg_uart_wait_retry(struct smsg_uart_channel *channel,
			 unsigned long deadline, unsigned long interval);
void smsg_uart_cancel(struct smsg_uart_channel *channel);
int smsg_uart_release(struct smsg_uart_channel *channel);

/* Conservative history, also valid after cancel/release. False only means
 * this handle never called the transport; it is NOT a remote-silence proof.
 */
bool smsg_uart_may_have_sent(struct smsg_uart_channel *channel);

/* Legacy SBUF creation must acquire BEFORE lookup/allocation/publication and
 * retain this claim until its manager has been fully destroyed. Every failure
 * after a successful acquire must release. Other tuples are no-ops. Internal
 * managed-UART creation must not call these legacy admission helpers.
 */
int smsg_uart_legacy_sbuf_acquire(u8 dst, u8 channel);
void smsg_uart_legacy_sbuf_release(u8 dst, u8 channel);

#endif
