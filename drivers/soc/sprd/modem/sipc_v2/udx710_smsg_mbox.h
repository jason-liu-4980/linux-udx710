/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef UDX710_SMSG_MBOX_H
#define UDX710_SMSG_MBOX_H

#include <linux/types.h>

struct device;
struct udx710_smsg_link;

/* CPU-endian representation; the link encodes the two 32-bit mailbox words. */
struct udx710_smsg {
	u8 channel;
	u8 type;
	u16 flag;
	u32 value;
};

typedef void (*udx710_smsg_rx_t)(void *context,
				 const struct udx710_smsg *message);

/* mbox_name must identify a standard DT mboxes/mailbox-names channel. */
struct udx710_smsg_link *udx710_smsg_link_open(struct device *dev,
					       const char *mbox_name,
					       udx710_smsg_rx_t receive,
					       void *context);

/* Atomic-safe. Zero means QUEUED, not delivered or acknowledged by the CP.
 * A private copy is retained until tx_done. The caller retains its message.
 * Returns -ENOBUFS on backpressure, -ESHUTDOWN once close starts.
 */
int udx710_smsg_link_send(struct udx710_smsg_link *link,
			  const struct udx710_smsg *message);

/* Process-context queue admission: timeout is in jiffies, negative waits
 * interruptibly without a time limit. Nonzero requires sleepable context;
 * atomic callers must pass zero or use the atomic API.
 * Success retains the same QUEUED-only meaning as the atomic API.
 */
int udx710_smsg_link_send_timeout(struct udx710_smsg_link *link,
				  const struct udx710_smsg *message,
				  int timeout);

/* Process context only; never call from receive. Caller must stop future
 * users before close, and use a provider obeying mailbox shutdown semantics.
 */
void udx710_smsg_link_close(struct udx710_smsg_link *link);

#endif
