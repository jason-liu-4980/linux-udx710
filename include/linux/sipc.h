/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2012-2019 Spreadtrum Communications Inc.
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

#ifndef __SIPC_H
#define __SIPC_H

#include <linux/poll.h>
#include <linux/uaccess.h>

/* ****************************************************************** */
/* SMSG interfaces */

/* sipc processor ID definition */
enum {
	SIPC_ID_AP = 0, /* Application Processor */
	SIPC_ID_MINIAP, /* mini AP processor */
	SIPC_ID_CPW, /* WCDMA processor */
	SIPC_ID_WCN, /* Wireless Connectivity */
	SIPC_ID_GNSS, /* Gps processor(gnss) */
	SIPC_ID_PSCP, /* Protocol stack processor */
	SIPC_ID_PM_SYS, /* Power management processor */
	SIPC_ID_NR_PHY, /* New Radio PHY processor */
	SIPC_ID_V3_PHY, /* MODEM v3 PHY processor */
	SIPC_ID_NR, /* Max processor number */
};

#define SIPC_ID_LTE SIPC_ID_PSCP

/* share-mem ring buffer short message */
struct smsg {
	u8 channel; /* channel index */
	u8 type; /* msg type */
	u16 flag; /* msg flag */
	u32 value; /* msg value */
};

/* smsg channel definition */
enum {
	SMSG_CH_CTRL = 0, /* some emergency control */
	SMSG_CH_COMM, /* general communication channel */
	SMSG_CH_IMSBR_DATA, /* ims bridge data channel */
	SMSG_CH_IMSBR_CTRL, /* ims bridge control channel */
	SMSG_CH_PIPE, /* general pipe channel */
	SMSG_CH_PLOG, /* pipe for debug log/dump */
	SMSG_CH_TTY, /* virtual serial for telephony */
	SMSG_CH_DATA0, /* 2G/3G wirleless data */
	SMSG_CH_DATA1, /* 2G/3G wirleless data */
	SMSG_CH_DATA2, /* 2G/3G wirleless data */
	SMSG_CH_VBC, /* audio conrol channel */
	SMSG_CH_PLAYBACK, /* audio playback channel */
	SMSG_CH_CAPTURE, /* audio capture channel */
	SMSG_CH_MONITOR_AUDIO, /* audio monitor channel */
	SMSG_CH_CTRL_VOIP, /* audio voip conrol channel */
	SMSG_CH_PLAYBACK_VOIP, /* audio voip playback channel */
	SMSG_CH_CAPTURE_VOIP, /* audio voip capture channel */
	SMSG_CH_MONITOR_VOIP, /* audio voip monitor channel */
	SMSG_CH_DATA3, /* 2G/3G wirleless data */
	SMSG_CH_DATA4, /* 2G/3G wirleless data */
	SMSG_CH_DATA5, /* 2G/3G wirleless data */
	SMSG_CH_DIAG, /* pipe for debug log/dump */
	SMSG_CH_PM_CTRL, /* power management control */
	SMSG_CH_DUAL_SIM_PLUG, /* dual sim plug channel */
	SMSG_CH_END /* will not allow add channel in here */
};

/* smsg channel definition */
enum {
	/* 2G/3G wirleless data, channel 24~39 */
	SMSG_CH_DATA_BASE = 24,
	SMSG_CH_DATA6 = SMSG_CH_DATA_BASE,
	SMSG_CH_DATA7,
	SMSG_CH_DATA8,
	SMSG_CH_DATA9,
	SMSG_CH_DATA10,
	SMSG_CH_DATA11,
	SMSG_CH_DATA12,
	SMSG_CH_DATA13,

	/* general pipe channel,  channel 40~59 */
	SMSG_CH_PIPE_BASE = 40,
	SMSG_CH_NV = SMSG_CH_PIPE_BASE,
	SMSG_CH_DVFS,
	SMSG_CH_PIPE2,
	SMSG_CH_PIPE3,

	/* pipe for debug log/dump channel 60~79 */
	SMSG_CH_PLOG_BASE = 60,
	SMSG_CH_PLOG0 = SMSG_CH_PLOG_BASE,
	SMSG_CH_PLOG1,
	SMSG_CH_PLOG2,
	SMSG_CH_PLOG3,

	/* virtual serial for telephony,  channel 80~99*/
	SMSG_CH_TTY_BASE = 80,
	SMSG_CH_TTY0 = SMSG_CH_TTY_BASE,
	SMSG_CH_TTY1,
	SMSG_CH_TTY2,
	SMSG_CH_TTY3,

	/* some emergency control, channel 100~119 */
	SMSG_CH_CTRL_BASE = 100,
	SMSG_CH_PMSYS_DBG = SMSG_CH_CTRL_BASE,
	SMSG_CH_CTRL1,
	SMSG_CH_CTRL2,
	SMSG_CH_CTRL3,

	/* general communication, channel 120~129 */
	SMSG_CH_COMM_BASE = 120,
	SMSG_CH_COMM_SIPA = SMSG_CH_COMM_BASE,
	SMSG_CH_COMM1,
	SMSG_CH_COMM2,
	SMSG_CH_COMM3,

	/* audio  channel, channel 130 ~149 */
	SMSG_CH_AUDIO_BASE = 130,
	SMSG_CH_AGDSP_ACCESS = SMSG_CH_AUDIO_BASE, /* audio conrol channel */
	SMSG_CH_PLAYBACK_DEEP,
	SMSG_CH_AUDIO2,
	SMSG_CH_AUDIO3,

	/* VOIP  channel, channel 150 ~169 */
	SMSG_CH_VOIP_BASE = 150,
	SMSG_CH_VOIP0 = SMSG_CH_VOIP_BASE, /* audio voip conrol channel */
	SMSG_CH_VOIP_DEEP, /* audio voip playback channel */
	SMSG_CH_VOIP2, /* audio voip capture channel */
	SMSG_CH_VOIP3, /* audio voip monitor channel */

	/* RPC server channel, channel 170~189  */
	SMSG_CH_RPC_BASE = 170,
	SMSG_CH_RPC0 = SMSG_CH_RPC_BASE,
	SMSG_CH_RPC1,
	SMSG_CH_RPC2,
	SMSG_CH_RPC3,

	/* RESERVE group 1, channel 190 ~209 */
	SMSG_CH_RESERVE1_BASE = 190,

	/* RESERVE group 2, channel 210 ~129 */
	SMSG_CH_RESERVE2_BASE = 210,

	/* RESERVE group 3, channel 230 ~244 */
	SMSG_CH_RESERVE3_BASE = 230,

	/* RESERVE group 4, channel 245 ~254 */
	SMSG_CH_RESERVE4_BASE = 245,

	/* total channel number  255, the max chanel number is 254*/
	SMSG_CH_NR = 255
};
#define INVALID_CHANEL_INDEX SMSG_CH_NR

#ifdef CONFIG_SPRD_SIPC_V2
/* modem type */
enum {
	SOC_MODEM = 0,
	PCIE_MODEM,
};
#endif

/* only be configured in sipc_config is valid channel */
struct sipc_config {
	u8 channel;
	char *name;
};

static const struct sipc_config sipc_cfg[] = {
	{ SMSG_CH_CTRL, "com control" }, /* chanel 0 */
	{ SMSG_CH_COMM, "com communication" }, /* chanel 1 */
	{ SMSG_CH_PM_CTRL, "pm contrl" }, /* chanel 22 */
	{ SMSG_CH_PMSYS_DBG, "pm debug contrl" }, /* chanel 100 */
	{ SMSG_CH_DUAL_SIM_PLUG, "dual sim plug" }, /* chanel 23 */
	{ SMSG_CH_PIPE, "pipe0" }, /* chanel 4 */
	{ SMSG_CH_PLOG, "plog" }, /* chanel 5 */
	{ SMSG_CH_DIAG, "diag" }, /* chanel 21 */
	{ SMSG_CH_TTY, "stty chanel" }, /* chanel 6 */
	{ SMSG_CH_DATA0, "seth0" }, /* chanel 7 */
	{ SMSG_CH_DATA1, "seth1" }, /* chanel 8 */
	{ SMSG_CH_DATA2, "seth2" }, /* chanel 9 */
	{ SMSG_CH_DATA3, "seth3" }, /* chanel 18 */
	{ SMSG_CH_DATA4, "seth4" }, /* chanel 19 */
	{ SMSG_CH_DATA5, "seth5" }, /* chanel 20 */
	{ SMSG_CH_DATA6, "seth6" }, /* chanel 24 */
	{ SMSG_CH_DATA7, "seth7" }, /* chanel 25 */
	{ SMSG_CH_DATA8, "seth8" }, /* chanel 26 */
	{ SMSG_CH_DATA9, "seth9" }, /* chanel 27 */
	{ SMSG_CH_DATA10, "seth10" }, /* chanel 28 */
	{ SMSG_CH_DATA11, "seth11" }, /* chanel 29 */
	{ SMSG_CH_DATA12, "seth12" }, /* chanel 30 */
	{ SMSG_CH_DATA13, "seth13" }, /* chanel 31 */
	{ SMSG_CH_VBC, "audio control" }, /* chanel 10 */
	{ SMSG_CH_PLAYBACK, "audio playback" }, /* chanel 11 */
	{ SMSG_CH_CAPTURE, "audio capture" }, /* chanel 12 */
	{ SMSG_CH_MONITOR_AUDIO, "audio monitor" }, /* chanel 13 */
	{ SMSG_CH_AGDSP_ACCESS, "agdsp access" }, /* chanel 13 */
	{ SMSG_CH_CTRL_VOIP, "VOIP conrol" }, /* chanel 14 */
	{ SMSG_CH_PLAYBACK_VOIP, "VOIP playback" }, /* chanel 15 */
	{ SMSG_CH_CAPTURE_VOIP, "VOIP capture" }, /* chanel 16 */
	{ SMSG_CH_MONITOR_VOIP, "VOIP monitor" }, /* chanel 17 */
	{ SMSG_CH_PLAYBACK_DEEP, "audio playback deep" }, /*channel 131*/
	{ SMSG_CH_IMSBR_DATA, "imsbr data" }, /* chanel 2 */
	{ SMSG_CH_IMSBR_CTRL, "imsbr control" }, /* channel 3 */
	{ SMSG_CH_VOIP_DEEP, "audio voip deep" }, /*channel 151*/
	{ SMSG_CH_DVFS, "dvfs" }, /* channel 41 */
	{ SMSG_CH_COMM_SIPA, "sipa" }, /* channel 120 */
	{ SMSG_CH_NV, "nvsync" }, /* channel 40 */
};

#define SMSG_VALID_CH_NR (sizeof(sipc_cfg) / sizeof(struct sipc_config))

/* smsg type definition */
enum {
	SMSG_TYPE_NONE = 0,
	SMSG_TYPE_OPEN, /* first msg to open a channel */
	SMSG_TYPE_CLOSE, /* last msg to close a channel */
	SMSG_TYPE_DATA, /* data, value=addr, no ack */
	SMSG_TYPE_EVENT, /* event with value, no ack */
	SMSG_TYPE_CMD, /* command, value=cmd */
	SMSG_TYPE_DONE, /* return of command */
	SMSG_TYPE_SMEM_ALLOC, /* allocate smem, flag=order */
	SMSG_TYPE_SMEM_FREE, /* free smem, flag=order, value=addr */
	SMSG_TYPE_SMEM_DONE, /* return of alloc/free smem */
	SMSG_TYPE_FUNC_CALL, /* RPC func, value=addr */
	SMSG_TYPE_FUNC_RETURN, /* return of RPC func */
	SMSG_TYPE_DIE,
	SMSG_TYPE_DFS,
	SMSG_TYPE_DFS_RSP,
	SMSG_TYPE_ASS_TRG,
	SMSG_TYPE_HIGH_OFFSET, /* client sipc get high offset from host */
	SMSG_TYPE_NR, /* total type number */
};

/* flag for OPEN/CLOSE msg type */
#define SMSG_OPEN_MAGIC 0xBEEE
#define SMSG_CLOSE_MAGIC 0xEDDD

/*
 * sipc_get_wakeup_flag
 * after the wakeup flag be set, the fist smsg will be print
 * @parameters: void
 * @return: int
 */
int sipc_get_wakeup_flag(void);

/*
 * sipc_set_wakeup_flag
 * after the wakeup flag be set, the fist smsg will be print
 * @parameters: void
 * @return: no return value
 */
void sipc_set_wakeup_flag(void);

/*
 * sipc_clear_wakeup_flag
 * clear the wake up flag
 * @parameters: void
 * @return: no return value
 */
void sipc_clear_wakeup_flag(void);

/**
 * smsg_ch_open -- open a channel for smsg
 *
 * @dst: dest processor ID
 * @channel: channel ID
 * @timeout: jiffies, 0 means no wait, -1 means unlimited
 * @return: 0 on success, <0 on failure
 */
int smsg_ch_open(u8 dst, u8 channel, int timeout);

/**
 * smsg_ch_close -- close a channel for smsg
 *
 * @dst: dest processor ID
 * @channel: channel ID
 * @timeout: jiffies, 0 means no wait, -1 means unlimited
 * @return: 0 on success, <0 on failure
 */
int smsg_ch_close(u8 dst, u8 channel, int timeout);

/**
 * smsg_senddie -- send a MSG_TYPE_DIE message to pubcp
 *
 * @dst: dest processor ID
 * @return: 0 on success, <0 on failure
 */
int smsg_senddie(u8 dst);

/**
 * smsg_send -- send smsg
 *
 * A nonzero timeout requires sleepable process context. Atomic callers must
 * pass zero. Success means accepted by the AP transport, not CP acknowledgment.
 *
 * @dst: dest processor ID
 * @msg: smsg body to be sent
 * @timeout: jiffies, 0 means no wait, -1 means unlimited
 * @return: 0 on success, <0 on failure
 */
int smsg_send(u8 dst, struct smsg *msg, int timeout);

/**
 * smsg_recv -- poll and recv smsg
 *
 * @dst: dest processor ID
 * @msg: smsg body to be received, channel should be filled as input
 * @timeout: jiffies, 0 means no wait, -1 means unlimited
 * @return: 0 on success, <0 on failure
 */
int smsg_recv(u8 dst, struct smsg *msg, int timeout);

/**
 * sipc_channel2index
 *
 * only be configured in sipc_config is valid channel
 * @ch: channel number
 * @return:  channel index ,if return index is INVALID_CHANEL_INDEX ,
 *               it indicate it is a invalid chanel
 */
u8 sipc_channel2index(u8 ch);

int smsg_ch_wake_unlock(u8 dst, u8 channel);

#if defined(CONFIG_DEBUG_FS)
void sipc_debug_putline(struct seq_file *m, char c, int n);
#endif

/* quickly fill a smsg body */
static inline void smsg_set(struct smsg *msg, u8 channel, u8 type, u16 flag,
			    u32 value)
{
	msg->channel = channel;
	msg->type = type;
	msg->flag = flag;
	msg->value = value;
}

/* ack an open msg for modem recovery */
static inline void smsg_open_ack(u8 dst, u16 channel)
{
	struct smsg mopen;

	pr_info("%s: channel %d-%d!\n", __func__, dst, channel);
	smsg_set(&mopen, channel, SMSG_TYPE_OPEN, SMSG_OPEN_MAGIC, 0);
	smsg_send(dst, &mopen, -1);
}

/* ack an close msg for modem recovery */
static inline void smsg_close_ack(u8 dst, u16 channel)
{
	struct smsg mclose;

	pr_info("%s: channel %d-%d!\n", __func__, dst, channel);
	smsg_set(&mclose, channel, SMSG_TYPE_CLOSE, SMSG_CLOSE_MAGIC, 0);
	smsg_send(dst, &mclose, -1);
}

/* ****************************************************************** */
/* SMEM interfaces */
#ifdef CONFIG_SPRD_SIPC_V2
/**
 * smem_alloc -- allocate shared memory block
 *
 * @dst: dest processor ID
 * @size: size to be allocated, page-aligned
 * @return: phys addr or 0 if failed
 */
u32 smem_alloc(u8 dst, u32 size);

/**
 * smem_free -- free shared memory block
 *
 * @dst: dest processor ID
 * @addr: smem phys addr to be freed
 * @order: size to be freed
 */
void smem_free(u8 dst, u32 addr, u32 size);

/**
 * shmem_ram_unmap -- for sipc unmap ram address
 *
 * @mem: vir mem
 */
void shmem_ram_unmap(u8 dst, const void *mem);

/**
 * shmem_ram_vmap_nocache -- for sipc map ram address
 *
 * @start: start address
 * @size: size to be allocated, page-aligned
 * @return: phys addr or 0 if failed
 */
void *shmem_ram_vmap_nocache(u8 dst, phys_addr_t start, size_t size);

/**
 * shmem_ram_vmap_cache -- for sipc map ram address
 *
 * @start: start address
 * @size: size to be allocated, page-aligned
 * @return: phys addr or 0 if failed
 */
void *shmem_ram_vmap_cache(u8 dst, phys_addr_t start, size_t size);

void smem_free(u8 dst, u32 addr, u32 size);

/**
 * modem_ram_unmap -- for modem unmap ram address
 *
 * @mem: vir mem
 * @modem_type: soc modem, pcie modem
 */
void modem_ram_unmap(u32 modem_type, const void *mem);

/**
 * shmem_ram_vmap_nocache -- for modem map ram address
 *
 * @modem_type: soc modem, pcie modem
 * @start: start address
 * @size: size to be allocated, page-aligned
 * @return: phys addr or 0 if failed
 */
void *modem_ram_vmap_nocache(u32 modem_type, phys_addr_t start, size_t size);

/**
 * modem_ram_vmap_cache -- for modem map ram address
 *
 * @modem_type: soc modem, pcie modem
 * @start: start address
 * @size: size to be allocated, page-aligned
 * @return: phys addr or 0 if failed
 */
void *modem_ram_vmap_cache(u32 modem_type, phys_addr_t start, size_t size);
#else

/**
 * smem_alloc -- allocate shared memory block
 *
 * @size: size to be allocated, page-aligned
 * @return: phys addr or 0 if failed
 */
u32 smem_alloc(u32 size);

/**
 * smem_alloc_ex -- allocate shared memory block
 *
 * @size: size to be allocated, page-aligned
 * @paddr: which pool wil used to malloc
 * @return: phys addr or 0 if failed
 */
u32 smem_alloc_ex(u32 size, u32 paddr);

/**
 * smem_free -- free shared memory block
 *
 * @addr: smem phys addr to be freed
 * @order: size to be freed
 */
void smem_free(u32 addr, u32 size);

/**
 * smem_free_ex -- free shared memory block
 *
 * @addr: smem phys addr to be freed
 * @paddr: which pool wil used to malloc
 * @order: size to be freed
 */
void smem_free_ex(u32 addr, u32 size, u32 paddr);

/**
 * shmem_ram_unmap -- for sipc unmap ram address
 *
 * @mem: vir mem
 */
void shmem_ram_unmap(const void *mem);

/**
 * shmem_ram_vmap_nocache -- for sipc map ram address
 *
 * @start: start address
 * @size: size to be allocated, page-aligned
 * @return: phys addr or 0 if failed
 */
void *shmem_ram_vmap_nocache(phys_addr_t start, size_t size);

/**
 * shmem_ram_vmap_cache -- for sipc map ram address
 *
 * @start: start address
 * @size: size to be allocated, page-aligned
 * @return: phys addr or 0 if failed
 */
void *shmem_ram_vmap_cache(phys_addr_t start, size_t size);
#endif
/**
 * sbuf_set_no_need_wake_lock
 *
 * @dst: dest processor ID
 * @bufnum: which buffer to be set
 * @return: none
 */

void sbuf_set_no_need_wake_lock(u8 dst, u8 channel, u32 bufnum);

#ifdef CONFIG_SPRD_SIPC_V2
/**
 * sbuf_create -- create pipe ring buffers on a channel
 *
 * @dst: dest processor ID
 * @channel: channel ID
 * @txbufsize: tx buffer size
 * @rxbufsize: rx buffer size
 * @bufnum: how many buffers to be created
 * @return: 0 on success, <0 on failure
 */

int sbuf_create(u8 dst, u8 channel, u32 bufnum, u32 txbufsize, u32 rxbufsize);
#else
/**
 * sbuf_create_ex -- create pipe ring buffers on a channel
 *
 * @dst: dest processor ID
 * @channel: channel ID
 * @smem: smem ID,default is 0
 * @txbufsize: tx buffer size
 * @rxbufsize: rx buffer size
 * @bufnum: how many buffers to be created
 * @return: 0 on success, <0 on failure
 */
int sbuf_create_ex(u8 dst, u8 channel, u32 smem, u32 bufnum, u32 txbufsize,
		   u32 rxbufsize);
#define sbuf_create(dst, channel, bufnum, txbufsize, rxbufsize) \
	sbuf_create_ex(dst, channel, 0, bufnum, txbufsize, rxbufsize)
#endif

/**
 * sbuf_destroy -- destroy the pipe ring buffers on a channel
 *
 * @dst: dest processor ID
 * @channel: channel ID
 * @return: 0 on success, <0 on failure
 */
void sbuf_destroy(u8 dst, u8 channel);

/**
 * sbuf_write -- write data to a sbuf
 *
 * @dst: dest processor ID
 * @channel: channel ID
 * @bufid: buffer ID
 * @buf: data to be written
 * @len: data length
 * @timeout: jiffies, 0 means no wait, -1 means unlimited
 * @return: written bytes on success, <0 on failure
 */
int sbuf_write(u8 dst, u8 channel, u32 bufid, void *buf, u32 len, int timeout);

/**
 * sbuf_read -- write data to a sbuf
 *
 * @dst: dest processor ID
 * @channel: channel ID
 * @bufid: buffer ID
 * @buf: data to be written
 * @len: data length
 * @timeout: jiffies, 0 means no wait, -1 means unlimited
 * @return: read bytes on success, <0 on failure
 */
int sbuf_read(u8 dst, u8 channel, u32 bufid, void *buf, u32 len, int timeout);

/**
 * sbuf_poll_wait -- poll sbuf read/write, used in spipe driver
 *
 * @dst: dest processor ID
 * @channel: channel ID
 * @bufid: buffer ID
 * @file: struct file handler
 * @wait: poll table
 * @return: POLLIN or POLLOUT
 */
/* Explicit user-buffer entry points; sbuf_read/write accept kernel memory only. */
int sbuf_read_user(u8 dst, u8 channel, u32 bufid, void __user *buf, u32 len,
		   int timeout);
int sbuf_write_user(u8 dst, u8 channel, u32 bufid, const void __user *buf,
		    u32 len, int timeout);

/*
 * Caller owns the SBUF manager lifetime. 0=idle, 1=transport ready, 2=stopping;
 * neither state nor generation authenticates the remote processor.
 */
int sbuf_get_session(u8 dst, u8 channel, u32 *generation, u32 *state);
int sbuf_poll_wait(u8 dst, u8 channel, u32 bufid, struct file *file,
		   poll_table *wait);

/**
 * sbuf_status -- get sbuf status
 *
 * @dst: dest processor ID
 * @channel: channel ID
 * @return: 0 when ready, <0 when broken
 */
int sbuf_status(u8 dst, u8 channel);

#define SBUF_NOTIFY_READY 0x00
#define SBUF_NOTIFY_READ 0x01
#define SBUF_NOTIFY_WRITE 0x02
/* Private managed-UART subscription only; never a wire message. */
#define SBUF_UART_NOTIFY_STOP 0x100
/**
 * sbuf_register_notifier -- register a callback that's called
 *		when a tx sbuf is available or a rx sbuf is received.
 *		non-blocked sbuf_read can be called.
 *
 * @dst: dest processor ID
 * @channel: channel ID
 * @bufid: buf ID
 * @handler: a callback handler
 * @event: NOTIFY_READ, NOTIFY_WRITE, or both
 * @data: opaque data passed to the receiver
 * @return: 0 on success, <0 on failure
 */
int sbuf_register_notifier(u8 dst, u8 channel, u32 bufid,
			   void (*handler)(int event, void *data), void *data);

/* UART-ex opt-in API (SIPC_V2 only), restricted to PM_SYS/COMM/ring0 and a
 * caller-owned 1 x 512/512 SBUF. No CP/AT clients are converted to this API.
 *
 * The creator must serialize create/register/unregister/destroy and keep the
 * parent SIPC endpoint bound/alive. Lookup is NOT independently lifetime-safe.
 * No legacy data/notifier call may be in flight at attach or used while the
 * owner is retained. Nonempty legacy data calls then fail -EBUSY as a guard,
 * not as independent lifetime synchronization. A retained UART owner rejects
 * sbuf_destroy until unregister succeeds.
 * Registration may synchronously call notify before it returns; initialize
 * all callback state first. notify must only enqueue local work, never wait
 * or call backend I/O (READY may be delivered under the control mutex).
 * Each notification carries its acquisition-time AP session generation.
 *
 * cancel is sticky, non-waiting and callable by notify. synchronize requires
 * cancel first, sleeps outside all locks, and returns -EDEADLK for a callback
 * waiting on itself. Before unregister, the caller must also join every
 * queued worker, API caller and registration/control call; notifier/I/O refs
 * do not own caller queues. unregister cancels, synchronizes, releases the
 * retained owner, and frees the handle. No handle access is legal afterwards.
 *
 * try_read/try_ack are process-context kernel-buffer calls without transport
 * or mutex waits; never hold a UART spinlock. Empty/busy returns immediately.
 * Read length is 1..128. A failed read may have copied bytes: discard output
 * on error. try_ack writes exactly 0x01 and returns 1 if committed; do not retry
 * a positive result. Both validate expected generation through pointer commit.
 * cancel prevents later commits, but cannot undo an earlier committed ACK.
 * This API does not decide closed-port, TX-ready, suspend or clock policy.
 */

struct sbuf_uart_backend;
struct sbuf_uart_notifier;
int sbuf_uart_backend_register(struct sbuf_uart_backend *backend,
			       void (*notify)(int event, u32 generation,
					      void *data),
			       void *data, struct sbuf_uart_notifier **out);
int sbuf_uart_register(u8 dst, u8 channel, u32 bufid,
		       void (*notify)(int event, u32 generation, void *data),
		       void *data, struct sbuf_uart_notifier **out);
void sbuf_uart_cancel(struct sbuf_uart_notifier *uart);
int sbuf_uart_synchronize(struct sbuf_uart_notifier *uart);
int sbuf_uart_unregister(struct sbuf_uart_notifier *uart);
int sbuf_uart_try_read(struct sbuf_uart_notifier *uart, u32 generation,
		       void *buf, u32 len);
int sbuf_uart_try_ack(struct sbuf_uart_notifier *uart, u32 generation);

/* Optional UART-only AP reset lease, same creator/endpoint lifetime as above.
 * Process context only; begin returns -EAGAIN instead of waiting for the
 * control mutex. It returns -EDEADLK from notify or for a nested same-task
 * lease, and the existing status errors on a stale/canceled session.
 * A successful begin owns one reference and control_lock until same-task end.
 * End returns -EPERM for a non-owner task or an unmatched call.
 *
 * Hold only across a bounded MMIO/try-I/O transaction. Never carry the lease
 * into queued work or across a retry. Do not wait, poll, sleep, synchronize,
 * unregister, or synchronously cancel work while holding it. Backend try-I/O
 * may queue its normal asynchronous notification. Release the UART port
 * spinlock before backend try-I/O or end. Callback must still only enqueue;
 * READY can run under control_lock.
 * Every retry needs a fresh begin with the original expected generation.
 *
 * This excludes the AP generation/IDLE transition in sbuf_reset_session, not
 * remote hardware reset, other producers or clock/PM changes. cancel can
 * occur after begin: admitted MMIO may still finish, while try-I/O rejects
 * later commits. cancel + synchronize waits the lease reference; cancel alone
 * is not a zero-MMIO-after-return guarantee. Shutdown must close admission,
 * release all locks needed by workers, join work and synchronize before free.
 *
 * No UART platform consumer or remote protocol/closed-port policy is added.
 */
int sbuf_uart_lease_try_begin(struct sbuf_uart_notifier *uart, u32 generation);
int sbuf_uart_lease_end(struct sbuf_uart_notifier *uart);

/* Fixed host UART backend, separate from legacy sbufs[][] ownership.
 * Caller must own and pin the enclosing SIPC endpoint through release, and
 * serialize release against all external API calls. No device/driver pin is
 * inferred from this handle. First successful reserve consumes 6/1 this boot.
 * create returns a fully initialized sleeping thread without protocol I/O.
 * start is once-only; timeout is a total OPEN/CMD/DONE/READY budget in ticks.
 * No reset/reopen/remote CLOSE/PM policy is supplied. On peer reset, overflow,
 * cancel or deadline failure this instance remains permanently quarantined.
 *
 * cancel is nonblocking, closes callback/I/O admission and wakes AP waits.
 * Existing notifier must still be canceled/synchronized, all consumer work
 * joined and notifier unregistered before backend_join. join has a timeout;
 * on error/timeout the entire backend and endpoint stay owned and retained.
 * release requires successful join and no external users. It frees AP-only
 * resources but permanently retains physical SMEM; AP exit is not remote
 * quiescence. There is no reclaim or second-create operation this boot.
 * A READY backend failure may deliver SBUF_UART_NOTIFY_STOP using a callback
 * reference admitted before cancellation. It never carries serial data.
 * Callback code remains enqueue-only; no waiter holds a port spinlock/lease.
 * All non-cancel APIs require process context; no hard-real-time claim.
 */
struct sbuf_uart_backend;
int sbuf_uart_backend_create(struct sbuf_uart_backend **out);
int sbuf_uart_backend_start(struct sbuf_uart_backend *backend,
			    unsigned long timeout);
int sbuf_uart_backend_wait_ready(struct sbuf_uart_backend *backend,
				 unsigned long timeout);
void sbuf_uart_backend_cancel(struct sbuf_uart_backend *backend);
int sbuf_uart_backend_join(struct sbuf_uart_backend *backend,
			   unsigned long timeout);
int sbuf_uart_backend_release(struct sbuf_uart_backend *backend);
int sbuf_uart_backend_get_generation(struct sbuf_uart_backend *backend,
				     u32 *generation);

/* ****************************************************************** */
/* SBLOCK interfaces */

/* sblock structure: addr is the uncached virtual address */
struct sblock {
	void *addr;
	u32 length;
	/* AP-local ownership token. Never part of the CP wire descriptor. */
	u32 generation;
	u64 cookie;
#ifdef CONFIG_SPRD_SIPC_ZERO_COPY_SIPX
	u16 index;
	u16 offset;
#endif
};

/**
 * sblock_create -- create sblock manager on a channel
 *
 * @dst: dest processor ID
 * @channel: channel ID
 * @txblocknum: tx block number
 * @txblocksize: tx block size
 * @rxblocknum: rx block number
 * @rxblocksize: rx block size
 * @return: 0 on success, <0 on failure
 */
int sblock_create(u8 dst, u8 channel, u32 txblocknum, u32 txblocksize,
		  u32 rxblocknum, u32 rxblocksize);

/**
 * sblock_create_ex -- merge sblock_create and block_register_notifier
 * in one function
 *
 * @dst: dest processor ID
 * @channel: channel ID
 * @txblocknum: tx block number
 * @txblocksize: tx block size
 * @rxblocknum: rx block number
 * @rxblocksize: rx block size
 * @event: SBLOCK_NOTIFY_GET, SBLOCK_NOTIFY_RECV, or both
 * @data: opaque data passed to the receiver
 * @return: 0 on success, <0 on failure
 */
int sblock_create_ex(u8 dst, u8 channel, u32 txblocknum, u32 txblocksize,
		     u32 rxblocknum, u32 rxblocksize,
		     void (*handler)(int event, void *data), void *data);

#ifdef CONFIG_SPRD_SIPC_V2
/* sblock_pcfg_create -- create preconfigured SBLOCK channel.
 *
 * @dst: dest processor ID
 * @channel: channel ID
 * @tx_blk_num: tx block number
 * @tx_blk_sz: tx block size
 * @rx_blk_num: rx block number
 * @rx_blk_sz: rx block size
 * @return: 0 on success, <0 on failure
 *
 * The function only allocates the memory for the channel, and will not
 * open the channel. The client shall open the channel using
 * sblock_pcfg_open.
 */
int sblock_pcfg_create(u8 dst, u8 channel, u32 tx_blk_num, u32 tx_blk_sz,
		       u32 rx_blk_num, u32 rx_blk_sz);

/* sblock_pcfg_open -- request to open preconfigured SBLOCK channel.
 *
 * @dst: dest processor ID
 * @channel: channel ID
 * @notifier: the event notification callback function. This function can
 *	      not sleep. If this parameter is NULL, no event will be
 *	      reported.
 * @event: SBLOCK_NOTIFY_GET, SBLOCK_NOTIFY_RECV, or both
 * @client: opaque data passed to the receiver
 * @return: if the channel is established, return 0; if the open procedure
 *          is started and not finished, return SIPC_ERR_IN_PROGRESS;
 *	    otherwise return a negative error code.
 *
 * The function starts the open procedure. If the open procedure is not
 * finished when the function returns, the SBLOCK system will report
 * the open result later through the notifier callback.
 */
int sblock_pcfg_open(uint8_t dest, uint8_t channel,
		     void (*notifier)(int event, void *client), void *client);

/* sblock_get_smem_cp_addr - get the shared memory CP address.
 * @dest: destination ID
 * @channel: channel number
 * @paddr: pointer to the variable to receive the address.
 */
int sblock_get_smem_cp_addr(uint8_t dest, uint8_t channel, uint32_t *paddr);
#endif
/**
 * sblock_destroy -- destroy sblock manager on a channel
 *
 * @dst: dest processor ID
 * @channel: channel ID
 */
void sblock_destroy(u8 dst, u8 channel);

#define SBLOCK_NOTIFY_GET 0x01
#define SBLOCK_NOTIFY_RECV 0x02
#define SBLOCK_NOTIFY_STATUS 0x04
#define SBLOCK_NOTIFY_OPEN 0x08
#define SBLOCK_NOTIFY_CLOSE 0x10
#define SBLOCK_NOTIFY_OPEN_FAILED 0x20

/**
 * sblock_register_notifier -- register a callback that's called
 *		when a tx sblock is available or a rx block is received.
 *		non-blocked sblock_get or sblock_receive can be called.
 *
 * @dst: dest processor ID
 * @channel: channel ID
 * @handler: a callback handler
 * @event: SBLOCK_NOTIFY_GET, SBLOCK_NOTIFY_RECV, or both
 * @data: opaque data passed to the receiver
 * @return: 0 on success, <0 on failure
 */
int sblock_register_notifier(u8 dst, u8 channel,
			     void (*handler)(int event, void *data),
			     void *data);

/**
 * sblock_get  -- get a free sblock for sender
 *
 * @dst: dest processor ID
 * @channel: channel ID
 * @blk: return a gotten sblock pointer
 * @timeout: jiffies, 0 means no wait, -1 means unlimited
 * @return: 0 on success, <0 on failure
 */
int sblock_get(u8 dst, u8 channel, struct sblock *blk, int timeout);

/**
 * sblock_send  -- send a sblock with smsg, it should be from sblock_get
 *
 * @dst: dest processor ID
 * @channel: channel ID
 * @blk: the sblock to be sent
 * @return: 0 on success, <0 on failure
 */
/* Zero transfers ownership after ring publication; EVENT retry is internal.
 * A negative result leaves ownership with the caller, which must sblock_put().
 */
int sblock_send(u8 dst, u8 channel, struct sblock *blk);

/**
 * sblock_send_prepare  -- send a sblock without smsg,
 * it should be from sblock_get
 *
 * @dst: dest processor ID
 * @channel: channel ID
 * @blk: the sblock to be sent
 * @return: 0 on success, <0 on failure
 */
int sblock_send_prepare(u8 dst, u8 channel, struct sblock *blk);

/**
 * sblock_send_finish  -- trigger an smsg to notify that sblock has been sent
 *
 * @dst: dest processor ID
 * @channel: channel ID
 * @return: 0 on success, <0 on failure
 */
int sblock_send_finish(u8 dst, u8 channel);

/**
 * sblock_receive  -- receive a sblock, it should be released after it's handled
 *
 * @dst: dest processor ID
 * @channel: channel ID
 * @blk: return a received sblock pointer
 * @timeout: jiffies, 0 means no wait, -1 means unlimited
 * @return: 0 on success, <0 on failure
 */
int sblock_receive(u8 dst, u8 channel, struct sblock *blk, int timeout);

/**
 * sblock_release  -- release a sblock from reveiver
 *
 * @dst: dest processor ID
 * @channel: channel ID
 * @return: 0 on success, <0 on failure
 */
int sblock_release(u8 dst, u8 channel, struct sblock *blk);

/**
 * sblock_get_arrived_count  -- get the count of sblock(s) arrived at
 * AP (sblock_send on CP) but not received (sblock_receive on AP).
 *
 * @dst: dest processor ID
 * @channel: channel ID
 * @return: >=0  the count of blocks
 */
int sblock_get_arrived_count(u8 dst, u8 channel);

/**
 * sblock_get_free_count  -- get the count of available sblock(s) resident in
 * sblock pool on AP.
 *
 * @dst: dest processor ID
 * @channel: channel ID
 * @return: >=0  the count of blocks
 */
int sblock_get_free_count(u8 dst, u8 channel);

/**
 * sblock_put  -- put a free sblock for sender
 *
 * @dst: dest processor ID
 * @channel: channel ID
 * @blk: sblock pointer
 * @return: void
 */
void sblock_put(u8 dst, u8 channel, struct sblock *blk);

/**
 * sblock_poll_wait  -- poll sblock read/write
 *
 * @dst: dest processor ID
 * @channel: channel ID
 * @filp: strcut file handle
 * @wait: poll table
 * @return: POLLIN or POLLOUT
 */
unsigned int sblock_poll_wait(u8 dst, u8 channel, struct file *filp,
			      poll_table *wait);

/**
 * sblock_query  -- sblock query status
 *
 * @dst: dest processor ID
 * @channel: channel ID
 * @return: 0 on success, <0 on failure
 */
int sblock_query(u8 dst, u8 channel);

/* Snapshot a ready generation; held loans retain their generation on reset. */
int sblock_get_generation(u8 dst, u8 channel, u32 *generation);
/* Generation-bound requests fail with -EPIPE if reset overtakes the wait. */
int sblock_get_for_generation(u8 dst, u8 channel, struct sblock *blk,
			      int timeout, u32 generation);
int sblock_receive_for_generation(u8 dst, u8 channel, struct sblock *blk,
				  int timeout, u32 generation);

/* ****************************************************************** */


#define SBLOCK_CREATE(dst, channel, txblocknum, txblocksize, txpoolsize, \
		      rxblocknum, rxblocksize, rxpoolsize)               \
	sblock_create(dst, channel, txblocknum, txblocksize, rxblocknum, \
		      rxblocksize)

#define SBLOCK_DESTROY(dst, channel) sblock_destroy(dst, channel)

#define SBLOCK_GET(dst, channel, blk, ack, timeout) \
	sblock_get(dst, channel, blk, timeout)

#define SBLOCK_REGISTER_NOTIFIER(dst, channel, handler, data) \
	sblock_register_notifier(dst, channel, handler, data)

#define SBLOCK_SEND(dst, channel, blk) sblock_send(dst, channel, blk)

#define SBLOCK_SEND_PREPARE(dst, channel, blk) \
	sblock_send_prepare(dst, channel, blk)

#define SBLOCK_SEND_FINISH(dst, channel) sblock_send_finish(dst, channel)

#define SBLOCK_RECEIVE(dst, channel, blk, timeout) \
	sblock_receive(dst, channel, blk, timeout)

#define SBLOCK_RELEASE(dst, channel, blk) sblock_release(dst, channel, blk)

#define SBLOCK_GET_ARRIVED_COUNT(dst, channel) \
	sblock_get_arrived_count(dst, channel)

#define SBLOCK_GET_FREE_COUNT(dst, channel) sblock_get_free_count(dst, channel)

#define SBLOCK_PUT(dst, channel, blk) sblock_put(dst, channel, blk)


/*
 * Shared RAM may be mapped with Device attributes. Use byte-width accesses:
 * mainline arm64 memcpy/uaccess implementations may issue unaligned/vector
 * accesses even for some aligned inputs. Optimize only after mapping/coherency
 * and CP sharing attributes are established for this board.
 */
static inline unsigned long
unalign_copy_to_user(void __user *to, const void *from, unsigned long n)
{
	const u8 *src = from;
	u8 __user *dst = to;

	while (n) {
		u8 byte = READ_ONCE(*src);

		if (put_user(byte, dst))
			break;
		src++;
		dst++;
		n--;
	}
	return n;
}

static inline unsigned long
unalign_copy_from_user(void *to, const void __user *from, unsigned long n)
{
	u8 *dst = to;
	const u8 __user *src = from;

	while (n) {
		u8 byte;

		if (get_user(byte, src))
			break;
		WRITE_ONCE(*dst++, byte);
		src++;
		n--;
	}
	return n;
}

static inline void *unalign_memcpy(void *to, const void *from, size_t n)
{
	u8 *dst = to;
	const u8 *src = from;

	while (n--)
		WRITE_ONCE(*dst++, READ_ONCE(*src++));
	return to;
}

#endif /* __SIPC_H */
