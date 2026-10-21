/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2012-2015 Spreadtrum Communications Inc.
 *
 * Shared register, termios and PIO RX helpers for Spreadtrum UARTs.
 * These helpers contain no IRQ, DMA, clock or firmware notification policy.
 * RX requires the caller's UART port lock and a live port->state.
 */
#ifndef _SPRD_SERIAL_COMMON_H
#define _SPRD_SERIAL_COMMON_H

#include <linux/bits.h>
#include <linux/io.h>
#include <linux/serial_core.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>

extern struct uart_driver sprd_uart_driver;

#if IS_ENABLED(CONFIG_SERIAL_SPRD_EX)
int sprd_serial_ex_register(void);
#else
static inline int sprd_serial_ex_register(void)
{
	return 0;
}
#endif

#define SPRD_FIFO_SIZE 128
#define SPRD_DEF_RATE 26000000
#define SPRD_BAUD_IO_LIMIT 3000000
#define SPRD_TIMEOUT 256000

/* the offset of serial registers and BITs for them */
/* data registers */
#define SPRD_TXD 0x0000
#define SPRD_RXD 0x0004

/* line status register and its BITs  */
#define SPRD_LSR 0x0008
#define SPRD_LSR_OE BIT(4)
#define SPRD_LSR_FE BIT(3)
#define SPRD_LSR_PE BIT(2)
#define SPRD_LSR_BI BIT(7)
#define SPRD_LSR_TX_OVER BIT(15)

/* data number in TX and RX fifo */
#define SPRD_STS1 0x000C
#define SPRD_RX_FIFO_CNT_MASK GENMASK(7, 0)
#define SPRD_TX_FIFO_CNT_MASK GENMASK(15, 8)

/* interrupt enable register and its BITs */
#define SPRD_IEN 0x0010
#define SPRD_IEN_RX_FULL BIT(0)
#define SPRD_IEN_TX_EMPTY BIT(1)
#define SPRD_IEN_BREAK_DETECT BIT(7)
#define SPRD_IEN_TIMEOUT BIT(13)
#define SPRD_IEN_DATA_TIMEOUT BIT(17)

/* interrupt clear register */
#define SPRD_ICLR 0x0014
#define SPRD_ICLR_TIMEOUT BIT(13)
#define SPRD_ICLR_DATA_TIMEOUT BIT(17)

/* line control register */
#define SPRD_LCR 0x0018
#define SPRD_LCR_STOP_1BIT 0x10
#define SPRD_LCR_STOP_2BIT 0x30
#define SPRD_LCR_DATA_LEN (BIT(2) | BIT(3))
#define SPRD_LCR_DATA_LEN5 0x0
#define SPRD_LCR_DATA_LEN6 0x4
#define SPRD_LCR_DATA_LEN7 0x8
#define SPRD_LCR_DATA_LEN8 0xc
#define SPRD_LCR_PARITY (BIT(0) | BIT(1))
#define SPRD_LCR_PARITY_EN 0x2
#define SPRD_LCR_EVEN_PAR 0x0
#define SPRD_LCR_ODD_PAR 0x1

/* control register 1 */
#define SPRD_CTL1 0x001C
#define SPRD_DMA_EN BIT(15)
#define SPRD_LOOPBACK_EN BIT(14)
#define RX_HW_FLOW_CTL_THLD BIT(6)
#define RX_HW_FLOW_CTL_EN BIT(7)
#define TX_HW_FLOW_CTL_EN BIT(8)
#define RX_TOUT_THLD_DEF 0x3E00
#define RX_HFC_THLD_DEF 0x40

/* fifo threshold register */
#define SPRD_CTL2 0x0020
#define THLD_TX_EMPTY 0x40
#define THLD_TX_EMPTY_SHIFT 8
#define THLD_RX_FULL 0x40
#define THLD_RX_FULL_MASK GENMASK(6, 0)

/* config baud rate register */
#define SPRD_CLKD0 0x0024
#define SPRD_CLKD0_MASK GENMASK(15, 0)
#define SPRD_CLKD1 0x0028
#define SPRD_CLKD1_MASK GENMASK(20, 16)
#define SPRD_CLKD1_SHIFT 16

/* interrupt mask status register */
#define SPRD_IMSR 0x002C
#define SPRD_IMSR_RX_FIFO_FULL BIT(0)
#define SPRD_IMSR_TX_FIFO_EMPTY BIT(1)
#define SPRD_IMSR_BREAK_DETECT BIT(7)
#define SPRD_IMSR_TIMEOUT BIT(13)
#define SPRD_IMSR_DATA_TIMEOUT BIT(17)
#define SPRD_DEFAULT_SOURCE_CLK 26000000

static inline unsigned int serial_in(struct uart_port *port,
				     unsigned int offset)
{
	return readl_relaxed(port->membase + offset);
}

static inline void serial_out(struct uart_port *port, unsigned int offset,
			      int value)
{
	writel_relaxed(value, port->membase + offset);
}

static inline int handle_lsr_errors(struct uart_port *port, u8 *flag,
				    unsigned int *lsr)
{
	int ret = 0;

	/* statistics */
	if (*lsr & SPRD_LSR_BI) {
		*lsr &= ~(SPRD_LSR_FE | SPRD_LSR_PE);
		port->icount.brk++;
		ret = uart_handle_break(port);
		if (ret)
			return ret;
	} else if (*lsr & SPRD_LSR_PE)
		port->icount.parity++;
	else if (*lsr & SPRD_LSR_FE)
		port->icount.frame++;
	if (*lsr & SPRD_LSR_OE)
		port->icount.overrun++;

	/* mask off conditions which should be ignored */
	*lsr &= port->read_status_mask;
	if (*lsr & SPRD_LSR_BI)
		*flag = TTY_BREAK;
	else if (*lsr & SPRD_LSR_PE)
		*flag = TTY_PARITY;
	else if (*lsr & SPRD_LSR_FE)
		*flag = TTY_FRAME;

	return ret;
}

static inline void sprd_rx_pio_budget(struct uart_port *port,
				      unsigned int budget, bool deliver)
{
	struct tty_port *tty = &port->state->port;
	unsigned int lsr, max_count = budget;
	u8 ch, flag;

	while ((serial_in(port, SPRD_STS1) & SPRD_RX_FIFO_CNT_MASK) &&
	       max_count--) {
		lsr = serial_in(port, SPRD_LSR);
		ch = serial_in(port, SPRD_RXD);
		flag = TTY_NORMAL;
		port->icount.rx++;

		if (lsr &
		    (SPRD_LSR_BI | SPRD_LSR_PE | SPRD_LSR_FE | SPRD_LSR_OE))
			if (handle_lsr_errors(port, &flag, &lsr))
				continue;
		if (uart_handle_sysrq_char(port, ch))
			continue;

		if (deliver)
			uart_insert_char(port, lsr, SPRD_LSR_OE, ch, flag);
	}

	if (deliver)
		tty_flip_buffer_push(tty);
}

static inline void sprd_rx_pio(struct uart_port *port)
{
	sprd_rx_pio_budget(port, SPRD_TIMEOUT, true);
}

/* Caller holds the port lock and has established space for count bytes.
 * This helper does not poll hardware or define a UART-ex TX-ready policy.
 * Return value excludes x_char, as documented by serial-core.
 */
static inline unsigned int sprd_tx_pio(struct uart_port *port,
				       unsigned int count, unsigned int flags)
{
	u8 ch;

	return uart_port_tx_limited_flags(port, ch, flags, count, true,
					  serial_out(port, SPRD_TXD, ch), ({}));
}

static inline void sprd_rx_status_mask(struct uart_port *port,
				       const struct ktermios *termios)
{
	port->read_status_mask = SPRD_LSR_OE;
	if (termios->c_iflag & INPCK)
		port->read_status_mask |= SPRD_LSR_FE | SPRD_LSR_PE;
	if (termios->c_iflag & (IGNBRK | BRKINT | PARMRK))
		port->read_status_mask |= SPRD_LSR_BI;

	/* characters to ignore */
	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		port->ignore_status_mask |= SPRD_LSR_PE | SPRD_LSR_FE;
	if (termios->c_iflag & IGNBRK) {
		port->ignore_status_mask |= SPRD_LSR_BI;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			port->ignore_status_mask |= SPRD_LSR_OE;
	}
}

static inline void sprd_set_termios(struct uart_port *port,
				    struct ktermios *termios,
				    const struct ktermios *old)
{
	unsigned int baud, quot;
	unsigned int lcr = 0, fc;
	unsigned long flags;

	/* ask the core to calculate the divisor for us */
	baud = uart_get_baud_rate(port, termios, old, 0, SPRD_BAUD_IO_LIMIT);

	quot = port->uartclk / baud;

	/* set data length */
	switch (termios->c_cflag & CSIZE) {
	case CS5:
		lcr |= SPRD_LCR_DATA_LEN5;
		break;
	case CS6:
		lcr |= SPRD_LCR_DATA_LEN6;
		break;
	case CS7:
		lcr |= SPRD_LCR_DATA_LEN7;
		break;
	case CS8:
	default:
		lcr |= SPRD_LCR_DATA_LEN8;
		break;
	}

	/* calculate stop bits */
	lcr &= ~(SPRD_LCR_STOP_1BIT | SPRD_LCR_STOP_2BIT);
	if (termios->c_cflag & CSTOPB)
		lcr |= SPRD_LCR_STOP_2BIT;
	else
		lcr |= SPRD_LCR_STOP_1BIT;

	/* calculate parity */
	lcr &= ~SPRD_LCR_PARITY;
	termios->c_cflag &= ~CMSPAR; /* no support mark/space */
	if (termios->c_cflag & PARENB) {
		lcr |= SPRD_LCR_PARITY_EN;
		if (termios->c_cflag & PARODD)
			lcr |= SPRD_LCR_ODD_PAR;
		else
			lcr |= SPRD_LCR_EVEN_PAR;
	}

	uart_port_lock_irqsave(port, &flags);

	/* update the per-port timeout */
	uart_update_timeout(port, termios->c_cflag, baud);

	sprd_rx_status_mask(port, termios);
	/* flow control */
	fc = serial_in(port, SPRD_CTL1);
	fc &= ~(RX_HW_FLOW_CTL_THLD | RX_HW_FLOW_CTL_EN | TX_HW_FLOW_CTL_EN);
	if (termios->c_cflag & CRTSCTS) {
		fc |= RX_HW_FLOW_CTL_THLD;
		fc |= RX_HW_FLOW_CTL_EN;
		fc |= TX_HW_FLOW_CTL_EN;
	}

	/* clock divider bit0~bit15 */
	serial_out(port, SPRD_CLKD0, quot & SPRD_CLKD0_MASK);

	/* clock divider bit16~bit20 */
	serial_out(port, SPRD_CLKD1,
		   (quot & SPRD_CLKD1_MASK) >> SPRD_CLKD1_SHIFT);
	serial_out(port, SPRD_LCR, lcr);
	fc |= RX_TOUT_THLD_DEF | RX_HFC_THLD_DEF;
	serial_out(port, SPRD_CTL1, fc);

	uart_port_unlock_irqrestore(port, flags);

	/* Don't rewrite B0 */
	if (tty_termios_baud_rate(termios))
		tty_termios_encode_baud_rate(termios, baud, baud);
}

#endif /* _SPRD_SERIAL_COMMON_H */
