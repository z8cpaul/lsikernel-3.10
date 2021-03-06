/*
 *  drivers/tty/serial/lsi_acp_serial.c
 *
 *  Driver for AMBA serial ports on LSI's PPC based ACP.
 *
 *  Based on drivers/char/serial.c, by Linus Torvalds, Theodore Ts'o.
 *
 *  Copyright 1999 ARM Limited
 *  Copyright (C) 2000 Deep Blue Solutions Ltd.
 *  Copyright 2009 LSI
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307	 USA
 *
 * This is a generic driver for ARM AMBA-type serial ports.  They
 * have a lot of 16550-like features, but are not register compatible.
 * Note that although they do have CTS, DCD and DSR inputs, they do
 * not have an RI input, nor do they have DTR or RTS outputs.  If
 * required, these have to be supplied via some other means (eg, GPIO)
 * and hooked into this driver.
 */

#if defined(CONFIG_SERIAL_ACP_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/device.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/slab.h>
#include <linux/amba/bus.h>
#include <linux/amba/serial.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/io.h>

#define SZ_4K (4*1024)
#define UART_NR			2
#define SERIAL_AMBA_MAJOR	204
#define SERIAL_AMBA_MINOR	64
#define SERIAL_AMBA_NR		UART_NR
#define AMBA_ISR_PASS_LIMIT	256
#define UART_DR_ERROR		\
(UART011_DR_OE|UART011_DR_BE|UART011_DR_PE|UART011_DR_FE)
#define UART_DUMMY_DR_RX	(1 << 16)

#define MAX_BAUD_RATE 115200

static int dt_baud_rate;
static unsigned long per_clock;

/*
  ======================================================================
  ======================================================================
  A modified uart_port structure.
  ======================================================================
  ======================================================================
*/

struct uart_acp_port {
	struct uart_port port;
	irq_hw_number_t hwirq;
	unsigned int interrupt_mask;
	unsigned int old_status;
	void *timer_base;
	unsigned short ibrd;
	unsigned short fbrd;
};

/*
  ======================================================================
  ======================================================================
  Both UARTs share the same clock input, the output of the 2nd timer,
  or timer 1, from the APB Dual Input Timers (there are two of these,
  thus 4 timers).
  ======================================================================
  ======================================================================
*/

#define TIMER_LOAD		       0x00
#define TIMER_VALUE		       0x04
#define TIMER_CONTROL		       0x08
#define TIMER_CONTROL_ENABLE	       0x80
#define TIMER_CONTROL_MODE	       0x40
#define TIMER_CONTROL_INTERRUPT_ENABLE 0x20
#define TIMER_CONTROL_OUTPUT_MODE      0x10
#define TIMER_CONTROL_PRESCALER	       0x0c
#define TIMER_CONTROL_SIZE	       0x02
#define TIMER_CONTROL_ONE_SHOT	       0x01
#define TIMER_INTCLR		       0x0C
#define TIMER_RIS		       0x10
#define TIMER_MIS		       0x14
#define TIMER_BGLOAD		       0x18

/*
  ----------------------------------------------------------------------
  get_clock_stuff
*/

static int
get_clock_stuff(struct uart_acp_port *port, int baud_rate)
{
	unsigned long ibrd;
	unsigned long fbrd;

	ibrd = per_clock / (16 * baud_rate);

	/*
	 * The following formula is from the ARM document (ARM DDI 0183E).
	 *
	 * Baud Rate Divisor = (Uart Clock / (16 * Baud Rate))
	 *
	 * Baud Rate Divisor is then split into integral and fractional
	 * parts.  The IBRD value is simply the itegral part.  The FBRD is
	 * calculated as follows.
	 *
	 * FBRD = fractional part of the Baud Rate Divisor * 64 + 0.5
	 *
	 * The fractional part of the Baud Rate Divisor can be represented as
	 * follows:
	 *
	 *    (Uart Clock % (16 * baud_rate)) / (16 * baud_rate)
	 *
	 * As long as the division isn't done until the end. So, the above
	 * "* 64 + 0.5" is the FBRD. Also note that x/y + 1/2 = (2x+y)/2y.
	 * This *  leads to:
	 *
	 *    ((Uart Clock % (16 * baud_rate)) * 64 * 2 + (16 * baud_rate))
	 *  -----------------------------------------------------------------
	 *                     2 * (16 * baud_rate)
	 */

	port->port.uartclk = per_clock;

	fbrd = port->port.uartclk % (16 * baud_rate);
	fbrd *= 128;
	fbrd += (16 * baud_rate);
	fbrd /= (2 * (16 * baud_rate));

	port->ibrd = (unsigned short) ibrd;
	port->fbrd = (unsigned short) fbrd;

	return 0;
}

/*
  ======================================================================
  ======================================================================
  Serial operations.
  ======================================================================
  ======================================================================
*/

/*
  ------------------------------------------------------------------------------
  acp_serial_wac

  This was added to allow an easy debugging breakpoint.
*/

static void
acp_serial_wac(u32 *address, int character, u32 line)
{
	out_le32(address, character);

	return;
}

/*
  ------------------------------------------------------------------------------
  acp_serial_tx_empty
*/

static unsigned int
acp_serial_tx_empty(struct uart_port *port)
{
	struct uart_acp_port *uap = (struct uart_acp_port *) port;
	unsigned int status =
		in_le32((u32 *)(uap->port.membase + UART01x_FR));
	return status &
		(UART01x_FR_BUSY | UART01x_FR_TXFF) ? 0 : TIOCSER_TEMT;
}

/*
  ----------------------------------------------------------------------
  acp_serial_stop_tx
*/

static void
acp_serial_stop_tx(struct uart_port *port)
{
	struct uart_acp_port *uap = (struct uart_acp_port *)port;

	uap->interrupt_mask &= ~UART011_TXIM;
	out_le32((u32 *)(uap->port.membase + UART011_IMSC),
		 uap->interrupt_mask);
}

/*
  ----------------------------------------------------------------------
  acp_serial_start_tx
*/

static void
acp_serial_start_tx(struct uart_port *port)
{
	struct uart_acp_port *uap = (struct uart_acp_port *)port;

	uap->interrupt_mask |= UART011_TXIM;
	out_le32((u32 *)(uap->port.membase + UART011_IMSC),
		 uap->interrupt_mask);
}

void acp_serial_stop_rx(struct uart_port *port)
{
	struct uart_acp_port *uap = (struct uart_acp_port *)port;

	uap->interrupt_mask &= ~(UART011_RXIM|UART011_RTIM|UART011_FEIM|
				 UART011_PEIM|UART011_BEIM|UART011_OEIM);
	out_le32((u32 *)(uap->port.membase + UART011_IMSC),
		 uap->interrupt_mask);
}

void acp_serial_enable_ms(struct uart_port *port)
{
	struct uart_acp_port *uap = (struct uart_acp_port *)port;

	uap->interrupt_mask |=
		UART011_RIMIM|UART011_CTSMIM|UART011_DCDMIM|UART011_DSRMIM;
	out_le32((u32 *)(uap->port.membase + UART011_IMSC),
		 uap->interrupt_mask);
}

/*
  ----------------------------------------------------------------------
  acp_serial_rx_chars
*/

static void
acp_serial_rx_chars(struct uart_acp_port *uap)
{
	unsigned int status, ch, flag, max_count = 256;

	status = in_le32((u32 *)(uap->port.membase + UART01x_FR));
	while ((status & UART01x_FR_RXFE) == 0 && max_count--) {
		ch = in_le32((u32 *)(uap->port.membase + UART01x_DR)) |
			UART_DUMMY_DR_RX;
		flag = TTY_NORMAL;
		uap->port.icount.rx++;

		/*
		 * Note that the error handling code is
		 * out of the main execution path
		 */
		if (unlikely(ch & UART_DR_ERROR)) {
			if (ch & UART011_DR_BE) {
				ch &= ~(UART011_DR_FE | UART011_DR_PE);
				uap->port.icount.brk++;
				if (uart_handle_break(&uap->port))
					goto ignore_char;
			} else if (ch & UART011_DR_PE)
				uap->port.icount.parity++;
			else if (ch & UART011_DR_FE)
				uap->port.icount.frame++;
			if (ch & UART011_DR_OE)
				uap->port.icount.overrun++;

			ch &= uap->port.read_status_mask;

			if (ch & UART011_DR_BE)
				flag = TTY_BREAK;
			else if (ch & UART011_DR_PE)
				flag = TTY_PARITY;
			else if (ch & UART011_DR_FE)
				flag = TTY_FRAME;
		}

		if (uart_handle_sysrq_char(&uap->port, ch & 255))
			goto ignore_char;

		uart_insert_char(&uap->port, ch, UART011_DR_OE, ch, flag);

ignore_char:
		status = in_le32((u32 *)(uap->port.membase + UART01x_FR));
	}
	spin_unlock(&uap->port.lock);
	tty_flip_buffer_push(&uap->port.state->port);
	spin_lock(&uap->port.lock);
}

/*
  ----------------------------------------------------------------------
  acp_serial_tx_chars
*/

static void
acp_serial_tx_chars(struct uart_acp_port *uap)
{
	struct circ_buf *xmit = &uap->port.state->xmit;
	int count;

	if (uap->port.x_char) {
		acp_serial_wac((u32 *)(uap->port.membase + UART01x_DR),
				uap->port.x_char, __LINE__);
		uap->port.icount.tx++;
		uap->port.x_char = 0;
		return;
	}
	if (uart_circ_empty(xmit) || uart_tx_stopped(&uap->port)) {
		acp_serial_stop_tx(&uap->port);
		return;
	}

	count = uap->port.fifosize >> 1;
	do {
		acp_serial_wac((u32 *) (uap->port.membase + UART01x_DR),
				xmit->buf[xmit->tail], __LINE__);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		uap->port.icount.tx++;
		if (uart_circ_empty(xmit))
			break;
	} while (--count > 0);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&uap->port);

	if (uart_circ_empty(xmit))
		acp_serial_stop_tx(&uap->port);
}

void acp_serial_modem_status(struct uart_acp_port *uap)
{
	unsigned int status, delta;

	status = in_le32((u32 *)(uap->port.membase + UART01x_FR)) &
		UART01x_FR_MODEM_ANY;

	delta = status ^ uap->old_status;
	uap->old_status = status;

	if (!delta)
		return;

	if (delta & UART01x_FR_DCD)
		uart_handle_dcd_change(&uap->port, status & UART01x_FR_DCD);

	if (delta & UART01x_FR_DSR)
		uap->port.icount.dsr++;

	if (delta & UART01x_FR_CTS)
		uart_handle_cts_change(&uap->port, status & UART01x_FR_CTS);

	wake_up_interruptible(&uap->port.state->port.delta_msr_wait);
}

/*
  ------------------------------------------------------------------------------
  acp_serial_isr
*/

static irqreturn_t
acp_serial_isr(int irq, void *dev_id)
{
	struct uart_acp_port *uap = dev_id;
	unsigned int status, pass_counter = AMBA_ISR_PASS_LIMIT;
	int handled = 0;

	spin_lock(&uap->port.lock);

	status = in_le32((u32 *)(uap->port.membase + UART011_MIS));
	if (status) {
		do {
			out_le32((u32 *)(uap->port.membase + UART011_ICR),
				 (status &
				  ~(UART011_TXIS|UART011_RTIS|UART011_RXIS)));

			if (status & (UART011_RTIS|UART011_RXIS))
				acp_serial_rx_chars(uap);
			if (status & (UART011_DSRMIS|UART011_DCDMIS|
				      UART011_CTSMIS|UART011_RIMIS))
				acp_serial_modem_status(uap);
			if (status & UART011_TXIS)
				acp_serial_tx_chars(uap);

			if (pass_counter-- == 0)
				break;

			status =
				in_le32((u32 *)(uap->port.membase +
					UART011_MIS));
		} while (status != 0);
		handled = 1;
	}

	spin_unlock(&uap->port.lock);

	return IRQ_RETVAL(handled);
}

unsigned int acp_serial_get_mctrl(struct uart_port *port)
{
	struct uart_acp_port *uap = (struct uart_acp_port *)port;
	unsigned int result = 0;
	unsigned int status = in_le32((u32 *)(uap->port.membase + UART01x_FR));

#define TIOCMBIT(uartbit, tiocmbit) do { \
	if (status & (uartbit))		 \
		result |= (tiocmbit);    \
} while (0);
	TIOCMBIT(UART01x_FR_DCD, TIOCM_CAR);
	TIOCMBIT(UART01x_FR_DSR, TIOCM_DSR);
	TIOCMBIT(UART01x_FR_CTS, TIOCM_CTS);
	TIOCMBIT(UART011_FR_RI, TIOCM_RNG);
#undef TIOCMBIT
	return result;
}

void acp_serial_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct uart_acp_port *uap = (struct uart_acp_port *)port;
	unsigned int cr;

	cr = in_le32((u32 *)(uap->port.membase + UART011_CR));

#define TIOCMBIT(tiocmbit, uartbit) do { \
	if (mctrl & tiocmbit)		 \
		cr |= uartbit;		 \
	else				 \
		cr &= ~uartbit;		 \
} while (0);

	TIOCMBIT(TIOCM_RTS, UART011_CR_RTS);
	TIOCMBIT(TIOCM_DTR, UART011_CR_DTR);
	TIOCMBIT(TIOCM_OUT1, UART011_CR_OUT1);
	TIOCMBIT(TIOCM_OUT2, UART011_CR_OUT2);
	TIOCMBIT(TIOCM_LOOP, UART011_CR_LBE);

#undef TIOCMBIT

	out_le32((u32 *)(uap->port.membase + UART011_CR), cr);
}

void acp_serial_break_ctl(struct uart_port *port, int break_state)
{
	struct uart_acp_port *uap = (struct uart_acp_port *)port;
	unsigned long flags;
	unsigned int lcr_h;

	spin_lock_irqsave(&uap->port.lock, flags);
	lcr_h = in_le32((u32 *)(uap->port.membase + UART011_LCRH));
	if (break_state == -1)
		lcr_h |= UART01x_LCRH_BRK;
	else
		lcr_h &= ~UART01x_LCRH_BRK;
	out_le32((u32 *)(uap->port.membase + UART011_LCRH), lcr_h);
	spin_unlock_irqrestore(&uap->port.lock, flags);
}

#ifdef CONFIG_CONSOLE_POLL

/*
  ----------------------------------------------------------------------
  acp_serial_poll_get_char
*/

static int
acp_serial_poll_get_char(struct uart_port *port)
{
	struct uart_acp_port *uap = (struct uart_acp_port *) port;
	unsigned int status;

	do {
		status = in_le32((u32 *)
				 (uap->port.membase + UART01x_FR));
	} while (status & UART01x_FR_RXFE);

	return in_le32((u32 *) (uap->port.membase + UART01x_DR));
}

/*
  ----------------------------------------------------------------------
  acp_serial_poll_put_char
*/

static void
acp_serial_poll_put_char(struct uart_port *port, unsigned char ch)
{
	struct uart_acp_port *uap = (struct uart_acp_port *)port;

	while (in_le32((u32 *) (uap->port.membase + UART01x_FR)) &
	       UART01x_FR_TXFF) {
		barrier();
	}

	acp_serial_wac((u32 *)(uap->port.membase + UART01x_DR), ch,
			__LINE__);

	return;
}
#endif /* CONFIG_CONSOLE_POLL */

/*
  ----------------------------------------------------------------------
  acp_serial_startup
*/

static int
acp_serial_startup(struct uart_port *port)
{
	struct uart_acp_port *uap = (struct uart_acp_port *)port;
	unsigned int cr;
	int retval = 0;

	/*
	 * Set up the interrupt.
	 */

	uap->port.irq = irq_create_mapping(NULL, uap->hwirq);

	if (NO_IRQ == uap->port.irq) {
		pr_err("irq_create_mapping() failed!\n");
		goto clk_dis;
	}

	retval = irq_set_irq_type(uap->port.irq, IRQ_TYPE_LEVEL_HIGH);

	if (0 != retval) {
		pr_err("set_irq_type(%d, 0x%x) failed!\n",
			uap->port.irq, IRQ_TYPE_LEVEL_HIGH);
		goto clk_dis;
	}

	retval = request_irq(uap->port.irq, acp_serial_isr,
			     IRQF_DISABLED, "uart-pl011", uap);

	if (retval) {
		pr_err("request_irq(%d) failed!\n", uap->port.irq);
		goto clk_dis;
	}

	out_le32((u32 *)(uap->port.membase + UART011_IFLS),
		 UART011_IFLS_RX4_8|UART011_IFLS_TX4_8);

	/*
	 * Provoke TX FIFO interrupt into asserting.
	 */
	cr = UART01x_CR_UARTEN | UART011_CR_TXE | UART011_CR_LBE;
	out_le32((u32 *)(uap->port.membase + UART011_CR), cr);
	out_le32((u32 *)(uap->port.membase + UART011_FBRD), 0);
	out_le32((u32 *)(uap->port.membase + UART011_IBRD), 1);
	out_le32((u32 *)(uap->port.membase + UART011_LCRH), 0);
	acp_serial_wac((u32 *) (uap->port.membase + UART01x_DR), 0,
			__LINE__);

	while (in_le32((u32 *)(uap->port.membase + UART01x_FR)) &
	       UART01x_FR_BUSY)
		barrier();

	cr = UART01x_CR_UARTEN | UART011_CR_RXE | UART011_CR_TXE;
	out_le32((u32 *)(uap->port.membase + UART011_CR), cr);

	/*
	 * initialise the old status of the modem signals
	 */

	uap->old_status =
		in_le32((u32 *)(uap->port.membase + UART01x_FR)) &
		UART01x_FR_MODEM_ANY;

	/*
	 * Finally, enable interrupts
	 */
	spin_lock_irq(&uap->port.lock);
	uap->interrupt_mask = UART011_RXIM | UART011_RTIM;
	out_le32((u32 *)(uap->port.membase + UART011_IMSC),
		 uap->interrupt_mask);
	spin_unlock_irq(&uap->port.lock);

	return 0;

 clk_dis:

	return retval;
}

/*
  ----------------------------------------------------------------------
  acp_serial_shutdown
*/

static void
acp_serial_shutdown(struct uart_port *port)
{
	struct uart_acp_port *uap = (struct uart_acp_port *)port;
	unsigned long val;

	/*
	 * disable all interrupts
	 */
	spin_lock_irq(&uap->port.lock);
	uap->interrupt_mask = 0;
	out_le32((u32 *)(uap->port.membase + UART011_IMSC),
		 uap->interrupt_mask);
	out_le32((u32 *)(uap->port.membase + UART011_ICR), 0xffff);
	spin_unlock_irq(&uap->port.lock);

	/*
	 * Free the interrupt
	 */
	free_irq(uap->port.irq, uap);

	/*
	 * disable the port
	 */
	out_le32((u32 *)(uap->port.membase + UART011_CR),
		 UART01x_CR_UARTEN | UART011_CR_TXE);

	/*
	 * disable break condition and fifos
	 */
	val = in_le32((u32 *)(uap->port.membase + UART011_LCRH));
	val &= ~(UART01x_LCRH_BRK | UART01x_LCRH_FEN);
	out_le32((u32 *)(uap->port.membase + UART011_LCRH), val);
}

/*
  ----------------------------------------------------------------------
  acp_serial_set_termios
*/

static void
acp_serial_set_termios(struct uart_port *port, struct ktermios *termios,
			struct ktermios *old)
{
	unsigned int lcr_h, old_cr;
	unsigned long flags;
	int baud;
	struct uart_acp_port *uap = (struct uart_acp_port *)port;

	/*
	 * Set up the clock, and calculate the divisors.
	 */

	baud = tty_termios_baud_rate(termios);
	get_clock_stuff(uap, baud);

	switch (termios->c_cflag & CSIZE) {
	case CS5:
		lcr_h = UART01x_LCRH_WLEN_5;
		break;
	case CS6:
		lcr_h = UART01x_LCRH_WLEN_6;
		break;
	case CS7:
		lcr_h = UART01x_LCRH_WLEN_7;
		break;
	default: /* CS8 */
		lcr_h = UART01x_LCRH_WLEN_8;
		break;
	}
	if (termios->c_cflag & CSTOPB)
		lcr_h |= UART01x_LCRH_STP2;
	if (termios->c_cflag & PARENB) {
		lcr_h |= UART01x_LCRH_PEN;
		if (!(termios->c_cflag & PARODD))
			lcr_h |= UART01x_LCRH_EPS;
	}
	if (port->fifosize > 1)
		lcr_h |= UART01x_LCRH_FEN;

	spin_lock_irqsave(&port->lock, flags);

	/*
	 * Update the per-port timeout.
	 */
	uart_update_timeout(port, termios->c_cflag, baud);

	port->read_status_mask = UART011_DR_OE | 255;
	if (termios->c_iflag & INPCK)
		port->read_status_mask |= UART011_DR_FE | UART011_DR_PE;
	if (termios->c_iflag & (BRKINT | PARMRK))
		port->read_status_mask |= UART011_DR_BE;

	/*
	 * Characters to ignore
	 */
	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		port->ignore_status_mask |= UART011_DR_FE | UART011_DR_PE;
	if (termios->c_iflag & IGNBRK) {
		port->ignore_status_mask |= UART011_DR_BE;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			port->ignore_status_mask |= UART011_DR_OE;
	}

	/*
	 * Ignore all characters if CREAD is not set.
	 */
	if ((termios->c_cflag & CREAD) == 0)
		port->ignore_status_mask |= UART_DUMMY_DR_RX;

	if (UART_ENABLE_MS(port, termios->c_cflag))
		acp_serial_enable_ms(port);

	/* first, disable everything */
	old_cr = in_le32((u32 *)(uap->port.membase + UART011_CR));
	out_le32((u32 *)(uap->port.membase + UART011_CR), 0);

	/* Set baud rate */
#if 1
	out_le32((u32 *)(uap->port.membase + UART011_FBRD), uap->fbrd);
	out_le32((u32 *)(uap->port.membase + UART011_IBRD), uap->ibrd);
#else
	out_le32((u32 *)(uap->port.membase + UART011_FBRD), 0x13);
	out_le32((u32 *)(uap->port.membase + UART011_IBRD), 0x598);
#endif

	/*
	 * ----------v----------v----------v----------v-----
	 * NOTE: MUST BE WRITTEN AFTER UARTLCR_M & UARTLCR_L
	 * ----------^----------^----------^----------^-----
	 */
	out_le32((u32 *)(uap->port.membase + UART011_LCRH), lcr_h);
	out_le32((u32 *)(uap->port.membase + UART011_CR), old_cr);

	spin_unlock_irqrestore(&port->lock, flags);
}

const char *acp_serial_type(struct uart_port *port)
{
	return port->type == PORT_AMBA ? "AMBA/PL011" : NULL;
}

/*
 * Release the memory region(s) being used by 'port'
 */
void acp_serial_release_port(struct uart_port *port)
{
	release_mem_region(port->mapbase, SZ_4K);
}

/*
 * Request the memory region(s) being used by 'port'
 */
int acp_serial_request_port(struct uart_port *port)
{
	return request_mem_region(port->mapbase, SZ_4K, "uart-pl011")
		!= NULL ? 0 : -EBUSY;
}

/*
 * Configure/autoconfigure the port.
 */
void acp_serial_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE) {
		port->type = PORT_AMBA;
		acp_serial_request_port(port);
	}
}

/*
 * verify the new serial_struct (for TIOCSSERIAL).
 */
int acp_serial_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	int ret = 0;
	if (ser->type != PORT_UNKNOWN && ser->type != PORT_AMBA)
		ret = -EINVAL;
	if (ser->irq < 0 || ser->irq >= nr_irqs)
		ret = -EINVAL;
	if (ser->baud_base < 9600)
		ret = -EINVAL;
	return ret;
}

static struct uart_ops amba_acp_pops = {
	.tx_empty	= acp_serial_tx_empty,
	.set_mctrl	= acp_serial_set_mctrl,
	.get_mctrl	= acp_serial_get_mctrl,
	.stop_tx	= acp_serial_stop_tx,
	.start_tx	= acp_serial_start_tx,
	.stop_rx	= acp_serial_stop_rx,
	.enable_ms	= acp_serial_enable_ms,
	.break_ctl	= acp_serial_break_ctl,
	.startup	= acp_serial_startup,
	.shutdown	= acp_serial_shutdown,
	.set_termios	= acp_serial_set_termios,
	.type		= acp_serial_type,
	.release_port	= acp_serial_release_port,
	.request_port	= acp_serial_request_port,
	.config_port	= acp_serial_config_port,
	.verify_port	= acp_serial_verify_port,
#ifdef CONFIG_CONSOLE_POLL
	.poll_get_char = acp_serial_poll_get_char,
	.poll_put_char = acp_serial_poll_put_char,
#endif
};

static struct uart_acp_port *acp_ports[2];

#ifdef CONFIG_SERIAL_ACP_CONSOLE

/*
  ----------------------------------------------------------------------
  acp_serial_console_putchar
*/

static void
acp_serial_console_putchar(struct uart_port *port, int ch)
{
	struct uart_acp_port *uap = (struct uart_acp_port *)port;

	while (in_le32((u32 *)(uap->port.membase + UART01x_FR)) &
	       UART01x_FR_TXFF) {
		barrier();
	}

	acp_serial_wac((u32 *) (uap->port.membase + UART01x_DR), ch,
			__LINE__);
}

void
acp_console_write(struct console *co, const char *s, unsigned int count)
{
	struct uart_acp_port *uap = acp_ports[co->index];
	unsigned int status, old_cr, new_cr;

	/*clk_enable(uap->clk);*/

	/*
	 *	First save the CR then disable the interrupts
	 */
	old_cr = in_le32((u32 *)(uap->port.membase + UART011_CR));
	new_cr = old_cr & ~UART011_CR_CTSEN;
	new_cr |= UART01x_CR_UARTEN | UART011_CR_TXE;
	out_le32((u32 *)(uap->port.membase + UART011_CR), new_cr);

	uart_console_write(&uap->port, s, count, acp_serial_console_putchar);

	/*
	 *	Finally, wait for transmitter to become empty
	 *	and restore the TCR
	 */
	do {
		status =
			in_le32((u32 *)(u32 *)(uap->port.membase + UART01x_FR));
	} while (status & UART01x_FR_BUSY);
	out_le32((u32 *)(uap->port.membase + UART011_CR), old_cr);

	/*clk_disable(uap->clk);*/
}

void __init
acp_console_get_options(struct uart_acp_port *uap, int *baud,
			int *parity, int *bits)
{
	if (in_le32((u32 *)(u32 *)(uap->port.membase + UART011_CR)) &
	    UART01x_CR_UARTEN) {
		unsigned int lcr_h, ibrd, fbrd;

		lcr_h = in_le32((u32 *)(u32 *)(uap->port.membase +
					       UART011_LCRH));

		*parity = 'n';
		if (lcr_h & UART01x_LCRH_PEN) {
			if (lcr_h & UART01x_LCRH_EPS)
				*parity = 'e';
			else
				*parity = 'o';
		}

		if ((lcr_h & 0x60) == UART01x_LCRH_WLEN_7)
			*bits = 7;
		else
			*bits = 8;

		ibrd = in_le32((u32 *)(u32 *)(uap->port.membase +
					      UART011_IBRD));
		fbrd = in_le32((u32 *)(u32 *)(uap->port.membase +
					      UART011_FBRD));
		*baud = (uap->port.uartclk * 4 / (64 * ibrd + fbrd));
		*baud += 50;
		*baud /= 10;
		*baud *= 10;
	}
}

int __init acp_console_setup(struct console *co, char *options)
{
	struct uart_acp_port *uap;
	int baud = 9600;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	/*
	 * Check whether an invalid uart number has been specified, and
	 * if so, search for the first available port that does have
	 * console support.
	 */
	if (co->index >= UART_NR)
		co->index = 0;
	uap = acp_ports[co->index];
	if (!uap)
		return -ENODEV;

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);
	else
		acp_console_get_options(uap, &baud, &parity, &bits);

	return uart_set_options(&uap->port, co, baud, parity, bits, flow);
}

static struct uart_driver acp_serial_driver;
static struct console acp_console = {
	.name		= "ttyS",
	.write		= acp_console_write,
	.device		= uart_console_device,
	.setup		= acp_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &acp_serial_driver,
};

static int __init
acp_console_init(void)
{
	register_console(&acp_console);

	return 0;
}

console_initcall(acp_console_init);

#define ACP_CONSOLE	(&acp_console)
#else
#define ACP_CONSOLE	NULL
#endif

static struct uart_driver acp_serial_driver = {
	.owner			= THIS_MODULE,
	.driver_name		= "serial",
	.dev_name		= "ttyS",
	.major			= TTY_MAJOR,
	.minor			= 64,
	.nr			= UART_NR,
	.cons			= ACP_CONSOLE,
};

/*
  ----------------------------------------------------------------------
  acp_serial_add_ports
*/

static int
acp_serial_add_ports(struct uart_driver *driver)
{
	struct uart_acp_port *uap;
	int i, ret;
	struct device_node *np = NULL;
	u64 addr = 0;
	const u32 *reg, *interrupts, *clk, *speed;
	int baud_rate = 9600;
	const int *enabled = NULL;

	for (i = 0; i < ARRAY_SIZE(acp_ports); ++i) {
		if (acp_ports[i] == NULL)
			break;
	}

	if (i == ARRAY_SIZE(acp_ports)) {
		ret = -EBUSY;
		goto out;
	}

	uap = kzalloc(sizeof(struct uart_acp_port), GFP_KERNEL);

	if (NULL == uap) {
		ret = -ENOMEM;
		goto out;
	}

	np = of_find_node_by_type(np, "serial");

	while (np && !of_device_is_compatible(np, "acp-uart0"))
		np = of_find_node_by_type(np, "serial");

	if (np)
		enabled = of_get_property(np, "enabled", NULL);

	if (!enabled) {
		/*
		  Older LSI U-Boot package (prior to 4.8.1.36).

		  Only use UART0.  The timer registers are defined
		  differently in the device tree.
		*/
		uap->timer_base = ioremap(0x002000408040ULL, 0x20);
	} else {
		/*
		  Newer LSI U-Boot package (4.8.1.36 on).

		  Only use a serial port if it is enabled.
		*/

		if (!np || (0 == *enabled)) {
			np = NULL;
			np = of_find_node_by_type(np, "serial");

			while (np && !of_device_is_compatible(np, "acp-uart1"))
				np = of_find_node_by_type(np, "serial");

			if (np)
				enabled = of_get_property(np, "enabled", NULL);
		}

		if (np && (0 != *enabled)) {
			reg = of_get_property(np, "clock-reg", NULL);

			if (reg) {
				addr = of_translate_address(np, reg);
				if (addr == OF_BAD_ADDR)
					addr = 0;
			}

			if (addr)
				uap->timer_base = ioremap(addr, reg[1]);
			else {
				pr_err("timer io address not found\n");
				ret = -ENOMEM;
			}
		}
	}

	if (np) {
		reg = of_get_property(np, "reg", NULL);

		if (reg) {
			addr = of_translate_address(np, reg);
			if (addr == OF_BAD_ADDR)
				addr = 0;
		}

		if (addr)
			uap->port.membase = ioremap(addr, reg[1]);
		else {
			pr_err("serial io address not found\n");
			ret = -ENOMEM;
		}

		interrupts = of_get_property(np, "interrupts", NULL);

		if (interrupts)
			uap->hwirq = interrupts[0];
		else {
			pr_err("serial irq not found\n");
			uap->hwirq = 22;
		}

		clk = of_get_property(np, "clock-frequency", NULL);

		if (clk && *clk)
			per_clock = *clk;
		else {
			pr_err("serial clock frequency not found\n");
			per_clock = 200000000;
		}

		speed = of_get_property(np, "current-speed", NULL);

		if (speed && *speed)
			baud_rate = *speed;
		else {
			pr_err("current speed not found\n");
			baud_rate = 9600;
		}
	} else {
		ret = -ENOMEM;
	}

	np = of_find_compatible_node(NULL, NULL, "lsi,acp3500");

	if (NULL == np) {
		unsigned long divisor;

		/*
		  In the 3500 case, the peripheral clock is connected
		  directly to the UART.  If this isn't 3500, set up
		  the second timer (which is in between the peripheral
		  clock and the UART) and adjust per_clock
		  accordingly.
		*/

		if (1000000 < per_clock) {
			divisor = per_clock / 25000000;
			per_clock = 25000000;
		} else {
			/* Emulation is much slower... */
			divisor = per_clock / 3250000;
			per_clock = 3250000;
		}

		--divisor;

		if (divisor != in_le32(uap->timer_base + TIMER_LOAD)) {
			while (0 ==
			       (in_le32((const volatile unsigned *)
					(uap->port.membase + UART01x_FR)) &
				UART011_FR_TXFE))
				;

			while (0 !=
			       (in_le32((const volatile unsigned *)
					(uap->port.membase + UART01x_FR)) &
				UART01x_FR_BUSY))
				;

			out_le32((uap->timer_base + TIMER_CONTROL), 0);
			out_le32((uap->timer_base + TIMER_LOAD), divisor);
			out_le32((uap->timer_base + TIMER_CONTROL),
				 (TIMER_CONTROL_ENABLE |
				  TIMER_CONTROL_MODE));
		}
	}

	dt_baud_rate = baud_rate;
	uap->port.iotype = UPIO_MEM;
	uap->port.fifosize = 16;
	uap->port.ops = &amba_acp_pops;
	uap->port.flags = UPF_BOOT_AUTOCONF;
	uap->port.line = i;
	get_clock_stuff(uap, baud_rate);
	acp_ports[i] = uap;
	ret = uart_add_one_port(driver, &uap->port);

	if (0 != ret) {
		acp_ports[i] = NULL;
		kfree(uap);
	}

 out:
	return ret;
}

/*
  ----------------------------------------------------------------------
  acp_serial_delete_ports
*/

static int
acp_serial_delete_ports(struct uart_driver *driver)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(acp_ports); ++i) {
		if (NULL != acp_ports[i]) {
			uart_remove_one_port(driver,
					     &(acp_ports[i])->port);
			kfree(acp_ports[i]);
			acp_ports[i] = NULL;
		}
	}

	return 0;
}

/*
  ======================================================================
  ======================================================================
  Linux module stuff.
  ======================================================================
  ======================================================================
*/

/*
  ----------------------------------------------------------------------
  acp_init
*/

int __init
acp_serial_init(void)
{
	int ret;

	pr_info("Serial: ACP Serial Driver\n");

	/* Clear the ports array */
	memset((void *) &acp_ports[0], 0,
		sizeof(struct uart_acp_port *) * ARRAY_SIZE(acp_ports));

	/* Register the driver */
	ret = uart_register_driver(&acp_serial_driver);

	if (0 != ret) {
		pr_err("uart_register_driver() failed with %d\n", ret);
		goto out;
	}

	/* Add ports */
	ret = acp_serial_add_ports(&acp_serial_driver);

	if (0 != ret) {
		pr_err("acp_serial_add_ports() failed with %d\n", ret);
		goto out;
	}

 out:
	return ret;
}
module_init(acp_serial_init);

/*
  ----------------------------------------------------------------------
  acp_serial_exit
*/

void __exit
acp_serial_exit(void)
{
	acp_serial_delete_ports(&acp_serial_driver);
	uart_unregister_driver(&acp_serial_driver);
}
module_exit(acp_serial_exit);

MODULE_AUTHOR("LSI Corporation");
MODULE_DESCRIPTION("ARM AMBA serial port on PPC476 driver");
MODULE_LICENSE("GPL");
