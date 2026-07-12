// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Serial driver for the Cortina-Access "Elnath" (CA77xx) SoC UART.
 *
 * Copyright (C) 2026 The Linux Kernel contributors
 *
 * Written from the hardware register map. The block is a simple
 * FIFO-based UART: 32-bit little-endian MMIO registers, one level-high
 * interrupt line, and a fixed input clock. The boot loader programs the
 * baud divisor (URX_SAMPLE); this driver deliberately leaves it alone
 * and runs the line at the loader-configured 115200 8N1.
 */

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/console.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>

/* Register map (32-bit, native/little-endian, no reg-shift) */
#define UCFG			0x00	/* config */
#define   UCFG_TX_EN		BIT(5)
#define   UCFG_RX_EN		BIT(6)
#define   UCFG_UART_EN		BIT(7)
#define   UCFG_BAUD_START	BIT(8)	/* latch a new URX_SAMPLE (unused) */
#define UFC			0x04	/* flow control - keep reset value */
#define URX_SAMPLE		0x08	/* baud divisor - loader-owned */
#define UTX_DATA		0x10	/* TX data (write one byte) */
#define URX_DATA		0x14	/* RX data (read one byte) */
#define UINFO			0x18	/* FIFO status (RO) */
#define   UINFO_RX_FULL		BIT(0)
#define   UINFO_RX_EMPTY	BIT(1)
#define   UINFO_TX_FULL		BIT(2)
#define   UINFO_TX_EMPTY	BIT(3)
#define UINT_EN			0x1c	/* interrupt enable (uses UCFG TX_EN/RX_EN bits) */
#define UINT_PEND		0x24	/* interrupt pending; write-back to acknowledge */

/*
 * The interrupt-enable register (UINT_EN) uses the same TX_EN/RX_EN bit
 * positions as UCFG. UINT_PEND is read for pending sources and written back
 * to acknowledge; the RX/TX work is then driven off the UINFO FIFO flags.
 */
#define UINT_RX_NOT_EMPTY	BIT(6)	/* == UCFG RX_EN: enable RX interrupt */
#define UINT_TX_EMPTY		BIT(5)	/* == UCFG TX_EN: enable TX interrupt */

#define CORTINA_UART_REGSIZE	0x30
#define CORTINA_UART_BAUD	115200
#define CORTINA_UART_CLK_HZ	125000000	/* fallback if no DT clock */
#define CORTINA_UART_FIFOSIZE	16

/* Local port type id, only compared within this driver (not in uapi) */
#define PORT_CORTINA		130

static struct uart_port cortina_uart_port;

static u32 cortina_uart_info(struct uart_port *port)
{
	return readl(port->membase + UINFO);
}

static unsigned int cortina_uart_tx_empty(struct uart_port *port)
{
	return (cortina_uart_info(port) & UINFO_TX_EMPTY) ? TIOCSER_TEMT : 0;
}

static void cortina_uart_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	/* No modem control lines */
}

static unsigned int cortina_uart_get_mctrl(struct uart_port *port)
{
	/* No modem control lines: report them permanently asserted */
	return TIOCM_CAR | TIOCM_DSR | TIOCM_CTS;
}

static void cortina_uart_irq_enable(struct uart_port *port, u32 mask)
{
	writel(readl(port->membase + UINT_EN) | mask, port->membase + UINT_EN);
}

static void cortina_uart_irq_disable(struct uart_port *port, u32 mask)
{
	writel(readl(port->membase + UINT_EN) & ~mask, port->membase + UINT_EN);
}

static void cortina_uart_tx_chars(struct uart_port *port)
{
	u8 ch;

	uart_port_tx(port, ch,
		     !(cortina_uart_info(port) & UINFO_TX_FULL),
		     writel(ch, port->membase + UTX_DATA));
}

static void cortina_uart_stop_tx(struct uart_port *port)
{
	cortina_uart_irq_disable(port, UINT_TX_EMPTY);
}

static void cortina_uart_start_tx(struct uart_port *port)
{
	cortina_uart_irq_enable(port, UINT_TX_EMPTY);
	cortina_uart_tx_chars(port);
}

static void cortina_uart_stop_rx(struct uart_port *port)
{
	cortina_uart_irq_disable(port, UINT_RX_NOT_EMPTY);
}

static void cortina_uart_rx_chars(struct uart_port *port)
{
	while (!(cortina_uart_info(port) & UINFO_RX_EMPTY)) {
		u8 ch = readl(port->membase + URX_DATA);

		port->icount.rx++;
		if (uart_handle_sysrq_char(port, ch))
			continue;
		uart_insert_char(port, 0, 0, ch, TTY_NORMAL);
	}

	tty_flip_buffer_push(&port->state->port);
}

static irqreturn_t cortina_uart_isr(int irq, void *dev_id)
{
	struct uart_port *port = dev_id;
	u32 stat;

	uart_port_lock(port);

	stat = readl(port->membase + UINT_PEND);
	if (!stat) {
		uart_port_unlock(port);
		return IRQ_NONE;
	}

	/* Ack by writing the pending bits back to the same register. */
	writel(stat, port->membase + UINT_PEND);

	cortina_uart_rx_chars(port);

	/* TX-empty source armed means transmission is in progress */
	if (readl(port->membase + UINT_EN) & UINT_TX_EMPTY)
		cortina_uart_tx_chars(port);

	uart_port_unlock(port);

	return IRQ_HANDLED;
}

static int cortina_uart_startup(struct uart_port *port)
{
	unsigned long flags;
	u32 cfg;
	int ret;

	ret = request_irq(port->irq, cortina_uart_isr, 0,
			  dev_name(port->dev), port);
	if (ret)
		return ret;

	uart_port_lock_irqsave(port, &flags);

	cfg = readl(port->membase + UCFG);
	cfg |= UCFG_TX_EN | UCFG_RX_EN | UCFG_UART_EN;
	writel(cfg, port->membase + UCFG);

	writel(~0u, port->membase + UINT_PEND);
	writel(UINT_RX_NOT_EMPTY, port->membase + UINT_EN);

	uart_port_unlock_irqrestore(port, flags);

	return 0;
}

static void cortina_uart_shutdown(struct uart_port *port)
{
	unsigned long flags;

	uart_port_lock_irqsave(port, &flags);

	/*
	 * Mask and clear all interrupt sources. Keep UCFG UART/TX/RX
	 * enabled: the (polled) console shares this port.
	 */
	writel(0, port->membase + UINT_EN);
	writel(~0u, port->membase + UINT_PEND);

	uart_port_unlock_irqrestore(port, flags);

	free_irq(port->irq, port);
}

static void cortina_uart_set_termios(struct uart_port *port,
				     struct ktermios *termios,
				     const struct ktermios *old)
{
	unsigned long flags;

	/*
	 * The boot loader has already programmed URX_SAMPLE for
	 * 115200 8N1; reprogramming the divisor here risks losing the
	 * console. Force the line to 115200 8N1 and only update the
	 * serial-core bookkeeping.
	 */
	termios->c_cflag &= ~(CSIZE | CSTOPB | PARENB | PARODD | CMSPAR);
	termios->c_cflag |= CS8;

	uart_port_lock_irqsave(port, &flags);
	uart_update_timeout(port, termios->c_cflag, CORTINA_UART_BAUD);
	uart_port_unlock_irqrestore(port, flags);

	tty_termios_encode_baud_rate(termios, CORTINA_UART_BAUD,
				     CORTINA_UART_BAUD);
}

static const char *cortina_uart_type(struct uart_port *port)
{
	return port->type == PORT_CORTINA ? "cortina-uart" : NULL;
}

static void cortina_uart_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE)
		port->type = PORT_CORTINA;
}

static int cortina_uart_request_port(struct uart_port *port)
{
	/* Resources are managed (devm) in probe */
	return 0;
}

static void cortina_uart_release_port(struct uart_port *port)
{
}

static int cortina_uart_verify_port(struct uart_port *port,
				    struct serial_struct *ser)
{
	if (ser->type != PORT_UNKNOWN && ser->type != PORT_CORTINA)
		return -EINVAL;
	return 0;
}

static const struct uart_ops cortina_uart_ops = {
	.tx_empty	= cortina_uart_tx_empty,
	.set_mctrl	= cortina_uart_set_mctrl,
	.get_mctrl	= cortina_uart_get_mctrl,
	.stop_tx	= cortina_uart_stop_tx,
	.start_tx	= cortina_uart_start_tx,
	.stop_rx	= cortina_uart_stop_rx,
	.startup	= cortina_uart_startup,
	.shutdown	= cortina_uart_shutdown,
	.set_termios	= cortina_uart_set_termios,
	.type		= cortina_uart_type,
	.config_port	= cortina_uart_config_port,
	.request_port	= cortina_uart_request_port,
	.release_port	= cortina_uart_release_port,
	.verify_port	= cortina_uart_verify_port,
};

#if defined(CONFIG_SERIAL_CORTINA_CONSOLE) || defined(CONFIG_SERIAL_EARLYCON)
static void cortina_uart_console_putchar(struct uart_port *port,
					 unsigned char ch)
{
	u32 val;

	/* Bounded spin for TX-FIFO empty; drop the char on timeout */
	if (readl_poll_timeout_atomic(port->membase + UINFO, val,
				      val & UINFO_TX_EMPTY, 0, 10000))
		return;

	writel(ch, port->membase + UTX_DATA);
}
#endif

#ifdef CONFIG_SERIAL_CORTINA_CONSOLE
static struct uart_driver cortina_uart_driver;

static void cortina_uart_console_write(struct console *co, const char *s,
				       unsigned int count)
{
	struct uart_port *port = &cortina_uart_port;
	unsigned long flags;
	int locked = 1;

	if (port->sysrq)
		locked = 0;
	else if (oops_in_progress)
		locked = uart_port_trylock_irqsave(port, &flags);
	else
		uart_port_lock_irqsave(port, &flags);

	uart_console_write(port, s, count, cortina_uart_console_putchar);

	if (locked)
		uart_port_unlock_irqrestore(port, flags);
}

static int cortina_uart_console_setup(struct console *co, char *options)
{
	struct uart_port *port = &cortina_uart_port;
	int baud = CORTINA_UART_BAUD;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	if (!port->membase)
		return -ENODEV;

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	return uart_set_options(port, co, baud, parity, bits, flow);
}

static struct console cortina_uart_console = {
	.name	= "ttyS",
	.write	= cortina_uart_console_write,
	.device	= uart_console_device,
	.setup	= cortina_uart_console_setup,
	.flags	= CON_PRINTBUFFER,
	.index	= -1,
	.data	= &cortina_uart_driver,
};

#define CORTINA_UART_CONSOLE	(&cortina_uart_console)
#else
#define CORTINA_UART_CONSOLE	NULL
#endif /* CONFIG_SERIAL_CORTINA_CONSOLE */

#ifdef CONFIG_SERIAL_EARLYCON
static void cortina_uart_earlycon_write(struct console *con, const char *s,
					unsigned int count)
{
	struct earlycon_device *dev = con->data;

	uart_console_write(&dev->port, s, count, cortina_uart_console_putchar);
}

static int __init cortina_uart_earlycon_setup(struct earlycon_device *device,
					      const char *options)
{
	if (!device->port.membase)
		return -ENODEV;

	device->con->write = cortina_uart_earlycon_write;

	return 0;
}
OF_EARLYCON_DECLARE(cortina, "cortina,serial", cortina_uart_earlycon_setup);
#endif /* CONFIG_SERIAL_EARLYCON */

static struct uart_driver cortina_uart_driver = {
	.owner		= THIS_MODULE,
	.driver_name	= "cortina-uart",
	.dev_name	= "ttyS",
	.major		= TTY_MAJOR,
	.minor		= 64,
	.nr		= 1,
	.cons		= CORTINA_UART_CONSOLE,
};

static int cortina_uart_probe(struct platform_device *pdev)
{
	struct uart_port *port = &cortina_uart_port;
	struct resource *res;
	unsigned long rate;
	struct clk *clk;
	int irq;

	if (port->dev)
		return -EBUSY;	/* single port */

	port->membase = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(port->membase))
		return PTR_ERR(port->membase);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	clk = devm_clk_get_optional_enabled(&pdev->dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(clk),
				     "failed to get clock\n");
	rate = clk_get_rate(clk);

	port->dev	= &pdev->dev;
	port->mapbase	= res->start;
	port->mapsize	= resource_size(res);
	port->irq	= irq;
	port->uartclk	= rate ? rate : CORTINA_UART_CLK_HZ;
	port->iotype	= UPIO_MEM32;
	port->fifosize	= CORTINA_UART_FIFOSIZE;
	port->flags	= UPF_BOOT_AUTOCONF;
	port->ops	= &cortina_uart_ops;
	port->line	= 0;
	port->type	= PORT_CORTINA;
	port->has_sysrq	= IS_ENABLED(CONFIG_SERIAL_CORTINA_CONSOLE);

	platform_set_drvdata(pdev, port);

	return uart_add_one_port(&cortina_uart_driver, port);
}

static void cortina_uart_remove(struct platform_device *pdev)
{
	struct uart_port *port = platform_get_drvdata(pdev);

	uart_remove_one_port(&cortina_uart_driver, port);
	port->dev = NULL;
	port->membase = NULL;
}

static const struct of_device_id cortina_uart_of_match[] = {
	{ .compatible = "cortina,serial" },
	{ }
};
MODULE_DEVICE_TABLE(of, cortina_uart_of_match);

static struct platform_driver cortina_uart_platform_driver = {
	.probe	= cortina_uart_probe,
	.remove	= cortina_uart_remove,
	.driver	= {
		.name		= "cortina-uart",
		.of_match_table	= cortina_uart_of_match,
	},
};

static int __init cortina_uart_init(void)
{
	int ret;

	ret = uart_register_driver(&cortina_uart_driver);
	if (ret)
		return ret;

	ret = platform_driver_register(&cortina_uart_platform_driver);
	if (ret)
		uart_unregister_driver(&cortina_uart_driver);

	return ret;
}

static void __exit cortina_uart_exit(void)
{
	platform_driver_unregister(&cortina_uart_platform_driver);
	uart_unregister_driver(&cortina_uart_driver);
}

module_init(cortina_uart_init);
module_exit(cortina_uart_exit);

MODULE_DESCRIPTION("Cortina-Access Elnath SoC UART driver");
MODULE_LICENSE("GPL");
