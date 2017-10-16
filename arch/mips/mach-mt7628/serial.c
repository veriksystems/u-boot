/*
 * SPDX-License-Identifier:	GPL-2.0+
 *
 */

#include <common.h>
#include <serial.h>
#include <asm/mt7628.h>

#define SERIAL_CLOCK_DIVISOR 16

static void mt7628_serial_setbrg(void);

/*
 * Initialise the serial port with the given baudrate. The settings
 * are always 8 data bits, no parity, 1 stop bit, no start bits.
 *
 */
static int mt7628_serial_init(void)
{
	mt7628_serial_setbrg ();
	return (0);
}

/* this function does not need to know the cpu and bus clock after RT3883. the clock is fix at 40Mhz */
static void mt7628_serial_setbrg(void)
{
	//DECLARE_GLOBAL_DATA_PTR;
	unsigned int clock_divisor = 0;

	/*
	 * CPU_CLK_SEL (bit 21:20)
	 */

	//reset uart lite and 2 uart full
	*(unsigned long *)(RALINK_SYSCTL_BASE + 0x0034) |= cpu_to_le32(1<<20|1<<19|1<<12);
	/* RST Control change from W1C to W1W0 to reset, update 20080812 */
	*(unsigned long *)(RALINK_SYSCTL_BASE + 0x0034) &= ~(1<<20|1<<19|1<<12);

	/* RST Control change from W1C to W1W0 to reset, update 20080812 */
	//*(unsigned long *)(RALINK_SYSCTL_BASE + 0x0034) = 0;
	//clock_divisor = (CPU_CLOCK_RATE / SERIAL_CLOCK_DIVISOR / gd->baudrate);
	clock_divisor = (40*1000*1000/ SERIAL_CLOCK_DIVISOR / CONFIG_BAUDRATE);

	IER(CFG_RALINK_CONSOLE) = 0;		/* Disable for now */
	FCR(CFG_RALINK_CONSOLE) = 0;		/* No fifos enabled */

	/* set baud rate */
	LCR(CFG_RALINK_CONSOLE) = LCR_WLS0 | LCR_WLS1 | LCR_DLAB;
	DLL(CFG_RALINK_CONSOLE) = clock_divisor & 0xff;
	DLM(CFG_RALINK_CONSOLE) = clock_divisor >> 8;
	LCR(CFG_RALINK_CONSOLE) = LCR_WLS0 | LCR_WLS1;
}

/*
 * Output a single byte to the serial port.
 */
static void mt7628_serial_putc(const char c)
{
	/* wait for room in the tx FIFO on UART */
	while ((LSR(CFG_RALINK_CONSOLE) & LSR_TEMT) == 0);

	TBR(CFG_RALINK_CONSOLE) = c;

	/* If \n, also do \r */
	if (c == '\n')
		mt7628_serial_putc ('\r');
}

/*
 * Read a single byte from the serial port. Returns 1 on success, 0
 * otherwise. When the function is succesfull, the character read is
 * written into its argument c.
 */
static int mt7628_serial_getc(void)
{
	while (!(LSR(CFG_RALINK_CONSOLE) & LSR_DR));
	return (char) RBR(CFG_RALINK_CONSOLE) & 0xff;
}

/*
 * Read a single byte from the serial port. Returns 1 on success, 0
 * otherwise. When the function is succesfull, the character read is
 * written into its argument c.
 */
static int mt7628_serial_tstc(void)
{
	return LSR(CFG_RALINK_CONSOLE) & LSR_DR;
}

static struct serial_device mt7628_serial_drv = {
	.name	= "mt7628_serial",
	.start	= mt7628_serial_init,
	.stop	= NULL,
	.setbrg	= mt7628_serial_setbrg,
	.putc	= mt7628_serial_putc,
	.puts	= default_serial_puts,
	.getc	= mt7628_serial_getc,
	.tstc	= mt7628_serial_tstc,
};

void mt7628_serial_initialize(void)
{
	serial_register(&mt7628_serial_drv);
}

__weak struct serial_device *default_serial_console(void)
{
	return &mt7628_serial_drv;
}
