/*
 * SPDX-License-Identifier:	GPL-2.0+
 *
 */

#ifndef __MT7628_REG__
#define __MT7628_REG__

#define RALINK_REG(x)		(*((volatile u32 *)(x)))
#define ra_inb(offset)		(*(volatile unsigned char *)(offset))
#define ra_inw(offset)		(*(volatile unsigned short *)(offset))
#define ra_inl(offset)		(*(volatile unsigned long *)(offset))
#define ra_outb(offset,val)	(*(volatile unsigned char *)(offset) = val)
#define ra_outw(offset,val)	(*(volatile unsigned short *)(offset) = val)
#define ra_outl(offset,val)	(*(volatile unsigned long *)(offset) = val)
#define ra_and(addr, value) ra_outl(addr, (ra_inl(addr) & (value)))
#define ra_or(addr, value) ra_outl(addr, (ra_inl(addr) | (value)))

/* Module Base Addresses */
#define RALINK_SYSCTL_BASE              0xB0000000
#define RALINK_TIMER_BASE               0xB0000100
#define RALINK_INTCL_BASE               0xB0000200
#define RALINK_MEMCTRL_BASE             0xB0000300
#define RALINK_RBUS_MATRIXCTL_BASE      0xB0000400
#define RALINK_MIPS_CNT_BASE            0x10000500
#define RALINK_PIO_BASE                 0xB0000600
#define RALINK_I2C_BASE                 0xB0000900
#define RALINK_I2S_BASE                 0xB0000A00
#define RALINK_SPI_BASE                 0xB0000B00
#define RALINK_UART_LITE_BASE           0x10000C00
#define RALINK_UART_LITE2_BASE          0x10000D00
#define RALINK_UART_LITE3_BASE          0x10000E00
#define RALINK_PCM_BASE                 0xB0002000
#define RALINK_GDMA_BASE                0xB0002800
#define RALINK_AES_ENGING_BASE          0xB0004000
#define RALINK_CRYPTO_ENGING_BASE       RALINK_AES_ENGING_BASE
#define RALINK_RGCTRL_BASE		0xB0001000
#define RALINK_FRAME_ENGINE_BASE        0xB0100000
#define RALINK_ETH_SW_BASE              0xB0110000
#define RALINK_USB_DEV_BASE             0x10120000
#define RALINK_MSDC_BASE                0xB0130000
#define RALINK_PCI_BASE                 0xB0140000
#define RALINK_11N_MAC_BASE             0xB0180000
#define RALINK_USB_HOST_BASE            0x101C0000
#define RALINK_MCNT_CFG                 0xB0000D00
#define RALINK_COMPARE                  0xB0000D04
#define RALINK_COUNT                    0xB0000D08

/*
 * System register
 */
#define RALINK_CHIP_REV_ID_REG		(RALINK_SYSCTL_BASE+0x0c)
#define RALINK_SYSCFG_REG		(RALINK_SYSCTL_BASE+0x10)
#define RALINK_SYSCFG1_REG		(RALINK_SYSCTL_BASE+0x14)
#define RALINK_ROMSTAT_REG		(RALINK_SYSCTL_BASE+0x28)
#define RALINK_CLKCFG0_REG		(RALINK_SYSCTL_BASE+0x2c)
#define RALINK_CLKCFG1_REG		(RALINK_SYSCTL_BASE+0x30)
#define RALINK_RSTCTRL_REG		(RALINK_SYSCTL_BASE+0x34)
#define RALINK_RSTSTAT_REG		(RALINK_SYSCTL_BASE+0x38)
#define RALINK_AGPIOCFG_REG		(RALINK_SYSCTL_BASE+0x3c)
#define RALINK_CPLLCFG0_REG		(RALINK_SYSCTL_BASE+0x54)
#define RALINK_CPLLCFG1_REG		(RALINK_SYSCTL_BASE+0x58)

/* Timer related */
#define RALINK_DYN_CFG0_REG		(RALINK_RBUS_MATRIXCTL_BASE+0x40)

/* CPLL related */
#define CPLL_SW_CONFIG                  (0x1UL << 31)
#define CPLL_MULT_RATIO_SHIFT           16
#define CPLL_MULT_RATIO                 (0x7UL << CPLL_MULT_RATIO_SHIFT)
#define CPLL_DIV_RATIO_SHIFT            10
#define CPLL_DIV_RATIO                  (0x3UL << CPLL_DIV_RATIO_SHIFT)
#define BASE_CLOCK                      40      /* Mhz */

/* Reset Control Register */
#define RALINK_SYS_RST                  (1<<0)
#define RALINK_TIMER_RST                (1<<8)
#define RALINK_INTC_RST                 (1<<9)
#define RALINK_MC_RST                   (1<<10)
#define RALINK_PCM_RST                  (1<<11)
#define RALINK_UART_RST                 (1<<12)
#define RALINK_PIO_RST                  (1<<13)
#define RALINK_DMA_RST                  (1<<14)
#define RALINK_I2C_RST                  (1<<16)
#define RALINK_I2S_RST                  (1<<17)
#define RALINK_SPI_RST                  (1<<18)
#define RALINK_UARTL_RST                (1<<19)
#define RALINK_FE_RST                   (1<<21)
#define RALINK_UHST_RST                 (1<<22)
#define RALINK_ESW_RST                  (1<<23)
#define RALINK_EPHY_RST                 (1<<24)
#define RALINK_UDEV_RST                 (1<<25)
#define RALINK_PCIE0_RST                (1<<26)
#define RALINK_PCIE1_RST                (1<<27)
#define RALINK_MIPS_CNT_RST             (1<<28)
#define RALINK_CRYPTO_RST               (1<<29)

/* Interrupt Controller Register */
#define RALINK_INTCTL_SYSCTL            (1<<0)
#define RALINK_INTCTL_TIMER0            (1<<1)
#define RALINK_INTCTL_WDTIMER           (1<<2)
#define RALINK_INTCTL_ILL_ACCESS        (1<<3)
#define RALINK_INTCTL_PCM               (1<<4)
#define RALINK_INTCTL_UART              (1<<5)
#define RALINK_INTCTL_PIO               (1<<6)
#define RALINK_INTCTL_DMA               (1<<7)
#define RALINK_INTCTL_PC                (1<<9)
#define RALINK_INTCTL_I2S               (1<<10)
#define RALINK_INTCTL_SPI               (1<<11)
#define RALINK_INTCTL_UARTLITE          (1<<12)
#define RALINK_INTCTL_CRYPTO            (1<<13)
#define RALINK_INTCTL_ESW               (1<<17)
#define RALINK_INTCTL_UHST              (1<<18)
#define RALINK_INTCTL_UDEV              (1<<19)
#define RALINK_INTCTL_GLOBAL            (1<<31)

/* Clock Conf Register */
#define RALINK_UPHY1_CLK_EN     (1<<22)
#define RALINK_UPHY0_CLK_EN     (1<<25)
#define RALINK_PCIE_CLK_EN    	(1<<26)

/*
 * EMC register
 */
#define RALINK_DDR_CFG0			(RALINK_MEMCTRL_BASE+0x40)
#define RALINK_DDR_CFG1			(RALINK_MEMCTRL_BASE+0x44)
#define RALINK_DDR_CFG2			(RALINK_MEMCTRL_BASE+0x48)
#define RALINK_DDR_CFG3			(RALINK_MEMCTRL_BASE+0x4c)
#define RALINK_DDR_CFG4			(RALINK_MEMCTRL_BASE+0x50)
#define RALINK_DDR_CFG8			(RALINK_MEMCTRL_BASE+0x60)
#define RALINK_DDR_CFG9			(RALINK_MEMCTRL_BASE+0x64)
#define RALINK_DDR_CFG10		(RALINK_MEMCTRL_BASE+0x68)

#define RAS_OFFSET	23
#define TRFC_OFFSET	13
#define TRFI_OFFSET	0
#define BL_VALUE	3 //BL=8

#define CAS_OFFSET	4
#define BL_OFFSET	0
#define AdditiveLatency_OFFSET 3

#if defined (W9751G6IB_25)
#define RAS_VALUE	7
#define TRFC_VALUE	9
#define TRFI_VALUE	650
#define CAS_VALUE	3
#define AdditiveLatency_VALUE  0
#endif //W9751G6IB_25

/*
 * UART registers
 */
#define RALINK_UART1	0x0C00  /* UART Lite */
#define RALINK_UART2	0x0D00  /* UART Lite */
#define RALINK_UART3	0x0E00  /* UART Lite */
#define CFG_RALINK_CONSOLE	RALINK_UART1


#define RALINK_UART_RBR_OFFSET	0x00
#define RALINK_UART_TBR_OFFSET	0x00
#define RALINK_UART_IER_OFFSET	0x04
#define RALINK_UART_IIR_OFFSET	0x08
#define RALINK_UART_FCR_OFFSET	0x08
#define RALINK_UART_LCR_OFFSET	0x0C
#define RALINK_UART_MCR_OFFSET	0x10
#define RALINK_UART_LSR_OFFSET	0x14
#define RALINK_UART_DLL_OFFSET	0x00
#define RALINK_UART_DLM_OFFSET	0x04

#define RBR(x)		RALINK_REG(RALINK_SYSCTL_BASE+(x)+RALINK_UART_RBR_OFFSET)
#define TBR(x)		RALINK_REG(RALINK_SYSCTL_BASE+(x)+RALINK_UART_TBR_OFFSET)
#define IER(x)		RALINK_REG(RALINK_SYSCTL_BASE+(x)+RALINK_UART_IER_OFFSET)
#define IIR(x)		RALINK_REG(RALINK_SYSCTL_BASE+(x)+RALINK_UART_IIR_OFFSET)
#define FCR(x)		RALINK_REG(RALINK_SYSCTL_BASE+(x)+RALINK_UART_FCR_OFFSET)
#define LCR(x)		RALINK_REG(RALINK_SYSCTL_BASE+(x)+RALINK_UART_LCR_OFFSET)
#define MCR(x)		RALINK_REG(RALINK_SYSCTL_BASE+(x)+RALINK_UART_MCR_OFFSET)
#define LSR(x)		RALINK_REG(RALINK_SYSCTL_BASE+(x)+RALINK_UART_LSR_OFFSET)
#define DLL(x)		RALINK_REG(RALINK_SYSCTL_BASE+(x)+RALINK_UART_DLL_OFFSET)
#define DLM(x)		RALINK_REG(RALINK_SYSCTL_BASE+(x)+RALINK_UART_DLM_OFFSET)

#define IER_ELSI	(1 << 2)	/* Receiver Line Status Interrupt Enable */
#define IER_ETBEI	(1 << 1)	/* Transmit Buffer Empty Interrupt Enable */
#define IER_ERBFI	(1 << 0)	/* Data Ready or Character Time-Out Interrupt Enable */

#define IIR_FIFOES1	(1 << 7)	/* FIFO Mode Enable Status */
#define IIR_FIFOES0	(1 << 6)	/* FIFO Mode Enable Status */
#define IIR_IID3	(1 << 3)	/* Interrupt Source Encoded */
#define IIR_IID2	(1 << 2)	/* Interrupt Source Encoded */
#define IIR_IID1	(1 << 1)	/* Interrupt Source Encoded */
#define IIR_IP		(1 << 0)	/* Interrupt Pending (active low) */

#define FCR_RXTRIG1	(1 << 7)	/* Receiver Interrupt Trigger Level */
#define FCR_RXTRIG0	(1 << 6)	/* Receiver Interrupt Trigger Level */
#define FCR_TXTRIG1	(1 << 5)	/* Transmitter Interrupt Trigger Level */
#define FCR_TXTRIG0	(1 << 4)	/* Transmitter Interrupt Trigger Level */
#define FCR_DMAMODE	(1 << 3)	/* Enable DMA transfers */
#define FCR_TXRST	(1 << 2)	/* Reset Transmitter FIFO */
#define FCR_RXRST	(1 << 1)	/* Reset Receiver FIFO */
#define FCR_FIFOE	(1 << 0)	/* Transmit and Receive FIFO Enable */


#define LCR_DLAB	(1 << 7)	/* Divisor Latch Access Bit */
#define LCR_SB		(1 << 6)	/* Set Break */
#define LCR_STKYP	(1 << 5)	/* Sticky Parity */
#define LCR_EPS		(1 << 4)	/* Even Parity Select */
#define LCR_PEN		(1 << 3)	/* Parity Enable */
#define LCR_STB		(1 << 2)	/* Stop Bit */
#define LCR_WLS1	(1 << 1)	/* Word Length Select */
#define LCR_WLS0	(1 << 0)	/* Word Length Select */

#define MCR_LOOP	(1 << 4)	/* Loop-back Mode Enable */

#define MSR_DCD		(1 << 7)	/* Data Carrier Detect */
#define MSR_RI		(1 << 6)	/* Ring Indicator */
#define MSR_DSR		(1 << 5)	/* Data Set Ready */
#define MSR_CTS		(1 << 4)	/* Clear To Send */
#define MSR_DDCD	(1 << 3)	/* Delta Data Carrier Detect */
#define MSR_TERI	(1 << 2)	/* Trailing Edge Ring Indicator */
#define MSR_DDSR	(1 << 1)	/* Delta Data Set Ready */
#define MSR_DCTS	(1 << 0)	/* Delta Clear To Send */


#define LSR_FIFOE	(1 << 7)	/* FIFO Error Status */
#define LSR_TEMT	(1 << 6)	/* Transmitter Empty */
#define LSR_TDRQ	(1 << 5)	/* Transmit Data Request */
#define LSR_BI		(1 << 4)	/* Break Interrupt */
#define LSR_FE		(1 << 3)	/* Framing Error */
#define LSR_PE		(1 << 2)	/* Parity Error */
#define LSR_OE		(1 << 1)	/* Overrun Error */
#define LSR_DR		(1 << 0)	/* Data Ready */

/*
 * SPI registers
 */
#define SPI_REG_CTL		(RALINK_SPI_BASE + 0x00)
#define SPI_REG_OPCODE		(RALINK_SPI_BASE + 0x04)
#define SPI_REG_DATA0		(RALINK_SPI_BASE + 0x08)
#define SPI_REG_DATA(x)		(SPI_REG_DATA0 + (x * 4))
#define SPI_REG_MASTER		(RALINK_SPI_BASE + 0x28)
#define SPI_REG_MOREBUF		(RALINK_SPI_BASE + 0x2c)
#define SPI_REG_Q_CTL		(RALINK_SPI_BASE + 0x30)
#define SPI_REG_SPACE_CR	(RALINK_SPI_BASE + 0x3c)

#define SPI_CTL_START		0x00000100
#define SPI_CTL_BUSY		0x00010000
#define SPI_CTL_TXCNT_MASK	0x0000000f
#define SPI_CTL_RXCNT_MASK	0x000000f0
#define SPI_CTL_TX_RX_CNT_MASK	0x000000ff
#define SPI_CTL_SIZE_MASK	0x00180000
#define SPI_CTL_ADDREXT_MASK	0xff000000

#define SPI_MBCTL_TXCNT_MASK		0x000001ff
#define SPI_MBCTL_RXCNT_MASK		0x001ff000
#define SPI_MBCTL_TX_RX_CNT_MASK	(SPI_MBCTL_TXCNT_MASK | SPI_MBCTL_RXCNT_MASK)
#define SPI_MBCTL_CMD_MASK		0x2f000000


#endif /* __MT7628_REG__ */
