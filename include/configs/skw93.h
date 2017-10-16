/*
 * SPDX-License-Identifier:	GPL-2.0+
 *
 */

/*
 * This file contains the configuration parameters for skw93 target.
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#define CONFIG_SKW93		1
#define CONFIG_SOC_MT7628	1

/*
 * System memory
 */
#if (CONFIG_SYS_MEMSIZE==64)
	#define MEM_SIZE 64
	#define ON_BOARD_512M_DRAM_COMPONENT
#elif (CONFIG_SYS_MEMSIZE==128)
	#define MEM_SIZE 128
	#define ON_BOARD_1024M_DRAM_COMPONENT
#else
	#error "Please define MEMSIZE=64/128 in config file"
#endif
#define RALINK_DDR_POWERSAVE
#define CONFIG_DDR_CALIB

/*
 * SPI setup
 */
#define CONFIG_HARD_SPI
#ifdef CONFIG_HARD_SPI
#define CONFIG_MT7628_SPI		1
#define CONFIG_SPI_FLASH		1
#define CONFIG_CMD_SF			1
#define CONFIG_SPI_FLASH_WINBOND	1
#define CONFIG_SPI_FLASH_USE_4K_SECTORS	1
#endif

/*
 * Ethernet config
 */
#define CONFIG_MT7628_ETH
#ifdef CONFIG_MT7628_ETH
#define CONFIG_EPHY_LED_ON		1
#define CONFIG_SYS_RX_ETH_BUFFER	24
#define CONFIG_CMD_PING			1
#endif

/*
 * Misc.
 */
#undef CONFIG_CMD_IMLS
#define CONFIG_SYS_NO_FLASH

#define CONFIG_DISPLAY_BOARDINFO
#define CONFIG_MISC_INIT_R

#define CONFIG_BOOTDELAY	4	/* autoboot after 4 seconds */

#define CONFIG_BAUDRATE		57600

#define CONFIG_TIMESTAMP		/* Print image info with timestamp */

#undef CONFIG_BOOTARGS

#define CONFIG_EXTRA_ENV_SETTINGS					\
	"addtty=setenv bootargs ${bootargs} "				\
		"console=ttyS0,${baudrate}\0"				\
	"addmtdparts=setenv bootargs ${bootargs} "			\
		"mtdparts=raspi:192(u-boot),64k(Config),64k(Factory),-(firmware)\0"\
	"addphys=setenv bootargs $ ${bootargs} "			\
		"ethaddr=${ethaddr} wifiaddr=${wifiaddr}\0"		\
	"addmisc=setenv bootargs ${bootargs} "				\
		"panic=1\0"						\
	"net_boot=tftp ramdisk.img;"					\
		"run addtty addmtdparts addphys addmisc;"		\
		"bootm\0"						\
	"upd_uboot=tftp u-boot.bin;"					\
		"sf probe;"						\
		"sf erase 0 +${filesize};"				\
		"sf write ${fileaddr} 0 ${filesize}\0"			\
	"upd_config=tftp config.bin;"					\
		"sf probe;"						\
		"sf erase 30000 +${filesize};"				\
		"sf write ${fileaddr} 30000 ${filesize};"		\
		"saveenv\0"						\
	"upd_factory=tftp factory.bin;"					\
		"sf probe;"						\
		"sf erase 40000 +${filesize};"				\
		"sf write ${fileaddr} 40000 ${filesize}\0"		\
	"upd_firmware=tftp firmware.bin;"				\
		"sf probe;"						\
		"sf erase 50000 +${filesize};"				\
		"sf write ${fileaddr} 50000 ${filesize}\0"		\
	""

#define CONFIG_PREBOOT		"setenv bootargs"
#define CONFIG_BOOTCOMMAND	"run addtty addmtdparts addphys addmisc;"\
				"bootm 0x9c050000"

/*
 * BOOTP options
 */
#define CONFIG_BOOTP_BOOTFILESIZE
#define CONFIG_BOOTP_BOOTPATH
#define CONFIG_BOOTP_GATEWAY
#define CONFIG_BOOTP_HOSTNAME

/*
 * Command line configuration.
 */
#define CONFIG_CMD_DHCP

#define CONFIG_CONS_INDEX		1

/*
 * Miscellaneous configurable options
 */
#define CONFIG_SYS_LONGHELP		/* undef to save memory */

/* Monitor Command Prompt */
#define CONFIG_AUTO_COMPLETE
#define CONFIG_CMDLINE_EDITING
#undef CONFIG_SYS_HUSH_PARSER

/* Console I/O Buffer Size */
#define CONFIG_SYS_CBSIZE		256
/* Print Buffer Size */
#define CONFIG_SYS_PBSIZE (CONFIG_SYS_CBSIZE+sizeof(CONFIG_SYS_PROMPT)+16)
/* max number of command args */
#define CONFIG_SYS_MAXARGS		16

#define CONFIG_SYS_MALLOC_LEN		256*1024

#define CONFIG_SYS_BOOTPARAMS_LEN	128*1024

#define CONFIG_SYS_MHZ			575

#define CONFIG_SYS_MIPS_TIMER_FREQ	(CONFIG_SYS_MHZ * 1000000 / 2)

/* Cached addr */
#define CONFIG_SYS_SDRAM_BASE		0x80000000

/* default load address */
#define CONFIG_SYS_LOAD_ADDR		0x81000000

#define CONFIG_SYS_MEMTEST_START	0x80100000
#define CONFIG_SYS_MEMTEST_END		0x80800000

/* Max image length in bootm */
#define CONFIG_SYS_BOOTM_LEN		0x2000000

/*-----------------------------------------------------------------------
 * FLASH and environment organization
 */
#define CONFIG_SYS_MONITOR_BASE		CONFIG_SYS_TEXT_BASE
#define CONFIG_SYS_MONITOR_LEN		(192 << 10)

#define CONFIG_SYS_INIT_SP_OFFSET	0x400000

/* We boot from spi flash */
#define CONFIG_ENV_IS_IN_SPI_FLASH
#ifdef CONFIG_ENV_IS_IN_SPI_FLASH
#define CONFIG_ENV_SPI_CS		0
#define CONFIG_ENV_OFFSET		0x30000
#define CONFIG_ENV_SIZE			0x1000
#define CONFIG_ENV_SECT_SIZE		0x1000
#endif

#define CONFIG_ENV_OVERWRITE		1

#define CONFIG_LZMA

/*-----------------------------------------------------------------------
 * Cache Configuration
 */
#define CONFIG_SYS_DCACHE_SIZE		32768
#define CONFIG_SYS_ICACHE_SIZE		65536
#define CONFIG_SYS_CACHELINE_SIZE	32

#endif /* __CONFIG_H */
