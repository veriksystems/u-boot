/*
 * SPDX-License-Identifier:	GPL-2.0+
 *
 */

#include <config.h>
#include <common.h>
#include <asm/asm.h>
#include <asm/regdef.h>
#include <asm/mipsregs.h>
#include <asm/mt7628.h>

	.text
	.set noreorder
	.set mips32

	.globl	lowlevel_init
lowlevel_init:

	/*
	 * Step 2) Establish Status Register
	 * (set BEV, clear ERL, clear EXL, clear IE)
	 */
	li	t1, 0x00400000
	mtc0	t1, CP0_STATUS

	/*
	 * Step 3) Establish CP0 Config0
	 * (set K0=3)
	 */
	li	t1, 0x00000003
	mtc0	t1, CP0_CONFIG

	/*
	 * Step 7) Establish Cause
	 * (set IV bit)
	 */
	li	t1, 0x00800000
	mtc0	t1, CP0_CAUSE

	/* Establish Wired (and Random) */
	mtc0	zero, CP0_WIRED
	nop

#define DELAY_USEC(us)              ((58*(us))/3)
CPLD_POLL:
	/* polling CPLL is ready */
	li	t1, DELAY_USEC(1000000)
	la	t5, RALINK_ROMSTAT_REG
1:
	lw		t2, 0(t5)
	andi	t2, t2, 0x1
	bnez	t2, CPLL_READY
	subu	t1, t1, 1
	bgtz	t1, 1b
	nop
	la      t0, RALINK_CLKCFG0_REG
	lw      t3, 0(t0)
	ori	t3, t3, 0x1
	sw	t3, 0(t0)
	j	CPLL_DONE
	nop

CPLL_READY:
	la	t0, RALINK_CLKCFG0_REG
	lw	t1, 0(t0)
	li	t2, ~0x0C
	and	t1, t1, t2
	ori	t1, t1, 0xC
	sw	t1, 0(t0)
	la	t0, RALINK_DYN_CFG0_REG
	lw	t3, 0(t0)
	li	t5, ~((0x0F<<8)|(0x0F<<0))
	and	t3, t3, t5
	li	t5, (1<<8)|(1<<0) //bits[11:8] cpu_fdiv = 10, cause boot slow due to mips counter freq reduce 10 times
	or	t3, t3, t5
	sw	t3, 0(t0)
	la	t0, RALINK_CLKCFG0_REG
	lw	t3, 0(t0)
	li	t4, ~0x0F
	and     t3, t3, t4
	ori	t3, t3, 0xC
	sw	t3, 0(t0)
	lw	t3, 0(t0)
	ori	t3, t3, 0x08
	sw	t3, 0(t0)
CPLL_DONE:

#if !defined(CONFIG_SYS_RAMBOOT)
INIT_DRAM:
	/* put MC to reset */
	li	t0, RALINK_RSTCTRL_REG
	lw	t1, 0(t0)
	ori	t1, t1, 1<<10
	sw	t1, 0(t0)

	/* SDR and DDR initialization: delay 200us
	 */
	li t0, DELAY_USEC(200+40)
	li t1, 0x1
1:
	sub t0, t0, t1
	bnez t0, 1b
	nop

	/* set DRAM IO PAD for MT7628IC */
	/* DDR LDO Enable  */
	li  t1, RALINK_RGCTRL_BASE+0x100
	lw  t4, 0(t1)
	li  t2, (1<<31)
	or  t4, t4, t2
	sw  t4, 0(t1)
	li  t1, RALINK_RGCTRL_BASE+0x10c
	lw  t4, 0(t1)
	j	LDO_1P8V
	nop
LDO_1P8V:
	li	t2, ~(1<<6)
	and	t4, t4, t2
	li  t1, RALINK_RGCTRL_BASE+0x10c
	sw	t4, 0(t1)
	j	DDRLDO_SOFT_START
DDRLDO_SOFT_START:
	nop
	li	t1, RALINK_RGCTRL_BASE+0x10c
	lw	t2, 0(t1)
	li	t3, 1<<16
	or	t2, t2, t3
	sw	t2, 0(t1)
	li	t3, DELAY_USEC(250*50)
LDO_DELAY:
	subu t3, t3, 1
	bnez t3, LDO_DELAY
	nop

	li  t1, RALINK_RGCTRL_BASE+0x10c
	lw  t2, 0(t1)
	li  t3, 1<<18
	or  t2, t2, t3
	sw  t2, 0(t1)

SET_RG_BUCK_FPWM:
	li	t1, RALINK_RGCTRL_BASE+0x104
	lw	t2, 0(t1)
	ori t2, t2, 1<<10
	sw	t2, 0(t1)

DDR_PAD_CFG:
	/* clean CLK PAD */
	li  t1, RALINK_RGCTRL_BASE+0x704
	lw  t2, 0(t1)
	li	t8, 0xFFFFF0F0
	and    t2, t2, t8
	/* clean CMD PAD */
	li  t1, RALINK_RGCTRL_BASE+0x70c
	lw  t3, 0(t1)
	li  t8, 0xFFFFF0F0
	and    t3, t3, t8
	/* clean DQ IPAD */
	li  t1, RALINK_RGCTRL_BASE+0x710
	lw  t4, 0(t1)
	li  t8, 0xFFFFF8FF
	and    t4, t4, t8
	/* clean DQ OPAD */
	li  t1, RALINK_RGCTRL_BASE+0x714
	lw  t5, 0(t1)
	li  t8, 0xFFFFF0F0
	and    t5, t5, t8
	/* clean DQS IPAD */
	li  t1, RALINK_RGCTRL_BASE+0x718
	lw  t6, 0(t1)
	li  t8, 0xFFFFF8FF
	and    t6, t6, t8
	/* clean DQS OPAD */
	li  t1, RALINK_RGCTRL_BASE+0x71c
	lw  t7, 0(t1)
	li  t8, 0xFFFFF0F0
	and    t7, t7, t8


	li	t1, RALINK_CHIP_REV_ID_REG
	lw	t9, 0(t1)
	srl	t9, t9, 16
	andi t9, t9, 0x1
	bnez t9, MT7628_AN_DDR1_PAD
MT7628_KN_PAD:
	li	t8, 0x00000303
	or	t2, t2, t8
	or	t3, t3, t8
	or	t5, t5, t8
	or	t7,	t7, t8
	li	t8, 0x00000000
	or	t4, t4, t8
	or	t6, t6, t8
	j	SET_PAD_CFG
MT7628_AN_DDR1_PAD:
	nop
	li	t1, RALINK_SYSCFG_REG
	lw	t1, 0(t1)
	andi	t1, t1, 0x1
	beqz t1, MT7628_AN_DDR2_PAD
	li  t8, 0x00000C0C
	or	t2, t2, t8
    li  t8, 0x00000202
	or	t3, t3, t8
    li  t8, 0x00000707
	or	t5, t5, t8
	li  t8, 0x00000C0C
	or	t7, t7, t8
	li  t8, 0x00000000
	or	t4, t4, t8
	or	t6, t6, t8
	j	SET_PAD_CFG
MT7628_AN_DDR2_PAD:
	li  t8, 0x00000C0C
	or  t2, t2, t8
	li  t8, 0x00000202
	or  t3, t3, t8
	li  t8, 0x00000404
	or  t5, t5, t8
	li  t8, 0x00000C0C
	or  t7, t7, t8
	//li  t8, 0x00000200
	li  t8, 0x00000000		//ODT off
	or  t4, t4, t8
	or  t6, t6, t8

SET_PAD_CFG:
	li  t1, RALINK_RGCTRL_BASE+0x704
	sw	t2, 0(t1)
	li  t1, RALINK_RGCTRL_BASE+0x70c
	sw	t3, 0(t1)
	li  t1, RALINK_RGCTRL_BASE+0x710
	sw	t4, 0(t1)
	li  t1, RALINK_RGCTRL_BASE+0x714
	sw	t5, 0(t1)
	li  t1, RALINK_RGCTRL_BASE+0x718
	sw	t6, 0(t1)
	li  t1, RALINK_RGCTRL_BASE+0x71c
	sw	t7, 0(t1)

	/* DDR initialization: reset pin to 0
	 */
	li t1, RALINK_RSTCTRL_REG
	lw t2, 0(t1)
	and t2, ~(0x1<<10)
	sw t2, 0(t1)
	nop

	li t0, DELAY_USEC(200+40)
	li t1, 0x1
1:
	sub t0, t0, t1
	bnez t0, 1b
	nop

	/* DDR initialization: wait til reg DDR_CFG1 bit 21 equal to 1 (ready)
	 */
#if defined (ON_BOARD_256M_DRAM_COMPONENT)
	#define RALINK_DDR_CFG0_VAL	0x2519E2E5
	#define RALINK_DDR_CFG1_VAL	0x222E2323
	#define RALINK_DDR_CFG2_VAL	0x68000C43
	#define RALINK_DDR_CFG3_VAL	0x00000452
	#define RALINK_DDR_CFG4_VAL	0x0000000A
#elif defined (ON_BOARD_512M_DRAM_COMPONENT)
	#define RALINK_DDR_CFG0_VAL	0x249AA2E5
	#define RALINK_DDR_CFG1_VAL	0x22322323
	#define RALINK_DDR_CFG2_VAL	0x68000C43
	#define RALINK_DDR_CFG3_VAL	0x00000452
	#define RALINK_DDR_CFG4_VAL	0x0000000A
#elif defined (ON_BOARD_1024M_DRAM_COMPONENT)
	#define RALINK_DDR_CFG0_VAL	0x249B42E5
	#define RALINK_DDR_CFG1_VAL	0x22362323
	#define RALINK_DDR_CFG2_VAL	0x68000C43
	#define RALINK_DDR_CFG3_VAL	0x00000452
	#define RALINK_DDR_CFG4_VAL	0x0000000A
#elif defined (ON_BOARD_2048M_DRAM_COMPONENT)
	#define RALINK_DDR_CFG0_VAL	0x249CE2E5
	#define RALINK_DDR_CFG1_VAL	0x223A2323
	#define RALINK_DDR_CFG2_VAL	0x68000C43
	#define RALINK_DDR_CFG3_VAL	0x00000452
	#define RALINK_DDR_CFG4_VAL	0x0000000A
#else
#error	"DRAM Component not defined, please define in include/configs/skw93.h"
#endif
DDR_READY:
	li t1, RALINK_DDR_CFG1
	lw t0, 0(t1)
	nop
	and t2, t0, (1<<21)
	beqz t2, DDR_READY
	nop

	li	t7, RALINK_DDR_CFG2_VAL
	nop
	la	t0, RALINK_DDR_CFG2
	sw	t7, 0(t0)
	nop
	li	t8, RALINK_DDR_CFG3_VAL
	nop
	la	t0, RALINK_DDR_CFG3
	sw	t8, 0(t0)
	nop
	li	t9, RALINK_DDR_CFG4_VAL
	nop
	la	t0, RALINK_DDR_CFG4
	sw	t9, 0(t0)
	nop

	li  t1, RALINK_CHIP_REV_ID_REG
	lw  t9, 0(t1)
	srl t9, t9, 16
	andi t9, t9, 0x1
	beqz t9, 1f
	nop
	la      t0, RALINK_DDR_CFG8
	li      t1, 0x00008282
	sw      t1, 0(t0)
	la      t0, RALINK_DDR_CFG9
	li      t1, 0x00008383
	sw      t1, 0(t0)

#define DDR_CFG0_MIPSREG		s7
#define DDR_CFG1_MIPSREG		s8
1:
	li	DDR_CFG0_MIPSREG, RALINK_DDR_CFG0_VAL
	nop
	la      t0, RALINK_DDR_CFG0
	sw	DDR_CFG0_MIPSREG, 0(t0)
	nop
	li	DDR_CFG1_MIPSREG, RALINK_DDR_CFG1_VAL
	nop
	la	t0, RALINK_DDR_CFG1
	sw	DDR_CFG1_MIPSREG, 0(t0)
	nop

tRFCinit:

#if defined(RALINK_DDR_POWERSAVE)
	/* DDR: enable self auto refresh for power saving
	 * enable it by default for both RAM and ROM version (for CoC)
	 */
	li	t0, RALINK_SYSCFG1_REG
	lw	t1, 0(t0)
	nop
	and	t1, 0xff000000
	or	t1, 0x01
	sw	t1, 0(t0)
	nop
	li	t0, RALINK_SYSCFG_REG
	lw	t1, 0(t0)
	nop
	or	t1, 0x10
	sw	t1, 0(t0)
	nop
#endif

#if defined (CONFIG_DDR_CAL)
	/* === fill I$ === */
fill_icache:
	la 	$14, dram_cali		# Get a KSeg0 address for cacheops
	li	$13, 0xDFFFFFFF
	and	$13, $13, $14
	li	$15, 512
	li	$11, 32
# Fill Cache Op
1:	cache	0x14, 0($13)
	add	$15, -1			# Decrement set counter
	nop
	bne	$15, $0, 1b
	add	$13, $11		# Get next line address
	nop
#endif

#endif /* !defined(CONFIG_SYS_RAMBOOT) */

	/* function return */
	jr	ra
	nop
