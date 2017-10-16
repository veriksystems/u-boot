/*
 * SPDX-License-Identifier:	GPL-2.0+
 *
 */

#include <common.h>
#include <asm/mipsregs.h>
#include <asm/mt7628.h>

#define SOFTRES_REG (RALINK_SYSCTL_BASE + 0x0034)
#define GORESET		(0x01)

void _machine_restart(void)
{
	*(volatile unsigned int*)(SOFTRES_REG) = GORESET;
}
