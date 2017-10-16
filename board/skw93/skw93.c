/*
 * SPDX-License-Identifier:	GPL-2.0+
 *
 */

#undef DEBUG
#include <common.h>
#include <command.h>
#include <asm/mipsregs.h>
#include <asm/cacheops.h>
#include <asm/io.h>
#include <netdev.h>
#include <asm/mt7628.h>

#if defined (CONFIG_DDR_CAL)
void dram_cali(void);
#endif

phys_size_t initdram(int board_type)
{

#if defined (CONFIG_DDR_CAL)
	/* Sdram is setup by assembler code */
	void (*ptr)(void);
	ptr = dram_cali;
	ptr = (void*)((u32)ptr & ~(1<<29));
	(*ptr)();
#endif

	/* If memory could be changed, we should return the true value here */
	return MEM_SIZE*1024*1024;
}

int checkboard(void)
{
	u32 proc_id;
	u32 config1;

	proc_id = read_c0_prid();
	printf("Board: SKW93 -M mips CPU: ");
	switch (proc_id) {
	case 0x00018000:
		printf("4Kc");
		break;
	case 0x00018400:
		printf("4KEcR1");
		break;
	case 0x00019000:
		printf("4KEc");
		break;
	case 0x00019300:
		config1 = read_c0_config1();
		if (config1 & 1)
			printf("24Kf");
		else
			printf("24Kc");
		break;
	case 0x00019500:
		printf("34Kf");
		break;
	case 0x00019655:
		printf("24KEc");
		break;
	case 0x00000400:
		printf("R4000");
		break;
	case 0x00018100:
		config1 = read_c0_config1();
		if (config1 & 1)
			printf("5Kf");
		else
			printf("5Kc");
		break;
	case 0x000182a0:
		printf("20Kc");
		break;

	default:
		printf("unknown");
	}
	printf(" proc_id=0x%x\n", proc_id);

	return 0;
}

__weak int misc_init_r(void)
{

	set_io_port_base(0);

	/* reset MIPS now also reset Andes */
	RALINK_REG(RALINK_SYSCTL_BASE+0x38) |= 0x200;

	/* Pin Mux gpio mode configure */
	/* GPIO1_MODE Register
	 * PWM1_MODE [31:30] = 1 - GPIO (nc)
	 * PWM0_MODE [29:28] = 1 - GPIO (nc)
	 * UART2_MODE[27:26] = 1 - GPIO (GPIO#20,#21 overriden by LAN_PORT2_TX+/-)
	 * UART1_MODE[25:24] = 0 - UART1
	 * I2C_MODE  [21:20] = 0 - I2C
	 * REFLCK_MODE[  18] = 1 - GPIO (nc)
	 * PERST_MODE[   16] = 0 - PCIe reset
	 * WDT_MODE  [   14] = 1 - GPIO (GPIO#38 as WPS/RST button)
	 * SPI_MODE  [   12] = 0 - SPI  (Internal)
	 * SD_MODE   [11:10] = 0 - SDXC (overriden by LAN_PORT3/4_TX/RX+/-)
	 * UART0_MODE[  9:8] = 0 - UART0 as debug port
	 * I2S_MODE  [  7:6] = 0 - I2S
	 * SPI_CS1_MODE[5:4] = 1 - GPIO (nc)
	 * SPIS_MODE [  3:2] = 1 - GPIO (nc)
	 * GPIO_MODE [  1:0] = 0 - GPIO (GPIO#11)
	 */
	RALINK_REG(RALINK_SYSCTL_BASE + 0x60) = (1<<30|1<<28|1<<26|1<<18|1<<14|1<<4|1<<2);

	/* GPIO2_MODE Register
	 * P4_LED_AN_MODE[11:10] = 00 - P4_LED
	 * P3_LED_AN_MODE[ 9:8 ] = 00 - P3_LED
	 * P2_LED_AN_MODE[ 7:6 ] = 00 - P2_LED
	 * P1_LED_AN_MODE[ 5:4 ] = 00 - P1_LED
	 * P0_LED_AN_MODE[ 3:2 ] = 00 - P0_LED
	 * WLED_AN_MODE  [ 1:0 ] = 00 - WLED
	 */
	RALINK_REG(RALINK_SYSCTL_BASE + 0x64) = 0;

	/* AGPIO configure
	 * WLED_OD_EN	   [    8] =    1 - (Open-Drain)
	 */
	RALINK_REG(RALINK_SYSCTL_BASE + 0x3c) |= (1<<8);

	/* GPIO 	Direction		Default
	 * GPIO[11] =	In  - Unsigned
	 * --------------------------------------
	 * GPIO[38] =	In  - WPS/RST Button
	 */
	/* GPIO direction */
	RALINK_REG(RALINK_PIO_BASE) 	  = 0;
	RALINK_REG(RALINK_PIO_BASE + 0x4) = 0;
	/* Default output */

	return 0;
}

#if defined (CONFIG_MT7628_ETH)
extern int rt2880_eth_initialize(bd_t *bis);
int board_eth_init(bd_t *bis)
{
	return rt2880_eth_initialize(bis);
}
#endif

#if defined (CONFIG_DDR_CAL)

#define CPU_FRAC_DIV		0x1
#define NUM_OF_CACHELINE	128
#define MIN_START		6
#define MIN_FINE_START		0xF
#define MAX_START 		7
#define MAX_FINE_START		0x0

#define DRAM_BUTTOM (MEM_SIZE << 20)

#define pref_op(hint,addr)						\
	__asm__ __volatile__(						\
			"       .set    push                                    \n"	\
			"       .set    noreorder                               \n"	\
			"       pref   %0, %1                                  \n"	\
			"       .set    pop                                     \n"	\
			:								\
			: "i" (hint), "R" (*(unsigned char *)(addr)))

#define cache_op(op,addr)						\
	__asm__ __volatile__(						\
			"       .set    push                                    \n"	\
			"       .set    noreorder                               \n"	\
			"       .set    mips3\n\t                               \n"	\
			"       cache   %0, %1                                  \n"	\
			"       .set    pop                                     \n"	\
			:								\
			: "i" (op), "R" (*(unsigned char *)(addr)))

static inline void cal_memcpy(void* src, void* dst, unsigned int size)
{
	int i;
	unsigned char* psrc = (unsigned char*)src, *pdst=(unsigned char*)dst;
	for (i = 0; i < size; i++, psrc++, pdst++)
		(*pdst) = (*psrc);
	return;
}
static inline void cal_memset(void* src, unsigned char pat, unsigned int size)
{
	int i;
	unsigned char* psrc = (unsigned char*)src;
	for (i = 0; i < size; i++, psrc++)
		(*psrc) = pat;
	return;
}


static void inline cal_invalidate_dcache_range(ulong start_addr, ulong stop)
{
	unsigned long lsize = CONFIG_SYS_CACHELINE_SIZE;
	unsigned long addr = start_addr & ~(lsize - 1);
	unsigned long aend = (stop - 1) & ~(lsize - 1);

	while (1) {
		cache_op(HIT_INVALIDATE_D, addr);
		if (addr == aend)
			break;
		addr += lsize;
	}
}

static void inline cal_patgen(unsigned long* start_addr, unsigned int size, unsigned bias)
{
	int i = 0;
	for (i = 0; i < size; i++)
		start_addr[i] = ((ulong)start_addr+i+bias);

	return;
}

void dram_cali(void)
{
	unsigned int * nc_addr = 0xA0000000+DRAM_BUTTOM-0x0400;
	unsigned int * c_addr = 0x80000000+DRAM_BUTTOM-0x0400;
	unsigned int min_coarse_dqs[2];
	unsigned int max_coarse_dqs[2];
	unsigned int min_fine_dqs[2];
	unsigned int max_fine_dqs[2];
	unsigned int coarse_dqs[2];
	unsigned int fine_dqs[2];
	unsigned int min_dqs[2];
	unsigned int max_dqs[2];
	int reg = 0, ddr_cfg2_reg = 0;
	int ret = 0;
	int flag = 0, min_failed_pos[2], max_failed_pos[2], min_fine_failed_pos[2], max_fine_failed_pos[2];
	int i,j, k;
	int dqs = 0;
	unsigned int min_coarse_dqs_bnd, min_fine_dqs_bnd, coarse_dqs_dll, fine_dqs_dll;
#if (NUM_OF_CACHELINE > 40)
#else
	unsigned int cache_pat[8*40];
#endif
	u32 value, test_count = 0;;
	u32 fdiv = 0, step = 0, frac = 0;

	value = RALINK_REG(RALINK_DYN_CFG0_REG);
	fdiv = (unsigned long)((value>>8)&0x0F);
	if ((CPU_FRAC_DIV < 1) || (CPU_FRAC_DIV > 10))
		frac = (unsigned long)(value&0x0F);
	else
		frac = CPU_FRAC_DIV;
	i = 0;

	while(frac < fdiv) {
		value = RALINK_REG(RALINK_DYN_CFG0_REG);
		fdiv = ((value>>8)&0x0F);
		fdiv--;
		value &= ~(0x0F<<8);
		value |= (fdiv<<8);
		RALINK_REG(RALINK_DYN_CFG0_REG) = value;
		udelay(500);
		i++;
		value = RALINK_REG(RALINK_DYN_CFG0_REG);
		fdiv = ((value>>8)&0x0F);
	}
#if (NUM_OF_CACHELINE > 40)
#else
	cal_memcpy(cache_pat, dram_patterns, 32*6);
	cal_memcpy(cache_pat+32*6, line_toggle_pattern, 32);
	cal_memcpy(cache_pat+32*6+32, pattern_ken, 32*13);
#endif

	RALINK_REG(RALINK_MEMCTRL_BASE+0x10) &= ~(0x1<<4);
	ddr_cfg2_reg = RALINK_REG(RALINK_MEMCTRL_BASE+0x48);
	RALINK_REG(RALINK_MEMCTRL_BASE+0x48)&=(~((0x3<<28)|(0x3<<26)));

TEST_LOOP:
	min_coarse_dqs[0] = MIN_START;
	min_coarse_dqs[1] = MIN_START;
	min_fine_dqs[0] = MIN_FINE_START;
	min_fine_dqs[1] = MIN_FINE_START;
	max_coarse_dqs[0] = MAX_START;
	max_coarse_dqs[1] = MAX_START;
	max_fine_dqs[0] = MAX_FINE_START;
	max_fine_dqs[1] = MAX_FINE_START;
	min_failed_pos[0] = 0xFF;
	min_fine_failed_pos[0] = 0;
	min_failed_pos[1] = 0xFF;
	min_fine_failed_pos[1] = 0;
	max_failed_pos[0] = 0xFF;
	max_fine_failed_pos[0] = 0;
	max_failed_pos[1] = 0xFF;
	max_fine_failed_pos[1] = 0;
	dqs = 0;

	// Add by KP, DQS MIN boundary
	reg = RALINK_REG(RALINK_MEMCTRL_BASE+0x20);
	coarse_dqs_dll = (reg & 0xF00) >> 8;
	fine_dqs_dll = (reg & 0xF0) >> 4;
	if (coarse_dqs_dll<=8)
		min_coarse_dqs_bnd = 8 - coarse_dqs_dll;
	else
		min_coarse_dqs_bnd = 0;

	if (fine_dqs_dll<=8)
		min_fine_dqs_bnd = 8 - fine_dqs_dll;
	else
		min_fine_dqs_bnd = 0;
	// DQS MIN boundary

DQS_CAL:
	flag = 0;
	j = 0;

	for (k = 0; k < 2; k ++)
	{
		unsigned int test_dqs, failed_pos = 0;
		if (k == 0)
			test_dqs = MAX_START;
		else
			test_dqs = MAX_FINE_START;
		flag = 0;
		do
		{
			flag = 0;
			for (nc_addr = 0xA0000000; nc_addr < (0xA0000000+DRAM_BUTTOM-NUM_OF_CACHELINE*32); nc_addr+=((DRAM_BUTTOM>>6)+1*0x400))
			{
				RALINK_REG(RALINK_MEMCTRL_BASE+0x64) = 0x00007474;
				wmb();
				c_addr = (unsigned int*)((ulong)nc_addr & 0xDFFFFFFF);
				cal_memset(((unsigned char*)c_addr), 0x1F, NUM_OF_CACHELINE*32);
#if (NUM_OF_CACHELINE > 40)
				cal_patgen(nc_addr, NUM_OF_CACHELINE*8, 3);
#else
				cal_memcpy(((unsigned char*)nc_addr), ((unsigned char*)cache_pat), NUM_OF_CACHELINE*32);
#endif

				if (dqs > 0)
					RALINK_REG(RALINK_MEMCTRL_BASE+0x64) = 0x00000074|(((k==1) ? max_coarse_dqs[dqs] : test_dqs)<<12)|(((k==0) ? 0xF : test_dqs)<<8);
				else
					RALINK_REG(RALINK_MEMCTRL_BASE+0x64) = 0x00007400|(((k==1) ? max_coarse_dqs[dqs] : test_dqs)<<4)|(((k==0) ? 0xF : test_dqs)<<0);
				wmb();

				cal_invalidate_dcache_range(((unsigned char*)c_addr), ((unsigned char*)c_addr)+NUM_OF_CACHELINE*32);
				wmb();
				for (i = 0; i < NUM_OF_CACHELINE*8; i ++)
				{
					if (i % 8 ==0)
						pref_op(0, &c_addr[i]);
				}
				for (i = 0; i < NUM_OF_CACHELINE*8; i ++)
				{
#if (NUM_OF_CACHELINE > 40)
					if (c_addr[i] != (ulong)nc_addr+i+3)
#else
						if (c_addr[i] != cache_pat[i])
#endif
						{
							flag = -1;
							failed_pos = i;
							goto MAX_FAILED;
						}
				}
			}
MAX_FAILED:
			if (flag==-1)
			{
				break;
			}
			else
				test_dqs++;
		}while(test_dqs<=0xF);

		if (k==0)
		{
			max_coarse_dqs[dqs] = test_dqs;
			max_failed_pos[dqs] = failed_pos;
		}
		else
		{
			test_dqs--;

			if (test_dqs==MAX_FINE_START-1)
			{
				max_coarse_dqs[dqs]--;
				max_fine_dqs[dqs] = 0xF;
			}
			else
			{
				max_fine_dqs[dqs] = test_dqs;
			}
			max_fine_failed_pos[dqs] = failed_pos;
		}
	}

	for (k = 0; k < 2; k ++)
	{
		unsigned int test_dqs, failed_pos = 0;
		if (k == 0)
			test_dqs = MIN_START;
		else
			test_dqs = MIN_FINE_START;
		flag = 0;
		do
		{
			for (nc_addr = 0xA0000000; nc_addr < (0xA0000000+DRAM_BUTTOM-NUM_OF_CACHELINE*32); (nc_addr+=(DRAM_BUTTOM>>6)+1*0x480))
			{
				RALINK_REG(RALINK_MEMCTRL_BASE+0x64) = 0x00007474;
				wmb();
				c_addr = (unsigned int*)((ulong)nc_addr & 0xDFFFFFFF);
				RALINK_REG(RALINK_MEMCTRL_BASE+0x64) = 0x00007474;
				wmb();
				cal_memset(((unsigned char*)c_addr), 0x1F, NUM_OF_CACHELINE*32);
#if (NUM_OF_CACHELINE > 40)
				cal_patgen(nc_addr, NUM_OF_CACHELINE*8, 1);
#else
				cal_memcpy(((unsigned char*)nc_addr), ((unsigned char*)cache_pat), NUM_OF_CACHELINE*32);
#endif
				if (dqs > 0)
					RALINK_REG(RALINK_MEMCTRL_BASE+0x64) = 0x00000074|(((k==1) ? min_coarse_dqs[dqs] : test_dqs)<<12)|(((k==0) ? 0x0 : test_dqs)<<8);
				else
					RALINK_REG(RALINK_MEMCTRL_BASE+0x64) = 0x00007400|(((k==1) ? min_coarse_dqs[dqs] : test_dqs)<<4)|(((k==0) ? 0x0 : test_dqs)<<0);
				wmb();
				cal_invalidate_dcache_range(((unsigned char*)c_addr), ((unsigned char*)c_addr)+NUM_OF_CACHELINE*32);
				wmb();
				for (i = 0; i < NUM_OF_CACHELINE*8; i ++)
				{
					if (i % 8 ==0)
						pref_op(0, &c_addr[i]);
				}
				for (i = 0; i < NUM_OF_CACHELINE*8; i ++)
				{
#if (NUM_OF_CACHELINE > 40)
					if (c_addr[i] != (ulong)nc_addr+i+1)
#else
						if (c_addr[i] != cache_pat[i])
#endif
						{
							flag = -1;
							failed_pos = i;
							goto MIN_FAILED;
						}
				}
			}

MIN_FAILED:

			if (k==0)
			{
				if ((flag==-1)||(test_dqs==min_coarse_dqs_bnd))
				{
					break;
				}
				else
					test_dqs--;

				if (test_dqs < min_coarse_dqs_bnd)
					break;
			}
			else
			{
				if (flag==-1)
				{
					test_dqs++;
					break;
				}
				else if (test_dqs==min_fine_dqs_bnd)
				{
					break;
				}
				else
				{
					test_dqs--;
				}

				if (test_dqs < min_fine_dqs_bnd)
					break;

			}
		}while(test_dqs>=0);

		if (k==0)
		{
			min_coarse_dqs[dqs] = test_dqs;
			min_failed_pos[dqs] = failed_pos;
		}
		else
		{
			if (test_dqs==MIN_FINE_START+1)
			{
				min_coarse_dqs[dqs]++;
				min_fine_dqs[dqs] = 0x0;
			}
			else
			{
				min_fine_dqs[dqs] = test_dqs;
			}
			min_fine_failed_pos[dqs] = failed_pos;
		}
	}

	if (dqs==0)
	{
		dqs = 1;
		goto DQS_CAL;
	}

	for (i=0 ; i < 2; i++)
	{
		unsigned int temp;
		coarse_dqs[i] = (max_coarse_dqs[i] + min_coarse_dqs[i])>>1;
		temp = (((max_coarse_dqs[i] + min_coarse_dqs[i])%2)*4)  +  ((max_fine_dqs[i] + min_fine_dqs[i])>>1);
		if (temp >= 0x10)
		{
			coarse_dqs[i] ++;
			fine_dqs[i] = (temp-0x10) +0x8;
		}
		else
		{
			fine_dqs[i] = temp;
		}
#if (MAX_TEST_LOOP > 1)
		min_statistic[i][min_coarse_dqs[i]][min_fine_dqs[i]]++;
		max_statistic[i][max_coarse_dqs[i]][max_fine_dqs[i]]++;
		center_statistic[i][coarse_dqs[i]][fine_dqs[i]]++;
#endif
	}
	reg = (coarse_dqs[1]<<12)|(fine_dqs[1]<<8)|(coarse_dqs[0]<<4)|fine_dqs[0];

	RALINK_REG(RALINK_MEMCTRL_BASE+0x10) &= ~(0x1<<4);
	RALINK_REG(RALINK_MEMCTRL_BASE+0x64) = reg;
	RALINK_REG(RALINK_MEMCTRL_BASE+0x48) = ddr_cfg2_reg;
	RALINK_REG(RALINK_MEMCTRL_BASE+0x10) |= (0x1<<4);

	test_count++;


FINAL:
	for (j = 0; j < 2; j++)
		debug("[%02X%02X%02X%02X]",min_coarse_dqs[j],min_fine_dqs[j], max_coarse_dqs[j],max_fine_dqs[j]);
	debug("\nDDR Calibration DQS reg = %08X\n",reg);

	return ;
}

#endif /* #if defined (CONFIG_DDR_CAL) */
