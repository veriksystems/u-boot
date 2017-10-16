/*
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <command.h>
#include <spi.h>
#include <malloc.h>
#include <asm/mt7628.h>

/*-----------------------------------------------------------------------
 * Definitions
 */
#undef DEBUG_SPI
#ifdef DEBUG_SPI
#define PRINTD(fmt,args...)	printf (fmt ,##args)
#else
#define PRINTD(fmt,args...)
#endif

#define ENABLE	1
#define DISABLE	0
#define spi_busy_loop 3000
#define max_ee_busy_loop 500
#define IS_BUSY		(RALINK_REG(RALINK_SPISTAT_REG) & 0x01)
#define SPIC_BUF_LEN	32

struct mt7628_spi_slave {
	struct spi_slave slave;
	u8 cmdlen;
	u8 cmd[4];
};

/*
 * TODOs:
 * 	+ support mmap with fs_page_sel
 * 	+ 4-byte address mode
 * 	+ support cs, speed, duplex, cpha, cpol
 * 	+ support serial mode: dual, quad
 */

/*=====================================================================*/
/*                         Private Functions                           */
/*=====================================================================*/
static inline struct mt7628_spi_slave *to_mt7628_spi(struct spi_slave *slave)
{
	return container_of(slave, struct mt7628_spi_slave, slave);
}

static inline void spi_flash_addr_inc(u8 *cmd, u8 step)
{
	u32 addr = cmd[3] | (cmd[2] << 8) | (cmd[1] << 16);
	addr += step;
	cmd[1] = addr >> 16;
	cmd[2] = addr >> 8;
	cmd[3] = addr >> 0;
}

static int bbu_spic_busy_wait(void)
{
	int n = 100000;
	do {
		if ((ra_inl(SPI_REG_CTL) & SPI_CTL_BUSY) == 0)
			return 0;
		udelay(1);
	} while (--n > 0);

	PRINTD("%s: fail \n", __func__);
	return -1;
}

static u8 bbu_spic_write(u8 const *cmd, u8 cmdlen, u8 const *buf, u8 len, u8 more_buf_mode)
{
	int i;
	u32 reg, val;
	u8 nbytes;

	/* command */
	val = 0;
	if (more_buf_mode) {
		if (cmdlen > 0) { val  = cmd[0] << 24; /* cmd     */
		if (cmdlen > 1) { val |= cmd[1] << 16; /* addr[2] */
		if (cmdlen > 2) { val |= cmd[2] <<  8; /* addr[1] */
		if (cmdlen > 3) { val |= cmd[3] ;      /* addr[0] */ }}}}
	}
	else {
		if (cmdlen > 0) { val  = cmd[0];       /* cmd     */
		if (cmdlen > 1) { val |= cmd[1] << 24; /* addr[2] */
		if (cmdlen > 2) { val |= cmd[2] << 16; /* addr[1] */
		if (cmdlen > 3) { val |= cmd[3] <<  8; /* addr[0] */ }}}}
	}
	ra_outl(SPI_REG_OPCODE, val);

	reg = SPI_REG_DATA0;
	nbytes = (u8)(len < SPIC_BUF_LEN ? len : SPIC_BUF_LEN);
	for (i = 0; i < nbytes; i++) {
		u8 r = i & 0x3; // Optimize for (i % 4)
		if (r == 0) {
			reg = SPI_REG_DATA0 + i;
			ra_outl(reg, 0);
		}
		ra_or(reg, buf[i] << (r << 3)); // Optimize for (r * 8)
	}

	/* set tx count */
	if (more_buf_mode) {
		ra_and(SPI_REG_MOREBUF, ~(SPI_MBCTL_CMD_MASK | SPI_MBCTL_TXCNT_MASK));
		ra_or(SPI_REG_MOREBUF, (cmdlen << 3) << 24);
		ra_or(SPI_REG_MOREBUF, nbytes << 3);
	}
	else {
		ra_and(SPI_REG_CTL, ~SPI_CTL_TXCNT_MASK);
		ra_or(SPI_REG_CTL, (cmdlen + nbytes) & 0xF);
	}

	return nbytes;
}

static inline void bbu_spi_set_rxcnt(u32 len, u8 more_buf_mode)
{
	u8 n_rx = (u8)(len < SPIC_BUF_LEN ? len : SPIC_BUF_LEN);
	if (more_buf_mode) {
		ra_and(SPI_REG_MOREBUF, ~SPI_MBCTL_RXCNT_MASK);
		ra_or(SPI_REG_MOREBUF, (n_rx << 3) << 12);
	}
	else {
		ra_and(SPI_REG_CTL, ~SPI_CTL_RXCNT_MASK);
		ra_or(SPI_REG_CTL, n_rx << 4);
	}
}

static u8 bbu_spic_read(u8 *buf, int len)
{
	int i;
	u32 val = 0;
	u8 nbytes = (u8)(len < SPIC_BUF_LEN ? len : SPIC_BUF_LEN);

	for (i = 0; i < nbytes; i++) {
		u8 r = i & 0x3; // Optimize for (i % 4)
		if (r == 0)
			val = ra_inl(SPI_REG_DATA0 + i);
		buf[i] = (u8)(val >> (r << 3)); // Optimize for (r * 8)
	}

	return nbytes;
}

/*=====================================================================*/
/*                         Public Functions                            */
/*=====================================================================*/

/*-----------------------------------------------------------------------
 * Initialization
 */
void spi_init(void)
{
	/* step 0. disable more_buf_mode */
	ra_and(SPI_REG_MASTER, ~(1 << 2));
	/* setup spi clock to hclk/3 */
	ra_and(SPI_REG_MASTER, ~0x0FFF0000);
	ra_or(SPI_REG_MASTER, 1 << 16);
}

void inline spi_cs_activate(struct spi_slave *slave)
{
}

void inline spi_cs_deactivate(struct spi_slave *slave)
{
}

struct spi_slave *spi_setup_slave(unsigned int bus, unsigned int cs,
		unsigned int max_hz, unsigned int mode)
{
	struct mt7628_spi_slave *ms;

	ms = spi_alloc_slave(struct mt7628_spi_slave, bus, cs);
	if (!ms)
		return NULL;

	ms->cmdlen = 0;
	ms->slave.max_write_size = SPIC_BUF_LEN;
	ms->slave.op_mode_rx = SPI_OPM_RX_AS;

	return &ms->slave;
}

void spi_free_slave(struct spi_slave *slave)
{
	struct mt7628_spi_slave *ms = to_mt7628_spi(slave);

	free(ms);
}

int spi_claim_bus(struct spi_slave *slave)
{
	return 0;
}

void spi_release_bus(struct spi_slave *slave)
{
	/* Nothing to do */
}

/*-----------------------------------------------------------------------
 * SPI transfer
 *
 */
int spi_xfer(struct spi_slave *slave, unsigned int bitlen,
		const void *dout, void *din, unsigned long flags)
{
	struct mt7628_spi_slave *ms = to_mt7628_spi(slave);
	const u8	*txd = dout;
	u8		*rxd = din;
	int 		n_tx = txd ? (bitlen >> 3) : 0; // Optimize for (bitlen / 8)
	int 		n_rx = rxd ? (bitlen >> 3) : 0; // Optimize for (bitlen / 8)
	u8		more_buf_mode;

	PRINTD("spi_xfer: slave %u:%u dout %08X din %08X bitlen %u\n",
		slave->bus, slave->cs, txd ? *(uint *)txd : 0, rxd ? *(uint *)rxd : 0, bitlen);

	/* save command from XFER_BEGIN message */
	if (flags & SPI_XFER_BEGIN) {
		spi_cs_activate(slave);

		if (n_tx > 0) { ms->cmd[0] = txd[0];
		if (n_tx > 1) { ms->cmd[1] = txd[1];
		if (n_tx > 2) { ms->cmd[2] = txd[2];
		if (n_tx > 3) { ms->cmd[3] = txd[3]; }}}}
		ms->cmdlen = n_tx;

		if (flags & SPI_XFER_END)
			spi_xfer(slave, 0, NULL, NULL, SPI_XFER_END);

		return(0);
	}

	/* step 0. enable more byte mode for data only */
	more_buf_mode = (ms->cmdlen == 4 && bitlen > 32);
	if (more_buf_mode)
		ra_or(SPI_REG_MASTER, (1 << 2));

	/* step 1. set opcode & address */
	/* step 2. write DI/DO data #0..#7 */
	bbu_spic_write(ms->cmd, ms->cmdlen, txd, n_tx, more_buf_mode);

	/* step 3. set rx (miso_bit_cnt) and tx (mosi_bit_cnt) bit count */
	bbu_spi_set_rxcnt(n_rx, more_buf_mode);

	PRINTD("1. cmdlen=%d opcode=%08lx n_tx=%d n_rx=%d %s=%08lx\n",
			ms->cmdlen, ra_inl(SPI_REG_OPCODE),
			n_tx, n_rx, more_buf_mode ? "mb" : "ctl",
			ra_inl(more_buf_mode ? SPI_REG_MOREBUF : SPI_REG_CTL));

	/* step 4. kick */
	ra_or(SPI_REG_CTL, SPI_CTL_START);

	/* step 5. wait spi_master_busy */
	bbu_spic_busy_wait();

	/* step 6. read DI/DO data #0 */
	while (n_rx > 0) {
		u8 nbytes = bbu_spic_read(rxd, n_rx);
		if (nbytes == n_rx)
			break;
		n_rx -= nbytes;
		rxd += nbytes;
		/* reload read command */
		spi_flash_addr_inc(ms->cmd, nbytes);
		bbu_spic_write(ms->cmd, ms->cmdlen, NULL, 0, more_buf_mode);
		bbu_spi_set_rxcnt(n_rx, more_buf_mode);
		PRINTD("1. cmdlen=%d opcode=%08lx n_tx=%d n_rx=%d %s=%08lx\n",
				ms->cmdlen, ra_inl(SPI_REG_OPCODE),
				n_tx, n_rx, more_buf_mode ? "mb" : "ctl",
				ra_inl(more_buf_mode ? SPI_REG_MOREBUF : SPI_REG_CTL));
		ra_or(SPI_REG_CTL, SPI_CTL_START);
		bbu_spic_busy_wait();
	}

	/* step 7. disable more byte mode */
	ra_and(SPI_REG_MASTER, ~(1 << 2));

	if (flags & SPI_XFER_END) {
		spi_cs_deactivate(slave);
		ms->cmdlen = 0;
	}

	return(0);
}
