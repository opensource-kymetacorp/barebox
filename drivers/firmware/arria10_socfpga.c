/*
 *
 * Copyright (C) 2012 Altera Corporation <www.altera.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  - Neither the name of the Altera Corporation nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL ALTERA CORPORATION BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <firmware.h>
#include <command.h>
#include <common.h>
#include <malloc.h>
#include <clock.h>
#include <fcntl.h>
#include <init.h>
#include <io.h>
#include <mach/arria10-system-manager.h>
#include <mach/arria10-reset-manager.h>
#include <mach/arria10-regs.h>
#include <mach/arria10-sdram.h>

#define FPGAMGRREGS_DCLKCNT		0x8
#define FPGAMGRREGS_DCLKSTAT		0xc

#define FPGAMGRREGS_GPO			0x10
#define FPGAMGRREGS_GPI			0x14

#define FPGAMGRREGS_MISCI		0x18

#define FPGAMGRREGS_EMR_DATA0		0x30
#define FPGAMGRREGS_EMR_DATA1		0x34
#define FPGAMGRREGS_EMR_DATA2		0x38
#define FPGAMGRREGS_EMR_DATA3		0x3C
#define FPGAMGRREGS_EMR_DATA4		0x40
#define FPGAMGRREGS_EMR_DATA5		0x44
#define FPGAMGRREGS_EMR_VALID		0x48
#define FPGAMGRREGS_EMR_EN		0x4C

#define FPGAMGRREGS_JTAG_CONFIG		0x50
#define FPGAMGRREGS_JTAG_STATUS		0x54
#define FPGAMGRREGS_JTAG_KICK		0x58
#define FPGAMGRREGS_JTAG_W		0x60
#define FPGAMGRREGS_JTAG_R		0x64

#define FPGAMGRREGS_IMGCFG_CTRL_00		0x70
#define FPGAMGRREGS_S2FCONDONE_OE_MASK		0x1000000
#define FPGAMGRREGS_S2FNSTATUS_OE_MASK		0x10000
#define FPGAMGRREGS_S2FNCONFIG_MASK		0x100
#define FPGAMGRREGS_S2FNENABLE_NCNDN_LSB	2
#define FPGAMGRREGS_S2FNENABLE_NSTAT_LSB	1
#define FPGAMGRREGS_S2FNENABLE_NCFG_MASK	0x1

#define FPGAMGRREGS_IMGCFG_CTRL_01		0x74
#define FPGAMGRREGS_S2FNCE_MASK			0x1000000
#define FPGAMGRREGS_S2FPR_REQ_MASK		0x10000
#define FPGAMGRREGS_S2FNENABLE_MASK		0x1

#define FPGAMGRREGS_IMGCFG_CTRL_02		0x78
#define FPGAMGRREGS_ENCFGCTL_MASK		0x1
#define FPGAMGRREGS_CFGWDTH_MASK		0x01000000
#define FPGAMGRREGS_CDRATIO_LSB			16

#define FPGAMGRREGS_IMGCFG_STAT			0x80
#define FPGAMGRREGS_MSEL_MASK			0x00070000
#define FPGAMGRREGS_MSEL_LSB			16
#define FPGAMGRREGS_USERMODE_MASK		0x4
#define FPGAMGRREGS_CONDONE_MASK		0x40
#define FPGAMGRREGS_CONDONE_OE_MASK		0x80
#define FPGAMGRREGS_NCONFIG_MASK		0x1000
#define FPGAMGRREGS_NSTATUS_MASK		0x10

#define FPGAMGRREGS_INTR_MSK_STAT		0x84
#define FPGAMGRREGS_INTR_MSK			0x88
#define FPGAMGRREGS_INTR_POL			0x8C


/* FPGA Mode */
#define FPGAMGRREGS_MODE_FPGAOFF	0x0
#define FPGAMGRREGS_MODE_RESETPHASE	0x1
#define FPGAMGRREGS_MODE_CFGPHASE	0x2
#define FPGAMGRREGS_MODE_INITPHASE	0x3
#define FPGAMGRREGS_MODE_USERMODE	0x4
#define FPGAMGRREGS_MODE_UNKNOWN	0x5

/* FPGA CD Ratio Value */
#define CDRATIO_x1	0x0
#define CDRATIO_x2	0x1
#define CDRATIO_x4	0x2
#define CDRATIO_x8	0x3

struct fpgamgr {
	struct firmware_handler fh;
	struct device_d dev;
	void __iomem *regs;
	void __iomem *regs_data;
	int programmed;
};

/* Get the FPGA mode */
static uint32_t fpgamgr_get_mode(struct fpgamgr *mgr)
{
	return readl(mgr->regs + FPGAMGRREGS_IMGCFG_STAT) & FPGAMGRREGS_USERMODE_MASK;
}

static int fpgamgr_dclkcnt_set(struct fpgamgr *mgr, unsigned long cnt)
{
	uint64_t start;

	/* clear any existing done status */
	if (readl(mgr->regs + FPGAMGRREGS_DCLKSTAT))
		writel(0x1, mgr->regs + FPGAMGRREGS_DCLKSTAT);

	writel(cnt, mgr->regs + FPGAMGRREGS_DCLKCNT);

	/* wait till the dclkcnt done */
	start = get_time_ns();
	while (1) {
		if (readl(mgr->regs + FPGAMGRREGS_DCLKSTAT)) {
			writel(0x1, mgr->regs + FPGAMGRREGS_DCLKSTAT);
			return 0;
		}

		if (is_timeout(start, 100 * MSECOND))
			return -ETIMEDOUT;
	}
}

/* Start the FPGA programming by initialize the FPGA Manager */
static int fpgamgr_program_init(struct fpgamgr *mgr)
{
	uint32_t reg;
	uint32_t ctrl = 0, ratio;

	// 1) get the MSEL value
	dev_dbg(&mgr->dev, "step 1...");
	reg = readl(mgr->regs + FPGAMGRREGS_IMGCFG_STAT);
	reg = ((reg & FPGAMGRREGS_MSEL_MASK) >> FPGAMGRREGS_MSEL_LSB);
	dev_dbg(&mgr->dev, "0x%08x\n", reg);

	if (reg > 0x1) {
		dev_err(&mgr->dev, "program init aborted:\n"
				"    MSEL pins not set for HPS configuration\n");
		return -EPERM;
	}

	// 2) Write cfgwidth bit of imgcfg_ctrl_02
	dev_dbg(&mgr->dev, "step 2...");
	ctrl = readl(mgr->regs + FPGAMGRREGS_IMGCFG_CTRL_02);
	ctrl |= FPGAMGRREGS_CFGWDTH_MASK;
	ctrl = writel(ctrl, mgr->regs + FPGAMGRREGS_IMGCFG_CTRL_02);
	dev_dbg(&mgr->dev, "0x%08x\n", ctrl);

	// 3) Configure cdratio
	dev_dbg(&mgr->dev, "step 3...");
	ratio = CDRATIO_x1; /* CDRATIO_x8;*/

	ctrl = readl(mgr->regs + FPGAMGRREGS_IMGCFG_CTRL_02);
	ctrl |= (ratio << FPGAMGRREGS_CDRATIO_LSB);
	ctrl = writel(ctrl, mgr->regs + FPGAMGRREGS_IMGCFG_CTRL_02);
	dev_dbg(&mgr->dev, "0x%08x\n", ctrl);

	// 4) Poll nconfig_pin and nstatus_pin until both 1
	dev_dbg(&mgr->dev, "step 4...");
	while (1) {
		reg = readl(mgr->regs + FPGAMGRREGS_IMGCFG_STAT);
		if ((reg & FPGAMGRREGS_NCONFIG_MASK) && (reg & FPGAMGRREGS_NSTATUS_MASK))
			break;
	}
	dev_dbg(&mgr->dev, "0x%08x\n", reg);

	// 5) Deassert signal drives from HPS
	// nce=1, pr_request=0
	dev_dbg(&mgr->dev, "step 5...");
	ctrl = readl(mgr->regs + FPGAMGRREGS_IMGCFG_CTRL_01);
	ctrl |= FPGAMGRREGS_S2FNCE_MASK;
	ctrl &= ~FPGAMGRREGS_S2FPR_REQ_MASK;
	ctrl = writel(ctrl, mgr->regs + FPGAMGRREGS_IMGCFG_CTRL_01);
	dev_dbg(&mgr->dev, "0x%08x\n", ctrl);

	// cfg_ctrl=0
	dev_dbg(&mgr->dev, "step 5 A...");
	ctrl = readl(mgr->regs + FPGAMGRREGS_IMGCFG_CTRL_02);
	ctrl &= ~FPGAMGRREGS_ENCFGCTL_MASK;
	ctrl = writel(ctrl, mgr->regs + FPGAMGRREGS_IMGCFG_CTRL_02);
	dev_dbg(&mgr->dev, "0x%08x\n", ctrl);

	// nconfig=1, nstatus_oe=0, condone_oe=0
	dev_dbg(&mgr->dev, "step 5 B...");
	ctrl = readl(mgr->regs + FPGAMGRREGS_IMGCFG_CTRL_00);
	ctrl &= (~FPGAMGRREGS_S2FNSTATUS_OE_MASK & ~FPGAMGRREGS_S2FCONDONE_OE_MASK);
	ctrl |= FPGAMGRREGS_S2FNCONFIG_MASK;
	ctrl = writel(ctrl, mgr->regs + FPGAMGRREGS_IMGCFG_CTRL_00);
	dev_dbg(&mgr->dev, "0x%08x\n", ctrl);

	// 6) Enable overides for DATA, DCLK, nCE, PR_REQUEST, and nCONFIG
	dev_dbg(&mgr->dev, "step 6...");
	ctrl = readl(mgr->regs + FPGAMGRREGS_IMGCFG_CTRL_01);
	ctrl &= ~FPGAMGRREGS_S2FNENABLE_MASK;
	ctrl = writel(ctrl, mgr->regs + FPGAMGRREGS_IMGCFG_CTRL_01);
	dev_dbg(&mgr->dev, "0x%08x\n", ctrl);

	dev_dbg(&mgr->dev, "step 6 B...");
	ctrl = readl(mgr->regs + FPGAMGRREGS_IMGCFG_CTRL_00);
	ctrl &= ~FPGAMGRREGS_S2FNENABLE_NCFG_MASK;
	ctrl = writel(ctrl, mgr->regs + FPGAMGRREGS_IMGCFG_CTRL_00);
	dev_dbg(&mgr->dev, "0x%08x\n", ctrl);

	// 7) Disable overides for nSTATUS, CONF_DONE
	dev_dbg(&mgr->dev, "step 7...");
	ctrl = readl(mgr->regs + FPGAMGRREGS_IMGCFG_CTRL_00);
	ctrl |= (1 << FPGAMGRREGS_S2FNENABLE_NCNDN_LSB | 1 << FPGAMGRREGS_S2FNENABLE_NSTAT_LSB);
	ctrl = writel(ctrl, mgr->regs + FPGAMGRREGS_IMGCFG_CTRL_00);
	dev_dbg(&mgr->dev, "0x%08x\n", ctrl);

	// 8) Assert Chip Select
	dev_dbg(&mgr->dev, "step 8...");
	ctrl = readl(mgr->regs + FPGAMGRREGS_IMGCFG_CTRL_01);
	ctrl &= ~FPGAMGRREGS_S2FNCE_MASK;
	ctrl = writel(ctrl, mgr->regs + FPGAMGRREGS_IMGCFG_CTRL_01);
	dev_dbg(&mgr->dev, "0x%08x\n", ctrl);

	// 9) Repeat step 4
	dev_dbg(&mgr->dev, "step 9...");
	while (1) {
		reg = readl(mgr->regs + FPGAMGRREGS_IMGCFG_STAT);
		if ((reg & FPGAMGRREGS_NCONFIG_MASK) && (reg & FPGAMGRREGS_NSTATUS_MASK))
			break;
	}
	dev_dbg(&mgr->dev, "0x%08x\n", reg);

	// 10) Reset Configuration
	dev_dbg(&mgr->dev, "step 10...");
	// a) Write 0 to s2f_nconfig bit
	ctrl = readl(mgr->regs + FPGAMGRREGS_IMGCFG_CTRL_00);
	ctrl &= ~FPGAMGRREGS_S2FNCONFIG_MASK;
	ctrl = writel(ctrl, mgr->regs + FPGAMGRREGS_IMGCFG_CTRL_00);
	dev_dbg(&mgr->dev, "0x%08x\n", ctrl);

	// b) Poll f2x_nstatus_pin bit until cleared
	while (1) {
		reg = readl(mgr->regs + FPGAMGRREGS_IMGCFG_STAT);
		if (!(reg & FPGAMGRREGS_NSTATUS_MASK))
			break;
	}
	dev_dbg(&mgr->dev, "0x%08x\n", reg);

	// c) Write 1 to s2f_nconfig bit
	ctrl = readl(mgr->regs + FPGAMGRREGS_IMGCFG_CTRL_00);
	ctrl |= FPGAMGRREGS_S2FNCONFIG_MASK;
	ctrl = writel(ctrl, mgr->regs + FPGAMGRREGS_IMGCFG_CTRL_00);
	dev_dbg(&mgr->dev, "0x%08x\n", ctrl);

	// d) Poll f2s_nstatus_pin bit until set; confirm f2s_condone_pin is 0,f2s_condone_oe is set
	while (1) {
		reg = readl(mgr->regs + FPGAMGRREGS_IMGCFG_STAT);
		if (reg & FPGAMGRREGS_NSTATUS_MASK)
			break;
	}
	dev_dbg(&mgr->dev, "0x%08x\n", reg);

	if ((reg & FPGAMGRREGS_CONDONE_MASK)) {
		dev_dbg(&mgr->dev, "step 10 Condone is 1\n");
	}
	if (!(reg & FPGAMGRREGS_CONDONE_OE_MASK)) {
		dev_dbg(&mgr->dev, "step 10 condone_oe is 0\n");
	}

	// 11) Enable DCLK, DATA path by setting en_cfg_ctrl bit in imgcfg_ctrl_02
	dev_dbg(&mgr->dev, "step 11...");
	ctrl = readl(mgr->regs + FPGAMGRREGS_IMGCFG_CTRL_02);
	ctrl |= FPGAMGRREGS_ENCFGCTL_MASK;
	ctrl = writel(ctrl, mgr->regs + FPGAMGRREGS_IMGCFG_CTRL_02);
	dev_dbg(&mgr->dev, "0x%08x\n", ctrl);

	return 0;
}

/* Ensure the FPGA entering config done */
static int fpgamgr_program_poll_cd(struct fpgamgr *mgr)
{
	uint32_t reg;
	uint64_t start;

	start = get_time_ns();
	while (1) {
		reg = readl(mgr->regs + FPGAMGRREGS_IMGCFG_STAT);

		/* config done without error */
		if (reg & FPGAMGRREGS_CONDONE_MASK)
			break;

		if (is_timeout(start, 100 * MSECOND)) {
			reg = readl(mgr->regs + FPGAMGRREGS_IMGCFG_STAT);
			dev_dbg(&mgr->dev, "imgcfg_stat: 0x%08x\n", reg);
			return -ETIMEDOUT;
		}
	}

#if 0
	/* disable AXI configuration */
	//val = readl(mgr->regs + FPGAMGRREGS_CTRL);
	//val &= ~FPGAMGRREGS_CTRL_AXICFGEN_MASK;
	//writel(val, mgr->regs + FPGAMGRREGS_CTRL);

	reg = readl(mgr->regs + FPGAMGRREGS_IMGCFG_STAT);
	if (reg & FPGAMGRREGS_NSTATUS_MASK)
		return -EIO;
#endif

	return 0;
}

/* Ensure the FPGA entering init phase */
static int fpgamgr_program_poll_initphase(struct fpgamgr *mgr)
{
	uint64_t start;

	dev_dbg(&mgr->dev, "step 14/15...\n");
	/* additional clocks for the CB to enter initialization phase */
	// 14) Write dclkcnt with 0xF and pull dclkcnstat until 0
	if (fpgamgr_dclkcnt_set(mgr, 0xF) != 0)
		return -5;

	// 15) Wait for init sequence to complete. Poll f2s_usermode bit until reads 1
	dev_dbg(&mgr->dev, "waiting for user mode...\n");

	start = get_time_ns();
	while (1) {
		int mode = fpgamgr_get_mode(mgr);

		if (mode == FPGAMGRREGS_MODE_USERMODE)
			break;

		if (is_timeout(start, 100 * MSECOND))
			return -ETIMEDOUT;
	}

	return 0;
}

/* Ensure the FPGA entering user mode */
static int fpgamgr_program_poll_usermode(struct fpgamgr *mgr)
{
	uint32_t val;

	/* additional clocks for the CB to exit initialization phase */
	//if (fpgamgr_dclkcnt_set(mgr, 0x5000) != 0)
		//return -7;

	// 16) Disable DATA, DCLK by clearing en_cfg_ctrl
	dev_dbg(&mgr->dev, "step 16...");
	val = readl(mgr->regs + FPGAMGRREGS_IMGCFG_CTRL_02);
	val &= ~FPGAMGRREGS_ENCFGCTL_MASK;
	writel(val, mgr->regs + FPGAMGRREGS_IMGCFG_CTRL_02);
	dev_dbg(&mgr->dev, "0x%08x\n", val);

	// 17) Disable chip select, set s2f_nce bit
	dev_dbg(&mgr->dev, "step 17...");
	val = readl(mgr->regs + FPGAMGRREGS_IMGCFG_CTRL_01);
	val |= FPGAMGRREGS_S2FNCE_MASK;
	writel(val, mgr->regs + FPGAMGRREGS_IMGCFG_CTRL_01);
	dev_dbg(&mgr->dev, "0x%08x\n", val);

	// 18) Disable overrides to nCONFIG, DATA, and DCLK
	dev_dbg(&mgr->dev, "step 18 A...");
	val = readl(mgr->regs + FPGAMGRREGS_IMGCFG_CTRL_01);
	val |= FPGAMGRREGS_S2FNENABLE_MASK;
	writel(val, mgr->regs + FPGAMGRREGS_IMGCFG_CTRL_01);
	dev_dbg(&mgr->dev, "0x%08x\n", val);

	dev_dbg(&mgr->dev, "step 18 B...");
	val = readl(mgr->regs + FPGAMGRREGS_IMGCFG_CTRL_00);
	val |= FPGAMGRREGS_S2FNENABLE_NCFG_MASK;
	writel(val, mgr->regs + FPGAMGRREGS_IMGCFG_CTRL_00);
	dev_dbg(&mgr->dev, "0x%08x\n", val);

	// 19) Check usermode enabled; and configuration is done
	dev_dbg(&mgr->dev, "step 19 A...");
	if (fpgamgr_get_mode(mgr) != FPGAMGRREGS_MODE_USERMODE) {
		dev_dbg(&mgr->dev, "Not entereing usermode\n");
		return -ETIMEDOUT;
	}
	dev_dbg(&mgr->dev, "Entereing usermode\n");

	dev_dbg(&mgr->dev, "step 19 BC...");
	val = readl(mgr->regs + FPGAMGRREGS_IMGCFG_STAT);
	if (!(val & FPGAMGRREGS_NSTATUS_MASK) && !(val & FPGAMGRREGS_CONDONE_MASK)) {
		dev_dbg(&mgr->dev, "timeout 0x%08x\n", val);
		return -ETIMEDOUT;
	}
	dev_dbg(&mgr->dev, "0x%08x\n", val);

	return 0;
}

/*
 * Using FPGA Manager to program the FPGA
 * Return 0 for sucess
 */
static int fpgamgr_program_start(struct firmware_handler *fh)
{
	struct fpgamgr *mgr = container_of(fh, struct fpgamgr, fh);
	int status;

	dev_dbg(&mgr->dev, "start programming...\n");

	/* initialize the FPGA Manager */
	status = fpgamgr_program_init(mgr);
	if (status) {
		dev_err(&mgr->dev, "program init failed with: %s\n",
				strerror(-status));
		return status;
	}

	return 0;
}

/* Write the RBF data to FPGA Manager */
static int fpgamgr_program_write_buf(struct firmware_handler *fh, const void *buf,
		size_t size)
{
	// 12) Write bitsream data to to img_data_w register;
	//      Periodically read and confirm that f2s_nstatus_pis bit is set;
	//      if not restart at step (1)
	struct fpgamgr *mgr = container_of(fh, struct fpgamgr, fh);
	const uint32_t *buf32 = buf;
	unsigned long reg;
	static bool once = true;

	if (once) {
		bool encrypt = ((buf32[69] >> 2) & 3) != 0;
		bool compress = !((buf32[229] >> 1) & 1);
		dev_dbg(&mgr->dev, "step 12...\n");
		dev_dbg(&mgr->dev, "encrypt(%d) %d = 0x%08x\n", encrypt, 69, buf32[69]);
		dev_dbg(&mgr->dev, "compress(%d) %d = 0x%08x\n", compress, 229, buf32[229]);
		once = false;
		if (compress || encrypt) {
			dev_err(&mgr->dev, "encrypt(%d), compress(%d)\n", encrypt, compress);
			return -EIO;
		}
	}

	/* write to FPGA Manager AXI data */
	while (size >= sizeof(uint32_t)) {
		writel(*buf32, mgr->regs_data);
		//readl(mgr->regs + FPGAMGRREGS_MON_GPIO_EXT_PORTA_ADDRESS);
		buf32++;
		size -= sizeof(uint32_t);
	}

	if (size) {
		const uint8_t *buf8 = (const uint8_t *)buf32;
		uint32_t word = 0;

		while (size--) {
			word |= *buf8;
			word <<= 8;
			buf8++;
		}

		writel(word, mgr->regs_data);
		//readl(mgr->regs + FPGAMGRREGS_MON_GPIO_EXT_PORTA_ADDRESS);
	}
	reg = readl(mgr->regs + FPGAMGRREGS_IMGCFG_STAT);
	if (!(reg & FPGAMGRREGS_NSTATUS_MASK)) {
		dev_dbg(&mgr->dev, "failed step 12.\n");
		return -EIO;
	}

	return 0;
}

static int fpgamgr_program_finish(struct firmware_handler *fh)
{
	struct fpgamgr *mgr = container_of(fh, struct fpgamgr, fh);
	int status;

	// 13) Ensure the FPGA entering config done
	dev_dbg(&mgr->dev, "step 13...\n");
	status = fpgamgr_program_poll_cd(mgr);
	if (status) {
		dev_err(&mgr->dev, "poll for config done failed with: %s\n",
				strerror(-status));
		return status;
	}

	dev_dbg(&mgr->dev, "waiting for init phase...\n");

	/* Ensure the FPGA entering init phase */
	status = fpgamgr_program_poll_initphase(mgr);
	if (status) {
		dev_err(&mgr->dev, "poll for init phase failed with: %s\n",
				strerror(-status));
		return status;
	}

	/* Ensure the FPGA entering user mode */
	status = fpgamgr_program_poll_usermode(mgr);
	if (status) {
		dev_err(&mgr->dev, "poll for user mode with: %s\n",
				strerror(-status));
		return status;
	}

	return 0;
}

/* Get current programmed state of fpga and put in "programmed" parameter */
static int programmed_get(struct param_d *p, void *priv)
{
	struct fpgamgr *mgr = priv;
	mgr->programmed = fpgamgr_get_mode(mgr) == FPGAMGRREGS_MODE_USERMODE;
	return 0;
}

static int fpgamgr_probe(struct device_d *dev)
{
	struct resource *iores;
	struct fpgamgr *mgr;
	struct firmware_handler *fh;
	const char *alias = of_alias_get(dev->device_node);
	const char *model = NULL;
	struct param_d *p;
	int ret;

	dev_dbg(dev, "Probing FPGA firmware programmer\n");

	mgr = xzalloc(sizeof(*mgr));
	fh = &mgr->fh;

	iores = dev_request_mem_resource(dev, 0);
	if (IS_ERR(iores)) {
		ret = PTR_ERR(iores);
		goto out;
	}
	mgr->regs = IOMEM(iores->start);

	iores = dev_request_mem_resource(dev, 1);
	if (IS_ERR(iores)) {
		ret = PTR_ERR(iores);
		goto out;
	}
	mgr->regs_data = IOMEM(iores->start);

	if (alias)
		fh->id = xstrdup(alias);
	else
		fh->id = xstrdup("socfpga-fpga");

	fh->open = fpgamgr_program_start;

	fh->write = fpgamgr_program_write_buf;
	fh->close = fpgamgr_program_finish;

	of_property_read_string(dev->device_node, "compatible", &model);
	if (model)
		fh->model = xstrdup(model);
	fh->dev = dev;

	dev_dbg(dev, "Registering FPGA firmware programmer\n");

	mgr->dev.id = DEVICE_ID_SINGLE;
	strcpy(mgr->dev.name, "fpga");
	mgr->dev.parent = dev;
	ret = register_device(&mgr->dev);
	if (ret) {
		goto out;
	}

	if (IS_ENABLED(CONFIG_PARAMETER)) {
		p = dev_add_param_bool(&mgr->dev, "programmed", NULL, programmed_get,
			&mgr->programmed, mgr);
		if (IS_ERR(p)) {
			ret = PTR_ERR(p);
			goto out_unreg;
		}
	}

	fh->dev = &mgr->dev;
	ret = firmwaremgr_register(fh);
	if (ret != 0) {
		free(mgr);
		goto out_unreg;
	}

	return 0;
out_unreg:
	unregister_device(&mgr->dev);
out:
	free(fh->id);
	free(mgr);

	return ret;
}

static struct of_device_id fpgamgr_id_table[] = {
	{
		.compatible = "altr,socfpga-a10-fpga-mgr",
	},
};

static struct driver_d fpgamgr_driver = {
	.name = "socfpa-fpgamgr",
	.of_compatible = DRV_OF_COMPAT(fpgamgr_id_table),
	.probe = fpgamgr_probe,
};
device_platform_driver(fpgamgr_driver);
