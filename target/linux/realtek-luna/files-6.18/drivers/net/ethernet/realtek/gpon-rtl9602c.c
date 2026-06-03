// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Realtek RTL9602C GPON MAC — foundation driver.
 *
 * Independent implementation from the SoC's register interface and the
 * G.984/G.988 protocols. The GPON MAC is a sub-block of the "Luna" switch core
 * (SWCORE, phys 0x1B000000); its register window begins at SWCORE + 0x700000
 * (phys 0x1B700000). Register offsets and field positions are hardware facts
 * taken from the SoC register map.
 *
 * This stage brings up the register-access layer, initialises the PON SerDes
 * (whose CMU/PLL provides the MAC core clock — without it the MAC cannot leave
 * reset and the GTC banks read floating), and reports the MAC identity plus the
 * live GPON activation state (the ONU FSM O1..O5, ONU-ID and ranging
 * equalisation delay) via /proc/gpon. The GPON
 * activation FSM (downstream sync, ranging) runs autonomously in hardware once
 * the MAC is out of soft-reset and fed valid downstream GTC; the upstream PLOAM
 * message FSM (serial-number / password / OMCI transport) that drives O3..O5
 * builds on this foundation.
 *
 * The GPON MAC window responds without any extra gating (it shares the SWCORE
 * window the Ethernet driver already maps); GPON_TEST (off 0x14) reads the
 * power-on scratch pattern 0x12345678.
 *
 * GPON register block (offsets from the GPON base 0x1B700000):
 *   0x00000  GPON_INT_DLT        aggregate interrupt delta
 *   0x0000c  GPON_RESET          [8] RST_DONE, [0] SOFT_RST (1=assert)
 *   0x00010  GPON_VERSION        [7:0] VER_ID
 *   0x00014  GPON_TEST           scratch (power-on pattern 0x12345678)
 *   0x00020  GPON_AES_BYPASS
 *   0x00040  GPON_INTR_MASK
 *   0x00044  GPON_INTR_STS
 *   0x01000  GPON_GTC_DS_INTR_DLT   downstream GTC interrupt delta
 *   0x01004  GPON_GTC_DS_INTR_MASK
 *   0x01008  GPON_GTC_DS_INTR_STS
 *   0x01010  GPON_GTC_DS_ONU_STATUS [15:8] ONU_ID, [3:0] ONU_STATE (O1..O7)
 *   0x05010  GPON_GTC_US_ONU_ID     [15:8] ONU_ID (upstream copy)
 *   0x05040  GPON_GTC_US_MIN_DELAY  [15:7] MIN_DELAY1, [6:0] MIN_DELAY2
 *   0x05044  GPON_GTC_US_EQD        [26:24] EQD multiframe, [17:0] EQD in-frame
 */

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/timer.h>
#include "rtl9602c_gpon_nic.h"

#define GPON_PHYS_BASE	0x1b700000u
#define GPON_REG_SIZE	0x00010000u	/* covers GTC DS block at +0x1000 */

/*
 * SoC system-controller "IP enable" bank. Bit 5 powers the PON packet datapath
 * ("PONPBO" / PON-IP at phys 0x1bf00000); the neighbouring bits in this bank
 * gate other on-chip IPs (e.g. PCIe). U-Boot never sets the PON bit because it
 * does not run GPON, so the PON-IP block is unclocked and any access to it
 * hard-hangs the CPU bus until this bit is set.
 */
#define SOC_IP_ENABLE_PHYS	0x1800063cu
#define   SOC_IP_EN_PON		BIT(5)

#define GPON_INT_DLT		0x0000
#define GPON_RESET		0x000c
#define   GPON_SOFT_RST		BIT(0)		/* 1 = assert soft reset      */
#define   GPON_RST_DONE		BIT(8)		/* 1 = reset cycle complete   */
#define GPON_VERSION		0x0010
#define   GPON_VER_ID_MASK	0xffu
#define GPON_TEST		0x0014
#define GPON_AES_BYPASS		0x0020
#define GPON_INTR_MASK		0x0040
#define GPON_INTR_STS		0x0044
#define GPON_GTC_DS_INTR_DLT	0x1000
#define GPON_GTC_DS_INTR_MASK	0x1004
#define GPON_GTC_DS_INTR_STS	0x1008
#define GPON_GTC_DS_LOS_CFG_STS	0x1040		/* downstream LOS status/cfg  */
#define   GPON_CDR_LOS_SIG	BIT(10)		/* 1 = CDR not recovering clk */
#define   GPON_OPTIC_LOS_SIG	BIT(8)		/* 1 = no optical signal      */
#define   GPON_OPTIC_LOS_POLAR	BIT(1)		/* invert optical-LOS input   */
#define   GPON_OPTIC_LOS_EN	BIT(0)		/* enable optical-LOS monitor */
#define GPON_GTC_DS_ONU_STATUS	0x1010
#define   GPON_ONU_STATE_MASK	0xfu		/* [3:0]  FSM state O1..O7    */
#define   GPON_ONU_ID_SHIFT	8		/* [15:8] ONU-ID             */
#define   GPON_ONU_ID_MASK	0xffu
#define GPON_GTC_US_ONU_ID	0x5010
#define GPON_GTC_US_MIN_DELAY	0x5040
#define GPON_GTC_US_EQD		0x5044
#define   GPON_EQD_INFRAME_MASK	0x3ffffu	/* [17:0]  in-frame delay    */
#define   GPON_EQD_MF_SHIFT	24		/* [26:24] multiframe count  */
#define   GPON_EQD_MF_MASK	0x7u
#define   GPON_EQD_FRAME_LEN	(19440 * 8)	/* one upstream frame, in bits */
#define GPON_GTC_US_WRITE_PROTECT 0x5018	/* gate for US config writes   */
#define   GPON_US_WP_UNLOCK	0xcc19u		/* magic: enable protected US writes */
#define   GPON_US_WP_LOCK	0x0000u
#define GPON_GTC_US_CFG		0x5014		/* [11]LESS_RANDOM [10]IND_NRM_PLM
						 * [9]PLM_DIS [4]ENA_AUTO_DG
						 * [3]US_BEN_POLAR [0]SCRM_DIS */
#define   GPON_US_CFG_VAL	0x0c18u		/* online operating value: BEN_POLAR=1,
						 * scrambler on, PLOAM on (LESS_RANDOM|
						 * IND_NRM_PLM|ENA_AUTO_DG|US_BEN_POLAR) */
#define GPON_GTC_US_LASER	0x504c		/* [13:8] LON_TIME, [5:0] LOFF_TIME */
#define   GPON_US_LASER_VAL	0x2028u		/* LON=32, LOFF=40 burst-window edges */
#define GPON_GTC_US_BOH_CFG	0x5054		/* [11:8] BOH_REPEAT, [7:0] BOH_LENGTH */
#define GPON_GTC_US_BOH_DATA	0x5080		/* 12-entry burst-overhead byte array, stride 4 */
#define   GPON_BOH_LEN		12		/* stored bytes (TOTAL_OVERHEAD_BITS(96)/8); HW extends via REPEAT */
#define   GPON_BOH_MAX_LEN	252		/* hardware BOH_LENGTH field cap */

/*
 * PLOAM message path (ITU-T G.984.3 management channel that drives the ONU
 * through O3..O5). This MAC is a software-PLOAM design: received downstream
 * PLOAM messages land in an 8-word buffer that firmware dequeues, and upstream
 * PLOAM messages (Serial_Number_ONU, Password, Acknowledge, ...) are composed
 * in an 8-word buffer and enqueued for transmission. The indicator registers
 * expose buffer-occupancy and the dequeue/enqueue triggers. Offsets are the
 * true GPON-block offsets read from the SoC register map.
 */
#define GPON_GTC_US_ONU_ID_SHIFT 8		/* [15:8] OLT-assigned ONU-ID  */
#define GPON_GTC_DS_PLOAM_CFG	0x101c		/* [9]BC_ACC [8]ONUID_FLT [7:0]NOMSG_ID */
#define GPON_GTC_DS_PLOAM_IND	0x1080		/* DS receive-buffer indicator */
#define   GPON_DS_PLM_BUF_EMPTY	BIT(5)		/* 1 = no DS PLOAM pending     */
#define   GPON_DS_PLM_BUF_FULL	BIT(4)
#define   GPON_DS_PLM_DEQ	BIT(0)		/* W: advance to next message  */
#define GPON_GTC_DS_PLOAM_MSG	0x10a0		/* 8-word received-message buf */
#define GPON_GTC_US_PLOAM_IND	0x50c0		/* US transmit-queue indicator */
#define   GPON_US_PLM_TYPE_SHIFT 8		/* [10:8] queue/type select    */
#define   GPON_US_PLM_NRM_EMPTY	BIT(7)		/* normal queue empty          */
#define   GPON_US_PLM_NRM_FULL	BIT(6)
#define   GPON_US_PLM_URG_EMPTY	BIT(5)		/* urgent queue empty          */
#define   GPON_US_PLM_URG_FULL	BIT(4)
#define   GPON_US_PLM_ENQ	BIT(0)		/* W: queue the composed msg   */
#define GPON_GTC_US_PLOAM_DATA	0x50e0		/* 8-word transmit-message buf */
#define GPON_GTC_US_PLOAM_CFG	0x5100		/* US PLOAM buffer control     */
#define   GPON_US_PLM_FLUSH_BUF	BIT(4)
#define   GPON_US_PLM_CRC_GEN_EN BIT(1)		/* HW computes US PLOAM CRC    */
#define   GPON_US_PLM_ONUID_OVRD BIT(0)		/* override ONU-ID field       */

#define GPON_TEST_SCRATCH	0x12345678u
#define GPON_RST_POLL_MAX	1000		/* bounded RST_DONE poll      */

/*
 * PON SerDes (SDS) analog block. It lives in the SWCORE window (phys
 * 0x1B000000), NOT the GPON datapath sub-block — these are plain MMIO offsets
 * off the switch-core base. The SDS CMU/PLL recovers the line clock that feeds
 * the GPON MAC core; until it is configured and its analog-ready flag asserts,
 * the MAC core has no clock, the soft-reset never completes (RST_DONE stays 0)
 * and the GTC register banks read a floating pattern. The GPON-MAC reset is
 * issued as part of this sequence (the SDS_RST also resets the MAC).
 */
#define SWCORE_PHYS_BASE	0x1b000000u
#define SWCORE_REG_SIZE		0x00041000u	/* covers up to FIB_EXT_REG21  */

/*
 * Register offsets here are the TRUE switch-core offsets (verified against the
 * SoC register map). The SerDes digital/analog banks live at SWCORE + 0x22xxx
 * (phys 0x1b022xxx), NOT 0x40xxx — a direct access to 0x40xxx hits an unmapped
 * hole that returns the bus abort-fill 0xbad0bad0. SDS_CFG and SOFTWARE_RST are
 * in the low control page.
 */
#define SW_SOFTWARE_RST		0x00104
#define   SW_SDS_RST_PS		BIT(0)		/* [0]  CMD_SDS_RST_PS pulse   */
#define   SW_SDS_CFG_RST_PS	BIT(7)		/* [7]  CMD_SDS_CFG_RST_PS     */
#define   SW_PONMAC_RST		BIT(6)		/* [6]  GPON-MAC core reset    */
#define SDS_CFG			0x001d0		/* [4:0] CFG_SDS_MODE          */
#define   SDS_MODE_OFF		0x1fu
#define   SDS_MODE_GPON		0x08u
#define SDS_FIB_STATUS		0x001e4		/* [17] SDS_SDET [2] FIB100_SDET */
#define   SDS_FIB_SDS_SDET	BIT(17)		/* SDS-level optical sig-detect */
#define FIB_REG16		0x22c40		/* [10] FP_CFG_FRC_SD [2] SEL_RX_SD */
#define   FIB_FP_CFG_FRC_SD	BIT(10)

/*
 * SoC hardware I2C master (SWCORE register file). The external RTL8290B BOSA
 * optical transceiver hangs off I2C bus 0; it must be initialised over this
 * master before the real optical signal-detect asserts
 * (SDS_FIB_STATUS.SDS_SDET). The master is indirect: program the per-bus
 * I2C_CONFIG (slave addr + addr/data width + clock divider), write the target
 * register offset into I2C_IND_ADR, kick I2C_IND_CMD (CMD_EN | RW_EN), poll
 * BUSY, then read I2C_IND_RD. Confirmed on the hardware: I2C_CONFIG.DEV_ID
 * reads back 0x50; bus-0 is enabled in IO_MODE_EN bit13.
 */
#define I2C_CONFIG0		0x23004		/* bus0; stride 0x20 per bus   */
#define   I2C_CFG_DEV_ID_MSB	20		/* [20:14] 7-bit slave addr    */
#define   I2C_CFG_DEV_ID_LSB	14
#define   I2C_CFG_AW_MSB	13		/* [13:12] reg-addr width 0=8b */
#define   I2C_CFG_AW_LSB	12
#define   I2C_CFG_DW_MSB	11		/* [11:10] data width 0=8b     */
#define   I2C_CFG_DW_LSB	10
#define   I2C_CFG_CLKDIV_MSB	9		/* [9:0] clock divider         */
#define   I2C_CFG_CLKDIV_LSB	0
#define   I2C_CLKDIV_100K	0x270u		/* (62500/100)-1 -> ~100 kHz   */
#define I2C_IND_WD		0x000b0		/* [31:0] write data           */
#define I2C_IND_ADR		0x000b8		/* [31:0] target reg offset    */
#define I2C_IND_CMD		0x000c0		/* [0]CMD_EN [1]RW_EN [2]BUSY [3]NACK */
#define   I2C_CMD_EN		BIT(0)
#define   I2C_CMD_RW_WR		BIT(1)		/* 1=write 0=read              */
#define   I2C_CMD_BUSY		BIT(2)
#define   I2C_CMD_NACK		BIT(3)
#define I2C_IND_RD		0x000c8		/* [31:0] read data            */
#define I2C_BUSY_POLL_MAX	1000		/* x10us = up to 10 ms         */
/*
 * RTL8290B register space is paged by I2C slave address: the full 12-bit
 * register number's high byte selects a 256-register page (page0->0x50,
 * page1->0x51, page2->0x54, page3+ ->0x55) and the low byte is the offset
 * within that page. Confirmed: chip-ID reg 0x390 reads correctly at slave 0x55
 * offset 0x90. RX path registers (NUM/page mapping from the transceiver's
 * register map):
 */
#define BOSA_REG_NUM		0x390		/* chip NUM (0x8290), 2 bytes  */
#define BOSA_REG_VID		0x394		/* manufacturer ID (0x0001)    */
#define BOSA_REG_W4		0x204		/* [4] EN_L booster (1=on)     */
#define BOSA_REG_W41		0x229		/* [4] RXI_PWDN_L (0=RX on)    */
#define BOSA_REG_CONTROL2	0x254		/* [6] LOS_PIN_TRI (0=drive SD)*/
#define BOSA_REG_STATUS2	0x383		/* [2] RX_LOS_STATUS (0=signal)*/
#define WSDS_DIG_00		0x22030		/* SDS clock + soft-reset-B bank */
#define   WSDS_STOP_CLK		BIT(0)		/* 1 = GPON MAC core clock off */
#define   WSDS_FRC_125M_EN	BIT(4)		/* force 125M ref clock enable */
#define   WSDS_FRCV_125M_EN	BIT(5)		/* forced value for 125M       */
#define   WSDS_SFT_RSTB		BIT(8)		/* digital soft reset-B        */
#define   WSDS_SFT_RSTB_EPON	BIT(9)		/* EPON datapath reset-B       */
#define   WSDS_SFT_RSTB_GPON	BIT(10)		/* GPON datapath reset-B       */
#define   WSDS_SFT_RSB_ANA	BIT(11)		/* analog reset-B              */
#define   WSDS_DIG00_RUN	0xf30u		/* operational run state       */
#define WSDS_DIG_01		0x22034		/* [31:0] CFG_DMY0 (force-SDS)  */
#define WSDS_DIG_02		0x22038		/* [10]  EN_PDOWN_BEN          */
#define WSDS_DIG_03		0x2203c		/* [6:4] CFG_TXDIS_SEL_DLY     */
#define WSDS_DIG_18		0x22090		/* [12]  BEN_OE                */
#define WSDS_DIG_1D		0x220a4		/* interface reset-B releases  */
#define   WSDS_SFT_RSTB_INF	BIT(14)		/* interface soft reset-B      */
#define   WSDS_SFT_RSTB_INF_RX	BIT(15)		/* RX interface soft reset-B   */
#define   WSDS_SFT_RSTB_INF_TX	BIT(16)		/* TX interface soft reset-B   */
#define SDS_ANA_COM_REG03	0x2258c		/* [15:14] CMU_ISTANK_SEL_RX   */
#define SDS_ANA_COM_REG11	0x225ac		/* [7:0]  RX_FILT_CONFIG       */
#define SDS_ANA_COM_REG12	0x225b0		/* [14]   RX_SEL_CDR_AFEN      */
#define SDS_ANA_COM_REG22	0x225d8		/* [5:3] TX_AMP [2:0] TX_EMP   */
#define SDS_ANA_COM_REG26	0x225e8		/* [6:5] CMU_ISTANK_SEL_GPHY   */
#define SDS_ANA_GPON_REG42	0x22728		/* [2]   PCM_CMU_EN            */
#define SDS_ANA_GPON_REG46	0x22738		/* [9:7]KI [6:4]KP1 [3:1]KP2   */
#define SDS_ANA_MISC_REG00	0x22500		/* [5] FRC_RX_EN_VAL [4] _ON   */
#define SDS_ANA_MISC_REG01	0x22504		/* [7:5] SPDSEL_VAL [4] _ON    */
#define SDS_ANA_MISC_REG02	0x22508		/* [13] SD_VAL [12] SD_FORCE   */
#define FIB_EXT_REG21		0x22e54		/* [13]  FEP_V2ANALOG (lock)   */
#define   SDS_ANALOG_READY	BIT(13)
#define SDS_LOCK_POLL_MAX	1000		/* x200us = up to 200 ms       */

/*
 * SoC IO pad routing for the optical front-end (switch-core register file, so
 * these are plain SWCORE offsets like the SDS block above). IO_MODE_EN's OEM_EN
 * bit enables the optical "e-mode" pads (TX_DISABLE, optical TX_SD / RX signal-
 * detect); IO_GPIO_EN is a 1-bit-per-pin GPIO-function-enable array (32 pins per
 * 32-bit word). The optical RX signal-detect shares pad GPIO 13: while that pin
 * is in GPIO mode the BOSA's signal-detect never reaches the GPON LOS input, so
 * OPTIC_LOS_SIG reads "loss" even with real light. Releasing GPIO 13 (function
 * disabled) routes the pad to the optical-SD input.
 */
#define SOC_IO_MODE_EN		0x23018		/* [19] OEM_EN (optical pads)  */
#define   IO_OEM_EN		BIT(19)
#define SOC_IO_GPIO_EN		0x00048		/* GPIO func-enable, 1 bit/pin */
#define SOC_IO_GPIO_EN_W0	0x40202006u	/* enable GPIO 1,2,13,21,30    */
#define SOC_IO_GPIO_EN_W1	0x00000819u	/* enable GPIO 32,35,36,43     */

/*
 * SoC GPIO controller (its own register page at phys 0x18003300, outside the
 * switch-core window). The optical signal-detect is wired to a board GPIO; a
 * working (O5) unit enables a specific set of pins here with GPIO 21 as an
 * input (the lone enabled input). Configure the controller to the known-good
 * state so the signal-detect pin is sampled and reaches the GPON LOS input.
 */
#define GPIO_PHYS_BASE		0x18003300u
#define GPIO_REG_SIZE		0x40u
#define GPIO_CTRL_ABCD		0x00
#define GPIO_DIR_ABCD		0x08
#define GPIO_DATA_ABCD		0x0c
#define GPIO_CTRL_EFGH		0x1c
#define GPIO_DIR_EFGH		0x24
#define GPIO_DATA_EFGH		0x28
#define GPIO_GOLD_DIR_ABCD	0x40002006u	/* 1,2,13,30 out; 21 in        */
#define GPIO_GOLD_DATA_ABCD	0xdbff1246u
#define GPIO_GOLD_DIR_EFGH	0x00000819u
#define GPIO_GOLD_DATA_EFGH	0x000037e5u

/*
 * PON packet-buffer / datapath ("PON-IP", "PBO/PONNIC") block. Physically this
 * is the top window of the switch core at phys 0x1bf00000 (SWCORE + 0xF00000),
 * but it is far above the SWCORE control window mapped above, so it gets its own
 * ioremap. It is only reachable once the PON IP-enable bit (SOC_IP_ENABLE_PHYS)
 * is set. The GPON MAC drains downstream GEM frames into, and sources upstream
 * frames from, this datapath; it must be configured (page/SRAM accounting, GPON
 * mode, GMII enables) before the MAC soft-reset so the MAC reset handshake
 * (RST_DONE) completes and the datapath carries traffic.
 *
 * Offsets are relative to the PON-IP base (phys 0x1bf00000). The block is split
 * into an upstream (US) and downstream (DS) half plus PONNIC IO command pages.
 */
#define PONIP_PHYS_BASE		0x1bf00000u
#define PONIP_REG_SIZE		0x00010000u	/* covers up to IO_CMD_1_DS     */

#define PI_IO_CMD_0_US		0x05434		/* [5] GMII_RX_EN [4] GMII_TX_EN */
#define PI_IO_CMD_0_DS		0x0d434
#define PI_IO_CMD_1_US		0x05438		/* [5:4] RPAGE [1:0] TPAGE size  */
#define PI_IO_CMD_1_DS		0x0d438
#define   PI_GMII_RX_EN		BIT(5)
#define   PI_GMII_TX_EN		BIT(4)
#define PI_PONIP_CTL_US		0x020d8		/* US PON-IP control             */
#define PI_PONIP_CTL_DS		0x0a0ac		/* DS PON-IP control             */
#define   PI_CFG_PBUF_EN	BIT(0)		/* [0] packet-buffer enable      */
#define   PI_CFG_STOP_RXC_EN	BIT(1)		/* [1] stop RXC enable           */
#define   PI_CFG_EPON_MODE	BIT(2)		/* [2] 0=GPON 1=EPON             */
#define PI_PON_US_FIFO_CTL	0x020f0		/* [5:4] SPACE [3:0] START       */
#define PI_PON_DSC_CFG_US	0x0215c		/* [28:16] RAM_NO [12:0] SRAM_NO */
#define PI_PON_DSC_CFG_DS	0x0a0cc		/* [14:13] PAGE_SIZE             */
#define PI_DSCRUNOUT_US		0x020e0		/* [28:16] DRAM [12:0] SRAM out  */
#define PI_DSCRUNOUT_DS		0x0a0b4
#define PI_PON_SID_STOP_TH	0x02450		/* [12:0] global stop-all page threshold */
#define PI_PON_SID_GLB_TH	0x02454		/* [28:16] ON_TH [12:0] OFF_TH (global) */
#define PI_PON_SID_RPV_TH	0x02458		/* per-SID reserved-page threshold, +sid*4 */
#define PI_RPV_TH_STRIDE	4u
#define PI_SID_NUM		65u		/* SIDs 0..64 (64 = OMCI) */
#define PI_PON_FC_CONFIG_DS	0x0a0fc		/* [28:16] FC_ON_TH [12:0] FC_OFF_TH */
#define PI_CFG_US		0x0404c		/* [26] RFF_AFULL [17] TX_STOP   */
#define PI_CFG_DS		0x0c04c		/* [16] TXE_EXTRA                */
#define PI_TX_CFG_US		0x04040		/* [12:10] IFG [2:1] PRE [0] PAD */
#define PI_TX_CFG_DS		0x0c040
#define PI_RX_CFG_US		0x04044		/* [5] accept-CRC-error          */
#define PI_RX_CFG_DS		0x0c044
#define PI_PROBE_SELECT_US	0x05400		/* [1] debug func select         */
#define PI_PROBE_SELECT_DS	0x0d400

/*
 * SRAM page accounting for GPON, 128-byte pages, no DRAM reservation.
 * US descriptor ring = 128 pages, DS = 32 pages; the registers hold count-1.
 */
#define PI_US_SRAM_NO		127u		/* 128 pages - 1               */
#define PI_DS_SRAM_NO		31u		/* 32 pages - 1                */
#define PI_US_SRAM_RUNOUT	126u		/* SRAM_NO - 1                 */
#define PI_DS_SRAM_RUNOUT	30u

/* ONU activation FSM states (HW encodes the G.984.3 O-states directly). */
static const char * const gpon_onu_state_name[] = {
	[0] = "unknown",   [1] = "O1-initial",  [2] = "O2-standby",
	[3] = "O3-serial", [4] = "O4-ranging",  [5] = "O5-operation",
	[6] = "O6-popup",  [7] = "O7-emergency",
};

static void __iomem *gpon_base;
static void __iomem *swcore_base;
static void __iomem *ponip_base;

/* PLOAM activation FSM state (the FSM itself is defined below the proc dump).
 * onu_sn is a placeholder default; the real per-ONU serial number is provisioned
 * at runtime (onu_sn= module param, or the userspace provisioning service that
 * reads it from the board's factory configuration). */
static char *onu_sn = "XPON00000000";
module_param(onu_sn, charp, 0444);
MODULE_PARM_DESC(onu_sn, "ONU serial number (G.984.3 ONU-SN): 4 ASCII ID chars + 8 hex digits");
/* Diagnostic: skip BOSA cold-init so that, on a warm boot where the BOSA is
 * already in a working state, the SoC datapath/FSM runs on top of it. */
static bool skip_bosa;
module_param(skip_bosa, bool, 0444);
MODULE_PARM_DESC(skip_bosa, "leave external BOSA as-is (warm-boot bisection)");
/* Open the DS GEM unicast/broadcast pass gate (GEM_DS_MC_CFG). Default OFF: without
 * the PON-IP->GMAC-NIC OMCI drain, opening it backs up the DS path and stalls the US
 * (deactivate ~48s). Set =1 only when drain-path testing. */
static bool gem_gate_open;		/* default OFF = stable online; the downstream OMCI-to-host path is still being brought up */
module_param(gem_gate_open, bool, 0444);
MODULE_PARM_DESC(gem_gate_open, "open DS GEM pass gate (needs the PON-IP->host OMCI drain; default off = stable online)");
/* DIAGNOSTIC: force the upstream laser continuously on (US_CFG.FS_LON). Tests
 * whether the SoC SerDes-TX can drive the BOSA at all, independent of the GTC
 * burst scheduler. DEV-ONLY — continuous light jams a multi-ONU PON. */
static bool force_laser;
module_param(force_laser, bool, 0444);
MODULE_PARM_DESC(force_laser, "force US laser CW on (US_CFG FS_LON) — SerDes-TX emission diagnostic");
/* DIAGNOSTIC: skip BOSA TX power-on + APC ignition (keep RX golden / bosa_rx_enable)
 * to isolate whether laser emission is what destabilises the downstream framer
 * lock. Set true ONLY for the laser-vs-DS-RX bisection; normal operation = false. */
static bool laser_off;		/* default false; set via gpon.laser_off=1 for the isolation test */
module_param(laser_off, bool, 0444);
MODULE_PARM_DESC(laser_off, "skip laser TX-enable+APC (DS-RX-vs-laser isolation: laser-on deafens DS RX)");
/*
 * DEFAULT TRUE = THE RANGING FIX. Skip my clean-room APC ignition
 * (bosa_apc_calibrate: W77 handshake / FSU / BOOSTER / EN_L / DCL) and rely on
 * the A4 register image that bosa_tx_enable loads (0x200-0x27c), which already
 * configures the RTL8290B laser for correct BURST operation. apc_calibrate was
 * forcing the laser into a continuous-emission state that DEAFENED the shared-
 * BOSA downstream RX (gtc_ds_sts=0x0b LOS+LOF, optic_los=1, ds_rx frozen) — the
 * whole multi-session "OLT never ranges us" wall. With apc_off the ONU reaches
 * O5: DS RX locks (gtc_ds_sts=0x04, ds_rx climbs), the OLT sends Assign_ONU-ID +
 * Ranging_Time, FSM O1..O5. Set gpon.apc_off=0 only to revisit the (harmful)
 * ignition path. See bisection: laser_off (skip both) vs apc_off (skip only APC).
 */
static bool apc_off = true;
module_param(apc_off, bool, 0444);
MODULE_PARM_DESC(apc_off, "skip bosa_apc_calibrate (default 1 = ranging fix: A4 image alone bursts; apc_calibrate deafens RX)");
static u8 gpon_sn_bytes[8];		/* G.984.3 ONU-SN: 4-byte ID + 4-byte serial */
static struct timer_list gpon_fsm_timer;
static u8 gpon_fsm_state = 1;		/* O1 */
static u8 gpon_fsm_onu_id = 0xff;

/*
 * Last upstream-burst-overhead parameters the OLT dictated. guard/ptn/delim
 * come from Upstream_Overhead (PLOAM 0x01); t3pre is the Type-3 pre-ranged
 * preamble length from Extended_Burst_Length (PLOAM 0x14). Both PLOAMs arrive
 * independently and are broadcast repeatedly, so retain them and recompute the
 * BOH from whichever arrived (gpon_apply_boh).
 */
static u8 gpon_boh_guard;			/* Upstream_Overhead d[0]   = guard bits */
static u8 gpon_boh_ptn = 0xaa;			/* Upstream_Overhead d[3]   = Type-3 pattern */
static u8 gpon_boh_delim[3] = { 0xab, 0x59, 0x83 };	/* Upstream_Overhead d[4..6] */
static u8 gpon_boh_t3pre;			/* Extended_Burst_Length d[0] = Type-3 pre-ranged len */
static u8 gpon_boh_t3ranged;			/* Extended_Burst_Length d[1] = Type-3 ranged len */
static u32 gpon_fsm_sn_tx;
static u32 gpon_fsm_ticks;
static u8 gpon_sds_synced;	/* one-shot SDS TX re-sync done */
static u32 gpon_ds_rx;		/* total downstream PLOAMs drained (DS-lock liveness) */
static bool gpon_omcc_installed;	/* OMCC GEM datapath installed (one-shot, on Configure_Port-ID) */
static bool gpon_tcont_installed;	/* OMCC T-CONT/alloc-id bound (one-shot, on Assign_Alloc-ID) */

static inline u32 gpon_rd(u32 off) { return ioread32(gpon_base + off); }
static inline void gpon_wr(u32 off, u32 v) { iowrite32(v, gpon_base + off); }

static inline u32 sw_rd(u32 off) { return ioread32(swcore_base + off); }
static inline void sw_wr(u32 off, u32 v) { iowrite32(v, swcore_base + off); }

static inline u32 pi_rd(u32 off) { return ioread32(ponip_base + off); }
static inline void pi_wr(u32 off, u32 v) { iowrite32(v, ponip_base + off); }

/* Read-modify-write the bit-field [msb:lsb] of the SWCORE register at off. */
static void sw_field(u32 off, unsigned int msb, unsigned int lsb, u32 val)
{
	u32 mask = (msb - lsb == 31) ? 0xffffffffu
				     : (((1u << (msb - lsb + 1)) - 1) << lsb);

	sw_wr(off, (sw_rd(off) & ~mask) | ((val << lsb) & mask));
}

/* Read-modify-write the bit-field [msb:lsb] of the PON-IP register at off. */
static void pi_field(u32 off, unsigned int msb, unsigned int lsb, u32 val)
{
	u32 mask = (msb - lsb == 31) ? 0xffffffffu
				     : (((1u << (msb - lsb + 1)) - 1) << lsb);

	pi_wr(off, (pi_rd(off) & ~mask) | ((val << lsb) & mask));
}

/* Read-modify-write the bit-field [msb:lsb] of the GPON-block register at off. */
static void gpon_field(u32 off, unsigned int msb, unsigned int lsb, u32 val)
{
	u32 mask = (msb - lsb == 31) ? 0xffffffffu
				     : (((1u << (msb - lsb + 1)) - 1) << lsb);

	gpon_wr(off, (gpon_rd(off) & ~mask) | ((val << lsb) & mask));
}

/* RTL8290B BOSA state captured at probe for /proc display (-1 = not read). */
static int bosa_id_num __ro_after_init = -1;
static int bosa_id_vid __ro_after_init = -1;
static int bosa_w41 __ro_after_init = -1;
static int bosa_ctrl2 __ro_after_init = -1;
static int bosa_status2 __ro_after_init = -1;

/* Map an RTL8290B 12-bit register number to its paging I2C slave address. */
static u8 bosa_slave_for(u16 reg)
{
	switch (reg >> 8) {
	case 0:  return 0x50;
	case 1:  return 0x51;
	case 2:  return 0x54;
	default: return 0x55;		/* page 3 and up */
	}
}

/*
 * Read one 8-bit register from an I2C slave via the SoC hardware I2C master on
 * bus 0. Returns the byte (0..0xff), or negative on NACK/timeout. This is a
 * read-only path — nothing is written to the BOSA — so it is safe to run
 * unconditionally during bring-up.
 */
static int bosa_i2c_read8(u8 slave, u8 reg)
{
	u32 cfg;
	int i;

	/* Route I2C bus 0 to its pads (IO_MODE_EN.I2C_EN[14:13], bit13 = bus0). */
	sw_field(SOC_IO_MODE_EN, 13, 13, 1);

	/* CONFIG: slave addr, 8-bit reg-addr + 8-bit data, ~100 kHz. Preserve the
	 * electrical bits (open-drain / mode / delays) already programmed. */
	cfg = sw_rd(I2C_CONFIG0);
	cfg &= ~((((1u << 7) - 1) << I2C_CFG_DEV_ID_LSB) |
		 (0x3u << I2C_CFG_AW_LSB) | (0x3u << I2C_CFG_DW_LSB) |
		 (0x3ffu << I2C_CFG_CLKDIV_LSB));
	cfg |= ((u32)(slave & 0x7f) << I2C_CFG_DEV_ID_LSB) |
	       (I2C_CLKDIV_100K << I2C_CFG_CLKDIV_LSB);
	sw_wr(I2C_CONFIG0, cfg);

	sw_wr(I2C_IND_ADR, reg);
	sw_wr(I2C_IND_CMD, I2C_CMD_EN);			/* RW_EN=0 -> read */

	for (i = 0; i < I2C_BUSY_POLL_MAX; i++) {
		u32 cmd = sw_rd(I2C_IND_CMD);

		if (!(cmd & I2C_CMD_BUSY)) {
			if (cmd & I2C_CMD_NACK)
				return -EIO;
			return sw_rd(I2C_IND_RD) & 0xff;
		}
		udelay(10);
	}
	return -ETIMEDOUT;
}

/*
 * Probe the external RTL8290B "Europa" BOSA over I2C by reading its chip ID
 * (NUM=0x8290, VID=0x0001). Read-only — this validates the I2C transport
 * end-to-end against known-good values before any RX/signal-detect-enable
 * writes are added. The real optical signal-detect (SDS_FIB_STATUS.SDS_SDET)
 * only asserts once the BOSA RX path is brought up, which is the next step.
 */
static int bosa_read_reg(u16 reg)
{
	return bosa_i2c_read8(bosa_slave_for(reg), reg & 0xff);
}

/*
 * Write one 8-bit register to an I2C slave via the SoC HW I2C master (bus 0).
 * Returns 0 on success, negative on NACK/timeout. Same indirect kick as the
 * read, but with RW_EN set and the data staged in I2C_IND_WD.
 *
 * NB: not __init — the laser-maintenance work (bosa_maint) re-runs the BOSA
 * write path continuously at runtime to service TX faults, so the whole write
 * helper family below must survive past boot.
 */
static int bosa_i2c_write8(u8 slave, u8 reg, u8 val)
{
	u32 cfg;
	int i;

	sw_field(SOC_IO_MODE_EN, 13, 13, 1);

	cfg = sw_rd(I2C_CONFIG0);
	cfg &= ~((((1u << 7) - 1) << I2C_CFG_DEV_ID_LSB) |
		 (0x3u << I2C_CFG_AW_LSB) | (0x3u << I2C_CFG_DW_LSB) |
		 (0x3ffu << I2C_CFG_CLKDIV_LSB));
	cfg |= ((u32)(slave & 0x7f) << I2C_CFG_DEV_ID_LSB) |
	       (I2C_CLKDIV_100K << I2C_CFG_CLKDIV_LSB);
	sw_wr(I2C_CONFIG0, cfg);

	sw_wr(I2C_IND_ADR, reg);
	sw_wr(I2C_IND_WD, val);
	sw_wr(I2C_IND_CMD, I2C_CMD_EN | I2C_CMD_RW_WR);

	for (i = 0; i < I2C_BUSY_POLL_MAX; i++) {
		u32 cmd = sw_rd(I2C_IND_CMD);

		if (!(cmd & I2C_CMD_BUSY))
			return (cmd & I2C_CMD_NACK) ? -EIO : 0;
		udelay(10);
	}
	return -ETIMEDOUT;
}

static int bosa_write_reg(u16 reg, u8 val)
{
	return bosa_i2c_write8(bosa_slave_for(reg), reg & 0xff, val);
}

/* Single-bit read-modify-write of a BOSA register. */
static void bosa_set_bit(u16 reg, u8 bit, int set)
{
	int r = bosa_read_reg(reg);

	if (r < 0)
		return;
	if (set)
		r |= (1u << bit);
	else
		r &= ~(1u << bit);
	bosa_write_reg(reg, r);
}

/* Masked field read-modify-write: val is the field value, placed at the mask's
 * low bit. */
static void bosa_set_field(u16 reg, u8 mask, u8 val)
{
	int r = bosa_read_reg(reg);
	u8 shift;

	if (r < 0 || !mask)
		return;
	shift = __ffs(mask);
	bosa_write_reg(reg, (r & ~mask) | ((val << shift) & mask));
}

/* Read a masked field, right-justified. Returns 0 on I2C error. */
static u8 bosa_get_field(u16 reg, u8 mask)
{
	int r = bosa_read_reg(reg);

	if (r < 0 || !mask)
		return 0;
	return (r & mask) >> __ffs(mask);
}

/* Bounded poll of a BOSA status bit. Returns 1 if the bit reached @want before
 * the cap, 0 on timeout. @us is the per-iteration delay. */
static int bosa_poll_bit(u16 reg, u8 bit, int want, unsigned int us, int cap)
{
	int i;

	for (i = 0; i < cap; i++) {
		int r = bosa_read_reg(reg);

		if (r >= 0 && !!(r & (1u << bit)) == !!want)
			return 1;
		udelay(us);
	}
	return 0;
}

/*
 * Power up the RTL8290B optical receiver so its signal-detect asserts. On a
 * fresh boot the BOSA leaves the RX amplifier powered down (W41.RXI_PWDN_L=1),
 * so the SoC SerDes sees no signal-detect (SDS_FIB_STATUS.SDS_SDET=0) and the
 * GPON framer can never lock. Clearing RXI_PWDN_L (the only RX-path gate that
 * differs from a working unit; SD-pin tristate is already cleared) turns the
 * receiver on. This touches only the RX enable — not the laser/APC TX path.
 * Read-modify-write so the chip's other W41 calibration bits are preserved.
 */
/*
 * RTL8290B RX-path operating configuration written over the I2C master. These
 * are the steady-state values a registered ONU runs; applying them brings the
 * optical receiver (RX amplifier, signal-detect comparator reference, APD bias)
 * to the operating point at which the real signal-detect asserts
 * (SDS_FIB_STATUS.SDS_SDET). All page-2 (I2C slave 0x54) registers. The APD bias
 * here (REG 0x264 = 0x43) is the device's specified operating value — within the
 * receiver's rated range, no over-bias risk. Values are register facts.
 */
static const struct { u16 reg; u8 val; } bosa_rx_golden[] __initconst = {
	{ 0x204, 0x8e },	/* W4  booster/SS clock         */
	{ 0x223, 0x08 },	/* W35 RX DAC low               */
	{ 0x224, 0xba },	/* W36 RX DAC high              */
	{ 0x226, 0xd2 },	/* W38 RX mode/swing            */
	{ 0x227, 0xa7 },	/* W39 RX-LOS reference DAC     */
	{ 0x228, 0x63 },	/* W40 RX bias                  */
	{ 0x229, 0x2b },	/* W41 RX power (RXI_PWDN_L=0)  */
	{ 0x22a, 0xe4 },	/* W42 RX gain/impedance        */
	{ 0x22b, 0x00 },	/* W43 RX hysteresis            */
	{ 0x231, 0xac },	/* W49 RX/TX path config        */
	{ 0x254, 0x4d },	/* CONTROL2 (SD/LOS pin ctrl)   */
	{ 0x264, 0x43 },	/* APD bias DAC (operating value) */
	{ 0x269, 0x08 },	/* RX_TH LOS assert threshold   */
	{ 0x26a, 0x10 },	/* RX_DE_TH LOS de-assert       */
};

static void __init bosa_rx_enable(void)
{
	int i, sdet;

	/* Apply the RX operating point to the BOSA. */
	for (i = 0; i < ARRAY_SIZE(bosa_rx_golden); i++)
		bosa_write_reg(bosa_rx_golden[i].reg, bosa_rx_golden[i].val);
	mdelay(50);					/* RX amp + SD comparator settle */

	/* Re-read so /proc shows the post-config state. */
	bosa_w41     = bosa_read_reg(BOSA_REG_W41);
	bosa_ctrl2   = bosa_read_reg(BOSA_REG_CONTROL2);
	bosa_status2 = bosa_read_reg(BOSA_REG_STATUS2);
	sdet = !!(sw_rd(SDS_FIB_STATUS) & SDS_FIB_SDS_SDET);
	pr_info("rtl9602c-gpon: BOSA RX config applied: w4=0x%02x w41=0x%02x ctrl2=0x%02x status2=0x%02x apd=0x%02x w39=0x%02x sds_sdet=%d\n",
		bosa_read_reg(BOSA_REG_W4) & 0xff, bosa_w41 & 0xff,
		bosa_ctrl2 & 0xff, bosa_status2 & 0xff,
		bosa_read_reg(0x264) & 0xff, bosa_read_reg(0x227) & 0xff, sdet);
}

/*
 * Upstream-laser (TX) operating point — the values a registered (O5) unit runs
 * on this BOSA. The RX table above never touched the laser driver, so without
 * this the ONU receives downstream fine but cannot transmit its upstream PLOAM
 * bursts -> the OLT never hears Serial_Number_ONU and the ONU is stuck in O3.
 * These are the device's specified operating DAC/APC values (within the laser
 * driver's rated bias range -> inherently safe). Order follows the TX-enable
 * flow: bias power -> DAC codes -> DAC/APC power -> fault detect -> TXSD -> enable mode.
 */
static const struct { u16 reg; u8 val; } bosa_tx_golden[] __initconst = {
	{ 0x22e, 0xb0 },	/* W46 TX bias power + APC clocks  */
	{ 0x236, 0x19 },	/* W54 laser BIAS DAC high          */
	{ 0x237, 0x67 },	/* W55 laser MOD DAC high           */
	{ 0x238, 0x22 },	/* W56 BIAS/MOD DAC low bits        */
	{ 0x239, 0x2d },	/* W57 APCDIG bias DAC power        */
	{ 0x235, 0xcf },	/* W53 TX/APC fault detection       */
	{ 0x23c, 0x03 },	/* W60 TIA power config             */
	{ 0x284, 0xf2 },	/* W88 DSR TX APC set-point         */
	{ 0x27c, 0xe9 },	/* W80 TX backup/state             */
	{ 0x230, 0x0e },	/* W48 TX_ENMODE (enable, last)    */
};

/*
 * BOSA base/control config (page0 slave 0x50 + page3 slave 0x55) — the values a
 * registered (O5) unit runs. An earlier revision wrote only the page2 RX/TX
 * registers, so the BOSA control page (clocks / power / APC-digital enables) was
 * left at power-on defaults — the APC digital block never clocked (R30-R33 read
 * 0, laser dark). The 0xff entries (0x03/04/05/08) are master enable masks.
 * Values are register facts required for the control page to clock.
 */
static const struct { u16 reg; u8 val; } bosa_init_golden[] __initconst = {
	{0x000,0x02}, {0x001,0x04}, {0x002,0x0b}, {0x003,0xff}, {0x004,0xff}, {0x005,0xff},
	{0x006,0xff}, {0x007,0xff}, {0x008,0xff}, {0x009,0xff}, {0x00a,0xff}, {0x00b,0x03},
	{0x00c,0x0c}, {0x00d,0x00}, {0x00e,0x14}, {0x00f,0xc8}, {0x010,0x00}, {0x011,0x00},
	{0x012,0x00}, {0x013,0x00}, {0x014,0x52}, {0x015,0x45}, {0x016,0x41}, {0x017,0x4c},
	{0x018,0x54}, {0x019,0x45}, {0x01a,0x4b}, {0x01b,0x20}, {0x01c,0x20}, {0x01d,0x20},
	{0x01e,0x20}, {0x01f,0x20}, {0x020,0x20}, {0x021,0x20}, {0x022,0x20}, {0x023,0x20},
	{0x024,0x00}, {0x025,0x00}, {0x026,0x00}, {0x027,0x00}, {0x028,0x52}, {0x029,0x54},
	{0x02a,0x4c}, {0x02b,0x38}, {0x02c,0x32}, {0x02d,0x39}, {0x02e,0x30}, {0x02f,0x20},
	{0x030,0x20}, {0x031,0x20}, {0x032,0x20}, {0x033,0x20}, {0x034,0x20}, {0x035,0x20},
	{0x036,0x20}, {0x037,0x20}, {0x038,0x30}, {0x039,0x30}, {0x03a,0x30}, {0x03b,0x31},
	{0x03c,0x05}, {0x03d,0x1e}, {0x03e,0x00}, {0x03f,0xff}, {0x040,0x00}, {0x041,0x20},
	{0x042,0x00}, {0x043,0x00}, {0x044,0x76}, {0x045,0x65}, {0x046,0x6e}, {0x047,0x64},
	{0x048,0x6f}, {0x049,0x72}, {0x04a,0x70}, {0x04b,0x78}, {0x04c,0x72}, {0x04d,0x74},
	{0x04e,0x6e}, {0x04f,0x22}, {0x050,0x6d}, {0x051,0x62}, {0x052,0x65}, {0x053,0x72},
	{0x054,0x32}, {0x055,0x30}, {0x056,0x31}, {0x057,0x34}, {0x058,0x30}, {0x059,0x31},
	{0x05a,0x32}, {0x05b,0x33}, {0x05c,0x68}, {0x05d,0x80}, {0x05e,0x02}, {0x05f,0xff},
	{0x060,0xff}, {0x061,0xff}, {0x062,0xff}, {0x063,0xff}, {0x064,0xff}, {0x065,0xff},
	{0x066,0xff}, {0x067,0xff}, {0x068,0xff}, {0x069,0xff}, {0x06a,0xff}, {0x06b,0xff},
	{0x06c,0xff}, {0x06d,0xff}, {0x06e,0xff}, {0x06f,0xff}, {0x070,0xff}, {0x071,0xff},
	{0x072,0xff}, {0x073,0xff}, {0x074,0xff}, {0x075,0xff}, {0x076,0xff}, {0x077,0xff},
	{0x078,0xff}, {0x079,0xff}, {0x07a,0xff}, {0x07b,0xff}, {0x07c,0xff}, {0x07d,0xff},
	{0x07e,0xff}, {0x07f,0xff}, {0x080,0xff}, {0x081,0x10}, {0x082,0xff}, {0x083,0xff},
	{0x084,0xff}, {0x085,0xff}, {0x086,0xff}, {0x087,0xfd}, {0x088,0xff}, {0x089,0xff},
	{0x08a,0x54}, {0x08b,0xff}, {0x08c,0xff}, {0x08d,0xff}, {0x08e,0xff}, {0x08f,0xff},
	{0x090,0xff}, {0x091,0xff}, {0x092,0xff}, {0x093,0xff}, {0x094,0xff}, {0x095,0xff},
	{0x096,0xff}, {0x097,0xff}, {0x098,0xff}, {0x099,0xff}, {0x09a,0xff}, {0x09b,0xff},
	{0x09c,0xff}, {0x09d,0xff}, {0x09e,0xff}, {0x09f,0xff}, {0x0a0,0xff}, {0x0a1,0xff},
	{0x0a2,0xff}, {0x0a3,0xff}, {0x0a4,0xff}, {0x0a5,0xff}, {0x0a6,0xff}, {0x0a7,0xff},
	{0x0a8,0xff}, {0x0a9,0xff}, {0x0aa,0xff}, {0x0ab,0xff}, {0x0ac,0xff}, {0x0ad,0xff},
	{0x0ae,0xff}, {0x0af,0xfd}, {0x0b0,0xff}, {0x0b1,0xff}, {0x0b2,0x78}, {0x0b3,0xff},
	{0x0b4,0xff}, {0x0b5,0x4f}, {0x0b6,0xff}, {0x0b7,0xff}, {0x0b8,0xff}, {0x0b9,0xff},
	{0x0ba,0xff}, {0x0bb,0xff}, {0x0bc,0xff}, {0x0bd,0xff}, {0x0be,0xff}, {0x0bf,0xff},
	{0x0c0,0xff}, {0x0c1,0xff}, {0x0c2,0xff}, {0x0c3,0xff}, {0x0c4,0xff}, {0x0c5,0xff},
	{0x0c6,0xff}, {0x0c7,0xff}, {0x0c8,0xff}, {0x0c9,0xff}, {0x0ca,0xff}, {0x0cb,0xff},
	{0x0cc,0xff}, {0x0cd,0xff}, {0x0ce,0xff}, {0x0cf,0xff}, {0x0d0,0xff}, {0x0d1,0xff},
	{0x0d2,0xff}, {0x0d3,0xff}, {0x0d4,0x0c}, {0x0d5,0xff}, {0x0d6,0xff}, {0x0d7,0xff},
	{0x0d8,0xff}, {0x0d9,0xff}, {0x0da,0xff}, {0x0db,0xff}, {0x0dc,0xff}, {0x0dd,0xff},
	{0x0de,0xff}, {0x0df,0xff}, {0x0e0,0xff}, {0x0e1,0xff}, {0x0e2,0xff}, {0x0e3,0xff},
	{0x0e4,0xff}, {0x0e5,0xff}, {0x0e6,0xff}, {0x0e7,0xff}, {0x0e8,0xff}, {0x0e9,0xff},
	{0x0ea,0xff}, {0x0eb,0xff}, {0x0ec,0xff}, {0x0ed,0xff}, {0x0ee,0xff}, {0x0ef,0xff},
	{0x0f0,0xff}, {0x0f1,0xff}, {0x0f2,0xff}, {0x0f3,0xff}, {0x0f4,0x6d}, {0x0f5,0xff},
	{0x0f6,0xfc}, {0x0f7,0xff}, {0x0f8,0xff}, {0x0f9,0xff}, {0x0fa,0x70}, {0x0fb,0xff},
	{0x0fc,0xff}, {0x0fd,0xff}, {0x0fe,0xff}, {0x0ff,0xff}, {0x100,0x7f}, {0x101,0xff},
	{0x102,0xff}, {0x103,0xff}, {0x104,0x7f}, {0x105,0xff}, {0x106,0xff}, {0x107,0xff},
	{0x108,0x8e}, {0x109,0x94}, {0x10a,0x6d}, {0x10b,0x60}, {0x10c,0x8c}, {0x10d,0xa0},
	{0x10e,0x75}, {0x10f,0x30}, {0x110,0x75}, {0x111,0x30}, {0x112,0x05}, {0x113,0xdc},
	{0x114,0x61}, {0x115,0xa8}, {0x116,0x07}, {0x117,0xd0}, {0x118,0x00}, {0x119,0x00},
	{0x11a,0x0f}, {0x11b,0x8d}, {0x11c,0x00}, {0x11d,0x0a}, {0x11e,0x0c}, {0x11f,0x5a},
	{0x120,0x00}, {0x121,0x0c}, {0x122,0x00}, {0x123,0x00}, {0x124,0x00}, {0x125,0x00},
	{0x126,0x00}, {0x127,0x00}, {0x128,0x00}, {0x129,0x00}, {0x12a,0x00}, {0x12b,0x00},
	{0x12c,0x00}, {0x12d,0x00}, {0x12e,0x00}, {0x12f,0x00}, {0x130,0x00}, {0x131,0x00},
	{0x132,0x00}, {0x133,0x00}, {0x134,0x00}, {0x135,0x00}, {0x136,0x00}, {0x137,0x00},
	{0x138,0x00}, {0x139,0x00}, {0x13a,0x00}, {0x13b,0x00}, {0x13c,0x00}, {0x13d,0x00},
	{0x13e,0x3f}, {0x13f,0x80}, {0x140,0x00}, {0x141,0x00}, {0x142,0x00}, {0x143,0x00},
	{0x144,0x00}, {0x145,0x00}, {0x146,0x01}, {0x147,0x00}, {0x148,0x00}, {0x149,0x00},
	{0x14a,0x01}, {0x14b,0x00}, {0x14c,0x00}, {0x14d,0x00}, {0x14e,0x01}, {0x14f,0x00},
	{0x150,0x00}, {0x151,0x00}, {0x152,0x01}, {0x153,0x00}, {0x154,0x00}, {0x155,0x00},
	{0x156,0x00}, {0x157,0x00}, {0x158,0x00}, {0x159,0xff}, {0x15a,0xff}, {0x15b,0xff},
	{0x15c,0xff}, {0x15d,0xff}, {0x15e,0xff}, {0x15f,0xff}, {0x160,0x2c}, {0x161,0x38},
	{0x162,0x84}, {0x163,0x98}, {0x164,0x18}, {0x165,0x14}, {0x166,0x3e}, {0x167,0x52},
	{0x168,0x00}, {0x169,0xe5}, {0x16a,0xff}, {0x16b,0x00}, {0x16c,0x00}, {0x16d,0x00},
	{0x16e,0xff}, {0x16f,0x00}, {0x170,0x03}, {0x171,0xc0}, {0x172,0x00}, {0x173,0x00},
	{0x174,0x03}, {0x175,0xc0}, {0x176,0x00}, {0x177,0x00}, {0x178,0x00}, {0x179,0x00},
	{0x17a,0x00}, {0x17b,0x00}, {0x17c,0x00}, {0x17d,0x00}, {0x17e,0x00}, {0x17f,0x00},
	{0x180,0x73}, {0x181,0x11}, {0x182,0x7d}, {0x183,0x39}, {0x184,0x6d}, {0x185,0xdb},
	{0x186,0xf9}, {0x187,0x54}, {0x188,0xc3}, {0x189,0xc0}, {0x18a,0x59}, {0x18b,0x1a},
	{0x18c,0x4b}, {0x18d,0xe3}, {0x18e,0xfb}, {0x18f,0x94}, {0x190,0x0c}, {0x191,0xa4},
	{0x192,0x16}, {0x193,0xca}, {0x194,0xf0}, {0x195,0x51}, {0x196,0xc1}, {0x197,0xe4},
	{0x198,0x08}, {0x199,0x09}, {0x19a,0x6d}, {0x19b,0x43}, {0x19c,0xe0}, {0x19d,0x63},
	{0x19e,0x65}, {0x19f,0x1d}, {0x1a0,0x89}, {0x1a1,0x53}, {0x1a2,0x54}, {0x1a3,0x23},
	{0x1a4,0x7b}, {0x1a5,0xe5}, {0x1a6,0xda}, {0x1a7,0x2e}, {0x1a8,0x7c}, {0x1a9,0xf5},
	{0x1aa,0x7c}, {0x1ab,0xe7}, {0x1ac,0xce}, {0x1ad,0x2b}, {0x1ae,0xd1}, {0x1af,0x76},
	{0x1b0,0xf8}, {0x1b1,0xdc}, {0x1b2,0x72}, {0x1b3,0x92}, {0x1b4,0x94}, {0x1b5,0x34},
	{0x1b6,0x69}, {0x1b7,0x48}, {0x1b8,0x85}, {0x1b9,0xff}, {0x1ba,0x30}, {0x1bb,0x0c},
	{0x1bc,0x23}, {0x1bd,0xdd}, {0x1be,0x3c}, {0x1bf,0xdc}, {0x1c0,0x53}, {0x1c1,0xd3},
	{0x1c2,0x5d}, {0x1c3,0x5c}, {0x1c4,0xc4}, {0x1c5,0xef}, {0x1c6,0xdb}, {0x1c7,0xe5},
	{0x1c8,0xc8}, {0x1c9,0xff}, {0x1ca,0xdb}, {0x1cb,0x52}, {0x1cc,0x22}, {0x1cd,0x27},
	{0x1ce,0xfd}, {0x1cf,0x37}, {0x1d0,0x3b}, {0x1d1,0x33}, {0x1d2,0xc3}, {0x1d3,0x91},
	{0x1d4,0xa2}, {0x1d5,0x01}, {0x1d6,0xbd}, {0x1d7,0x7c}, {0x1d8,0x4e}, {0x1d9,0xe6},
	{0x1da,0x0d}, {0x1db,0x2d}, {0x1dc,0x2e}, {0x1dd,0x94}, {0x1de,0xa0}, {0x1df,0xeb},
	{0x1e0,0xd9}, {0x1e1,0x31}, {0x1e2,0x39}, {0x1e3,0x91}, {0x1e4,0x68}, {0x1e5,0xf6},
	{0x1e6,0x7b}, {0x1e7,0x4b}, {0x1e8,0x67}, {0x1e9,0xf4}, {0x1ea,0xc7}, {0x1eb,0x36},
	{0x1ec,0xfd}, {0x1ed,0x4d}, {0x1ee,0x86}, {0x1ef,0x76}, {0x1f0,0x36}, {0x1f1,0x2c},
	{0x1f2,0xf3}, {0x1f3,0x2e}, {0x1f4,0x3c}, {0x1f5,0xbd}, {0x1f6,0xb5}, {0x1f7,0x01},
	{0x1f8,0x37}, {0x1f9,0x11}, {0x1fa,0x04}, {0x1fb,0x7b}, {0x1fc,0x9d}, {0x1fd,0x01},
	{0x1fe,0x99}, {0x1ff,0x3a}, {0x200,0x02}, {0x201,0x89}, {0x202,0xa1}, {0x203,0xfe},
	{0x204,0x8e}, {0x205,0xb2}, {0x206,0x9b}, {0x207,0x90}, {0x208,0x00}, {0x209,0x49},
	{0x20a,0x9f}, {0x20b,0xff}, {0x20c,0x23}, {0x20d,0x04}, {0x20e,0x78}, {0x20f,0x7f},
	{0x210,0xff}, {0x211,0x00}, {0x212,0x82}, {0x213,0x05}, {0x214,0x00}, {0x215,0x00},
	{0x216,0x01}, {0x217,0xf6}, {0x218,0xce}, {0x219,0x90}, {0x21a,0xc0}, {0x21b,0x00},
	{0x21c,0x00}, {0x21d,0x38}, {0x21e,0x24}, {0x21f,0x40}, {0x220,0x40}, {0x221,0x00},
	{0x222,0x01}, {0x223,0x08}, {0x224,0xba}, {0x225,0x1e}, {0x226,0xd2}, {0x227,0xa7},
	{0x228,0x63}, {0x229,0x2b}, {0x22a,0xe4}, {0x22b,0x00}, {0x22c,0xe0}, {0x22d,0x01},
	{0x22e,0xb0}, {0x22f,0x44}, {0x230,0x0e}, {0x231,0xac}, {0x232,0x01}, {0x233,0x08},
	{0x234,0x80}, {0x235,0xcf}, {0x236,0x19}, {0x237,0x67}, {0x238,0x22}, {0x239,0x2d},
	{0x23a,0x62}, {0x23b,0xcf}, {0x23c,0x03}, {0x23d,0xa2}, {0x23e,0xfc}, {0x23f,0xfd},
	{0x240,0x02}, {0x241,0x57}, {0x242,0xd0}, {0x243,0x80}, {0x244,0x00}, {0x245,0x00},
	{0x246,0x3f}, {0x247,0xcc}, {0x248,0x4d}, {0x249,0x2a}, {0x24a,0x22}, {0x24b,0x89},
	{0x24c,0x85}, {0x24d,0xb0}, {0x24e,0x80}, {0x24f,0x3f}, {0x250,0x00}, {0x251,0x00},
	{0x252,0x00}, {0x253,0x00}, {0x254,0x4d}, {0x255,0x30}, {0x256,0x00}, {0x257,0xf4},
	{0x258,0x00}, {0x259,0xfe}, {0x25a,0xff}, {0x25b,0x01}, {0x25c,0x00}, {0x25d,0xff},
	{0x25e,0x00}, {0x25f,0x02}, {0x260,0x00}, {0x261,0x03}, {0x262,0xff}, {0x263,0x07},
	{0x264,0x43}, {0x265,0x00}, {0x266,0xa0}, {0x267,0xc0}, {0x268,0x00}, {0x269,0x08},
	{0x26a,0x10}, {0x26b,0xe0}, {0x26c,0xe0}, {0x26d,0xe0}, {0x26e,0xff}, {0x26f,0xf4},
	{0x270,0x84}, {0x271,0x82}, {0x272,0x50}, {0x273,0x00}, {0x274,0xff}, {0x275,0x00},
	{0x276,0x10}, {0x277,0x00}, {0x278,0x00}, {0x279,0xff}, {0x27a,0x00}, {0x27b,0x08},
	{0x27c,0xe9}, {0x27d,0x00}, {0x27e,0x00}, {0x27f,0x00}, {0x280,0x00}, {0x281,0x01},
	{0x282,0x00}, {0x283,0x88}, {0x284,0xf2}, {0x285,0x00}, {0x286,0x00}, {0x287,0x08},
	{0x288,0x00}, {0x289,0x00}, {0x28a,0x00}, {0x28b,0x00}, {0x28c,0x00}, {0x28d,0x00},
	{0x28e,0x00}, {0x28f,0x00}, {0x290,0x00}, {0x291,0x00}, {0x292,0x08}, {0x293,0x00},
	{0x294,0x00}, {0x295,0x00}, {0x296,0x00}, {0x297,0x00}, {0x298,0x00}, {0x299,0x00},
	{0x29a,0x00}, {0x29b,0x00}, {0x29c,0x00}, {0x29d,0x00}, {0x29e,0x00}, {0x29f,0x00},
	{0x2a0,0x00}, {0x2a1,0x00}, {0x2a2,0x00}, {0x2a3,0x00}, {0x2a4,0x00}, {0x2a5,0x00},
	{0x2a6,0x00}, {0x2a7,0x08}, {0x2a8,0x00}, {0x2a9,0x00}, {0x2aa,0x00}, {0x2ab,0x00},
	{0x2ac,0x00}, {0x2ad,0x00}, {0x2ae,0x00}, {0x2af,0x08}, {0x2b0,0x00}, {0x2b1,0x00},
	{0x2b2,0x00}, {0x2b3,0x00}, {0x2b4,0x00}, {0x2b5,0x00}, {0x2b6,0x00}, {0x2b7,0xca},
	{0x2b8,0x00}, {0x2b9,0x00}, {0x2ba,0x00}, {0x2bb,0x00}, {0x2bc,0x00}, {0x2bd,0x00},
	{0x2be,0x00}, {0x2bf,0x00}, {0x2c0,0x00}, {0x2c1,0xfc}, {0x2c2,0x00}, {0x2c3,0x00},
	{0x2c4,0x00}, {0x2c5,0x02}, {0x2c6,0x00}, {0x2c7,0xe3}, {0x2c8,0x00}, {0x2c9,0x00},
	{0x2ca,0x00}, {0x2cb,0x00}, {0x2cc,0x00}, {0x2cd,0x00}, {0x2ce,0x00}, {0x2cf,0x00},
	{0x2d0,0x00}, {0x2d1,0x00}, {0x2d2,0x00}, {0x2d3,0x00}, {0x2d4,0x00}, {0x2d5,0x00},
	{0x2d6,0x00}, {0x2d7,0x00}, {0x2d8,0x00}, {0x2d9,0x00}, {0x2da,0x00}, {0x2db,0x00},
	{0x2dc,0x00}, {0x2dd,0x00}, {0x2de,0x00}, {0x2df,0x00}, {0x2e0,0x00}, {0x2e1,0x00},
	{0x2e2,0x00}, {0x2e3,0x00}, {0x2e4,0x00}, {0x2e5,0x00}, {0x2e6,0x00}, {0x2e7,0x00},
	{0x2e8,0x00}, {0x2e9,0x00}, {0x2ea,0x00}, {0x2eb,0x08}, {0x2ec,0x00}, {0x2ed,0x00},
	{0x2ee,0x00}, {0x2ef,0x00}, {0x2f0,0x00}, {0x2f1,0x00}, {0x2f2,0x00}, {0x2f3,0x08},
	{0x2f4,0x00}, {0x2f5,0x00}, {0x2f6,0x00}, {0x2f7,0x00}, {0x2f8,0x00}, {0x2f9,0x00},
	{0x2fa,0x00}, {0x2fb,0x00}, {0x2fc,0x00}, {0x2fd,0x00}, {0x2fe,0x08}, {0x2ff,0x00},
	{0x300,0xd6}, {0x301,0xca}, {0x302,0xa9}, {0x303,0x08}, {0x304,0xc4}, {0x305,0xe4},
	{0x306,0x78}, {0x307,0x70}, {0x308,0xe5}, {0x309,0xcd}, {0x30a,0xf8}, {0x326,0x00},
	{0x327,0x00}, {0x328,0x00}, {0x329,0x08}, {0x32a,0x00}, {0x32b,0x00}, {0x32c,0x00},
	{0x32d,0x00}, {0x32e,0x00}, {0x32f,0x00}, {0x330,0x00}, {0x331,0x00}, {0x332,0x00},
	{0x333,0x00}, {0x334,0x00}, {0x335,0x00}, {0x336,0x00}, {0x337,0x00}, {0x338,0x00},
	{0x339,0x00}, {0x33a,0x00}, {0x33b,0x02}, {0x33c,0x00}, {0x33d,0x09}, {0x33e,0x00},
	{0x33f,0x00}, {0x340,0x00}, {0x341,0x00}, {0x342,0x00}, {0x343,0x00}, {0x344,0x00},
	{0x345,0x00}, {0x346,0x00}, {0x347,0x00}, {0x348,0x00}, {0x349,0x00}, {0x34a,0x00},
	{0x34b,0x00}, {0x34c,0x00}, {0x34d,0x00}, {0x34e,0x00}, {0x34f,0x00}, {0x350,0x00},
	{0x351,0x00}, {0x352,0x00}, {0x353,0x00}, {0x354,0x00}, {0x355,0x00}, {0x356,0x00},
	{0x357,0x00}, {0x358,0x00}, {0x359,0x00}, {0x35a,0x00}, {0x35b,0x00}, {0x35c,0x00},
	{0x35d,0x00}, {0x35e,0x00}, {0x35f,0x00}, {0x360,0x00}, {0x361,0x00}, {0x362,0x00},
	{0x363,0x00}, {0x364,0x00}, {0x365,0x00}, {0x366,0x00}, {0x367,0x00}, {0x368,0x00},
	{0x369,0x00}, {0x36a,0x00}, {0x36b,0x00}, {0x36c,0x00}, {0x36d,0x00}, {0x36e,0x00},
	{0x36f,0x00}, {0x370,0x00}, {0x371,0x00}, {0x372,0x00}, {0x373,0x00}, {0x374,0x00},
	{0x375,0x00}, {0x376,0x00}, {0x377,0x00}, {0x378,0x00}, {0x379,0x00}, {0x37a,0x00},
	{0x37b,0x00}, {0x37c,0x00}, {0x37d,0x00}, {0x37e,0x00}, {0x37f,0x00}, {0x380,0x01},
	{0x381,0x01}, {0x382,0x04}, {0x38b,0x00}, {0x38c,0x00}, {0x38d,0x21}, {0x38e,0x00},
	{0x38f,0x00}, {0x390,0x82}, {0x391,0x90}, {0x392,0x00}, {0x393,0x00}, {0x394,0x01},
	{0x395,0x00}, {0x396,0x00}, {0x397,0x00}, {0x398,0x00}, {0x399,0x00}, {0x39a,0x00},
	{0x39b,0x15}, {0x39c,0x00}, {0x39d,0x00}, {0x39e,0x00}, {0x3a0,0x00},	/* 0x39f (REG_LENGTH) intentionally not written */
	{0x3a1,0x00}, {0x3a2,0x00}, {0x3a3,0x02}, {0x3a4,0x00}, {0x3a5,0x00}, {0x3a6,0x00},
	{0x3a7,0x00}, {0x3a8,0x00}, {0x3a9,0x00}, {0x3aa,0x00}, {0x3ab,0x00}, {0x3ac,0x00},
	{0x3ad,0x00}, {0x3ae,0x00}, {0x3af,0x00}, {0x3b0,0x00}, {0x3b1,0x00}, {0x3b2,0x00},
	{0x3b3,0xb6}, {0x3b4,0x58}, {0x3b5,0xf8}, {0x3b6,0x01}, {0x3b7,0x00}, {0x3b8,0x00},
	{0x3b9,0x00}, {0x3ba,0x00}, {0x3bb,0x00}, {0x3bc,0x00}, {0x3bd,0x00}, {0x3be,0x00},
	{0x3bf,0x00}, {0x3c0,0x01}, {0x3c1,0xa0}, {0x3c2,0xac}, {0x3c3,0x40}, {0x3c4,0x30},
	{0x3c5,0x00}, {0x3c6,0x00}, {0x3c7,0x00}, {0x3c8,0x00}, {0x3c9,0x00}, {0x3ca,0x00},
	{0x3cb,0x00}, {0x3cc,0x00}, {0x3cd,0x14}, {0x3ce,0x00}, {0x3cf,0x00}, {0x3d0,0x00},
	{0x3d1,0x00}, {0x3d2,0x00}, {0x3d3,0x00}, {0x3d4,0x00}, {0x3d5,0x00}, {0x3d6,0x00},
	{0x3d7,0x00}, {0x3d8,0x00}, {0x3d9,0x00}, {0x3da,0x00}, {0x3db,0x00}, {0x3dc,0x00},
	{0x3dd,0x00}, {0x3de,0x00}, {0x3df,0x00}, {0x3e0,0x00}, {0x3e1,0x00}, {0x3e2,0x00},
	{0x3e3,0x00}, {0x3e4,0x00}, {0x3e5,0x00}, {0x3e6,0x00}, {0x3e7,0x00}, {0x3e8,0x44},
	{0x3e9,0x00}, {0x3ea,0x00}, {0x3eb,0x00}, {0x3ec,0x00}, {0x3ed,0x00}, {0x3ee,0x00},
	{0x3ef,0x00}, {0x3f0,0x00}, {0x3f1,0x08}, {0x3f2,0x00}, {0x3f3,0x00}, {0x3f4,0x00},
	{0x3f5,0x00}, {0x3f6,0x00}, {0x3f7,0x00}, {0x3f8,0x00}, {0x3f9,0x00}, {0x3fa,0x00},
	{0x3fb,0x00}, {0x3fc,0x00}, {0x3fd,0x00}, {0x3fe,0x00}, {0x3ff,0x00}
};

static void __init bosa_tx_enable(void)
{
	int i;

	/* Load the A4 register image (0x200-0x27c) + base/control config. This is the
	 * patch the BOSA core consumes — it is a plain register image, not a strobed
	 * "activation": no patch-length/activate register is written (REG_LENGTH 0x39f
	 * is intentionally left untouched). The actual laser ignition is the MCU-driven
	 * APC power-on flow run later in bosa_apc_calibrate() (after the SerDes/PON-IP
	 * TX clock is up). */
	for (i = 0; i < ARRAY_SIZE(bosa_init_golden); i++)
		bosa_write_reg(bosa_init_golden[i].reg, bosa_init_golden[i].val);
	mdelay(2);
	pr_info("rtl9602c-gpon: A4 image loaded: st1=0x%02x(cksum_err=%d) st2=0x%02x\n",
		bosa_read_reg(0x382) & 0xff,
		!!(bosa_read_reg(0x382) & 0x20), bosa_read_reg(0x383) & 0xff);

	for (i = 0; i < ARRAY_SIZE(bosa_tx_golden); i++)
		bosa_write_reg(bosa_tx_golden[i].reg, bosa_tx_golden[i].val);
	mdelay(10);
	pr_info("rtl9602c-gpon: BOSA TX/laser config applied (bias=0x%02x mod=0x%02x w46=0x%02x w48=0x%02x)\n",
		bosa_read_reg(0x236) & 0xff, bosa_read_reg(0x237) & 0xff,
		bosa_read_reg(0x22e) & 0xff, bosa_read_reg(0x230) & 0xff);
}

/* Set once the cold ignition has run, so the periodic fault-service (driven from
 * the GPON FSM timer) only touches the laser after DIGITAL_POWER_ON. */
static int bosa_laser_up;
static u32 bosa_maint_faults;		/* recovery attempts (rate-limited logging) */
static u32 bosa_stat_ticks;		/* heartbeat counter for the live status log */

/*
 * Laser fault re-arm — the BOSA "light re-arm" path for a recoverable fault:
 *   TX power control      -> CONTROL2 (0x254) bit2 TX_POW_CTL = 1
 *   laser-diode VDD       -> CONTROL2 (0x254) bit3 ENLD_L     = 1
 *   pulse CONTROL3 (0x255) bit1 (UNDER_RX_OVER_POWER_RELEASE): 1 -> 500us -> 0
 * The 1->0 edge on the release strobe clears the latched fault and re-arms the
 * laser. This DELIBERATELY does NOT write 0x399 bit0: that bit is TOTAL_CHIP_RESET
 * (a last-resort path that must then re-apply all RX/TX config) — an earlier
 * version pulsed it on every re-arm, resetting the BOSA and wiping the ignition.
 * No mdelay: callable from the FSM timer
 * (softirq) — only the bounded 500us release strobe runs in-context; recovery is
 * re-checked on the next tick. Never raises bias/mod (laser-safety: the operating
 * point stays clamped to the per-board calibrated LUT).
 */
static void bosa_fault_rearm(void)
{
	bosa_set_bit(0x254, 2, 1);		/* CONTROL2 TX_POW_CTL: re-enable TX drv */
	bosa_set_bit(0x254, 3, 1);		/* CONTROL2 ENLD_L: re-enable laser-diode */
	bosa_set_bit(0x255, 1, 1);		/* CONTROL3 release strobe: assert */
	udelay(500);				/* 500us release-strobe settle */
	bosa_set_bit(0x255, 1, 0);		/* de-assert: 1->500us->0 clears latch */
	bosa_set_field(0x254, 0x80, 0x00);	/* clear soft TX-disable (bit7) -> emit */
}

/*
 * One laser-maintenance pass — a continuous poll of the BOSA INT/fault status
 * (every ~50ms) plus a re-arm when a recoverable fault is seen. The cold
 * ignition is one-shot; a transient TX_FAULT after
 * DIGITAL_POWER_ON (or any later trip) would otherwise leave the BOSA latched
 * with the laser dark forever. This runs from the GPON FSM timer and, whenever
 * the laser is found faulted/disabled, re-arms it. Fault sources reacted to:
 *   STATUS_2 (0x383) b4  FAULT_STATUS  -- live "laser currently disabled" (authoritative)
 *   FAULT_STATUS (0x389) & 0xd1        -- genuine TX-kill: TX_FAULT(b0), TX_LV(b4),
 *                                         OVER_VOL(b6), OVER_TEMP(b7)
 * OVER_IMPD(b5)/MPD_VHIGH are excluded here: the bring-up disarms the MPD
 * high/low HW fault-detect (W53 0x235 bits[1:0]) so they neither latch the laser
 * nor thrash this loop; the bias/mod DACs stay clamped to the calibrated LUT, so
 * the laser cannot physically over-drive even with MPD detect off.
 */
static void bosa_laser_maint(void)
{
	int s2 = bosa_read_reg(0x383);
	int fs = bosa_read_reg(0x389);

	if (s2 < 0 || fs < 0)
		return;				/* I2C glitch — retry next tick */

	/* Live laser-state heartbeat (~every 2.5s) — bring-up visibility into whether
	 * the laser actually emits (mpd != 0) and holds bias once the FSM is in O3 and
	 * bursting upstream. EN_L = W4/0x204 bit4 (laser booster output enable). */
	if ((bosa_stat_ticks++ % 50) == 0)
		pr_info("rtl9602c-gpon: laser stat: 0x383=0x%02x 0x389=0x%02x R30=0x%02x bias=0x%02x mod=0x%02x mpd=%02x/%02x EN_L=%d state=O%u\n",
			s2 & 0xff, fs & 0xff, bosa_read_reg(0x31e) & 0xff,
			bosa_read_reg(0x236) & 0xff, bosa_read_reg(0x237) & 0xff,
			bosa_read_reg(0x320) & 0xff, bosa_read_reg(0x321) & 0xff,
			!!(bosa_read_reg(0x204) & 0x10), gpon_fsm_state);

	if (s2 & BIT(5))
		return;				/* STATUS_2 b5 DEBUG_MODE — BOSA wedged, don't poke */
	if (!(fs & 0xd1))
		return;				/* healthy: react only to genuine 0x389 TX-kill,
					 * not the benign aggregate 0x383 b4 (always set in
					 * the B-flow; re-arming on it tips the BOSA into
					 * DEBUG_MODE). */

	if ((bosa_maint_faults++ % 32) == 0)
		pr_info("rtl9602c-gpon: laser maint re-arm (0x383=0x%02x 0x389=0x%02x R30=0x%02x)\n",
			s2 & 0xff, fs & 0xff, bosa_read_reg(0x31e) & 0xff);
	bosa_fault_rearm();
}

/*
 * Cold laser ignition — the RTL8290B's MCU-driven APC power-on, expressed as the
 * register-level sequence the silicon requires (APC-enable flow then TX-enable
 * flow). The A4 register image (loaded into 0x200-0x27c by bosa_tx_enable) arms
 * the BOSA's on-chip APC core; this routine then runs the ignition the device
 * requires:
 *   MCU power-on gate -> CHECK_READY -> BIAS_POWER_ON -> DIGITAL_POWER_ON ->
 *   enable the hardware APC servo loop (W67/0x243 bit7) -> offset-cal lock loop ->
 *   TX-enable flow.
 * Once the APC loop is enabled the BOSA core servoes bias/modulation autonomously,
 * so there is no software servo here. Every laser-drive value is the device's own
 * ignition limit (bias-max 0x86->0x87, bias-min 0x06, ...) — none is raised.
 *
 * Slave banking (handled by bosa_write_reg): 0x2xx -> I2C slave 0x54 (page 2,
 * analog/APC), 0x3xx -> slave 0x55 (page 3, MCU status/control).
 *
 * MUST run after the SerDes/PON-IP TX clock is up (the APC-digital block is
 * clocked from it) — hence it is deferred until after the GPON MAC reset.
 */
static void __init bosa_apc_calibrate(void)
{
	int i, k, locked = 0;
	u8 v;

	/* APC power setpoints (DCL P0/P1/Pavg) — the laser power TARGET the BOSA
	 * servo regulates toward. These are PER-BOARD values from the optical
	 * calibration data; a wrong target over-drives the laser and trips the
	 * MPD-VHIGH fault. TODO: load from per-board calibration at startup like the
	 * MAC/SN; this is the Board-C calibrated DCL set. Written after the A4 image
	 * (which leaves patch bytes here) and before the ignition. */
	bosa_write_reg(0x23a, 0x26);		/* W58 DCL P0   */
	bosa_write_reg(0x23b, 0x50);		/* W59 DCL P1   */
	bosa_write_reg(0x23d, 0x50);		/* W61 DCL Pavg */

	/* MCU power-on gate: wait for the BOSA core boot/power-on-reset to finish
	 * (STATUS_2 0x383: LVCMP_TX_VALID(7) + TEMP_VALID(6); DEBUG_MODE(5) stays 0 in
	 * normal operation) plus the page-3 reset-done bit; then a 15ms settle. */
	for (i = 0; i < 2000; i++) {
		int s = bosa_read_reg(0x383);

		if (s >= 0 && (s & 0xc0) == 0xc0)
			break;
		udelay(1000);
	}
	if (i == 2000)
		pr_warn("rtl9602c-gpon: BOSA MCU power-on (0x383&0xc0) not ready\n");
	bosa_poll_bit(0x301, 7, 0, 1000, 2000);		/* page-3 reset-done clears */
	mdelay(15);

	/* --- APC-enable flow, step order 0,1,2,3,4,6,5,7 --- */
	bosa_set_bit(0x3c0, 0, 1);			/* idx0: entry reset/disable */

	/* idx1 CHECK_READY: STATUS_1(0x382) bit2 = READY_STATUS */
	if (!bosa_poll_bit(0x382, 2, 1, 1000, 2000))
		pr_warn("rtl9602c-gpon: BOSA APC CHECK_READY (0x382 bit2) timeout\n");

	/* idx2 BIAS_POWER_ON: arm the analog bias front-end + the APC bias targets.
	 * bias-max W72(0x248)=0x86 then 0x87, bias-min W73(0x249)=0x06 are the
	 * ignition ceilings; the servo converges the live bias well below them. */
	bosa_set_field(0x245, 0xff, 0x10);
	bosa_set_field(0x245, 0x0c, 0x00);		/* W6932 field (default 0) */
	bosa_write_reg(0x284, 0x01);
	bosa_write_reg(0x27c, 0x08);
	bosa_write_reg(0x247, 0x05);
	bosa_write_reg(0x248, 0x86);			/* W72 bias-max */
	bosa_write_reg(0x239, 0xfc);
	bosa_set_bit(0x24a, 3, 1);
	bosa_write_reg(0x249, 0x06);			/* W73 bias-min */
	bosa_write_reg(0x24c, 0x71);
	bosa_write_reg(0x24c, 0x72);
	bosa_write_reg(0x247, 0x06);
	bosa_write_reg(0x248, 0x87);			/* W72 bias-max final */
	/* loop_mode 0 (W69=0x00): 0x23e source = !(0x239 bit3) */
	v = bosa_get_field(0x239, BIT(3)) ^ 1;
	bosa_set_field(0x23e, 0xff, v);
	bosa_write_reg(0x232, 0x07);
	bosa_write_reg(0x244, 0xf8);
	bosa_set_bit(0x252, 3, 1);			/* W82 DAC/loop commit */

	bosa_set_field(0x239, 0xff, 0xfc);		/* idx3 */
	bosa_set_field(0x23c, 0xff, 0xfd);		/* idx4 */

	/* RTL8290B MCU bias/mod-MAX loadin handshake (the APC-init W77 sequence).
	 * Board C's BOSA is an RTL8290B whose on-chip 8051 MCU OWNS laser-enable + bias —
	 * the ignition must drive it through power-on by writing command bytes to W77/0x24d
	 * (each + ~10ms settle + an R29/0x31d status read the MCU consumes). An ignition
	 * path that omits this handshake leaves the MCU with the laser never enabled
	 * (EN_L/bias=0). The bytes strobe BIAS_MAX_EN/LOADIN (b7/b6) + MOD_MAX_EN/LOADIN
	 * (b5/b4) to latch the bias/mod max limits set just above. */
	{
		static const u8 w77_a[] = { 0xa8, 0xb0, 0xd0, 0xd8, 0xe8, 0xe0 };
		static const u8 w77_b[] = { 0xb0, 0xd0, 0xb8, 0xb0, 0xd0, 0xc0 };
		int j;

		bosa_set_bit(0x24e, 7, 1);		/* W78 b7 (apc_init prefix) */
		for (j = 0; j < ARRAY_SIZE(w77_a); j++) {
			bosa_write_reg(0x24d, w77_a[j]);	/* W77 MCU command */
			mdelay(10);
			bosa_read_reg(0x31d);		/* R29 status (MCU consumes) */
		}
		bosa_set_bit(0x243, 7, 1);		/* W67 b7 */
		bosa_set_field(0x27c, 0x08, 0x00);	/* W80 clear bit3 */
		for (j = 0; j < ARRAY_SIZE(w77_b); j++) {
			bosa_write_reg(0x24d, w77_b[j]);
			mdelay(10);
			bosa_read_reg(0x31d);
		}
		pr_info("rtl9602c-gpon: DBG post-W77hs: EN_L=%d bias=0x%02x R29=0x%02x R33=0x%02x 0x383=0x%02x R30=0x%02x\n",
			!!(bosa_read_reg(0x204) & 0x10), bosa_read_reg(0x236) & 0xff,
			bosa_read_reg(0x31d) & 0xff, bosa_read_reg(0x321) & 0xff,
			bosa_read_reg(0x383) & 0xff, bosa_read_reg(0x31e) & 0xff);
	}

	v = bosa_get_field(0x31f, 0x03);		/* idx6 */
	bosa_set_field(0x232, 0xc0, v);
	bosa_set_field(0x249, 0x18, v);

	/* TEST: pre-load this board's calibrated laser bias/mod (per-board optical
	 * calibration LUT @25C = 0x18/0x34, 12-bit DAC = byte<<4) BEFORE turning the
	 * laser on, so it ignites at the right optical power instead of the hotter
	 * default that trips MPD_VHIGH at DIGITAL_POWER_ON. */
	bosa_set_bit(0x23d, 7, 0);
	bosa_set_field(0x236, 0xff, 0x18);		/* bias hi-8 (calibrated LUT @25C). NOTE: lowering to 0x0a did NOT save DS RX — laser-on deafens RX independent of optical power; the fix is burst-gating, not bias level. */
	bosa_set_field(0x238, 0x0f, 0x00);
	bosa_set_bit(0x23d, 7, 1);
	bosa_set_bit(0x23d, 7, 0);
	bosa_set_field(0x237, 0xff, 0x34);		/* mod hi-8 */
	bosa_set_field(0x238, 0xf0, 0x00);
	bosa_set_bit(0x23d, 7, 1);

	/* Disarm W53/0x235 fault-detect for the rest of ignition (after the W77
	 * handshake — disarming it BEFORE the handshake regressed O3->O1). Keeps a
	 * (false) MPD_VHIGH from latching 0x383 b4 at DPO and zeroing the bias; the safe
	 * subset is re-armed at txEnableFlow end. */
	bosa_set_field(0x235, 0xff, 0x00);

	/* idx5 DIGITAL_POWER_ON: turn on the digital/laser power, then blind settle */
	bosa_set_bit(0x27c, 4, 1);			/* W80 bit4 = 1 */
	bosa_set_bit(0x380, 0, 1);
	mdelay(101);
	pr_info("rtl9602c-gpon: DBG post-DPO: bias(0x236)=0x%02x 0x389=0x%02x 0x383=0x%02x R30=0x%02x\n",
		bosa_read_reg(0x236) & 0xff, bosa_read_reg(0x389) & 0xff,
		bosa_read_reg(0x383) & 0xff, bosa_read_reg(0x31e) & 0xff);

	bosa_set_bit(0x23c, 0, 1);			/* idx7 */
	bosa_set_bit(0x254, 3, 1);			/* CONTROL2 bit3 = 1 */
	mdelay(5);

	/* HYPOTHESIS TEST: disable the laser TX (CONTROL2/0x254 bit7=1) during the
	 * offset-K calibration so the ADC zero-offset is measured with no emission and
	 * the TX path can't trip TX_FAULT while the analog block is mid-cal. The
	 * txEnableFlow below re-enables TX (0x254 bit7=0). */
	bosa_set_bit(0x254, 7, 1);

	/* RTL8290B FSU (Field Setup Unit) offset/gain auto-cal + DCL convergence — the
	 * RTL8290B offset cal. (A plain offset-K that polls R30 b7 OFFK_DONE never
	 * completes on this part.) The W77 handshake above started the MCU biasing
	 * (R33 0->0x0a), but the APC must CONVERGE here before TX is enabled in the
	 * TX-enable flow, or it collapses the bias to 0 ("thrashing without
	 * feedback"). Flow (FSU enable + FSU-done check):
	 * select DCL closed-loop mode (W80/0x27c[7:6]=3); arm the FSU (W80 b5 low, b4
	 * high, W14/0x20e b7 high, W80 b5 high = path strobe); fsuMode 0 (W65/0x241 b6);
	 * arm the done check (W77/0x24d=0xB0) and poll the FSU done nibble on R29/0x31d
	 * (& 0x3c == 0x3c) — NOT R30 OFFK_DONE; latch (W14 b7 low, W80 b4 low). */
	bosa_set_field(0x27c, 0xc0, 0x03);	/* apcLoopMode DCL: W80[7:6]=3 */
	bosa_set_bit(0x27c, 5, 0);		/* FSU arm: W80 b5 low */
	bosa_set_bit(0x27c, 4, 1);		/*          W80 b4 high */
	bosa_set_bit(0x20e, 7, 1);		/*          W14 b7 high (LOADIN) */
	bosa_set_bit(0x27c, 5, 1);		/*          W80 b5 high (path strobe) */
	bosa_set_bit(0x241, 6, 0);		/* fsuMode 0: W65 b6 */
	bosa_write_reg(0x24d, 0xb0);		/* W77=0xB0: BIAS_MAX_EN|MOD_MAX_EN arm done-check */
	for (k = 0; k < 250; k++) {
		int r29 = bosa_read_reg(0x31d);

		if (r29 >= 0 && (r29 & 0x3c) == 0x3c) {
			locked = 1;
			break;
		}
		udelay(200);
	}
	bosa_set_bit(0x20e, 7, 0);		/* finalize: de-assert W14 LOADIN */
	bosa_set_bit(0x27c, 4, 0);		/*           de-assert W80 b4 -> latch */
	pr_info("rtl9602c-gpon: DBG post-FSU: done=%d R29=0x%02x bias=0x%02x R33=0x%02x 0x383=0x%02x 0x27c=0x%02x\n",
		locked, bosa_read_reg(0x31d) & 0xff, bosa_read_reg(0x236) & 0xff,
		bosa_read_reg(0x321) & 0xff, bosa_read_reg(0x383) & 0xff,
		bosa_read_reg(0x27c) & 0xff);

	/* --- txEnableFlow (B-flow: laser output enable AFTER FSU convergence) ---
	 * NB: an earlier TX-enable flow rewrote W77/0x24d (=0xa5, then low-nibble=1),
	 * which CORRUPTED the converged MCU command state and tipped the BOSA into
	 * DEBUG_MODE (every reg 0x20) once TX was enabled. Those W77 writes are removed;
	 * the FSU/DCL above already owns W77. */
	bosa_set_field(0x254, 0xff, 0x8d);

	/* idx2 laser bias/mod LUT — the PER-BOARD calibrated operating point. The
	 * optical calibration holds a 151-entry {bias,mod} table indexed by
	 * temperature (stride 2, idx = temp_code - 233, i.e. -40..110C). The bias/mod
	 * are 12-bit DACs = (LUT byte << 4): bias -> 0x236 hi-8 / 0x238[3:0], mod ->
	 * 0x237 hi-8 / 0x238[7:4], each committed by the 0x23d bit7 strobe (0->1).
	 * Without this the DACs sit hotter than THIS laser's calibration -> the part
	 * emits above its monitor-photodiode high threshold -> R30 APC_FAULT_MPD_VHIGH
	 * -> the BOSA safety shuts the laser off. Loading the calibrated point lets the
	 * laser ignite at the right optical power and the APC servo hold it (steady
	 * state = MPD_VLOW).
	 *
	 * This is the Board-C room-temp entry (idx65 / 25C); the neighbourhood
	 * idx60..69 is flat so this is robust ~20-29C. PER-BOARD: load from the
	 * device's calibration data at startup (like MAC/SN) for the fleet image —
	 * NEVER exceed the per-temperature LUT byte (over-power / laser safety). */
	{
		u8 lut_bias = 0x18, lut_mod = 0x34;	/* laser LUT @ 25C (calibrated). lowering bias to 0x0a did NOT save DS RX (laser-on deafens RX regardless of optical power) -> the fix is burst-gating the TX path, not the bias level */

		bosa_set_bit(0x23d, 7, 0);		/* bias DAC: strobe low */
		bosa_set_field(0x236, 0xff, lut_bias);	/* W54 bias hi-8 (= bias12[11:4]) */
		bosa_set_field(0x238, 0x0f, 0x00);	/* W56 bias12[3:0] (LUT<<4 -> 0) */
		bosa_set_bit(0x23d, 7, 1);		/* latch */
		bosa_set_bit(0x23d, 7, 0);		/* mod DAC: strobe low */
		bosa_set_field(0x237, 0xff, lut_mod);	/* W55 mod hi-8 (= mod12[11:4]) */
		bosa_set_field(0x238, 0xf0, 0x00);	/* W56 mod12[3:0] (LUT<<4 -> 0) */
		bosa_set_bit(0x23d, 7, 1);		/* latch */
	}

	/* (removed an idx4 0x245 loop_mode write — the RTL8290B loop mode is W80[7:6],
	 * set by FSU/DCL) */
	bosa_set_field(0x230, 0xff, 0x00);		/* idx5 */
	bosa_set_field(0x27c, 0xff, 0xe9);		/* W80=0xe9: converged (DCL[7:6]=3 + b5 + 0x09) */
	mdelay(51);					/* idx6 */
	/* (removed a 0x24d low-nibble write — W77 owned by the FSU above) */

	/* BOOSTER — the laser-diode driver OUTPUT stage. It must be enabled here; if it
	 * is not, W4/0x204 bit4 EN_L stays 0 (the W4=0x8e operating value leaves bit4
	 * clear) and NO laser current flows even with the calibrated bias loaded -> the
	 * monitor photodiode reads 0 -> the APC servo, seeing no optical feedback,
	 * collapses the bias DAC to 0 and the laser stays dark. Sequence: CONTROL2 bit6
	 * LOS_PIN_TRI low, assert EN_L, ~200ms laser-bias settle, CONTROL2 bit6 high. */
	bosa_set_field(0x254, 0x40, 0x00);		/* CONTROL2 bit6 LOS_PIN_TRI = 0 */
	bosa_set_bit(0x204, 4, 1);			/* W4 EN_L = 1: laser booster ON */
	mdelay(200);					/* 200ms laser-bias settle */
	bosa_set_field(0x254, 0x40, 0x40);		/* CONTROL2 bit6 LOS_PIN_TRI = 1 */

	bosa_set_field(0x254, 0x80, 0x00);		/* idx8 CONTROL2 bit7 = 0 */
	/* idx7 W53/0x235 fault-detect enables. The device default arms ALL (0xff), but
	 * on this board the MPD high/low APC fault-detect (bits[1:0] APC_ENFD_MPD_HIGH/LOW)
	 * trips a (false) MPD_VHIGH the instant TX is enabled and latches the laser
	 * off — even though the bias/mod DACs are clamped to the per-board calibrated
	 * LUT (so no real over-power is possible). Arm everything EXCEPT the MPD
	 * high/low detect (0xfc) so the laser is not HW-killed during bring-up; the
	 * periodic bosa_laser_maint() still watches the genuine TX-kill faults and the
	 * R30 MPD bits are polled in software for visibility. (Laser safety is held by
	 * the LUT clamp, not by this comparator.) */
	bosa_set_field(0x235, 0xff, 0xfc);		/* idx7: arm faults, MPD hi/lo OFF */
	bosa_set_field(0x25f, 0xff, 0x02);
	bosa_set_field(0x260, 0xff, 0x00);

	/* Post-enable settle + diagnostic. NO init re-arm loop here: the old bounded
	 * bosa_fault_rearm loop fired on the benign aggregate 0x383 b4, and in the
	 * converged B-flow that corrupts the MCU state and tips the BOSA into DEBUG_MODE
	 * (every reg 0x20). Just log the live state; the FSM-timer maintenance services
	 * genuine 0x389 TX-kill faults from here (and skips a DEBUG_MODE-wedged BOSA). */
	mdelay(50);
	pr_info("rtl9602c-gpon: post-txen: 0x389=0x%02x 0x383=0x%02x R30=0x%02x bias=0x%02x R33=0x%02x mod=0x%02x EN_L=%d\n",
		bosa_read_reg(0x389) & 0xff, bosa_read_reg(0x383) & 0xff,
		bosa_read_reg(0x31e) & 0xff, bosa_read_reg(0x236) & 0xff,
		bosa_read_reg(0x321) & 0xff, bosa_read_reg(0x320) & 0xff,
		!!(bosa_read_reg(0x204) & 0x10));

	/*
	 * BURST-GATE the laser. EN_L (0x204 bit4) is the booster OUTPUT-enable: held
	 * =1 it forces continuous-wave emission, whose 1310nm light/coupling DEAFENS
	 * the shared-BOSA downstream RX (root cause of "OLT never ranges us": laser-on
	 * => gtc_ds_sts=0x0b LOS+LOF, optic_los=1, ds_rx frozen; laser-off => 0x04
	 * LOCKED, ds_rx climbs). The operational burst-mode value of this register at
	 * O5 (bursting, RX intact) is 0x204=0x8e, i.e. **EN_L=0** — the device does NOT
	 * pin EN_L on; the per-burst emission is gated downstream by the SoC BEN. EN_L=1
	 * is needed only TRANSIENTLY during ignition (above) to flow bias and seat the
	 * APC; the bias DAC stays loaded (R33) once seated. So deassert EN_L now to
	 * reach the burst-mode state and let DS RX survive between grants. (Laser-safe:
	 * this only turns an enable OFF.) If the bias collapses here the APC convergence
	 * is the real gap (R30 OFFK_DONE still 0) — the readback below makes that
	 * visible.
	 */
	bosa_set_bit(0x204, 4, 0);			/* EN_L = 0: burst-gate (0x8e) */
	mdelay(5);
	pr_info("rtl9602c-gpon: burst-gate EN_L=0 -> 0x204=0x%02x R33=0x%02x R30=0x%02x 0x383=0x%02x\n",
		bosa_read_reg(0x204) & 0xff, bosa_read_reg(0x321) & 0xff,
		bosa_read_reg(0x31e) & 0xff, bosa_read_reg(0x383) & 0xff);

	/* Hand off to the continuous fault-service driven from the GPON FSM timer. */
	bosa_laser_up = 1;

	pr_info("rtl9602c-gpon: laser ignite: lock=%d R30=0x%02x(offk_done=%d txsd=%d) bias=0x%02x mod=0x%02x mpd=%02x/%02x\n",
		locked, bosa_read_reg(0x31e) & 0xff,
		!!(bosa_read_reg(0x31e) & BIT(7)), !!(bosa_read_reg(0x31e) & BIT(6)),
		bosa_read_reg(0x236) & 0xff, bosa_read_reg(0x238) & 0xff,
		bosa_read_reg(0x320) & 0xff, bosa_read_reg(0x321) & 0xff);
}

static void __init bosa_probe(void)
{
	int hb  = bosa_read_reg(BOSA_REG_NUM);
	int lb  = bosa_read_reg(BOSA_REG_NUM + 1);
	int vid = bosa_read_reg(BOSA_REG_VID);

	if (hb < 0 || lb < 0 || vid < 0) {
		pr_warn("rtl9602c-gpon: BOSA I2C read failed (hb=%d lb=%d vid=%d)\n",
			hb, lb, vid);
		return;
	}
	bosa_id_num = (hb << 8) | lb;
	bosa_id_vid = vid;

	/* Read the RX-path registers (read-only): RX power-down, SD-pin tristate,
	 * and the live RX loss-of-signal status. These confirm page-0x54 access and
	 * show what the RX-enable writes will need to change. */
	bosa_w41     = bosa_read_reg(BOSA_REG_W41);
	bosa_ctrl2   = bosa_read_reg(BOSA_REG_CONTROL2);
	bosa_status2 = bosa_read_reg(BOSA_REG_STATUS2);

	pr_info("rtl9602c-gpon: BOSA RTL8290B num=0x%04x vid=0x%02x %s | w41=0x%02x(rxpwdn=%d) ctrl2=0x%02x(los_tri=%d) status2=0x%02x(rx_los=%d)\n",
		bosa_id_num, bosa_id_vid,
		(bosa_id_num == 0x8290) ? "detected" : "UNEXPECTED",
		bosa_w41 & 0xff, (bosa_w41 >> 4) & 1,
		bosa_ctrl2 & 0xff, (bosa_ctrl2 >> 6) & 1,
		bosa_status2 & 0xff, (bosa_status2 >> 2) & 1);
}

/*
 * Full SerDes analog + WSDS configuration — the operating point an ONU runs at
 * O5. The analog block (CMU, CDR, RX front-end incl. the optical signal-detect/LOS
 * comparator, TX driver) has dozens of per-silicon calibration/config registers;
 * programming only the handful that obviously differ from reset leaves other
 * parts of the RX/SD path un-powered, so the optical signal-detect never
 * asserts even with real downstream light. These offset/value pairs are the
 * operational values for this silicon — register facts. Status/monitor registers
 * and the digital reset-B/clock bank
 * (WSDS_DIG_00/18/1D) are deliberately excluded; those are driven by the
 * ordered sequence in gpon_serdes_init().
 */
static const struct { u32 off; u32 val; } sds_analog_golden[] __initconst = {
	/* WSDS analog front + digital RX-path config */
	{ 0x22000, 0x00000805 }, { 0x22008, 0x0000ffff }, { 0x2201c, 0x0000ffff },
	{ 0x22020, 0x0000ffff }, { 0x22038, 0x00000900 }, { 0x22048, 0x000000ff },
	{ 0x22050, 0x00022300 }, { 0x22054, 0x00022310 }, { 0x22058, 0x083d0100 },
	{ 0x22060, 0x00000fff }, { 0x22064, 0x0000cf45 }, { 0x22068, 0x00000f45 },
	/* SDS_ANA_MISC (RX-enable force, speed-select, force-SD) */
	{ 0x22500, 0x00000030 }, { 0x22504, 0x00000030 }, { 0x22508, 0x00003000 },
	/* SDS_ANA_COM (CMU, RX CDR front-end, filters, bias) */
	{ 0x22580, 0x00003400 }, { 0x22584, 0x000073a4 }, { 0x22588, 0x00006df8 },
	{ 0x2258c, 0x00008941 }, { 0x22590, 0x00008884 }, { 0x22594, 0x0000413f },
	{ 0x22598, 0x00004fc0 }, { 0x2259c, 0x00005682 }, { 0x225a0, 0x00000713 },
	{ 0x225a4, 0x000002f5 }, { 0x225a8, 0x00002793 }, { 0x225ac, 0x0000b000 },
	{ 0x225b0, 0x00004848 }, { 0x225b4, 0x000000c8 }, { 0x225bc, 0x000008f2 },
	{ 0x225c0, 0x00001042 }, { 0x225c4, 0x0000c391 }, { 0x225c8, 0x00006a00 },
	{ 0x225cc, 0x00006600 }, { 0x225d0, 0x0000c000 },
	/* 0x225d8 (COM_REG22 TX_AMP/EMP) is set later, in gpon_serdes_init's TX
	 * section, to the rev-A (ModeV1) value 0x29 (TX_AMP=0x5, TX_EMP=0x1) via
	 * field-writes. NOT a full write here so the upper bits keep their reset
	 * state. (Boot default 0x39/TX_AMP=0x7 over-drives.) */
	{ 0x225dc, 0x00000418 }, { 0x225e0, 0x00008001 }, { 0x225e4, 0x0000001f },
	{ 0x225e8, 0x000011e4 }, { 0x225ec, 0x00009422 }, { 0x225f0, 0x00008502 },
	{ 0x225f4, 0x00000ff0 }, { 0x225f8, 0x0000000a },
	/* SDS_ANA_GPON (GPON-rate CDR/PLL/PCM config) */
	{ 0x22708, 0x00000f00 }, { 0x2270c, 0x0000b8c6 }, { 0x22710, 0x0000a112 },
	{ 0x22714, 0x00004280 }, { 0x22718, 0x0000f53f }, { 0x2271c, 0x00004fdf },
	{ 0x22720, 0x00000001 }, { 0x22724, 0x0000309b }, { 0x22728, 0x0000225c },
	{ 0x2272c, 0x00001061 }, { 0x22730, 0x0000110d }, { 0x22734, 0x00004854 },
	{ 0x22738, 0x000080c5 }, { 0x2273c, 0x0000121e }, { 0x22740, 0x0000307b },
	{ 0x22744, 0x00000271 }, { 0x22748, 0x00000271 }, { 0x2274c, 0x00001012 },
	{ 0x22750, 0x0000f162 }, { 0x22754, 0x00003026 }, { 0x22758, 0x0000a780 },
	{ 0x2275c, 0x0000f000 },
	/* SDS_ANA_GPON additional per-rate/lane banks (the RX path selects among
	 * these; leaving them at reset starves the active RX/SD analog). */
	{ 0x22608, 0x00000f00 }, { 0x2260c, 0x0000b8c6 }, { 0x22610, 0x0000a112 },
	{ 0x22614, 0x00004280 }, { 0x22618, 0x0000f53f }, { 0x2261c, 0x00004fdf },
	{ 0x22620, 0x00000001 }, { 0x22624, 0x0000309b }, { 0x22628, 0x0000225c },
	{ 0x2262c, 0x00001061 }, { 0x22630, 0x0000110d }, { 0x22634, 0x00004854 },
	{ 0x22638, 0x000080c5 }, { 0x2263c, 0x0000121e }, { 0x22640, 0x0000307b },
	{ 0x22644, 0x00000271 }, { 0x22648, 0x00000271 }, { 0x2264c, 0x00001012 },
	{ 0x22650, 0x0000f162 }, { 0x22654, 0x00003026 }, { 0x22658, 0x0000a780 },
	{ 0x2265c, 0x0000f000 },
	{ 0x22688, 0x00000f00 }, { 0x2268c, 0x0000b8c6 }, { 0x22690, 0x0000a112 },
	{ 0x22694, 0x00004280 }, { 0x22698, 0x0000f53f }, { 0x2269c, 0x00004fdf },
	{ 0x226a0, 0x00000001 }, { 0x226a4, 0x0000309b }, { 0x226a8, 0x0000225c },
	{ 0x226ac, 0x00001062 }, { 0x226b0, 0x00002000 }, { 0x226b4, 0x00001050 },
	{ 0x226b8, 0x000080c1 }, { 0x226bc, 0x0000121e }, { 0x226c0, 0x0000107b },
	{ 0x226c4, 0x00000280 }, { 0x226c8, 0x00000280 }, { 0x226cc, 0x00001012 },
	{ 0x226d0, 0x0000f862 }, { 0x226d4, 0x00003938 }, { 0x226d8, 0x00003100 },
	{ 0x226dc, 0x0000f000 },
	{ 0x22788, 0x00000f00 }, { 0x2278c, 0x0000b8c6 }, { 0x22790, 0x0000a112 },
	{ 0x22794, 0x00004280 }, { 0x22798, 0x0000f53f }, { 0x2279c, 0x00004fdf },
	{ 0x227a0, 0x00000001 }, { 0x227a4, 0x0000309b }, { 0x227a8, 0x0000225c },
	{ 0x227ac, 0x00001062 }, { 0x227b0, 0x00002000 }, { 0x227b4, 0x00004850 },
	{ 0x227b8, 0x000080c5 }, { 0x227bc, 0x0000121e }, { 0x227c0, 0x0000103e },
	{ 0x227c4, 0x00000280 }, { 0x227c8, 0x00000280 }, { 0x227cc, 0x00001012 },
	{ 0x227d0, 0x0000f862 }, { 0x227d4, 0x00003938 }, { 0x227d8, 0x0000b100 },
	{ 0x227dc, 0x0000f000 },
	/* FIB (fiber optical front-end) config — 4 identical banks. This block
	 * powers and configures the optical RX/SD path; leaving it at reset keeps
	 * the optical front-end down so the signal-detect never asserts. FIB_REG0
	 * (bank base) carries FP_CFG_FIB_PDOWN at bit11, cleared separately below to
	 * turn fiber power on. */
	{ 0x22c00, 0x00001940 }, { 0x22c04, 0x00006109 }, { 0x22c08, 0x0000e001 },
	{ 0x22c0c, 0x00003290 }, { 0x22c10, 0x000001a0 }, { 0x22c1c, 0x00000004 },
	{ 0x22c3c, 0x00008000 }, { 0x22c40, 0x00000083 }, { 0x22c48, 0x00005000 },
	{ 0x22c58, 0x00000001 }, { 0x22c5c, 0x00004001 }, { 0x22c60, 0x00000004 },
	{ 0x22c64, 0x0000326a }, { 0x22c6c, 0x0000115d }, { 0x22c70, 0x000033fa },
	{ 0x22c74, 0x0000e46a }, { 0x22c78, 0x0000071e },
	{ 0x22c80, 0x00001940 }, { 0x22c84, 0x00006109 }, { 0x22c88, 0x0000e001 },
	{ 0x22c8c, 0x00003290 }, { 0x22c90, 0x000001a0 }, { 0x22c9c, 0x00000004 },
	{ 0x22cbc, 0x00008000 }, { 0x22cc0, 0x00000083 }, { 0x22cc8, 0x00005000 },
	{ 0x22cd8, 0x00000001 }, { 0x22cdc, 0x00004001 }, { 0x22ce0, 0x00000004 },
	{ 0x22ce4, 0x0000326a }, { 0x22cec, 0x0000115d }, { 0x22cf0, 0x000033fa },
	{ 0x22cf4, 0x0000e46a }, { 0x22cf8, 0x0000071e },
	{ 0x22d00, 0x00001940 }, { 0x22d04, 0x00006109 }, { 0x22d08, 0x0000e001 },
	{ 0x22d0c, 0x00003290 }, { 0x22d10, 0x000001a0 }, { 0x22d1c, 0x00000004 },
	{ 0x22d3c, 0x00008000 }, { 0x22d40, 0x00000083 }, { 0x22d48, 0x00005000 },
	{ 0x22d58, 0x00000001 }, { 0x22d5c, 0x00004001 }, { 0x22d60, 0x00000004 },
	{ 0x22d64, 0x0000326a }, { 0x22d6c, 0x0000115d }, { 0x22d70, 0x000033fa },
	{ 0x22d74, 0x0000e46a }, { 0x22d78, 0x0000071e },
	{ 0x22d80, 0x00001940 }, { 0x22d84, 0x00006109 }, { 0x22d88, 0x0000e001 },
	{ 0x22d8c, 0x00003290 }, { 0x22d90, 0x000001a0 }, { 0x22d9c, 0x00000004 },
	{ 0x22dbc, 0x00008000 }, { 0x22dc0, 0x00000083 }, { 0x22dc8, 0x00005000 },
	{ 0x22dd8, 0x00000001 }, { 0x22ddc, 0x00004001 }, { 0x22de0, 0x00000004 },
	{ 0x22de4, 0x0000326a }, { 0x22dec, 0x0000115d }, { 0x22df0, 0x000033fa },
	{ 0x22df4, 0x0000e46a }, { 0x22df8, 0x0000071e },
};

/* FIB_REG0 bank bases; FP_CFG_FIB_PDOWN (bit11) cleared = fiber power on. */
#define FIB_REG0_PDOWN		BIT(11)
static const u32 fib_reg0_banks[] __initconst = {
	0x22c00, 0x22c80, 0x22d00, 0x22d80,
};

/*
 * Bring up the PON SerDes (SDS) so the GPON MAC core gets its line clock AND so
 * the receiver recovers the downstream bitstream.
 *
 * Ordering is the whole game here. A working bring-up programs the analog
 * CMU/CDR block FIRST, selects GPON mode, THEN pulses the SDS+MAC reset, and
 * only AFTER that releases the per-datapath soft-reset-B lines (generic, EPON,
 * GPON, analog and the RX/TX interface reset-B) and forces the 125M reference
 * clock. The reset latches the freshly-written analog config; releasing the
 * reset-B lines afterwards lets the RX CDR re-lock against it. A naive
 * "reset-then-configure" sequence ends with the same final register values
 * yet a CDR that never locks the real downstream — the register contents are
 * identical but the receiver reports loss-of-frame and the ONU FSM is stuck in
 * O1. The operational run state is WSDS_DIG_00 = 0xf30,
 * WSDS_DIG_1D = 0x1c000 (RX+TX+common interface reset-B released).
 */
static int __init gpon_serdes_init(void)
{
	int i;

	/* 1. Park CFG_SDS_MODE at the illegal/off value (0x1f) while the analog
	 *    block is programmed and reset. The SDS must stay in the illegal mode for
	 *    the WHOLE bring-up and only switch to GPON (0x08) at the very end
	 *    (step 7). Selecting GPON before the RX is armed is exactly why a
	 *    register-identical naive sequence never locks the downstream. */
	sw_field(SDS_CFG, 4, 0, SDS_MODE_OFF);
	sw_wr(WSDS_DIG_01, 0);				/* clear force-SDS dummy   */
	sw_field(WSDS_DIG_00, 0, 0, 0);			/* STOP_CLK = 0            */

	/* 2. Program the FULL analog block to the operational values (the complete
	 *    RX/SD/CDR/CMU/TX config) and turn fiber power on (clear FP_CFG_FIB_PDOWN
	 *    on every FIB bank so the optical front-end + signal-detect power up). */
	for (i = 0; i < ARRAY_SIZE(sds_analog_golden); i++)
		sw_wr(sds_analog_golden[i].off, sds_analog_golden[i].val);
	for (i = 0; i < ARRAY_SIZE(fib_reg0_banks); i++)
		sw_wr(fib_reg0_banks[i],
		      sw_rd(fib_reg0_banks[i]) & ~FIB_REG0_PDOWN);

	/* 3. Pulse the SDS config + datapath reset to latch the analog config. */
	sw_field(SW_SOFTWARE_RST, 7, 7, 1);		/* CMD_SDS_CFG_RST_PS      */
	sw_field(SW_SOFTWARE_RST, 0, 0, 1);		/* CMD_SDS_RST_PS          */
	mdelay(10);

	/* 4. Release all datapath soft-reset-B lines and force the 125M ref clock
	 *    (golden WSDS_DIG_00 = 0xf30), then pulse the RX/TX interface reset-B
	 *    lines (golden WSDS_DIG_1D = 0x1c000). Re-clear FIB power-down, which the
	 *    reset re-asserts. */
	sw_wr(WSDS_DIG_00, WSDS_DIG00_RUN);
	sw_field(WSDS_DIG_1D, 15, 15, 0);		/* RX interface reset-B 0  */
	sw_field(WSDS_DIG_1D, 16, 16, 0);		/* TX interface reset-B 0  */
	sw_field(WSDS_DIG_1D, 14, 14, 1);		/* common interface rst-B  */
	sw_field(WSDS_DIG_1D, 15, 15, 1);		/* RX interface reset-B 1  */
	sw_field(WSDS_DIG_1D, 16, 16, 1);		/* TX interface reset-B 1  */
	mdelay(10);
	for (i = 0; i < ARRAY_SIZE(fib_reg0_banks); i++)
		sw_wr(fib_reg0_banks[i],
		      sw_rd(fib_reg0_banks[i]) & ~FIB_REG0_PDOWN);

	/* 5. Burst-enable output; leave optical-LOS un-forced so the real RX front-
	 *    end drives it (a working unit reaches O5 with FRC_OPTIC_LOS=0). */
	sw_field(WSDS_DIG_18, 12, 12, 1);		/* BEN_OE = 1              */
	sw_field(WSDS_DIG_18, 15, 15, 0);		/* OPTIC_LOS_SEL_EPON = 0  */
	/* Do NOT force optic_los. At O5 WSDS_DIG_18 = 0x1000 (no force) and the REAL
	 * optical signal-detect asserts — because the external RTL8290B BOSA is
	 * initialised over I2C. The SD here is driven by that real RX path (see the
	 * BOSA init above); forcing optic_los only masks a down RX and never reaches
	 * O5. */
	sw_field(WSDS_DIG_18, 14, 14, 0);		/* CFG_FRC_OPTIC_LOS = 0   */
	sw_field(WSDS_DIG_18, 13, 13, 0);		/* CFG_FRCV_OPTIC_LOS = 0  */

	/* 6. Arm the RX in the required order: enable the RX-CDR analog front
	 *    end, settle, force the line-rate select to the GPON rate, then drive the
	 *    forced RX-enable through a 0->1 edge to start the CDR. Only AFTER the
	 *    analog config + reset + reset-B release does this 0->1 edge actually
	 *    kick the receiver. Finish with EN_PDOWN_BEN=0 and TX-disable delay=0. */
	sw_field(SDS_ANA_COM_REG12, 14, 14, 0x1);	/* RX_SEL_CDR_AFEN = 1     */
	mdelay(10);
	sw_field(SDS_ANA_MISC_REG01, 7, 5, 0x1);	/* SPDSEL_VAL = GPON rate  */
	sw_field(SDS_ANA_MISC_REG01, 4, 4, 0x1);	/* SPDSEL force on         */
	sw_field(SDS_ANA_MISC_REG00, 4, 4, 0x1);	/* FRC_RX_EN_ON = 1        */
	sw_field(SDS_ANA_MISC_REG00, 5, 5, 0x0);	/* FRC_RX_EN_VAL 0 ...     */
	sw_field(SDS_ANA_MISC_REG00, 5, 5, 0x1);	/* ... -> 1 (start CDR)    */
	mdelay(50);
	sw_field(WSDS_DIG_02, 10, 10, 0x0);		/* EN_PDOWN_BEN = 0        */
	sw_field(WSDS_DIG_03, 6, 4, 0x0);		/* CFG_TXDIS_SEL_DLY = 0: the RTL9602C
							 * burst-mode TX-disable timing requires
							 * 0; 0x2 mis-times the burst TX-disable
							 * -> "Laser out". */
	sw_field(WSDS_DIG_03, 3, 0, 0x0);		/* CFG_D2ANLOG_SEL = 0 (TX data path) */
	/* FORCE_BEN (SDS 0x220e4) BEN_FORCE_MODE[0]=0: let the GTC framer drive the
	 * burst-enable (laser gate). If left at the forced default the laser is gated
	 * by BEN_FORCE_VALUE (off) and never fires the SN burst even though the MAC
	 * queue drains -> OLT "Power down". (Was wrongly written to 0x400e4, which is
	 * an unmapped address that bus-aborts — verified via /proc sds_tx readback.) */
	sw_field(0x220e4, 0, 0, 0x0);

	/*
	 * 6b. TX DATA PATH — route the digital US-framer data into the analog TX
	 * serializer AND set the TX-data sample-clock edges, *** BEFORE *** switching
	 * CFG_SDS_MODE to GPON. CRITICAL ORDERING for the rev-A ModeV1 SerDes: program
	 * the D2A interconnect (WSDS_DIG_1E) and the SP_CFG_NEG_CLKWR_A2D /
	 * SEP_CFG_NEG_CLKRD_D2A sample clocks before CFG_SDS_MODE=GPON, so the
	 * serializer latches the *connected* data-path mux + clocks at mode-entry. The
	 * earlier version set these AFTER GPON mode (with a TX reset-B toggle to
	 * compensate) — which left the already-running serializer latched on the disconnected
	 * pre-config: the laser was DC-biased but carried NO decodable burst, so the
	 * OLT received zero upstream. WSDS_DIG_1E[5]=CFG_ANALOG2D_SEL,
	 * [4]=CFG_D2ANLOG_INF_SEL; SDS_REG7[14]=SP_CFG_NEG_CLKWR_A2D;
	 * SDS_EXT_REG12[8]=SEP_CFG_NEG_CLKRD_D2A.
	 */
	sw_field(0x220a8, 5, 4, 0x3);			/* WSDS_DIG_1E D2A interconnect */
	sw_field(0x2281c, 14, 14, 0x1);			/* SDS_REG7 SP_CFG_NEG_CLKWR_A2D */
	sw_field(0x22a30, 8, 8, 0x1);			/* SDS_EXT_REG12 SEP_CFG_NEG_CLKRD_D2A */

	/*
	 * TX drive level (SDS_ANA_COM_REG22, 0x225d8): REG_TX_AMP[5:3]=0x5,
	 * REG_TX_EMP[2:0]=0x1 — the rev-A (ModeV1) TX drive this board requires. The
	 * SoC boot default (0x39 => TX_AMP=0x7) over-drives
	 * the serializer output feeding the laser modulation input, distorting the
	 * upstream burst eye so the OLT's burst-mode receiver cannot reliably decode
	 * our SN/ranging burst (detect-but-no-range). An earlier note mis-labelled the
	 * resulting 0x29 a "ModeV2" value and skipped it; 0x29 IS the ModeV1 TX drive.
	 */
	sw_field(0x225d8, 5, 3, 0x5);			/* SDS_ANA_COM_REG22 REG_TX_AMP = 0x5 */
	sw_field(0x225d8, 2, 0, 0x1);			/* SDS_ANA_COM_REG22 REG_TX_EMP = 0x1 */

	/*
	 * 7a. Force signal-detect on. RST_DONE is gated by signal-detect; force it so
	 * the MAC reset handshake completes (MISC_REG02 = 0x3000). This
	 * only ungates the handshake — real downstream lock still shows as LOF
	 * clearing and superframe_cnt incrementing, which force-SD does NOT fake.
	 */
	sw_field(SDS_ANA_MISC_REG02, 13, 13, 0x1);	/* signal-detect value=1   */
	sw_field(SDS_ANA_MISC_REG02, 12, 12, 0x1);	/* force signal-detect     */
	mdelay(10);

	/* 7b. Finally select GPON mode — the very last step, with the RX fully armed
	 *     (CFG_SDS_MODE switches to GPON only here). */
	sw_field(SDS_CFG, 4, 0, SDS_MODE_GPON);
	mdelay(50);

	/* TX-interface reset-B re-sync: with the D2A mux + sample clocks already set
	 * before mode-entry (step 6b), pulse WSDS_DIG_1D[16] (CFG_SFT_RSTB_INF_TX) 0->1
	 * once GPON mode is live so the TX serializer (re)locks onto the now-connected
	 * framer data. The SerDes-TX serializer lock is non-deterministic; this re-sync
	 * is the mechanism that historically caught the upstream-burst lock (the OLT
	 * occasionally ranged the ONU). Only the TX interface reset-B is toggled, not
	 * the PLL, so the locked RX downstream framer is undisturbed. */
	sw_field(WSDS_DIG_1D, 16, 16, 0);
	mdelay(2);
	sw_field(WSDS_DIG_1D, 16, 16, 1);
	mdelay(10);

	sw_field(WSDS_DIG_00, 0, 0, 0);			/* keep MAC clock ungated  */

	pr_info("rtl9602c-gpon: SDS cfg=0x%08x dig00=0x%08x dig1d=0x%08x fib21=0x%08x fib_reg0=0x%08x\n",
		sw_rd(SDS_CFG), sw_rd(WSDS_DIG_00), sw_rd(WSDS_DIG_1D),
		sw_rd(FIB_EXT_REG21), sw_rd(0x22c00));

	for (i = 0; i < SDS_LOCK_POLL_MAX; i++) {
		if (sw_rd(FIB_EXT_REG21) & SDS_ANALOG_READY)
			return 0;
		udelay(200);
	}
	return -ETIMEDOUT;
}

/*
 * Wait (bounded) for the GPON MAC to report RST_DONE after the SerDes sequence
 * has issued the SDS+MAC reset. Returns 0 on RST_DONE, -ETIMEDOUT otherwise.
 */
static int __init gpon_wait_rst_done(void)
{
	int i;

	for (i = 0; i < GPON_RST_POLL_MAX; i++) {
		if (gpon_rd(GPON_RESET) & GPON_RST_DONE)
			return 0;
		udelay(10);
	}
	return -ETIMEDOUT;
}

/*
 * Configure the PON packet datapath (PON-IP) for GPON before the MAC reset.
 *
 * The block is brought up disabled (GMII halted, packet buffers off), the SRAM
 * descriptor accounting is programmed for 128-byte pages with no DRAM
 * reservation (US 128 pages, DS 32 pages), GPON mode is selected, the upstream
 * FIFO thresholds and PONNIC TX/RX framing are set, and finally the upstream and
 * downstream packet buffers are enabled. Until this runs, the MAC has nowhere to
 * land downstream frames and the reset handshake does not settle. Ordering and
 * register/field facts are from the SoC PON-IP register map.
 */
static void __init gpon_pbo_init(void)
{
	/* 1. Halt GMII and disable both packet buffers while reconfiguring. */
	/* Enable the upstream GMII TX/RX framer to its O5 operating value (0x90101070,
	 * GMII_TX_EN|GMII_RX_EN + US datapath bits). This was forced to 0, leaving
	 * the PON-IP upstream datapath DISABLED — the ONU composed Serial_Number_ONU
	 * but could never transmit it upstream, so the OLT never heard it and the
	 * ONU was stuck in O3. Must be set here in pbo_init (before the MAC reset);
	 * setting it post-boot is too late. */
	pi_wr(PI_IO_CMD_0_US, 0x90101070);
	/* DS IO_CMD (the DMA/FIFO drain enable, 0x90081070) is written LAST, after the
	 * backpressure thresholds + PBUF_EN, so the DS engine drains out of a properly
	 * bounded buffer (see end of this function). */
	pi_field(PI_PONIP_CTL_US, 0, 0, 0);		/* CFG_PBUF_EN = 0        */
	pi_field(PI_PONIP_CTL_DS, 0, 0, 0);

	/* 2. SRAM descriptor accounting (128B pages, no DRAM): count-1 values. */
	pi_field(PI_PON_DSC_CFG_US, 12, 0, PI_US_SRAM_NO);
	pi_field(PI_PON_DSC_CFG_DS, 12, 0, PI_DS_SRAM_NO);
	pi_field(PI_PON_DSC_CFG_US, 28, 16, PI_US_SRAM_NO);	/* RAM_NO=SRAM_NO */
	pi_field(PI_PON_DSC_CFG_DS, 28, 16, PI_DS_SRAM_NO);
	pi_field(PI_DSCRUNOUT_US, 12, 0, PI_US_SRAM_RUNOUT);
	pi_field(PI_DSCRUNOUT_DS, 12, 0, PI_DS_SRAM_RUNOUT);
	pi_field(PI_DSCRUNOUT_US, 28, 16, 0);		/* no DRAM runout         */
	pi_field(PI_DSCRUNOUT_DS, 28, 16, 0);

	/* PBO backpressure thresholds (SRAM-only, 128B pages). Without these every
	 * threshold field reads 0, so the PBO treats the small DS SRAM pool as
	 * instantly over-threshold and never releases switch flow-control — which is
	 * why enabling the full DS DMA backs the buffer up and stalls the US. The
	 * per-SID RPV thresholds (0x02458, stride 4) are flow-control limits, NOT SID
	 * validity (writing them for every SID is safe; SIDVALID/SID2QID untouched).
	 * Values: us_sram_runt 126 -> stop 125, glb on/off 125/123; per-SID 150/130
	 * (page scale 1, non-tripping); DS flow-ctrl on/off 2/22. */
	pi_field(PI_PON_SID_STOP_TH, 12, 0, 125);
	pi_field(PI_PON_SID_GLB_TH, 28, 16, 125);
	pi_field(PI_PON_SID_GLB_TH, 12, 0, 123);
	{
		unsigned int sid;

		for (sid = 0; sid < PI_SID_NUM; sid++) {
			u32 off = PI_PON_SID_RPV_TH + sid * PI_RPV_TH_STRIDE;

			pi_field(off, 28, 16, 150);
			pi_field(off, 12, 0, 130);
		}
	}
	pi_field(PI_PON_FC_CONFIG_DS, 12, 0, 22);
	pi_field(PI_PON_FC_CONFIG_DS, 28, 16, 2);

	/* 3. GPON mode (not EPON) + upstream RXC stop + US FIFO thresholds. */
	pi_field(PI_PONIP_CTL_US, 2, 2, 0);		/* CFG_EPON_MODE = 0      */
	pi_field(PI_PONIP_CTL_DS, 2, 2, 0);
	pi_field(PI_PONIP_CTL_US, 1, 1, 1);		/* CFG_STOP_RXC_EN = 1    */
	pi_field(PI_PON_US_FIFO_CTL, 5, 4, 1);		/* USFIFO_SPACE = 1       */
	pi_field(PI_PON_US_FIFO_CTL, 3, 0, 3);		/* USFIFO_START = 3       */

	/* 4. 128-byte page size everywhere (PON-IP descriptors + PONNIC pages). */
	pi_field(PI_PON_DSC_CFG_US, 14, 13, 0);
	pi_field(PI_PON_DSC_CFG_DS, 14, 13, 0);
	pi_field(PI_IO_CMD_1_US, 5, 4, 0);		/* RPAGE_SIZE = 128B      */
	pi_field(PI_IO_CMD_1_US, 1, 0, 0);		/* TPAGE_SIZE = 128B      */
	pi_field(PI_IO_CMD_1_DS, 5, 4, 0);
	pi_field(PI_IO_CMD_1_DS, 1, 0, 0);
	pi_field(PI_IO_CMD_1_DS, 27, 27, 1);		/* PRECISE_DMA_EN — DS precise/aligned DMA transfers; the O5 DS IO_CMD_1 value is 0x08000000. Without it the DS RX DMA never lands a frame, so filled stays 0. */

	/* 5. PONNIC datapath: almost-full RX backpressure + TX stop/extra. */
	pi_field(PI_PROBE_SELECT_US, 1, 1, 1);
	pi_field(PI_CFG_US, 26, 26, 1);			/* E_EN_RFF_AFULL         */
	pi_field(PI_CFG_US, 17, 17, 1);			/* EN_TX_STOP             */
	pi_field(PI_CFG_US, 16, 16, 1);			/* EN_TXE_EXTRA           */
	pi_field(PI_PROBE_SELECT_DS, 1, 1, 1);
	pi_field(PI_CFG_DS, 26, 26, 1);
	pi_field(PI_CFG_DS, 17, 17, 1);
	pi_field(PI_CFG_DS, 16, 16, 1);

	/* 6. PONNIC TX framing (IFG, preamble, padding) + RX accept-CRC-error. */
	pi_field(PI_TX_CFG_US, 12, 10, 3);		/* IFG                    */
	pi_field(PI_TX_CFG_US, 2, 1, 1);		/* preamble length        */
	pi_field(PI_TX_CFG_US, 0, 0, 1);		/* TX padding             */
	pi_field(PI_RX_CFG_US, 5, 5, 1);		/* accept CRC error       */
	pi_field(PI_TX_CFG_DS, 12, 10, 3);
	pi_field(PI_TX_CFG_DS, 2, 1, 1);
	pi_field(PI_TX_CFG_DS, 0, 0, 1);
	pi_field(PI_RX_CFG_DS, 5, 5, 1);

	/* 7. Enable upstream and downstream packet buffers. (The O5 PONIP_CTL_DS value
	 * is 0x81 = +bit7, but setting bit7 here destabilised the link with our
	 * incomplete DS routing — left at bit0 only for stability; revisit with the
	 * full DS datapath.) */
	pi_field(PI_PONIP_CTL_US, 0, 0, 1);		/* CFG_PBUF_EN = 1        */
	pi_field(PI_PONIP_CTL_DS, 0, 0, 1);
	pi_field(PI_PONIP_CTL_DS, 7, 7, 1);		/* CFG_TX_PAUSE low bit -> O5 value 0x81 (DS buffer release; safe now thresholds bound the buffer) */

	/* 8. Enable the full downstream PONNIC DMA drain (LAST — after the thresholds
	 * + PBUF_EN above). 0x90081070 = MAX_DMA_SEL_0[31] | EARLY_TX_EN[28] |
	 * TX_FIFO_THR[20:19]=1 | RX_FIFO_THR[12:11]=2 | RX_MAX_DMA_SEL[7:6]=1 |
	 * GMII_RX_EN[5] | GMII_TX_EN[4]. The DMA/FIFO fields start the DS engine
	 * draining de-encapsulated GEM frames out of the (now bounded) buffer into the
	 * switch -> CPU-port GMAC; DS OMCI (SID 64) egresses on the GMAC RX with the
	 * cpu-tag stream-id 64 that the NIC's OMCI hook catches. */
	pi_wr(PI_IO_CMD_0_DS, 0x90081070u);

	/* 9. PON-IP DS NIC config that forwards de-encapsulated DS frames to the host
	 * GMAC NIC (OMCI bypasses the switch: the switch PON-port RX MIB stays 0, OMCI
	 * goes PON-IP -> GMAC NIC direct). These registers in the DS NIC block are 0 at
	 * reset and must be programmed for O5; without them the de-encapsulated frame is
	 * not handed to the GMAC NIC and backs up in the PON-IP. Operational values. */
	pi_wr(0x0d400u, 0x00000040u);	/* DS NIC: OMCI SID 64 */
	pi_wr(0x0d404u, 0x11100348u);	/* DS NIC forward/queue config */
	pi_wr(0x0d42cu, 0x00000040u);	/* DS NIC: OMCI SID 64 (mirror) */
	pi_wr(0x0d3f4u, 0x02d60000u);	/* DS NIC RX ring-size/CDO config */
}

/* Full BOSA page2 (slave 0x54) + page3 (slave 0x55) register dump for diagnostics.
 * Reads all 512 regs via the kernel I2C path. */
static int bosadump_proc_show(struct seq_file *s, void *v)
{
	int i, j;

	for (i = 0; i < 256; i += 16) {
		seq_printf(s, "P2_%02x:", i);
		for (j = 0; j < 16; j++)
			seq_printf(s, " %02x", bosa_read_reg(0x200 + i + j) & 0xff);
		seq_puts(s, "\n");
	}
	for (i = 0; i < 256; i += 16) {
		seq_printf(s, "P3_%02x:", i);
		for (j = 0; j < 16; j++)
			seq_printf(s, " %02x", bosa_read_reg(0x300 + i + j) & 0xff);
		seq_puts(s, "\n");
	}
	return 0;
}

/* Per-flow downstream GEM Ethernet RX packet count (indirect read): write the
 * flow index to GEM_DS_RX_CNTR_IND (0x4040), poll R_ACK (bit15), read the count
 * from GEM_DS_RX_CNTR_STAT (0x4044). Diagnoses whether the OLT is sending ANY DS
 * GEM frames on a flow — e.g. OMCI on the OMCC flow 64 (gem port 2). */
static u32 gpon_gem_ds_rx_cnt(u8 flow)
{
	int i;

	gpon_wr(0x4040, flow & 0x7f);
	for (i = 0; i < 1000; i++) {
		if (gpon_rd(0x4040) & BIT(15))		/* ETH_PKT_RX_R_ACK */
			break;
		udelay(1);
	}
	return gpon_rd(0x4044);				/* ETH_PKT_RX count */
}

static int gpon_proc_show(struct seq_file *s, void *v)
{
	u32 rst    = gpon_rd(GPON_RESET);
	u32 status = gpon_rd(GPON_GTC_DS_ONU_STATUS);
	u32 eqd    = gpon_rd(GPON_GTC_US_EQD);
	u32 state  = status & GPON_ONU_STATE_MASK;

	seq_printf(s, "version:     0x%02x\n", gpon_rd(GPON_VERSION) & GPON_VER_ID_MASK);
	seq_printf(s, "reset:       0x%08x (soft_rst=%d rst_done=%d)\n",
		   rst, !!(rst & GPON_SOFT_RST), !!(rst & GPON_RST_DONE));
	seq_printf(s, "onu_state:   O%u (%s)\n", state,
		   state < ARRAY_SIZE(gpon_onu_state_name) &&
		   gpon_onu_state_name[state] ? gpon_onu_state_name[state] : "?");
	seq_printf(s, "onu_id:      %u\n",
		   (status >> GPON_ONU_ID_SHIFT) & GPON_ONU_ID_MASK);
	seq_printf(s, "eqd:         inframe=%u multiframe=%u\n",
		   eqd & GPON_EQD_INFRAME_MASK,
		   (eqd >> GPON_EQD_MF_SHIFT) & GPON_EQD_MF_MASK);
	seq_printf(s, "min_delay:   0x%08x\n", gpon_rd(GPON_GTC_US_MIN_DELAY));
	seq_printf(s, "us_cfg:      0x%08x (ben_polar=%u scrm_dis=%u plm_dis=%u)\n",
		   gpon_rd(GPON_GTC_US_CFG),
		   (gpon_rd(GPON_GTC_US_CFG) >> 3) & 1,
		   gpon_rd(GPON_GTC_US_CFG) & 1,
		   (gpon_rd(GPON_GTC_US_CFG) >> 9) & 1);
	seq_printf(s, "us_laser:    0x%08x (lon=%u loff=%u)\n",
		   gpon_rd(GPON_GTC_US_LASER),
		   (gpon_rd(GPON_GTC_US_LASER) >> 8) & 0x3f,
		   gpon_rd(GPON_GTC_US_LASER) & 0x3f);
	{
		int i;

		seq_printf(s, "boh_cfg:     0x%08x (repeat=%u length=%u) data=",
			   gpon_rd(GPON_GTC_US_BOH_CFG),
			   (gpon_rd(GPON_GTC_US_BOH_CFG) >> 8) & 0xf,
			   gpon_rd(GPON_GTC_US_BOH_CFG) & 0xff);
		for (i = 0; i < GPON_BOH_LEN; i++)
			seq_printf(s, "%02x", gpon_rd(GPON_GTC_US_BOH_DATA + i * 4) & 0xff);
		seq_puts(s, "\n");
	}
	seq_printf(s, "test:        0x%08x\n", gpon_rd(GPON_TEST));
	seq_printf(s, "intr_mask:   0x%08x\n", gpon_rd(GPON_INTR_MASK));
	seq_printf(s, "intr_sts:    0x%08x\n", gpon_rd(GPON_INTR_STS));
	seq_printf(s, "gtc_ds_dlt:  0x%08x\n", gpon_rd(GPON_GTC_DS_INTR_DLT));
	seq_printf(s, "gtc_ds_mask: 0x%08x\n", gpon_rd(GPON_GTC_DS_INTR_MASK));
	seq_printf(s, "gtc_ds_sts:  0x%08x\n", gpon_rd(GPON_GTC_DS_INTR_STS));
	{
		u32 los = gpon_rd(GPON_GTC_DS_LOS_CFG_STS);

		seq_printf(s, "los_cfg_sts: 0x%08x (optic_los=%d cdr_los=%d en=%d polar=%d)\n",
			   los, !!(los & GPON_OPTIC_LOS_SIG),
			   !!(los & GPON_CDR_LOS_SIG),
			   !!(los & GPON_OPTIC_LOS_EN),
			   !!(los & GPON_OPTIC_LOS_POLAR));
	}

	/*
	 * PLOAM channel view. Read-only: this decodes the management-channel
	 * buffers/queues so the activation handshake can be observed. With no
	 * downstream PLOAM and nothing queued upstream, the buffers must read
	 * empty (ds buf_empty=1, us nrm/urg empty=1) and the assigned ONU-ID 255
	 * — a self-consistency check that validates the register offsets even
	 * before a live OLT is attached.
	 */
	{
		u32 ds_ind  = gpon_rd(GPON_GTC_DS_PLOAM_IND);
		u32 us_ind  = gpon_rd(GPON_GTC_US_PLOAM_IND);
		u32 us_cfg  = gpon_rd(GPON_GTC_US_PLOAM_CFG);
		u32 us_onu  = gpon_rd(GPON_GTC_US_ONU_ID);

		seq_printf(s, "ds_ploam_ind:  0x%08x (buf_empty=%d buf_full=%d)\n",
			   ds_ind, !!(ds_ind & GPON_DS_PLM_BUF_EMPTY),
			   !!(ds_ind & GPON_DS_PLM_BUF_FULL));
		seq_printf(s, "us_ploam_ind:  0x%08x (nrm_empty=%d nrm_full=%d urg_empty=%d urg_full=%d)\n",
			   us_ind, !!(us_ind & GPON_US_PLM_NRM_EMPTY),
			   !!(us_ind & GPON_US_PLM_NRM_FULL),
			   !!(us_ind & GPON_US_PLM_URG_EMPTY),
			   !!(us_ind & GPON_US_PLM_URG_FULL));
		seq_printf(s, "us_ploam_cfg:  0x%08x (crc_gen=%d onuid_ovrd=%d)\n",
			   us_cfg, !!(us_cfg & GPON_US_PLM_CRC_GEN_EN),
			   !!(us_cfg & GPON_US_PLM_ONUID_OVRD));
		seq_printf(s, "us_onu_id:     %u\n",
			   (us_onu >> GPON_GTC_US_ONU_ID_SHIFT) & GPON_ONU_ID_MASK);
		seq_printf(s, "ds_ploam_cfg:  0x%08x\n",
			   gpon_rd(GPON_GTC_DS_PLOAM_CFG));
	}

	/*
	 * DS framer config registers vs their O5 operating values:
	 * ds_cfg(0x1014)=0x620 intr_mask(0x1004)=0x70f r1048=superframe-cnt
	 * r104c=0x400003e8 r1050=0x00010fa0. A mismatch in ds_cfg is the prime
	 * suspect for "configured correctly yet won't frame-lock".
	 */
	seq_printf(s, "gtc_cfg: ds_cfg=0x%08x intr_mask=0x%08x r1048=0x%08x r104c=0x%08x r1050=0x%08x\n",
		   gpon_rd(0x1014), gpon_rd(0x1004), gpon_rd(0x1048),
		   gpon_rd(0x104c), gpon_rd(0x1050));
	/* DS_MISC counters (GTC-relative, chipdef 0x7011xx): do we even SEE/accept the
	 * OLT's BWmap grants + DS PLOAMs? ploam_acpt/bwm_acpt nonzero => GTC recognizes
	 * grants and asserts BEN (so a zero at the OLT = analog SerDes-TX emission);
	 * bwm_fail/inv nonzero => grants seen but rejected (CRC/format); all zero =>
	 * GTC never sees the OLT grants (downstream BWmap parse issue). */
	seq_printf(s, "ds_cntr: ploam_acpt=%u ploam_fail=%u bwm_acpt=%u bwm_fail=%u bwm_inv=%u active=%u\n",
		   gpon_rd(0x119c), gpon_rd(0x11a0), gpon_rd(0x11b0),
		   gpon_rd(0x11a4), gpon_rd(0x11a8), gpon_rd(0x11ac));
	seq_printf(s, "gem_ds_rx: omcc(f64)=%u f0=%u f1=%u  (>0 => OLT is sending DS GEM/OMCI)\n",
		   gpon_gem_ds_rx_cnt(64), gpon_gem_ds_rx_cnt(0), gpon_gem_ds_rx_cnt(1));
	seq_printf(s, "io: io_mode_en=0x%08x gpio_en0=0x%08x gpio_en1=0x%08x  (operational 0x12050/0x40202006/0x819)\n",
		   sw_rd(SOC_IO_MODE_EN), sw_rd(SOC_IO_GPIO_EN), sw_rd(SOC_IO_GPIO_EN + 4));

	/*
	 * SerDes RX/analog readback, for comparison against the known-good O5 values
	 * (light present): com03=0x8941 com26=0x11e4 gpon42=0x225c dig18=0x1000
	 * misc02=0x3000 fib21 bit13=ANALOG_READY. Mismatches here explain an RX that
	 * won't lock.
	 */
	seq_printf(s, "sds: dig00=0x%08x dig1d=0x%08x  (golden 0xf30 / 0x1c000)\n",
		   sw_rd(WSDS_DIG_00), sw_rd(WSDS_DIG_1D));
	seq_printf(s, "sds: com03=0x%08x com26=0x%08x gpon42=0x%08x dig18=0x%08x\n",
		   sw_rd(SDS_ANA_COM_REG03), sw_rd(SDS_ANA_COM_REG26),
		   sw_rd(SDS_ANA_GPON_REG42), sw_rd(WSDS_DIG_18));
	seq_printf(s, "sds: misc00=0x%08x misc01=0x%08x misc02=0x%08x fib21=0x%08x\n",
		   sw_rd(SDS_ANA_MISC_REG00), sw_rd(SDS_ANA_MISC_REG01),
		   sw_rd(SDS_ANA_MISC_REG02), sw_rd(FIB_EXT_REG21));
	/* TX-path verification: confirm my TX serializer writes actually landed. */
	seq_printf(s, "sds_tx: cfg=0x%08x com22_txamp=0x%08x reg24=0x%08x dig1e_d2a=0x%08x dig02_pdben=0x%08x dig03_txdis=0x%08x forceben=0x%08x\n",
		   sw_rd(SDS_CFG), sw_rd(0x225d8), sw_rd(0x225e0),
		   sw_rd(0x220a8), sw_rd(0x22038), sw_rd(0x2203c), sw_rd(0x220e4));
	{
		u32 fibsts = sw_rd(SDS_FIB_STATUS);

		seq_printf(s, "sds: fib_status=0x%08x (sds_sdet=%u fib100_sdet=%u link_ok=%u) fib_reg16=0x%08x\n",
			   fibsts, !!(fibsts & SDS_FIB_SDS_SDET),
			   !!(fibsts & BIT(2)), !!(fibsts & BIT(4)),
			   sw_rd(FIB_REG16));
	}
	seq_printf(s, "bosa: rtl8290b num=0x%04x vid=0x%02x w41=0x%02x ctrl2=0x%02x status2=0x%02x\n",
		   bosa_id_num, bosa_id_vid, bosa_w41, bosa_ctrl2, bosa_status2);
	seq_printf(s, "fsm: state=O%u onu_id=%u sn_tx=%u ds_rx=%u sds_sync=%u ticks=%u sn=%*phN\n",
		   gpon_fsm_state, gpon_fsm_onu_id, gpon_fsm_sn_tx, gpon_ds_rx,
		   gpon_sds_synced, gpon_fsm_ticks, 8, gpon_sn_bytes);
	/* Live BOSA laser-emission status: R30 (txsd/valid/apc-done/mpd-fault),
	 * R33 bias-DAC readback (nonzero => bias driven), R32 mod, FAULT_STATUS. */
	{
		int r30 = bosa_read_reg(0x31e), r33 = bosa_read_reg(0x321);
		int r32 = bosa_read_reg(0x320), fault = bosa_read_reg(0x389);

		seq_printf(s, "bosa_tx: R30=0x%02x txsd=%d valid=%d apc_done=%d mpd_vhi=%d mpd_vlo=%d bias=0x%02x mod=0x%02x fault=0x%02x\n",
			   r30 & 0xff, !!(r30 & BIT(6)), !!(r30 & BIT(5)),
			   !!(r30 & BIT(7)), !!(r30 & BIT(3)), !!(r30 & BIT(2)),
			   r33 & 0xff, r32 & 0xff, fault & 0xff);
	}
	/* BOSA page0 (slave 0x50) control regs — compare against the O5 operating
	 * state to find the APCDIG clock/power enable (O5 p0: 02 04 0b ff ff ff ff 0c
	 * .. 52 54 20). */
	seq_printf(s, "bosa_p0: 00=%02x 01=%02x 02=%02x 03=%02x 04=%02x 05=%02x 08=%02x 0c=%02x 10=%02x 14=%02x 18=%02x 1c=%02x\n",
		   bosa_read_reg(0x00) & 0xff, bosa_read_reg(0x01) & 0xff,
		   bosa_read_reg(0x02) & 0xff, bosa_read_reg(0x03) & 0xff,
		   bosa_read_reg(0x04) & 0xff, bosa_read_reg(0x05) & 0xff,
		   bosa_read_reg(0x08) & 0xff, bosa_read_reg(0x0c) & 0xff,
		   bosa_read_reg(0x10) & 0xff, bosa_read_reg(0x14) & 0xff,
		   bosa_read_reg(0x18) & 0xff, bosa_read_reg(0x1c) & 0xff);
	seq_printf(s, "bosa_p3: 31c=%02x 31d=%02x 31f=%02x 322=%02x 323=%02x (O5 00 33 00 da 00)\n",
		   bosa_read_reg(0x31c) & 0xff, bosa_read_reg(0x31d) & 0xff,
		   bosa_read_reg(0x31f) & 0xff, bosa_read_reg(0x322) & 0xff,
		   bosa_read_reg(0x323) & 0xff);
	seq_printf(s, "bosa_p2: W54_236=%02x W56_238=%02x W57_239=%02x W61_24d=%02x W88_284=%02x (O5 19 22 2d b0 76)\n",
		   bosa_read_reg(0x236) & 0xff, bosa_read_reg(0x238) & 0xff,
		   bosa_read_reg(0x239) & 0xff, bosa_read_reg(0x24d) & 0xff,
		   bosa_read_reg(0x284) & 0xff);
	seq_printf(s, "bosa_apc: W69_245=%02x(loopmode) W58_23a=%02x(iavg) W72_248=%02x(biasmax) W73_249=%02x(biasmin 0x2a)\n",
		   bosa_read_reg(0x245) & 0xff, bosa_read_reg(0x23a) & 0xff,
		   bosa_read_reg(0x248) & 0xff, bosa_read_reg(0x249) & 0xff);
	return 0;
}

/* FSM state exposed to /proc (defined with the FSM below). */

/*
 * ===== G.984.3 PLOAM activation FSM (drives the ONU O1 -> O5) =====
 *
 * The MAC is a software-PLOAM design: a poll timer drains the downstream PLOAM
 * receive buffer, runs the activation state machine, and composes upstream
 * Serial_Number_ONU / takes the OLT-assigned ONU-ID and ranging delay. The OLT
 * (observed live) broadcasts Upstream_Overhead (type 0x01, ONU-ID 0xff) to
 * acquire unregistered ONUs; we answer with our Serial_Number_ONU, then accept
 * Assign_ONU-ID (0x03) and Ranging_Time (0x04) to reach O5.
 *
 * Per-board serial number stays OUT of the image: default below is overridable
 * via the `gpon.onu_sn=` module/cmdline param (and is wired to gpon_provision's
 * factory value for the fleet). Format (G.984.3 ONU-SN): 4 ASCII ID chars + 8 hex digits.
 */
#define PLM_DS_UPSTREAM_OVERHEAD	0x01
#define PLM_DS_ASSIGN_ONU_ID		0x03
#define PLM_DS_RANGING_TIME		0x04
#define PLM_DS_DEACTIVATE_ONU		0x05
#define PLM_DS_DISABLE_SN		0x06
#define PLM_DS_EXT_BURST_LENGTH		0x14
#define PLM_DS_ENCRYPT_PORT		0x08	/* Encrypted_Port-ID (ACK) */
#define PLM_DS_ASSIGN_ALLOC_ID		0x0a	/* Assign_Alloc-ID (ACK) */
#define PLM_DS_CONFIG_PORT		0x0e	/* Configure_Port-ID (ACK) */
#define PLM_US_SERIAL_NUMBER		0x01
#define PLM_US_ACKNOWLEDGE		0x09	/* US Acknowledge message type */
#define PLM_US_QUEUE_SN			0x6	/* US_PLOAM_IND[10:8] auto-SN queue */
#define PLM_US_QUEUE_URG		0x1	/* US_PLOAM_IND[10:8] urgent queue (ACKs) */

/* Parse "XPON12345678" -> {'X','P','O','N',0x12,0x34,0x56,0x78}. */
static void gpon_parse_sn(const char *s)
{
	int i;

	for (i = 0; i < 4 && s[i]; i++)
		gpon_sn_bytes[i] = s[i];
	for (i = 0; i < 4; i++) {
		u8 hi = 0, lo = 0;

		if (s[4 + 2 * i])
			hi = hex_to_bin(s[4 + 2 * i]);
		if (s[4 + 2 * i + 1])
			lo = hex_to_bin(s[4 + 2 * i + 1]);
		gpon_sn_bytes[4 + i] = (hi << 4) | lo;
	}
}

/* Read the 13-byte downstream PLOAM message (2 bytes per 32-bit word). */
static void gpon_ploam_read(u8 *m)
{
	int i;

	for (i = 0; i < 6; i++) {
		u32 w = gpon_rd(GPON_GTC_DS_PLOAM_MSG + i * 4);

		m[2 * i]     = (w >> 8) & 0xff;
		m[2 * i + 1] = w & 0xff;
	}
	m[12] = (gpon_rd(GPON_GTC_DS_PLOAM_MSG + 6 * 4) >> 8) & 0xff;
}

/* Compose + enqueue an upstream Serial_Number_ONU PLOAM (HW fills CRC). */
/*
 * Compose + transmit a 12-byte upstream PLOAM on the given US_PLOAM_IND queue
 * (PLM_US_QUEUE_SN 0x6 = HW auto-SN slot; PLM_US_QUEUE_URG 0x1 = urgent, used for
 * Acknowledge). Required order: select TYPE and CLEAR ENQ, write the 6 data
 * words, enable HW CRC + ONU-ID override, THEN pulse ENQ 0->1. The
 * enqueue is edge-triggered, so ENQ must be cleared before being set or a repeat
 * is a no-op (the original bug: writing TYPE|ENQ every time left ENQ stuck high
 * with no 0->1 edge, so nothing transmitted).
 */
static void gpon_send_cpu_ploam(u8 queue, const u8 m[12])
{
	u32 ind;
	int i;

	ind = gpon_rd(GPON_GTC_US_PLOAM_IND);
	ind &= ~((0x7u << GPON_US_PLM_TYPE_SHIFT) | GPON_US_PLM_ENQ);
	ind |= ((u32)queue << GPON_US_PLM_TYPE_SHIFT);		/* select queue, ENQ=0 */
	gpon_wr(GPON_GTC_US_PLOAM_IND, ind);

	for (i = 0; i < 6; i++)
		gpon_wr(GPON_GTC_US_PLOAM_DATA + i * 4,
			((u32)m[2 * i] << 8) | m[2 * i + 1]);
	gpon_wr(GPON_GTC_US_PLOAM_CFG,
		GPON_US_PLM_CRC_GEN_EN | GPON_US_PLM_ONUID_OVRD);	/* = 0x13 */

	ind |= GPON_US_PLM_ENQ;			/* ENQ 0->1 edge: transmit */
	gpon_wr(GPON_GTC_US_PLOAM_IND, ind);
}

static void gpon_send_sn(void)
{
	u8 m[12];

	m[0] = 0xff;			/* ONU-ID (unassigned)            */
	m[1] = PLM_US_SERIAL_NUMBER;	/* 0x01                           */
	memcpy(&m[2], gpon_sn_bytes, 8);/* ONU-SN: ID(4) + serial(4)      */
	m[10] = 0x00;			/* random delay (HW may fill)     */
	m[11] = 0x04;			/* G-bit set, power level 0       */

	gpon_send_cpu_ploam(PLM_US_QUEUE_SN, m);
	gpon_fsm_sn_tx++;
}

/*
 * Transmit a US Acknowledge (G.984.3 msg type 0x09) for a downstream PLOAM that
 * requires one (Assign_Alloc-ID, Configure_Port-ID, Encrypted_Port-ID). The OLT
 * arms a post-ranging timer waiting for this ACK; with no reply it Deactivates
 * the ONU (~43s) — this is what stops the ONU staying online after O5. ds points
 * at the full 13-byte DS message (ds[0]=onu_id, ds[1]=type, ds[2..]=payload). The
 * ack echoes the acknowledged message; HW fills the CRC. Sent on the urgent
 * queue so it pre-empts the SN burst.
 */
static void gpon_send_ack(const u8 *ds)
{
	u8 a[12] = { 0 };

	a[0] = gpon_fsm_onu_id;		/* our assigned ONU-ID (HW may override) */
	a[1] = PLM_US_ACKNOWLEDGE;	/* 0x09 */
	a[2] = ds[1];			/* acknowledged message type */
	a[3] = ds[0];			/* acknowledged message ONU-ID */
	a[4] = ds[1];			/* acknowledged message type (echo) */
	memcpy(&a[5], &ds[2], 7);	/* first 7 payload octets */
	gpon_send_cpu_ploam(PLM_US_QUEUE_URG, a);
}

/*
 * OMCI channel (OMCC) GEM datapath. After ranging the OLT assigns the OMCC GEM
 * port via Configure_Port-ID; install it at the fixed RTL9602C OMCI flow/SID 64
 * (T-CONT 16) so DS OMCI GEM frames are de-encapsulated + trapped to the CPU and
 * US OMCI can egress. Register sequence: DS GEM-port CAM write, US GEM-port map,
 * and the PON-IP OMCC bind. GPON-block regs via gpon_wr
 * (offset = phys-0x1b700000); PON-IP datapath regs via pi_wr (offset =
 * phys-0x1bf00000), packed arrays -> read-modify-write.
 */
#define GPON_GTC_DS_PORT_IND	0x1100		/* CAM op: OP_MODE[9:8] OP_IDX[6:0] */
#define   DS_PORT_OP_REQ	BIT(15)
#define   DS_PORT_OP_COMPL	BIT(14)
#define   DS_PORT_OP_WRITE	BIT(8)		/* OP_MODE = WRITE(1) */
#define GPON_GTC_DS_PORT_WR	0x1104		/* [11:0] gemPortId */
#define GPON_GTC_DS_TRAFFIC_CFG	0x1400		/* +flow*4, [4:0] traffic-type */
#define   DS_TRAFFIC_IS_OMCI	BIT(2)
#define GPON_GTC_GEM_US_PORT_MAP 0x6400		/* +flow*4, [11:0] gemPortId */
#define GPON_GTC_DS_OMCI_PTI	0x1204		/* [6:4] PTI_MASK [2:0] END_PTI */
#define   DS_OMCI_PTI_VAL	((1u << 4) | 1u)	/* mask=1 ptn=1 -> 0x11 */
#define GPON_GEM_DS_MC_CFG	0x4080		/* [6] BROADCAST_PASS [4] NON_MULTICAST_PASS [3] FCS_CHK_EN */
#define   GEM_DS_MC_CFG_VAL	0x18u		/* NON_MULTICAST_PASS(4)|FCS_CHK_EN(3): pass unicast OMCI, NOT broadcast (avoids DS flood backup that stalls US); the full operating value is 0x59 (adds BROADCAST_PASS+bit0) */
#define PI_PON_SID2QID		0x020f8		/* packed 7b/SID: physical queue */
#define PI_PON_SIDVALID		0x0213c		/* packed 1b/SID */
#define PI_PON_OMCI_CFG		0x02154		/* [6:0] OMCI SID */
#define PI_PON_SID_Q_MAP_DS	0x0a0e4		/* packed 2b/SID: DS PBO queue */
#define GPON_OMCC_FLOW		64		/* RTL9602C fixed OMCI flow/SID */
#define GPON_OMCC_PHYS_QID	64		/* TCONT_QUEUE_MAX(32)*(TCONT16/8) */
#define GPON_OMCC_DSQ_HIGH	2

/* Set a `bits`-wide entry at index `idx` in the packed pi-register array based at
 * `base` (driver-relative). Read-modify-write. */
static void pi_packed_set(u32 base, unsigned int idx, unsigned int bits, u32 val)
{
	unsigned int per = 32 / bits;
	u32 off = base + (idx / per) * 4;
	unsigned int sh = (idx % per) * bits;
	u32 mask = ((1u << bits) - 1) << sh;

	pi_wr(off, (pi_rd(off) & ~mask) | ((val << sh) & mask));
}

static int gpon_install_omcc(u16 gem)
{
	int i;

	/* DS GEM-port CAM: map gem -> flow 64, mark isOMCI. */
	gpon_wr(GPON_GTC_DS_PORT_IND, DS_PORT_OP_WRITE | (GPON_OMCC_FLOW & 0x7f));
	gpon_wr(GPON_GTC_DS_PORT_WR, gem & 0xfff);
	gpon_wr(GPON_GTC_DS_PORT_IND,
		DS_PORT_OP_WRITE | (GPON_OMCC_FLOW & 0x7f) | DS_PORT_OP_REQ);
	for (i = 0; i < 1000; i++) {		/* bounded; timeout != success */
		if (gpon_rd(GPON_GTC_DS_PORT_IND) & DS_PORT_OP_COMPL)
			break;
		udelay(1);
	}
	if (i == 1000) {
		pr_err("rtl9602c-gpon: OMCC DS GEM install timeout\n");
		return -ETIMEDOUT;
	}
	gpon_wr(GPON_GTC_DS_TRAFFIC_CFG + GPON_OMCC_FLOW * 4, DS_TRAFFIC_IS_OMCI);

	/* DS OMCI PTI: tell the GTC how to detect the end of an OMCI GEM frame for
	 * reassembly (PTI_MASK[6:4]=1 compares GEM-header PTI bit0; END_PTI[2:0]=1 =
	 * that bit set marks end-of-OMCI-fragment). At reset this register is 0, so
	 * the GTC never recognises an OMCI frame boundary and drops every downstream
	 * OMCI frame — the reason DS OMCI never reaches the CPU. Operating value 0x11. */
	gpon_wr(GPON_GTC_DS_OMCI_PTI, DS_OMCI_PTI_VAL);

	/* GEM DS pass config: WITHOUT NON_MULTICAST_PASS (bit4) the GTC drops every
	 * unicast downstream GEM frame BEFORE de-encapsulation — including OMCI, which
	 * the OLT sends unicast on the OMCC GEM port — so OMCI never reaches the flow
	 * datapath or the CPU (DS GEM RX counter stays 0). At reset this register is 0.
	 * The O5 operating value 0x59 = BROADCAST_PASS(6) | NON_MULTICAST_PASS(4) | FCS_CHK_EN(3).
	 * DISABLED for now: opening this gate lets de-encapsulated unicast OMCI flow, but
	 * the PON-IP -> GMAC-NIC drain is not yet built, so the frames back up and the US
	 * stalls (deactivate ~48s). Re-enable once the PON-IP->host OMCI DMA path lands. */
	if (gem_gate_open)
		gpon_wr(GPON_GEM_DS_MC_CFG, GEM_DS_MC_CFG_VAL);

	/* US GEM-port map for flow 64. */
	gpon_wr(GPON_GTC_GEM_US_PORT_MAP + GPON_OMCC_FLOW * 4, gem & 0xfff);

	/* PON-IP: flow->queue, SID-valid, OMCI-SID, DS PBO high queue. */
	pi_packed_set(PI_PON_SID2QID, GPON_OMCC_FLOW, 7, GPON_OMCC_PHYS_QID & 0x7f);
	pi_packed_set(PI_PON_SIDVALID, GPON_OMCC_FLOW, 1, 1);
	pi_field(PI_PON_OMCI_CFG, 6, 0, GPON_OMCC_FLOW);
	pi_packed_set(PI_PON_SID_Q_MAP_DS, GPON_OMCC_FLOW, 2, GPON_OMCC_DSQ_HIGH);

	/* Arm the NIC OMCI trap so DS stream-64 frames reach the CPU netdev. */
	rtl9602c_eth_set_omci_sid(GPON_OMCC_FLOW);

	pr_info("rtl9602c-gpon: OMCC installed gem=%u flow=%u (compl %d)\n",
		gem, GPON_OMCC_FLOW, i);
	return 0;
}

#define GPON_GTC_DS_ALLOC_IND	0x10c0		/* T-CONT alloc CAM: OP_IDX[4:0]=tcont */
#define GPON_GTC_DS_ALLOC_WR	0x10c4		/* [11:0] allocateId */
#define GPON_OMCC_TCONT		16

/*
 * Bind an OLT-assigned Alloc-ID to a T-CONT in the GTC alloc CAM (Assign_Alloc-ID
 * handler). Without this the GTC does not associate the OLT's BWMAP grants for
 * the Alloc-ID with a T-CONT, so the ONU never transmits upstream on it and the
 * OLT sees the upstream as dead (and US OMCI has no T-CONT to egress on). Same
 * indirect-CAM op as the DS GEM-port CAM (OP bit positions shared).
 */
static int gpon_install_tcont(u8 tcont, u16 alloc)
{
	int i;

	gpon_wr(GPON_GTC_DS_ALLOC_IND, DS_PORT_OP_WRITE | (tcont & 0x1f));
	gpon_wr(GPON_GTC_DS_ALLOC_WR, alloc & 0xfff);
	gpon_wr(GPON_GTC_DS_ALLOC_IND,
		DS_PORT_OP_WRITE | (tcont & 0x1f) | DS_PORT_OP_REQ);
	for (i = 0; i < 1000; i++) {		/* bounded; timeout != success */
		if (gpon_rd(GPON_GTC_DS_ALLOC_IND) & DS_PORT_OP_COMPL)
			break;
		udelay(1);
	}
	if (i == 1000) {
		pr_err("rtl9602c-gpon: T-CONT alloc bind timeout\n");
		return -ETIMEDOUT;
	}
	pr_info("rtl9602c-gpon: T-CONT %u <- alloc 0x%x bound (compl %d)\n",
		tcont, alloc, i);
	return 0;
}

/*
 * Several upstream-config registers (US_CFG, US_LASER, MIN_DELAY) sit behind a
 * write-protect gate: a write only takes effect while the protect register holds
 * the unlock magic. Arm it, write, relock — this bracket wraps each protected US
 * write. (BOH_DATA/EQD are not gated and are written directly.)
 */
static void gpon_wr_us_protected(u32 off, u32 val)
{
	gpon_wr(GPON_GTC_US_WRITE_PROTECT, GPON_US_WP_UNLOCK);
	gpon_wr(off, val);
	gpon_wr(GPON_GTC_US_WRITE_PROTECT, GPON_US_WP_LOCK);
}

/*
 * Program the upstream burst-mode overhead (PLOu preamble + delimiter) the OLT
 * asks for in its Upstream_Overhead PLOAM (G.984.3, type 0x01). The OLT's burst
 * receiver locks its CDR on the preamble run and frames on the 3-byte delimiter;
 * if our SN burst carries the wrong overhead the OLT cannot decode it, never
 * sees our serial number and never ranges us (it reports "laser out"). The byte
 * layout follows the G.984.3 pre-ranged-overhead computation for the 12-byte
 * (96-bit) pre-ranged overhead:
 *   oh = [0xAA * guard_bytes][type3_ptn * fill][delim0,delim1,delim2]
 * with guard_bytes = min(guard_bits,32)/8, fill making 9 preamble bytes total
 * and the last 3 bytes the delimiter. BOH_CFG.REPEAT marks the last preamble
 * index (size-4); BOH_CFG.LENGTH is the byte count. These registers are not
 * behind the US write-protect gate (only US_CFG is), so no unlock is needed.
 */
/*
 * Program the upstream burst-overhead (preamble + delimiter) from the retained
 * OLT-dictated parameters. Only GPON_BOH_LEN(12) bytes are stored in BOH_DATA:
 *   oh = [0xAA x rep][type3_ptn x (9-rep)][delim0,delim1,delim2]
 * The HW emits a BOH_LENGTH-byte burst by repeating oh[REPEAT] (REPEAT =
 * (size-4)&0xF = 8, i.e. the last Type-3 pattern byte) out to LENGTH-3, then the
 * 3 stored delimiter bytes. The crucial field is BOH_LENGTH: the OLT's
 * Extended_Burst_Length (0x14) sets the Type-3 lengths: t3pre for the
 * pre-ranged (SN/ranging) burst, t3ranged for the ranged (operation) burst:
 *   LENGTH = rep + t3{pre,ranged} + 3   (G.984.3 upstream-overhead length).
 * Without 0x14 (t3==0) fall back to the 96-bit/12-byte default. A too-short
 * pre-ranged preamble is detectable by the OLT but not lockable -> no ranging;
 * an over-long ranged preamble wastes the operation burst -> switch at O5.
 */
static void gpon_apply_boh(bool ranged)
{
	u8 oh[GPON_BOH_LEN];
	u8 guard = gpon_boh_guard, t3 = ranged ? gpon_boh_t3ranged : gpon_boh_t3pre;
	u8 rep, i, len;

	if (guard > 32)
		guard = 32;
	rep = guard / 8;			/* whole guard bytes (0xAA) */

	for (i = 0; i < rep; i++)
		oh[i] = 0xaa;
	for (; i < GPON_BOH_LEN - 3; i++)	/* Type-3 preamble run */
		oh[i] = gpon_boh_ptn;
	oh[GPON_BOH_LEN - 3] = gpon_boh_delim[0];
	oh[GPON_BOH_LEN - 2] = gpon_boh_delim[1];
	oh[GPON_BOH_LEN - 1] = gpon_boh_delim[2];

	len = t3 ? rep + t3 + 3 : GPON_BOH_LEN;
	if (len > GPON_BOH_MAX_LEN)
		len = GPON_BOH_MAX_LEN;

	gpon_wr(GPON_GTC_US_BOH_CFG,
		(((GPON_BOH_LEN - 4) & 0xf) << 8) | (len & 0xff));
	for (i = 0; i < GPON_BOH_LEN; i++)
		gpon_wr(GPON_GTC_US_BOH_DATA + i * 4, oh[i]);

	pr_info("rtl9602c-gpon: BOH %s guard=%u ptn=0x%02x delim=%02x%02x%02x t3=%u len=%u oh=%*phN\n",
		ranged ? "ranged" : "prerng", guard, gpon_boh_ptn,
		gpon_boh_delim[0], gpon_boh_delim[1], gpon_boh_delim[2],
		t3, len, GPON_BOH_LEN, oh);
}

/*
 * Upstream equalization delay. The OLT-visible burst time is value + the local
 * MIN_DELAY1 (read back from the timing register, 290 bits here) scaled to bits
 * (x16x8 = x128), then split across the 19440x8-bit upstream frame into a
 * multiframe count and an in-frame offset. Pre-ranging (value 0) yields
 * 290*128 = 37120 (0x9100) — the correct burst position before the OLT hands us
 * a ranging EqD. (Board eqd_offset = 0.)
 */
static void gpon_set_eqd(u32 value)
{
	u32 min_delay1 = (gpon_rd(GPON_GTC_US_MIN_DELAY) >> 7) & 0x1ff;
	u32 eqd1  = value + min_delay1 * 128;
	u32 multi = eqd1 / GPON_EQD_FRAME_LEN;
	u32 intra = eqd1 - multi * GPON_EQD_FRAME_LEN;

	gpon_wr(GPON_GTC_US_EQD,
		((multi & GPON_EQD_MF_MASK) << GPON_EQD_MF_SHIFT) |
		(intra & GPON_EQD_INFRAME_MASK));
}

static void gpon_fsm_set_state(u8 st)
{
	if (gpon_fsm_state != st)
		pr_info("rtl9602c-gpon: ONU state O%u -> O%u\n", gpon_fsm_state, st);
	gpon_fsm_state = st;
	/* The HW ONU_STATE field uses the same 1-based encoding as our state numbers:
	 * UNKNOWN=0, O1=1, O2=2, O3=3, O4=4, O5=5. So O3 (Serial-Number, where the
	 * GTC's auto-SN-burst transmitter is gated) = register value 3 = our st.
	 * Write st. */
	gpon_field(GPON_GTC_DS_ONU_STATUS, 3, 0, st);
}

static void gpon_fsm_handle(const u8 *m)
{
	u8 onu_id = m[0], type = m[1];
	const u8 *d = &m[2];		/* 10 data octets */

	/* Surface any DS PLOAM that is not the repetitive broadcast acquisition
	 * traffic (Upstream_Overhead 0x01 / profile 0x14) — e.g. Assign_ONU-ID or
	 * anything addressed to us — so activation progress is visible. */
	if (type != PLM_DS_UPSTREAM_OVERHEAD && type != PLM_DS_EXT_BURST_LENGTH)
		pr_info_ratelimited("rtl9602c-gpon: DS PLOAM onu_id=0x%02x type=0x%02x d=%*phN\n",
				    onu_id, type, 8, d);

	switch (type) {
	case PLM_DS_UPSTREAM_OVERHEAD:
		/* OLT is acquiring ONUs (it broadcasts this continuously). On the
		 * O1/O2 -> O3 edge, program the burst overhead + pre-ranging EqD the
		 * OLT dictates BEFORE the first SN, then move to O3; the SN is (re)sent,
		 * throttled, from the poll loop so we don't flood the US PLOAM queue.
		 * G.984.3 Upstream_Overhead payload: d[0]=guard bits, d[3]=preamble
		 * (type3) pattern, d[4..6]=delimiter, d[7] bit5=pre-EqD present with
		 * value d[8:9] (x32x8 bits). */
		if (gpon_fsm_state < 3) {
			u32 pre_eqd = ((d[7] >> 5) & 1) ?
				(((u32)d[8] << 8) | d[9]) * 32 * 8 : 0;

			gpon_boh_guard    = d[0];
			gpon_boh_ptn      = d[3];
			gpon_boh_delim[0] = d[4];
			gpon_boh_delim[1] = d[5];
			gpon_boh_delim[2] = d[6];
			gpon_apply_boh(false);	/* folds in any prior 0x14 t3pre */
			gpon_set_eqd(pre_eqd);
			gpon_fsm_set_state(2);
			gpon_fsm_set_state(3);
			gpon_send_sn();		/* first SN immediately */
		}
		break;
	case PLM_DS_ASSIGN_ONU_ID:
		/* d[0] = assigned ONU-ID, d[1..8] = serial number to match */
		if (!memcmp(&d[1], gpon_sn_bytes, 8)) {
			gpon_fsm_onu_id = d[0];
			gpon_field(GPON_GTC_US_ONU_ID, 15, 8, gpon_fsm_onu_id);
			gpon_field(GPON_GTC_DS_ONU_STATUS, 15, 8, gpon_fsm_onu_id);
			pr_info("rtl9602c-gpon: OLT assigned ONU-ID %u\n",
				gpon_fsm_onu_id);
			gpon_fsm_set_state(4);
		}
		break;
	case PLM_DS_RANGING_TIME:
		/* Accept only the main-path EqD (d[0] bit0 == 0); protect-path EqD is
		 * not configurable. EqD is d[1..4] big-endian and is folded with
		 * MIN_DELAY1 by gpon_set_eqd (same as the pre-ranging path). */
		if (onu_id == gpon_fsm_onu_id && !(d[0] & 0x01)) {
			u32 eqd = ((u32)d[1] << 24) | ((u32)d[2] << 16) |
				  ((u32)d[3] << 8) | d[4];

			gpon_set_eqd(eqd);
			gpon_apply_boh(true);	/* switch to the ranged operation burst */
			pr_info("rtl9602c-gpon: Ranging_Time EqD=0x%x -> O5\n", eqd);
			gpon_fsm_set_state(5);
		}
		break;
	case PLM_DS_DEACTIVATE_ONU:
	case PLM_DS_DISABLE_SN:
		if (onu_id == gpon_fsm_onu_id || onu_id == 0xff) {
			gpon_fsm_onu_id = 0xff;
			gpon_fsm_set_state(1);
		}
		break;
	case PLM_DS_EXT_BURST_LENGTH:
		/* Extended_Burst_Length (G.984.3): d[0] = Type-3 preamble length
		 * for the PRE-RANGED (SN/ranging) burst, d[1] = for the ranged
		 * (operation) burst. The OLT broadcasts this during acquisition;
		 * honoring d[0] lengthens our SN-burst preamble (BOH_LENGTH) so
		 * the OLT's burst receiver can lock and range us. Re-arm while
		 * still broadcast-addressed/pre-ranging. The Extended_Burst_Length
		 * PLOAM is acted on at O3. */
		gpon_boh_t3ranged = d[1];	/* applied at the O5 transition */
		if (gpon_fsm_onu_id == 0xff && gpon_boh_t3pre != d[0]) {
			gpon_boh_t3pre = d[0];
			gpon_apply_boh(false);
			pr_info("rtl9602c-gpon: Extended_Burst_Length type3_preranged=%u ranged=%u\n",
				gpon_boh_t3pre, gpon_boh_t3ranged);
		}
		break;
	case PLM_DS_CONFIG_PORT:
		/* Configure_Port-ID (0x0e): the OLT assigns the OMCC GEM port for OMCI
		 * (d[0] bit0 = enable, gem = (d[1]<<4)|(d[2]>>4)). Install the OMCC GEM
		 * datapath (one-shot) so DS OMCI reaches the CPU, THEN Acknowledge (so
		 * the ONU is RX-ready before the OLT proceeds). */
		if (onu_id == gpon_fsm_onu_id) {
			u16 gem = ((u16)d[1] << 4) | (d[2] >> 4);

			if ((d[0] & 0x1) && !gpon_omcc_installed) {
				if (!gpon_install_omcc(gem))
					gpon_omcc_installed = true;
			}
			gpon_send_ack(m);
			pr_info_ratelimited("rtl9602c-gpon: ACK type=0x%02x d=%*phN\n",
					    type, 8, d);
		}
		break;
	case PLM_DS_ASSIGN_ALLOC_ID:
		/* Assign_Alloc-ID (0x0a): bind the OLT's Alloc-ID (alloc=(d[0]<<4)|
		 * (d[1]>>4)) to the OMCC T-CONT so the ONU answers BWMAP grants for it
		 * (d[2]: 0x01=allocate, 0xff=deallocate). Then Acknowledge. */
		if (onu_id == gpon_fsm_onu_id) {
			u16 alloc = ((u16)d[0] << 4) | (d[1] >> 4);

			if (d[2] == 0x01 && !gpon_tcont_installed) {
				if (!gpon_install_tcont(GPON_OMCC_TCONT, alloc))
					gpon_tcont_installed = true;
			}
			gpon_send_ack(m);
			pr_info_ratelimited("rtl9602c-gpon: ACK type=0x%02x d=%*phN\n",
					    type, 8, d);
		}
		break;
	case PLM_DS_ENCRYPT_PORT:
		/* Encrypted_Port-ID (0x08): G.984.3 requires a US Acknowledge; the OLT
		 * arms a ~43s timer and Deactivates us if none arrives. */
		if (onu_id == gpon_fsm_onu_id) {
			gpon_send_ack(m);
			pr_info_ratelimited("rtl9602c-gpon: ACK type=0x%02x d=%*phN\n",
					    type, 8, d);
		}
		break;
	}
}

static void gpon_fsm_poll(struct timer_list *t)
{
	int guard = 0;

	gpon_fsm_ticks++;
	while (!(gpon_rd(GPON_GTC_DS_PLOAM_IND) & GPON_DS_PLM_BUF_EMPTY) &&
	       guard++ < 16) {
		u8 m[13];

		gpon_ploam_read(m);
		gpon_fsm_handle(m);
		gpon_ds_rx++;					/* DS-lock liveness */
		gpon_wr(GPON_GTC_DS_PLOAM_IND, GPON_DS_PLM_DEQ);	/* advance */
	}
	/* Periodic SerDes-TX re-sync while UN-RANGED. The upstream-burst serializer
	 * lock is non-deterministic (the OLT decodes our SN burst only intermittently —
	 * confirmed by the OLT alarm log: "authorization success" appears, but not on
	 * demand). Re-pulse the TX-interface reset-B (WSDS_DIG_1D[16] CFG_SFT_RSTB_INF_TX
	 * 0->1) ~every 2s so the TX serializer keeps re-attempting to lock onto the
	 * framer burst data — this re-sync is the mechanism that historically caught the
	 * lock and got the OLT to range the ONU. TX-interface only (not the PLL), so the
	 * locked RX downstream framer is undisturbed. Short udelay only (softirq). The
	 * old version wrote WRONG ModeV2 values (0x225ac/0x225d8) and corrupted the rev-A
	 * ModeV1 TX config — that is removed; this toggles only the reset-B. */
	if (gpon_fsm_state >= 3 && gpon_fsm_onu_id == 0xff &&
	    (gpon_fsm_ticks % 200) == 0) {
		sw_field(WSDS_DIG_1D, 16, 16, 0);
		udelay(500);
		sw_field(WSDS_DIG_1D, 16, 16, 1);
		gpon_sds_synced++;
	}
	/* While unregistered in O3, re-offer our Serial_Number_ONU ~twice a second
	 * (the OLT grants SN windows intermittently). */
	if (gpon_fsm_state >= 3 && gpon_fsm_onu_id == 0xff &&
	    (gpon_fsm_ticks % 50) == 0)
		gpon_send_sn();
	/* Continuous laser keep-lit: once ignited, service any BOSA TX fault every
	 * ~50ms (5 x 10ms ticks) so a transient TX_FAULT after DIGITAL_POWER_ON does
	 * not leave the laser latched dark. This is the continuous laser INT/fault poll.
	 * Runs in softirq — bosa_laser_maint() does at most a bounded 500us strobe. */
	if (bosa_laser_up && (gpon_fsm_ticks % 5) == 0)
		bosa_laser_maint();
	mod_timer(&gpon_fsm_timer, jiffies + msecs_to_jiffies(10));
}

static int __init rtl9602c_gpon_init(void)
{
	u32 ver, rst, test;

	gpon_base = ioremap(GPON_PHYS_BASE, GPON_REG_SIZE);
	if (!gpon_base) {
		pr_err("rtl9602c-gpon: ioremap 0x%08x failed\n", GPON_PHYS_BASE);
		return -ENOMEM;
	}
	swcore_base = ioremap(SWCORE_PHYS_BASE, SWCORE_REG_SIZE);
	if (!swcore_base) {
		pr_err("rtl9602c-gpon: ioremap 0x%08x failed\n", SWCORE_PHYS_BASE);
		iounmap(gpon_base);
		return -ENOMEM;
	}

	/* Power up the PON packet-datapath IP domain (see SOC_IP_ENABLE_PHYS). */
	{
		void __iomem *ipen = ioremap(SOC_IP_ENABLE_PHYS, 4);

		if (ipen) {
			writel(readl(ipen) | SOC_IP_EN_PON, ipen);
			(void)readl(ipen);		/* post the write */
			iounmap(ipen);
		}
	}

	/* PON-IP datapath window — only reachable now the IP-enable bit is set. */
	ponip_base = ioremap(PONIP_PHYS_BASE, PONIP_REG_SIZE);
	if (!ponip_base) {
		pr_err("rtl9602c-gpon: ioremap 0x%08x failed\n", PONIP_PHYS_BASE);
		iounmap(swcore_base);
		iounmap(gpon_base);
		return -ENOMEM;
	}

	ver  = gpon_rd(GPON_VERSION) & GPON_VER_ID_MASK;
	rst  = gpon_rd(GPON_RESET);
	test = gpon_rd(GPON_TEST);

	pr_info("rtl9602c-gpon: MAC @0x%08x ver=0x%02x reset=0x%08x test=0x%08x\n",
		GPON_PHYS_BASE, ver, rst, test);

	/*
	 * Self-test the register window with the GPON_TEST scratch register:
	 * write a pattern, read it back, then restore the power-on value.
	 */
	gpon_wr(GPON_TEST, 0xa5a5a5a5u);
	if (gpon_rd(GPON_TEST) == 0xa5a5a5a5u)
		pr_info("rtl9602c-gpon: register R/W OK (scratch verified)\n");
	else
		pr_warn("rtl9602c-gpon: scratch R/W failed — MAC may be gated\n");
	gpon_wr(GPON_TEST, GPON_TEST_SCRATCH);

	/*
	 * Configure the GPIO pads to the known-good (O5) state so the optical
	 * signal-detect pin is enabled and sampled. The function-enable bits live
	 * in the switch-core IO_GPIO_EN words; the direction/data live in the SoC
	 * GPIO controller at phys 0x18003300 (its own window).
	 */
	sw_wr(SOC_IO_GPIO_EN, SOC_IO_GPIO_EN_W0);
	sw_wr(SOC_IO_GPIO_EN + 4, SOC_IO_GPIO_EN_W1);
	{
		void __iomem *gpio = ioremap(GPIO_PHYS_BASE, GPIO_REG_SIZE);

		if (gpio) {
			iowrite32(GPIO_GOLD_DIR_ABCD,  gpio + GPIO_DIR_ABCD);
			iowrite32(GPIO_GOLD_DATA_ABCD, gpio + GPIO_DATA_ABCD);
			iowrite32(GPIO_GOLD_DIR_EFGH,  gpio + GPIO_DIR_EFGH);
			iowrite32(GPIO_GOLD_DATA_EFGH, gpio + GPIO_DATA_EFGH);
			(void)ioread32(gpio + GPIO_DIR_ABCD);	/* post writes */
			iounmap(gpio);
		}
	}
	pr_info("rtl9602c-gpon: GPIO pads set (gpio_en0=0x%08x gpio_en1=0x%08x)\n",
		sw_rd(SOC_IO_GPIO_EN), sw_rd(SOC_IO_GPIO_EN + 4));

	/*
	 * Bring up the PON SerDes so the MAC core gets its clock, then confirm the
	 * MAC completed reset. Neither failing is fatal — the register window stays
	 * usable and /proc/gpon reports the live state for diagnosis.
	 */
	if (gpon_serdes_init())
		pr_warn("rtl9602c-gpon: SerDes analog-ready not seen (FIB_EXT_REG21=0x%08x)\n",
			sw_rd(FIB_EXT_REG21));
	else
		pr_info("rtl9602c-gpon: PON SerDes up (GPON mode, analog ready)\n");

	/*
	 * Probe the external RTL8290B BOSA over I2C (read-only chip-ID check). The
	 * optical RX signal-detect comes from this chip; a working unit initialises
	 * it over I2C and only then does SDS_FIB_STATUS.SDS_SDET assert. This
	 * validates the I2C transport before the RX-enable writes are added.
	 */
	bosa_probe();

	/*
	 * BISECTION: when skip_bosa=1 (warm boot), leave the external BOSA in whatever
	 * state it is already in (a working BOSA config persists across a SoC warm
	 * reset since the BOSA is externally powered) and only run the SoC-side
	 * SerDes/PON-IP/MAC/FSM. If the ONU then ranges online, the datapath is correct
	 * and the ONLY gap is the BOSA cold-init.
	 */
	if (skip_bosa) {
		pr_info("rtl9602c-gpon: skip_bosa=1 -> leaving BOSA as-is (bisection)\n");
		goto skip_bosa_init;
	}

	/*
	 * Power on the BOSA optical receiver (clears its RX power-down). This is
	 * what makes the real optical signal-detect assert — run it before the GPON
	 * MAC reset below so the downstream framer locks on real recovered bits.
	 */
	bosa_rx_enable();

	/* Power on the BOSA optical transmitter (laser bias/modulation/APC) so the
	 * ONU can send upstream PLOAM bursts during activation. The APC offset
	 * calibration is deferred until after the PON-IP datapath + MAC reset below,
	 * because the APC digital block only clocks once the SerDes/PON TX clock is
	 * running (calibrating earlier leaves its readout dead -> OFFK_DONE never
	 * asserts). */
	if (!laser_off)
		bosa_tx_enable();

skip_bosa_init:
	/*
	 * Configure the PON-IP packet datapath (page accounting, GPON mode, GMII)
	 * so the MAC has a place to land downstream frames before it is reset.
	 */
	gpon_pbo_init();
	pr_info("rtl9602c-gpon: PON-IP datapath configured (ctl_us=0x%08x ctl_ds=0x%08x)\n",
		pi_rd(PI_PONIP_CTL_US), pi_rd(PI_PONIP_CTL_DS));

	/*
	 * With the SerDes clock now present, soft-reset the GPON MAC block so its
	 * RST_DONE handshake can complete and the GTC banks come out of reset.
	 */
	gpon_wr(GPON_RESET, GPON_SOFT_RST);
	gpon_wr(GPON_RESET, 0);
	if (gpon_wait_rst_done())
		pr_warn("rtl9602c-gpon: RST_DONE not seen (reset=0x%08x)\n",
			gpon_rd(GPON_RESET));
	else
		pr_info("rtl9602c-gpon: MAC reset done, ONU state O%u\n",
			gpon_rd(GPON_GTC_DS_ONU_STATUS) & GPON_ONU_STATE_MASK);

	/*
	 * Enable optical loss-of-signal monitoring with inverted polarity. The
	 * downstream framer gates on OPTIC_LOS_SIG; until the LOS input is enabled
	 * and given the correct (inverted) polarity, that status reads "loss" even
	 * with real downstream light, holding the FSM in O1. A working (O5) unit
	 * runs this register at 0x03 (OPTIC_LOS_EN=1, OPTIC_LOS_POLAR=1).
	 */
	gpon_field(GPON_GTC_DS_LOS_CFG_STS, 0, 0, 1);	/* OPTIC_LOS_EN = 1     */
	gpon_field(GPON_GTC_DS_LOS_CFG_STS, 1, 1, 1);	/* OPTIC_LOS_POLAR = 1  */
	pr_info("rtl9602c-gpon: optical-LOS monitor enabled (los_cfg=0x%08x)\n",
		gpon_rd(GPON_GTC_DS_LOS_CFG_STS));

	/* Now that the PON-IP/MAC (and thus the SerDes TX clock) are running, run
	 * the laser APC offset calibration so the laser actually biases. */
	if (!skip_bosa && !laser_off && !apc_off)
		bosa_apc_calibrate();

	proc_create_single("gpon", 0444, NULL, gpon_proc_show);
	proc_create_single("bosadump", 0444, NULL, bosadump_proc_show);

	/*
	 * Upstream burst CONFIG + laser-enable timing.  The GTC MAC reset above
	 * clears US_CFG, so it must be (re)programmed here or the burst-enable
	 * polarity and laser on/off window default wrong and the laser never
	 * modulates a burst the OLT can see.  These are the operating values for this
	 * board while ranged online:
	 *   US_CFG   0x0c18 = US_BEN_POLAR=1, scrambler on, PLOAM on, auto-DG on
	 *   US_LASER 0x2028 = LON_TIME=32, LOFF_TIME=40 (laser-enable burst edges)
	 * Both sit behind the US write-protect gate.
	 */
	gpon_wr_us_protected(GPON_GTC_US_CFG,
			     GPON_US_CFG_VAL | (force_laser ? BIT(15) : 0));
	if (force_laser)
		pr_info("rtl9602c-gpon: force_laser=1 -> US_CFG.FS_LON set (CW diagnostic)\n");
	gpon_wr_us_protected(GPON_GTC_US_LASER, GPON_US_LASER_VAL);

	/*
	 * (Removed the brief-CW OFFK-converge step: it DID converge OFFK (R30=0xa0)
	 * but drove the bias to the CW operating point 0x4c — higher idle emission,
	 * DS RX still dead. Elimination across experiments shows the RX-killer is the
	 * IDLE-emission LEVEL, i.e. the bias sitting ABOVE the lasing threshold during
	 * acquisition: 0x18 dead, 0x4c dead; an O3 acquisition bias ~0x0a (below
	 * threshold) keeps idle light minimal so DS RX survives, and the SN burst's
	 * modulation rides on top. OFFK converges later, during ranging/O5. So the
	 * laser bias is loaded LOW for acquisition (see bosa_apc_calibrate).)
	 */

	/*
	 * ★ Upstream SN-burst ARMING — the GTC-level conditions the silicon requires.
	 * The GTC auto-fires the loaded Serial_Number PLOAM into
	 * the OLT's broadcast SN grant ONLY if all of these are armed; without them the
	 * software sn_tx counter climbs (template enqueued) but NO burst is transmitted
	 * into a grant -> OLT "Received Ploams = 0" (the exact symptom: ranged once
	 * historically, but the current build never emits a decodable SN burst).
	 *
	 * (1) ONU-ID = 0xFF (broadcast) into BOTH the DS and US ONU-ID register fields.
	 *     DS_CFG has BWM_FILT_ONUID set, so the GTC only ACTS on BWmap grants whose
	 *     ONU-ID matches the PROGRAMMED field; the OLT's pre-assignment SN grant is
	 *     addressed to 0xFF, so the field must be 0xFF or the grant is filtered out
	 *     (no grant serviced -> no burst). Both ONU-ID fields are written at init.
	 * (2) BWM_NO_FLT (DS_CFG 0x1014 bit11) = 1: belt-and-suspenders for bring-up —
	 *     bypass the BWmap ONU-ID filter entirely so EVERY grant is accepted (in
	 *     case the 0xFF compare is off). Removable once ranging is confirmed.
	 * (3) US_PLOAM_CFG = CRC_GEN_EN|ONUID_OVRD armed at INIT (not only per-send):
	 *     the auto-SN burst needs a HW PLOAM CRC8 + ONU-ID-stamped header or the OLT
	 *     silently discards it.
	 * (4) AUTO_PROC_SSTART (US_PROC_MODE 0x5200 bit0, behind the US write-protect):
	 *     HW auto-aligns the small SN burst to the BWmap-granted StartTime, so the
	 *     burst lands inside the OLT's RX window.
	 * (5) DS_PLOAM_CFG broadcast-accept + ONU-ID filter (accept the broadcast
	 *     Serial_Number_Request / Assign_ONU-ID PLOAMs).
	 */
	gpon_field(GPON_GTC_DS_ONU_STATUS, 15, 8, 0xff);	/* DS ONU-ID = broadcast */
	gpon_field(GPON_GTC_US_ONU_ID, 15, 8, 0xff);		/* US ONU-ID = broadcast */
	gpon_field(0x1014, 11, 11, 1);				/* DS_CFG BWM_NO_FLT = 1 */
	gpon_wr(GPON_GTC_US_PLOAM_CFG,
		GPON_US_PLM_CRC_GEN_EN | GPON_US_PLM_ONUID_OVRD);
	gpon_wr(GPON_GTC_US_WRITE_PROTECT, GPON_US_WP_UNLOCK);
	gpon_field(0x5200, 0, 0, 1);				/* US_PROC_MODE AUTO_PROC_SSTART */
	gpon_wr(GPON_GTC_US_WRITE_PROTECT, GPON_US_WP_LOCK);
	gpon_field(GPON_GTC_DS_PLOAM_CFG, 9, 9, 1);		/* DS PLOAM BC_ACCEPT */
	gpon_field(GPON_GTC_DS_PLOAM_CFG, 8, 8, 0);		/* DS PLOAM ONUID_FILTER OFF
		* (bring-up): accept ALL DS PLOAMs regardless of ONU-ID, so an Assign_ONU-ID
		* sent non-broadcast (directed to the assigned id) is still delivered to the
		* FSM. The OLT authorizes our SN but the FSM never saw a type-0x03 PLOAM with
		* the filter on. */
	/* (6) DS_INTR_MASK = 0x070f, the O5 operating value. At reset it is 0x00000000
	 * (all GTC interrupts off); the O5 value 0x070f = LOS/LOF/FEC/LOM (b0-3) +
	 * **SN_REQ(b8)/RNG_REQ(b9)/PLM_BUF(b10)**.
	 * The SN_REQ/RNG_REQ/PLM_BUF unmask bits gate the GTC's upstream serial-number /
	 * ranging / PLOAM-buffer event handling; the MAC reset clears this reg, so it
	 * must be re-set here or the GTC never services the OLT's SN grant. */
	gpon_wr(0x1004, 0x070f);

	/*
	 * Upstream burst TIMING.  MIN_DELAY1 = 290 bits, MIN_DELAY2 = 50 guard bits
	 * (0x9132, also write-protected).  The pre-ranging EqD is then MIN_DELAY1-
	 * folded by gpon_set_eqd to 37120 (0x9100) — the correct one-frame burst
	 * position before the OLT assigns a ranging delay.  (The bogus 0x5000/0x5004/
	 * 0x5008 writes that used to live here actually hit the US interrupt delete/
	 * mask/status registers, NOT the burst overhead — removed.)
	 */
	gpon_wr_us_protected(GPON_GTC_US_MIN_DELAY, 0x9132);
	gpon_set_eqd(0);			/* pre-ranging EqD = 290*128 = 0x9100 */

	/*
	 * Default upstream burst overhead (G.984.3): 0xAA preamble run + the
	 * standard 0xAB,0x59,0x83 delimiter, no extra guard bytes (the gpon_boh_*
	 * defaults). The OLT's Upstream_Overhead (0x01) + Extended_Burst_Length
	 * (0x14) PLOAMs reprogram this with exact values/length before our first
	 * SN burst (gpon_apply_boh in the FSM); this is just a sane state for the
	 * window between GTC bring-up and those PLOAMs arriving.
	 */
	gpon_apply_boh(false);

	/*
	 * NOTE: the SDS upstream-TX serializer regs (0x22584/0x225ac/0x225d8) are NOT
	 * forced here. Setting them at init (= boot-default already right) did not
	 * get the ONU online, but transitioning them wrong->right AFTER the laser is
	 * up (a live register write) once got the ONU fully online on the OLT (47s).
	 * So the FSM applies them once, a few seconds into O3, to reproduce that
	 * post-laser-up SDS-TX re-sync. (Left at their wrong boot-default here.)
	 */

	/* Start the PLOAM activation FSM: parse the per-board serial number and
	 * begin draining downstream PLOAM to drive O1 -> O5. */
	gpon_parse_sn(onu_sn);
	pr_info("rtl9602c-gpon: PLOAM FSM start, SN '%s' = %*phN\n",
		onu_sn, 8, gpon_sn_bytes);
	timer_setup(&gpon_fsm_timer, gpon_fsm_poll, 0);
	mod_timer(&gpon_fsm_timer, jiffies + msecs_to_jiffies(50));
	return 0;
}

static void __exit rtl9602c_gpon_exit(void)
{
	timer_delete_sync(&gpon_fsm_timer);
	remove_proc_entry("gpon", NULL);
	if (ponip_base)
		iounmap(ponip_base);
	if (swcore_base)
		iounmap(swcore_base);
	if (gpon_base)
		iounmap(gpon_base);
}

module_init(rtl9602c_gpon_init);
module_exit(rtl9602c_gpon_exit);

MODULE_DESCRIPTION("Realtek RTL9602C GPON MAC foundation driver");
MODULE_LICENSE("GPL");
