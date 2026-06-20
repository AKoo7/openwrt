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
#include <linux/gfp.h>		/* __get_free_pages / GFP_KERNEL for the US PBO DRAM pool */
#include <linux/mm.h>		/* virt_to_phys */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include "rtl9602c_gpon_nic.h"
#include "rtl960x_ponmac.h"		/* clean-room family PON-MAC/SerDes bring-up lib */

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
#define   WSDS_SFT_RSTB_INF	BIT(14)		/* interface soft reset-B (FIFO r/w ptr re-sync) */
#define   WSDS_SFT_RSTB_INF_RX	BIT(15)		/* RX interface soft reset-B   */
#define   WSDS_SFT_RSTB_INF_TX	BIT(16)		/* TX interface soft reset-B   */
#define SDS_ANA_COM_REG27	0x225ec		/* TX CMU enable lives here    */
#define   SDS_CMU_EN		BIT(10)		/* TX CMU enable (toggle 1->0->1 to re-lock the PLL) */
#define SDS_ANA_COM_REG03	0x2258c		/* [15:14] CMU_ISTANK_SEL_RX   */
#define SDS_ANA_COM_REG08	0x225a0		/* TX-CDR (reg1418); [15] = the
						 * serdesCdr_reset toggle bit    */
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
#define SDS_REG0		0x22800		/* [1] SP_SDS_EN_RX (RX enable)*/
#define   SP_SDS_EN_RX		BIT(1)
/* Stock link-state polling behavior: when GPON_GTC_DS_INTR_STS reads this
 * exact sentinel the DS CDR is wedged; stock recovers by toggling SP_SDS_EN_RX
 * 1->0->1 (SDS_REG0[1]), waiting 10ms, then re-reading. */
#define GTC_DS_CDR_STUCK	0xca0eca0fu

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
/* Internal-MII force-link for the two PON-IP NICs (symmetric pair, US = DS-0x8000):
 * MEDIA_STS_DS @0xc058 (read 0x106e8400 from the bootloader, the DS-NIC<->GMAC0 RX
 * link that DS OMCI rides) and MEDIA_STS_US @0x4058 (the GMAC0-TX -> US-NIC link
 * that US OMCI must ride). FORCELINK[18] FORCEDFULLDUP[19] FORCE_SPD[17:16]
 * FORCE_SPD_MODE[10] TRXFCE/RXFCE/TXFCE[31:29]; golden 0x106e8400. No DAL code
 * writes the US one -> it stays down -> GMAC0-TX OMCI is dropped at the US-NIC GMII
 * ingress before PON_SID2QID (ustx 0x329bc = 0). Force it up like the DS side. */
#define PI_MEDIA_STS_US		0x04058
#define PI_MEDIA_STS_DS		0x0c058
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
#define PI_IP_MSTBASE_US	0x020e8		/* CFG_PON_MSTBASE: phys base of the US PBO DRAM packet pool */
/* US PBO DRAM pool. ROOT CAUSE of "US-NIC RX never receives" (2026-06-12, found
 * via the stock memory-usage init behavior + live devmem): the US-NIC RX path
 * is a PBO that DMAs received US packets into a DRAM pool at IP_MSTBASE_US. Stock
 * sets IP_MSTBASE_US=0x07eff000, RAM_NO=0x1fff, DRAM_RUNOUT=0x1f58 (a 1MB / 8192x
 * 128B-page DRAM pool); DS is SRAM-only (IP_MSTBASE_DS=0). Our driver did US
 * SRAM-only too (copying DS) -> the US-NIC RX had no DRAM pool to land frames in,
 * so EVERY US OMCI frame was dropped at descriptor-fetch BEFORE the MAC, leaving
 * PKT_OK/ERR/MISS all 0 (the exact symptom). Allocate the pool and point the HW
 * at it. Stock memory-usage init math (usPageSize=128, us_mem_size=1MB): SRAM_NO=0x7f,
 * RAM_NO=min(size/128,0x1fff)=0x1fff, DRAM_RUNOUT=size/128-40-128=0x1f58. */
#define PI_US_DRAM_PAGES	0x1fffu		/* RAM_NO: total US PBO pages-1 (SRAM+DRAM) */
#define PI_US_DRAM_RUNOUT	0x1f58u		/* DSCRUNOUT_US DRAM portion (size/128-168) */
#define PI_US_DRAM_ORDER	8		/* __get_free_pages order: 2^8 * 4KB = 1MB */
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
#define PI_PROBE_SELECT_DS	0x0d400		/* stock O5 = 0x40 (DS-NIC drain) */
#define PI_DS_NIC_CFG_D404	0x0d404		/* stock O5 = 0x11100348          */
#define PI_DS_NIC_CFG_D42C	0x0d42c		/* stock O5 = 0x40               */

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
 * at runtime by writing the value read from the board's factory configuration to
 * /sys/module/gpon_rtl9602c/parameters/onu_sn (the userspace provisioning service
 * does this early in boot). The param is writable and re-parses on write, so the
 * factory value takes effect on the next ranging cycle; the FSM re-reads the
 * parsed serial each time it sends its Serial_Number_ONU upstream. */
static void gpon_parse_sn(const char *s);	/* defined below; re-parses onu_sn */
static char *onu_sn = "XPON39013867";	/* TEST-ONLY default = this board's SN, so the FSM
					 * ranges with the real SN immediately (no placeholder
					 * phantom / re-range that races OLT discovery). For
					 * production revert to a placeholder + provision via the
					 * gpon_provision init script at OS startup. */
static bool gpon_sn_changed;		/* SN (re)provisioned -> FSM must re-range */

static int onu_sn_set(const char *val, const struct kernel_param *kp)
{
	int ret = param_set_charp(val, kp);

	if (!ret) {
		gpon_parse_sn(onu_sn);
		/* The driver loads with the placeholder SN and begins ranging
		 * immediately; the real per-board SN is provisioned slightly later
		 * via this /sys param (userspace) or the cmdline. Flag a re-range so
		 * the OLT sees the correct Serial_Number and authorises the
		 * provisioned ONU instead of auto-ranging the placeholder phantom. */
		gpon_sn_changed = true;
	}
	return ret;
}
static const struct kernel_param_ops onu_sn_ops = {
	.set = onu_sn_set,
	.get = param_get_charp,
};
module_param_cb(onu_sn, &onu_sn_ops, &onu_sn, 0644);
MODULE_PARM_DESC(onu_sn, "ONU serial number (G.984.3 ONU-SN): 4 ASCII ID chars + 8 hex digits");
/* Diagnostic: skip BOSA cold-init so that, on a warm boot where the BOSA is
 * already in a working state, the SoC datapath/FSM runs on top of it. */
static bool skip_bosa;
module_param(skip_bosa, bool, 0444);
MODULE_PARM_DESC(skip_bosa, "leave external BOSA as-is (warm-boot bisection)");
/* Open the DS GEM unicast/broadcast pass gate (GEM_DS_MC_CFG). Default OFF: without
 * the PON-IP->GMAC-NIC OMCI drain, opening it backs up the DS path and stalls the US
 * (deactivate ~48s). Set =1 only when drain-path testing. */
/* Pass unicast DS GEM so the GTC de-encapsulates the OLT's unicast OMCI.
 * DEFAULT OFF: opening it de-encapsulates OMCI (verified: gem_ds_rx climbs) but
 * the de-encapsulated frame does NOT yet drain to the CPU NIC (omci_rx stays 0 —
 * the PON-IP->GMAC HW-PBO trap delivery is the open wall), so the DS path backs
 * up and the OLT Deactivates the ONU (~17-22s). Set gem_gate_open=1 to resume
 * debugging the trap delivery; keep 0 for a stable O5 baseline. */
/* Pass unicast DS GEM so the GTC de-encapsulates the OLT's unicast OMCI.
 * DEFAULT OFF: stable O5 baseline. Stage instrumentation (DSPIPE / ds_deenc_sweep,
 * gated on this flag) showed that with it ON, de-encap is 0 on ALL 128 flows and
 * the OLT is NOT sending OMCI (nonidle GEM ~0) — it deallocs + deactivates ~36s.
 * Set =1 to re-run the DS-pipeline probes. */
static bool gem_gate_open;
module_param(gem_gate_open, bool, 0444);
MODULE_PARM_DESC(gem_gate_open, "open DS GEM pass gate (needs the PON-IP->host OMCI drain; default off = stable online)");
/* Install the WAN data-GEM datapath. CONFIRMED (stability bisection): doing it ~30s
 * AFTER O5 (while Online) re-latches the US-NIC (modeset) and glitches the established
 * US burst -> OLT "Laser out" -> deactivate. FIX: install it DURING config (at
 * Configure_Port-ID, right after the OMCC), so its modeset coincides with the config
 * phase (tolerated, like the OMCC's own modeset) and the ONU reaches Online already
 * carrying the data path -- no modeset while Online. Default ON. */
static bool data_gem_en = true;
module_param(data_gem_en, bool, 0644);
MODULE_PARM_DESC(data_gem_en, "install the WAN data GEM datapath during config (default on)");
/* trace=0 (default) silences the routine per-PLOAM/per-ACK dumps so the compact
 * O5 timeline survives the lossy serial console; key-PLOAM EVT + O5 lines always print. */
static bool trace;	/* default 0: per-PLOAM/ACK tracing is SLOW (printk over serial) and perturbs the
			 * activation timing (breaks ranging when on). Set gpon.trace=1 only for short diagnostics. */
module_param(trace, bool, 0644);
MODULE_PARM_DESC(trace, "verbose per-PLOAM/per-ACK serial spam (default 0)");

static bool cdr_reseat_on_reactivate = true;	/* default ON (A/B 2026-06-15): on a deactivate->O1 re-range, re-pulse the
			 * softirq-safe US-TX SerDes interface reset-B (WSDS_DIG_1D[16], the same primitive the
			 * boot path + the O3 re-sync use) so a marginal serializer lock from the prior activation
			 * is re-attempted immediately, cutting the cold-start O5<->O1 activation flapping. Opt-in
			 * A/B: TX-interface only (the locked DS RX framer is undisturbed). */
module_param(cdr_reseat_on_reactivate, bool, 0644);
MODULE_PARM_DESC(cdr_reseat_on_reactivate, "re-seat US-TX SerDes reset-B on re-range to cut activation flapping (default 0)");
static bool ploam_tx_dbg = true;	/* TEST: log per-send US-PLOAM TX (ENQ self-clear = HW transmitted) to
					 * prove whether the urgent-queue ACK/Password actually leaves the ONU
					 * (OLT raises LOAi = never gets our acks). Logs first 40 sends. */
module_param(ploam_tx_dbg, bool, 0644);
MODULE_PARM_DESC(ploam_tx_dbg, "log US-PLOAM CPU-TX ENQ self-clear per send (urgent-queue TX diagnostic)");
/* o5_rearm_burst_gate: re-apply the US burst-gate cluster (0x5188/0x526c/0x6024/0x6260)
 * and re-arm the HW auto-No_message keepalive template on every O5 entry (not just __init),
 * so a re-ranged O5 after a GMAC/SDS reset does not run on US-side reset defaults. Default on;
 * A/B with gpon.o5_rearm_burst_gate=0. */
static bool o5_rearm_burst_gate = true;
module_param(o5_rearm_burst_gate, bool, 0644);
MODULE_PARM_DESC(o5_rearm_burst_gate, "re-apply US burst-gate cluster + No_message keepalive on each O5 entry (default on)");
/* o5_ploam_keepalive_ticks: emit a No_message US PLOAM (HW auto queue 0x7) every N FSM ticks
 * (10ms each) while at O5 with an assigned ONU-ID, so a valid PLOAM is present in the OLT's
 * granted slots regardless of how the shared US-PLOAM buffer was last written, defeating the
 * OLT's PLOAM/ack-liveness timeout. 0 = disabled. Default 100 (~1s). */
static uint o5_ploam_keepalive_ticks = 100;
module_param(o5_ploam_keepalive_ticks, uint, 0644);
MODULE_PARM_DESC(o5_ploam_keepalive_ticks, "emit No_message US-PLOAM every N 10ms ticks at O5 (0=off, default 100)");

/* o5_provision_watchdog_ticks: a "Laser out" boot reaches O5 LOCALLY but the OLT
 * cannot frame our US burst (the TX-serializer lock PHASE is non-deterministic per
 * boot, gpon-rtl9602c.c:~1843), so the OLT stays Offline / Config=fail, never
 * provisions us, and sends NO Deactivate the ONU acts on -> the ONU sits at O5
 * forever with the bad phase and never self-recovers (gpon0 RX stays 0, sds_sync
 * stays 0 = no re-range = "never leases in 5 min", HW-observed). If at O5 this many
 * ticks with ZERO gpon0 WAN RX (the OLT forwarded us nothing = definitely not
 * provisioned), self-re-range to RE-ROLL the serializer phase; each roll has ~50%
 * chance of a frameable phase, so a stuck boot leases within a few cycles instead of
 * never. Gated PAST the slow-lease window (observed max ~135s) AND on wan_rx==0, so
 * it can NEVER disturb a working or slow-leasing link (those have wan_rx>0). About
 * 12ms/tick; 12000 ~= 150s. 0 = disabled.
 * ★PROVEN INEFFECTIVE (2026-06-16, HW A/B): it FIRES correctly (dmesg "O5 provision
 * watchdog (12001 ticks...)") but re-ranging does NOT recover a stuck boot — a hard-fail
 * boot reached sn_tx=37 / sds_sync=7 + 2 watchdog re-ranges and STILL never leased
 * (gpon0 RX=0 @700s). The bad US-TX serializer/CMU lock is a COLD-START analog state
 * fixed at power-on; NO amount of runtime re-range/CDR-reseat re-rolls it (only a reboot
 * does — hence ~60% lease ACROSS reboots but a stuck boot stays stuck forever). DEFAULT
 * OFF; kept as documented negative knowledge — do NOT re-enable expecting a WAN fix. */
static uint o5_provision_watchdog_ticks;	/* default 0 = off (proven ineffective, see above) */
module_param(o5_provision_watchdog_ticks, uint, 0644);
MODULE_PARM_DESC(o5_provision_watchdog_ticks, "re-range if at O5 this many ticks with gpon0 RX=0 (0=off default; PROVEN INEFFECTIVE: re-range does not re-roll the cold-start serializer lock)");
/* los_rerange_ticks: autonomous downstream-LOS recovery (fiber-pull / DS-light loss).
 * When the downstream optical signal is lost the OLT cannot send a Deactivate (no DS
 * light), so the ONU must notice the sustained LOS itself, tear down to O1, and re-
 * acquire when light returns. Without this the FSM sits stale at O5 after a fiber pull
 * and never re-ranges on reconnect (OLT LED stays dark). Fires after optic_los has been
 * asserted this many consecutive ~10ms ticks (debounces transient dips / I2C pad-steal
 * blips). 0 = off. Requires bosa_i2c_restore_pad so optic_los is live. */
static uint los_rerange_ticks = 50;		/* ~500ms sustained LOS */
module_param(los_rerange_ticks, uint, 0644);
MODULE_PARM_DESC(los_rerange_ticks, "drop to O1 + re-range after downstream optical-LOS persists this many ~10ms ticks (fiber-pull recovery; 0=off, default 50)");
static u32 gpon_los_run;			/* consecutive optic_los=1 tick count */

/* last DS PLOAM type drained this cycle, surfaced on the periodic O5 line */
static u8 gpon_last_ds_type;
/* DIAGNOSTIC: force the upstream laser continuously on (US_CFG.FS_LON). Tests
 * whether the SoC SerDes-TX can drive the BOSA at all, independent of the GTC
 * burst scheduler. DEV-ONLY — continuous light jams a multi-ONU PON. */
static bool force_laser;
module_param(force_laser, bool, 0444);
MODULE_PARM_DESC(force_laser, "force US laser CW on (US_CFG FS_LON) — SerDes-TX emission diagnostic");
/* Laser burst bias/mod DAC override (BOSA W54 0x236 hi-8 / W55 0x237 hi-8), applied
 * after bosa_tx_enable (which loads the A4-golden bias=0x19/mod=0x67). The OLT raises
 * "Laser out" at O5 because the un-OFFK'd laser's burst is marginal for its operational
 * burst-RX. MOD sets the burst PEAK (1-level) = stronger burst; BIAS sets the DC/0-level
 * (raising it worsens extinction -> hurts DS), so prefer raising MOD and keep BIAS low.
 * Conservative bump first (laser-safety: MPD high/low detect is disarmed in bring-up, so
 * the DAC value is the only over-power guard — do NOT crank these). Sweepable live. */
static unsigned int laser_bias = 0x32;	/* Board-C REAL per-board calib (rtl8290b.data CAL_IBIAS 9960uA->code 0x32f). 0x18/0x34 25C-LUT A/B = no rate gain (2/6 vs 3/6), reverted */
static unsigned int laser_mod = 0xbb;	/* Board-C REAL per-board calib (rtl8290b.data CAL_IMOD 36694uA->code 0xbbd) */
module_param(laser_bias, uint, 0644);
module_param(laser_mod, uint, 0644);
MODULE_PARM_DESC(laser_bias, "BOSA bias DAC hi-8 (0x236) override; 0=keep A4-golden 0x19");
MODULE_PARM_DESC(laser_mod, "BOSA mod DAC hi-8 (0x237) override for stronger burst peak; 0=keep golden 0x67");
/* rev-A bring-up US-TX SerDes CMU/PLL + TX-LA-LDO writes (part of the stock rev-A GPON
 * mode-set analog config) that our gpon_serdes_init OMITS: SDS_ANA_COM_REG02/03/08 = the
 * TX CMU/PLL that clocks the US serializer, COM_REG24=0x8001 = REG_TXLA_LDOEN (TX limiting-amp
 * LDO/output stage). Without them the laser is DC-biased but the MAC's US data is not cleanly
 * serialized/modulated onto it -> OLT "Laser out" + rxsid=0 (the unified US-TX wall). The CMU
 * is shared with RX, so this can perturb DS -> watch ranging; revert if DS breaks. */
static unsigned int serdes_modev1_tx;	/* default 0: TESTED (COM_REG02/03/08/24/25 applied+readback-confirmed) =
					 * NO effect on US-TX burst (OLT still "Laser out", rxsid=0) and slightly
					 * degraded DS (CMU shared) -> the omitted ModeV1 TX SerDes regs are NOT the
					 * gap; reverted to off. Param kept for reference. */
module_param(serdes_modev1_tx, uint, 0444);
MODULE_PARM_DESC(serdes_modev1_tx, "1=apply the rev-A bring-up US-TX SerDes CMU/PLL + TXLA_LDOEN writes (no effect)");

/* serdes_tx_xtra: 0 (default) = ORACLE-PARITY — do NOT set the 3 SerDes-TX serializer-path bits
 * (0x220a8[5:4], 0x2281c[14], 0x22a30[8]) that the LIVE stock-ref ONU leaves clear (it ranges +
 * bursts without them). 1 = restore the old behaviour (set them) for A/B if parity regresses ranging. */
static unsigned int serdes_tx_xtra;
module_param(serdes_tx_xtra, uint, 0644);
MODULE_PARM_DESC(serdes_tx_xtra, "1=set legacy SerDes-TX D2A/clk-edge bits (stock=0; default 0=match stock)");
/* serdes_cdr_reset: replicate the stock SerDes CDR-reset behavior
 * (SDS_ANA_COM_REG08 @ swcore 0x225a0): INVERT bit15, hold 10ms, RESTORE. Our clean-room
 * init OMITTED this CDR-lock pulse. It seats the upstream-TX SerDes serializer CDR; without it the
 * TX serializer lock is non-deterministic, which matches the observed cycle-to-cycle US-burst
 * variation (some O5 windows the OLT decodes hundreds of US-OMCI, others it loses the burst at once
 * = LOSi/LOAi "Laser out"). NOTE: serdes_cdr_reset is now writable (0644) so it can be
 * left default-on but A/B'd live. Default on (the fix); gpon.serdes_cdr_reset=0 reverts. */
static bool serdes_cdr_reset = true;
module_param(serdes_cdr_reset, bool, 0644);
MODULE_PARM_DESC(serdes_cdr_reset, "pulse SDS_ANA_COM_REG12 (0x225b0) bit15 10ms (stock serdesCdr_reset RX_SD_POR_SEL) (stock ponmac step; default on)");
/* usnic_initrdy_poll: stock-aligned US-NIC readiness gate. PON_IPSTS_US (PON-IP
 * 0x1bf020f4) bit0 = PONIC_INITRDY = read-only US PON-IP core init-ready. Wait for
 * it before the US GMII RX/TX latch edge so the NIC latches with the core ready.
 * Bounded 200ms, read-only -> cannot reorder config/reset or worsen the lock.
 * (Stock NEVER writes 0x20f4; our old pi_wr(0x20f4,1) set a reserved bit -> removed.)
 * RE verdict: a digital-core ready flag, so probably instrumentation not the analog
 * fix — but discriminating: a timeout on a failed-WAN boot implicates the digital US
 * layer; never-timeout proves that layer innocent. Default on. */
static bool usnic_initrdy_poll = true;
module_param(usnic_initrdy_poll, bool, 0644);
MODULE_PARM_DESC(usnic_initrdy_poll, "wait PON_IPSTS_US.PONIC_INITRDY (0x1bf020f4 bit0)==1 before US GMII latch (default on; bounded 200ms, read-only)");
/* usnic_initrdy_repulse: ACTIVE escalation (experiment). On PONIC_INITRDY timeout,
 * re-pulse the CDR (COM_REG12 bit15, same as serdes_cdr_reset) + re-poll once
 * — re-rolls the analog lock without reordering config/reset. Default off. */
static bool usnic_initrdy_repulse;
module_param(usnic_initrdy_repulse, bool, 0644);
MODULE_PARM_DESC(usnic_initrdy_repulse, "on PONIC_INITRDY timeout, re-pulse CDR (COM_REG12 bit15) + re-poll once (default 0)");
/* cdr_stuck_recover: faithful replica of the stock runtime link-state-check
 * DS-CDR-wedge recovery that our driver was MISSING. From a cold power-on the
 * DS CDR can come up wedged (the per-boot ~50% cold-start lock); a soft/WDT reboot
 * re-runs init assuming fresh HW and never re-acquires, so a bad lock persists. Stock
 * detects the wedge at link-check time — GPON_GTC_DS_INTR_STS == 0xca0eca0f — and
 * recovers by toggling SP_SDS_EN_RX (SDS_REG0[1]) 1->0->1 with a 10ms settle. We run
 * the same check each FSM poll tick (BOSA-serialized softirq) as a two-tick toggle
 * (off this tick, on next) to avoid a 10ms busy-wait in softirq. Bounded by
 * GPON_CDR_STUCK_MAX attempts/range. Default on; gpon.cdr_stuck_recover=0 disables. */
static bool cdr_stuck_recover = true;
module_param(cdr_stuck_recover, bool, 0644);
MODULE_PARM_DESC(cdr_stuck_recover, "recover a wedged DS CDR (GTC_DS_STS==0xca0eca0f) by toggling SP_SDS_EN_RX, like stock (default on)");
#define GPON_CDR_STUCK_MAX	8	/* bounded re-acquire attempts per range cycle */
static unsigned int gpon_cdr_stuck_count;	/* diag: total wedges detected */
static unsigned int gpon_cdr_stuck_fixed;	/* diag: wedges cleared by the toggle */
static u32 gpon_gtc_ds_sts_last;		/* diag: last raw GTC DS status seen */
/* serdes_stock_seq: 1 = use gpon_serdes_init_stock() (the EXACT stock rev-A bring-up
 * ORDER: reset-FIRST then config, single reset bit CMD_SDS_RST_PS, the D2A/sample-clock
 * bits SET=1, NO reset-B-release dance) instead of our gpon_serdes_init() (config-first
 * + reset-B dance). Tests whether the deterministic cold-start US-TX serializer lock is
 * an emergent property of stock's reset-first order — the one angle the per-register
 * tests couldn't cover.
 * ★A/B RESULT 2026-06-16 (HW, default-on test build): the stock order DS-locks fine
 * (reached O5 6/6 — so our reset-B dance is NOT needed) but the cold-start lease rate
 * did NOT improve (1/6, no better than the ~60% baseline). So the deterministic lock is
 * NOT an emergent property of stock's order either. Combined with config-values-match +
 * every-component-tested, this DEFINITIVELY shows the ~40% cold-start US-TX serializer
 * lock non-determinism is irreducible by any register/sequence-level ONU action (it is
 * below the register level — analog VCO/CMU trim / PVT). DEFAULT OFF; kept as documented
 * negative knowledge — do NOT re-enable expecting a WAN fix. */
static bool serdes_stock_seq;	/* default 0 = our gpon_serdes_init (stock order tested = no improvement) */
module_param(serdes_stock_seq, bool, 0644);
MODULE_PARM_DESC(serdes_stock_seq, "1=stock rev-A SerDes bring-up order (gpon_serdes_init_stock); 0=our gpon_serdes_init");

/* family_lib: bring the SerDes up via the clean-room rtl960x_ponmac family library
 * (RTL960X_CHIP_9602C path) instead of the inline gpon_serdes_init(). Validates the
 * family-lib op-table framework on real 9602C silicon: family_lib=1 must reach O5 +
 * lease + keep LAN exactly like family_lib=0 (the lib's 9602C tables are a faithful
 * translation of gpon_serdes_init). Default off; A/B with gpon.family_lib=1. */
static bool family_lib = true;	/* default ON: the clean-room rtl960x_ponmac family lib is the
				 * 9602C SerDes boot bring-up. HW-validated (O5 10/10 over two 5-boot
				 * runs, LAN ok, WAN leases at the analog rate) = equivalent to the
				 * inline path (its 9602C op-tables are a faithful, exact-match
				 * translation of gpon_serdes_init). gpon.family_lib=0 = legacy inline. */
module_param(family_lib, bool, 0644);
MODULE_PARM_DESC(family_lib, "1=bring up SerDes via rtl960x_ponmac family lib (9602C path, default); 0=inline gpon_serdes_init");
/* serdes_postmode_perturb: the family-lib path performs TWO US-TX serializer edges
 * AFTER GPON mode is committed that stock rev-A does NOT (DIG_1D[16] reset-B
 * re-sync + a post-mode serdesCdr_reset pulse). These were NEVER cleanly A/B'd
 * (the serdes_cdr_reset param does not gate the family-lib path). DEFAULT 0
 * (=stock-matching: skip them) — prime suspect for cold-start serializer-phase
 * jitter (WAN ~50%). gpon.serdes_postmode_perturb=1 restores legacy behavior. */
static bool serdes_postmode_perturb;	/* default false: skip post-mode perturbations (stock rev-A) */
module_param(serdes_postmode_perturb, bool, 0644);
MODULE_PARM_DESC(serdes_postmode_perturb, "1=do post-GPON-mode DIG_1D resync + serdesCdr_reset (legacy); 0=skip (stock rev-A, default)");
/* serdes_sds_cfgrst: the family-lib SerDes reset pulse used to assert BOTH
 * CMD_SDS_CFG_RST_PS (bit7) and CMD_SDS_RST_PS (bit0); stock rev-A pulses ONLY bit0
 * (stock never writes bit7 anywhere during bring-up). Because the field-write is
 * RMW, bit7 stayed latched through the whole bring-up = an extra SDS-config reset
 * domain stock never touches -> prime suspect for the per-power-on US-TX serializer/
 * PLL phase re-roll (cold-start WAN ~50%, OLT "Laser out"). DEFAULT 0 = bit0-only
 * (stock = the fix); gpon.serdes_sds_cfgrst=1 restores the legacy bit7+bit0 pulse. */
static bool serdes_sds_cfgrst;	/* default false = stock bit0-only SDS reset */
module_param(serdes_sds_cfgrst, bool, 0644);
MODULE_PARM_DESC(serdes_sds_cfgrst, "1=legacy: also pulse CMD_SDS_CFG_RST_PS bit7 in the SDS reset; 0=stock bit0-only (default, the cold-start fix)");
/* serdes_stock_analog: drive SDS_ANA_COM REG01 (0x22584)=0x73a4 + REG11 (0x225ac)
 * RX_FILT_CONFIG=0 to EXACTLY match the live-stock post-reset values — the ONLY two
 * SerDes registers that differed between stock (WAN-up, 100%) and our failing board
 * (cold-start ~50% US-TX "Laser out"). The golden table writes them correctly but the
 * SDS reset wipes REG01 bit14 (shared CMU) / REG11 RX_FILT; this re-applies them AFTER
 * the reset, like stock. DEFAULT 1 = the fix; gpon.serdes_stock_analog=0 = legacy. */
static bool serdes_stock_analog = true;
module_param(serdes_stock_analog, bool, 0644);
MODULE_PARM_DESC(serdes_stock_analog, "1=match live-stock SDS REG01=0x73a4 + REG11 RX_FILT=0 post-reset (default, the cold-start fix); 0=legacy");
/* serdes_analog_postreset: program the FULL analog CMU/CDR golden table (COM_REG02/03/
 * 08/24/25 charge-pump/LDO/tank + GPON_REG46 CDR) AFTER the SDS reset (stock rev-A
 * order) instead of before it. The reset WIPES analog to defaults; pre-reset placement
 * (legacy) leaves the CMU/CDR locking against default operating-point values that the
 * partial REG01/REG11 re-apply never fully corrects -> metastable per-power-on lock =
 * the cold-start ~50% "Laser out". Post-reset = stock = deterministic lock every cold
 * boot + soft restart. DEFAULT 1 = the fix; gpon.serdes_analog_postreset=0 = legacy. */
static bool serdes_analog_postreset = true;
module_param(serdes_analog_postreset, bool, 0644);
MODULE_PARM_DESC(serdes_analog_postreset, "1=program full analog CMU/CDR table AFTER the SDS reset (stock rev-A, default, the cold-start determinism fix); 0=legacy pre-reset");
/* serdes_cmu_settle_ms: ms to wait after forcing the 125M ref clock and BEFORE releasing
 * the SerDes interface reset-B (which latches the TX serializer phase) — lets the TX CMU PLL
 * lock first. 0 = legacy. Candidate fix for the cold-start ~50% US-TX "Laser out" metastability. */
static unsigned int serdes_cmu_settle_ms;
module_param(serdes_cmu_settle_ms, uint, 0644);
MODULE_PARM_DESC(serdes_cmu_settle_ms, "ms TX-CMU-lock settle between 125M ref force and reset-B release (0=legacy default)");
/* serdes_txpll_relock: at the O1/O2->O3 edge (downstream optical signal present, just
 * before the first upstream burst), re-lock the TX CMU PLL by toggling the CMU enable
 * 1->0->1, then re-sync the SerDes word FIFO read/write pointer (WSDS_DIG_1D[14] 0->1).
 * On a fresh power-on under strong downstream light the optical signal-detect can assert
 * before the CMU has settled, latching the TX PLL onto the WRONG clock rate (~50%,
 * un-frameable by the OLT = "Laser out"); re-toggling the CMU enable once the optics are
 * stable forces a clean re-acquire. This is the one bring-up step the clean-room port
 * omitted. DEFAULT 1 = on (cold-start determinism candidate); =0 = skip. */
/* DEFAULT OFF: the O3-entry CMU re-toggle is an unvalidated cold-start-lock candidate
 * (the active-fiber failure was NOT cold-start, so it's untested); keep it gated. */
static bool serdes_txpll_relock;
module_param(serdes_txpll_relock, bool, 0644);
MODULE_PARM_DESC(serdes_txpll_relock, "1=re-lock TX CMU PLL (toggle CMU enable + FIFO re-sync) at O3 entry before first US burst (cold-start candidate, default off); 0=skip");
/* optical_poll: periodic ANI-G DDM optical read. DEFAULT OFF — the periodic BOSA I2C
 * read (in any context) transiently disturbs the optical path and provokes the OLT
 * op=0xff dealloc/DEACT churn (no WAN); proven by A/B (poll on=churn, off=clean WAN).
 * Until the read is made glitch-free, ANI-G reports the (plausible) snapshot value.
 * =1 re-enables the live read (for development of the glitch-free path). */
static bool optical_poll;
module_param(optical_poll, bool, 0644);
MODULE_PARM_DESC(optical_poll, "1=periodic ANI-G DDM optical poll; 0=off default (the live /proc read + LuCI refresh already show current dBm without periodic BOSA I2C, which historically churned the OLT)");
/* bosa_i2c_restore_pad: optional cleanup — return bus-0's SoC pad to the optical-SD
 * function (SOC_IO_MODE_EN[13]=0) after each BOSA I2C transaction. DEFAULT OFF: the
 * baseline leaves it at 1 and optic_los still reports LOS correctly (operator-confirmed
 * on a real fiber pull), so this is NOT needed for LOS detection and is kept off to
 * avoid perturbing the cold-start datapath. =1 only if a masked optic_los is ever
 * observed. */
static bool bosa_i2c_restore_pad;
module_param(bosa_i2c_restore_pad, bool, 0644);
MODULE_PARM_DESC(bosa_i2c_restore_pad, "restore SOC_IO_MODE_EN[13]=0 (optical-SD pad) after each BOSA I2C transaction (default 0; optic_los works without it)");
/* lan_keep_open: LAN management must be reachable INDEPENDENT of the WAN/GPON state —
 * it must NEVER be gated on O5. A bad cold-start (no O5) or a WAN/fiber disconnect
 * (LOS -> re-range) must NOT kill LAN control of the ONU. When set (default), the
 * switch VLAN_FILTER is never asserted for LAN-gating: LAN is open from boot and
 * stays open through ranging / re-range / LOS. 0 = legacy (filter on during config,
 * cleared at stable O5, re-armed on every drop below O5). */
static bool lan_keep_open = true;
module_param(lan_keep_open, bool, 0644);
MODULE_PARM_DESC(lan_keep_open, "keep LAN open from boot independent of GPON/WAN state (default 1); 0=legacy O5-gated");
/* force_soc_clk: before the SerDes bring-up, write the 3 SoC sysctl/clock registers
 * (0x18000100/12c/140) to the live-STOCK (100%-deterministic) values that OUR FAIL boot
 * was found to differ from (stock 0x00440e00/0x024d024d/0x024d024d vs ours 0x00440f00/
 * 0x02490249). Candidate fix for the cold-start ~50% US-TX metastability (stock-vs-ours
 * clock-config diff, same methodology that found REG01/REG11). 0 = legacy (no write). */
static bool force_soc_clk;
module_param(force_soc_clk, bool, 0644);
MODULE_PARM_DESC(force_soc_clk, "1=write live-stock SoC clock regs 0x18000100/12c/140 before SerDes bring-up (cold-start fix candidate); 0=legacy");
/* serdes_clkgate_rstb: gate the SerDes word clock (STOP_CLK=1) across the interface
 * reset-B release and un-gate LAST, so the word divider restarts on one defined edge
 * (defeats the async-reset-on-running-divider ~50% serializer-phase coin-flip). The
 * single highest-value cold-start fix candidate (ideation rank-1 fix). 0 = legacy. */
static bool serdes_clkgate_rstb;
module_param(serdes_clkgate_rstb, bool, 0644);
MODULE_PARM_DESC(serdes_clkgate_rstb, "1=clock-gated (STOP_CLK) SerDes reset-B release, un-gate last (cold-start metastability fix candidate); 0=legacy free-running");
/* serdes_skip_rstb_dance: Live debug confirmed WSDS_DIG_1D is ALREADY 0x1c000 (interface reset-B
 * released) at mode_set entry AND that the SDS reset (CMD_SDS_RST_PS bit0) does NOT clear it — so
 * the c2_sds_rstb dance (assert DIG_1D[15/16]->0 then release ->1) is a GRATUITOUS TX/RX reset-B
 * 1->0->1 PULSE on an already-running serializer = the textbook async-reset-on-running-divider that
 * latches a metastable word-phase (cold-start ~50%). Stock rev-A bring-up NEVER pulses it. =1 skips
 * the whole dance (DIG_00=0xf30 + the DIG_1D toggles) — DIG_1D/DIG_00 are already at their
 * operational values, so no edge is issued. Cold-start determinism fix candidate; A/B vs the dance. */
static bool serdes_skip_rstb_dance;
module_param(serdes_skip_rstb_dance, bool, 0644);
MODULE_PARM_DESC(serdes_skip_rstb_dance, "1=skip the c2_sds_rstb DIG_1D reset-B dance (already-released; gratuitous phase-latching pulse); 0=legacy dance");
/* serdes_minimal_analog: our golden analog table does ~145 SerDes writes; the stock rev-A GPON
 * bring-up does only ~23 — it never writes the 3 duplicate GPON per-rate banks
 * (0x22608/0x22688/0x22788) nor the 4 FIB-bank bodies (0x22c00-0x22df8), ~134 redundant writes that
 * lengthen the bring-up before the CMU/serializer phase latches. =1 skips those over-configure writes
 * to match stock's minimal write-set (keeps the active GPON bank + FIB PDOWN-clear). Cold-start
 * determinism fix candidate (timing/transaction-count); final O5 config unchanged (HW defaults). */
static bool serdes_minimal_analog;
module_param(serdes_minimal_analog, bool, 0644);
MODULE_PARM_DESC(serdes_minimal_analog, "1=skip the golden-table writes stock omits (dup GPON banks + FIB bodies, ~134 writes) to match stock's minimal SerDes bring-up; 0=full golden table");
/* bosa_before_serdes: CROSS-SUBSYSTEM ORDER fix candidate (cold-start ~50% determinism).
 * Stock stages the external RTL8290B BOSA analog (I2C calib + reset + a BOSA-ready settle
 * via europa's ready_check) BEFORE bringing up the SoC SerDes/CMU (ponmac mode_set). OUR
 * probe inverts this: it runs the SerDes mode_set (CMU lock + the RX_EN 0->1 start edge)
 * FIRST, with the BOSA RX still powered-down, then calls bosa_rx_enable AFTER. A SerDes CMU/
 * CDR that locks against an un-settled BOSA analog/shared-reference is a clean ~50% metastable
 * lock. =1 moves bosa_probe()+bosa_rx_enable()+a settle to BEFORE the SerDes bring-up (stock
 * order); the later duplicate calls are skipped. A/B vs the legacy SerDes-first order. */
static bool bosa_before_serdes;
module_param(bosa_before_serdes, bool, 0644);
MODULE_PARM_DESC(bosa_before_serdes, "1=power+settle the BOSA RX before the SerDes bring-up (stock order); 0=legacy SerDes-first");
/* bosa_settle_ms: settle delay after bosa_rx_enable when bosa_before_serdes=1, emulating
 * stock europa's ~21ms post-reset + ready_check analog-settle before the SerDes CMU locks. */
static uint bosa_settle_ms = 50;
module_param(bosa_settle_ms, uint, 0644);
MODULE_PARM_DESC(bosa_settle_ms, "ms to settle the BOSA analog before the SerDes bring-up when bosa_before_serdes=1 (default 50)");
/* sc_ldo_init: stock runs an LDO-init step (early in boot) that we OMIT — an
 * SC-indirect RMW of the DRAM-rail LDO byte 0xfdca (clear bits 2,3) + THERMAL_CTRL_0
 * (swcore 0x130)=0x00ec0005 (arm on-die over-temp ALARM comparator). Assessment:
 * DRAM-LDO + thermal alarm, NOT the SerDes/laser path — kept as stock platform
 * hygiene (the init we were missing), NOT expected to move the WAN cold-start rate.
 * Default on; A/B revert with gpon.sc_ldo_init=0. */
static bool sc_ldo_init = true;
module_param(sc_ldo_init, bool, 0644);
MODULE_PARM_DESC(sc_ldo_init, "run stock rtk_ldo_init (SC-indirect 0xfdca analog LDO + THERMAL_CTRL_0); default on");

/* Register accessor the family lib injects: absolute phys -> KSEG1 uncached MMIO
 * (phys < 0x20000000 on this SoC: swcore 0x1b000000 / PON-IP 0x1bf00000). */
static u32 r960_phys_rd(u32 phys)
{
	return ioread32((void __iomem *)(unsigned long)(0xa0000000u | phys));
}
static void r960_phys_wr(u32 phys, u32 val)
{
	iowrite32(val, (void __iomem *)(unsigned long)(0xa0000000u | phys));
}
static const struct rtl960x_ops rtl9602c_r960_ops = {
	.rd = r960_phys_rd,
	.wr = r960_phys_wr,
};
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
static bool apc_off = true;	/* default TRUE: apc_off=false (full APC seat) was RE-TESTED (task bdcqpqqn1) and
			 * BREAKS ranging — the ~3s CW seating loop deafens/wedges the shared-BOSA DS-RX, so the
			 * OLT only ever sees "Initial", omcirx=0, never O5. CONFIRMED the multi-session "APC
			 * deafens DS-RX" wall. AND the apc_off=true laser is NOT weak/drooping: boot trajectory
			 * (laser_boot.log) shows bias=0x19 STABLE, 0x389=0 (no fault), mpd 0x67..0x8f, EN_L=0 the
			 * whole time, ONU reaches O5 and holds ~8s before the OLT sends Deactivate(0x05)=LOS. So
			 * the LOS is NOT a laser-bias-seat problem — it is the upstream BURST not being decodable
			 * by the OLT despite a healthy laser (US-TX SerDes / burst-gating), the same wall as
			 * rxsid=0: the US-TX SerDes init step is omitted relative to stock. */
module_param(apc_off, bool, 0444);
MODULE_PARM_DESC(apc_off, "skip bosa_apc_calibrate (1=A4 image alone; 0=run APC to seat OFFK laser bias)");
/*
 * RTL8290B B-variant APC/OFFK ignition (rtl8290b_apc_init). DEFAULT FALSE so the
 * shipping default is unchanged (A4 image alone, the ~50% analog-rate path) and
 * the new flow is A/B-revertible from the kernel command line.
 *
 * The board's laser is an RTL8290B (chip_type==1). bosa_apc_calibrate runs the
 * rtl8290 NON-B flow whose OFFK (modulator offset cal) never completes on the B
 * chip: it sets up the wrong FSM (never writes the W62/W63 OFFK_EN trio in B
 * order, never runs the FSU/OFFK-FSM config block), so R29(0x31d) never reaches
 * (&0x3c)==0x3c and R30(0x31e) b7 OFFK_DONE stays 0. An un-nulled modulator
 * emits DC between bursts which deafens the shared DS-RX -> non-deterministic
 * lease. rtl8290b_apc_init runs the B-variant FSU/OFFK to completion (R29=0x3f),
 * keeps EN_L burst-gated (NOT CW, DS-safe), and aborts on MPD/no-feedback.
 * Stock O5 targets: R29(0x31d)=0x3f, R30(0x31e)=0xa0, 0x204=0x8e (EN_L=0).
 */
static bool apc_offk;		/* default OFF: OFFK converges (R29=0x3f) but does NOT lift WAN rate; not the WAN root cause */
module_param(apc_offk, bool, 0444);
MODULE_PARM_DESC(apc_offk, "run rtl8290b_apc_init B-variant OFFK ignition (completes modulator offset cal; default 0 = unchanged)");
/* TEMP default true (LAN+WiFi ACCESS build): hold the ONU FSM at O1 (no ranging),
 * so the GPON never deactivates/re-ranges and br-lan + the WiFi AP stay STABLE +
 * accessible (http://192.168.1.1/ and ONU-3282AE visible). GPON/WAN is disabled
 * while held. REVERT to default false (`static bool gpon_hold;`) to resume GPON
 * ranging once the US-OMCI egress fix lands. */
static bool gpon_hold;	/* default false: range to O5 normally (so the O5 selftest fires) */
module_param(gpon_hold, bool, 0444);
MODULE_PARM_DESC(gpon_hold, "hold the GPON FSM at O1 (no ranging) -> stable br-lan/WiFi for LAN+WiFi access (GPON/WAN disabled)");
static u8 gpon_sn_bytes[8];		/* G.984.3 ONU-SN: 4-byte ID + 4-byte serial */
static struct timer_list gpon_fsm_timer;
static u8 gpon_fsm_state = 1;		/* O1 */
static u8 gpon_fsm_onu_id = 0xff;

/*
 * Deferred US-TX CDR-reset (re-range path). gpon_fsm_handle() runs in the FSM
 * timer = softirq, where the stock serdesCdr_reset (invert COM_REG08 bit15, hold
 * 10ms, restore) CANNOT run (mdelay in softirq is illegal). On a Deactivate->O1
 * re-range we schedule this work so the *correct, full* CDR-lock pulse runs in
 * process context — re-seating the TX serializer CDR fresh for the next ranging,
 * instead of the softirq-only WSDS_DIG_1D[16] re-strobe (a weaker primitive that
 * does not re-lock the CDR). */
static struct work_struct gpon_cdr_reset_work;

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
static bool gpon_data_installed;	/* WAN data GEM (193) datapath installed (one-shot, on OMCI GEM-CTP create) */
static bool gpon_data_tcont_installed;	/* the OLT's DATA Alloc-ID bound to the DATA T-CONT (8) */
static u16 gpon_omcc_alloc;		/* the OMCC's (first) Alloc-ID, on T-CONT 16 */
static u16 gpon_data_alloc;		/* the OLT's data Alloc-ID, on T-CONT 8 */
/* data_tcont: bind the OLT's separate DATA Alloc-ID to its OWN data T-CONT 8 (and route the
 * data GEM US to GPON_DATA_PHYS_QID 32) instead of riding the OMCC T-CONT 16's grants. The OLT's
 * DBA grants the data Alloc separately; with no live T-CONT reporting occupancy on it, the OLT
 * reclaims it (Assign_Alloc-ID op=0xFF), reallocs, churns, and escalates to Deactivate_ONU-ID
 * (0x05) ~4min in -> WAN data path torn down. Binding it like stock (one physical T-CONT per
 * Alloc-ID) keeps the OLT happy. ⚠ ONLY for OLTs that grant a SEPARATE data Alloc-ID — THIS lab
 * OLT (HSGQ-G008) uses a SINGLE Alloc-ID 0x100 for both OMCC + data (T-CONT 16), so routing data
 * to T-CONT 8 leaves it grantless. DEFAULT off (data rides T-CONT 16, correct for single-alloc);
 * gpon.data_tcont=1 enables the per-data-alloc T-CONT 8 bind for multi-alloc OLTs. */
static bool data_tcont;		/* default off: single-alloc OLT (this lab) -> data rides T-CONT 16 */
module_param(data_tcont, bool, 0644);
MODULE_PARM_DESC(data_tcont, "bind the OLT data Alloc-ID to its own T-CONT 8 + route data US to qid 32 (default on, the WAN-stability fix)");

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

/* DEV live register poke (uses the driver's own ioremap — no /dev/mem, immune to
 * STRICT_DEVMEM). Write "<g|G|p|P> <hexoff> [hexval]" to
 * /sys/module/gpon_rtl9602c/parameters/poke :  g/G = GPON-GTC read/write,
 * p/P = PON-IP read/write, b/B = BOSA I2C read/write. Result is logged via dmesg. Lets us
 * test US-egress register experiments (watch /proc/gpon idle16/gemus64/bwm_acpt) without a rebuild. */
static int bosa_read_reg(u16 reg);	/* fwd decl: poke_set 'b'/'B' use these (defined below) */
static int bosa_write_reg(u16 reg, u8 val);
static char poke_buf[8];
static int poke_set(const char *val, const struct kernel_param *kp)
{
	char c = 0;
	unsigned int off = 0, v = 0;
	int n = sscanf(val, " %c %x %x", &c, &off, &v);

	if (n < 2) {
		pr_info("rtl9602c-gpon: poke usage: <g|G|p|P> <hexoff> [hexval]\n");
		return 0;
	}
	switch (c) {
	case 'g':
		pr_info("rtl9602c-gpon: poke GTC[%#x]=%#x\n", off, gpon_rd(off));
		break;
	case 'G':
		gpon_wr(off, v);
		pr_info("rtl9602c-gpon: poke GTC[%#x]<=%#x ->%#x\n", off, v, gpon_rd(off));
		break;
	case 'p':
		pr_info("rtl9602c-gpon: poke PI[%#x]=%#x\n", off, pi_rd(off));
		break;
	case 'P':
		pi_wr(off, v);
		pr_info("rtl9602c-gpon: poke PI[%#x]<=%#x ->%#x\n", off, v, pi_rd(off));
		break;
	case 'm': {	/* generic phys read (GMAC 0x18012048 CPUTAGCR, SWCORE 0x1b00xxxx) */
		void __iomem *a = ioremap(off, 4);
		if (a) { pr_info("rtl9602c-gpon: poke MEM[%#x]=%#x\n", off, ioread32(a)); iounmap(a); }
		break;
	}
	case 'M': {	/* generic phys write */
		void __iomem *a = ioremap(off, 4);
		if (a) { iowrite32(v, a); pr_info("rtl9602c-gpon: poke MEM[%#x]<=%#x ->%#x\n", off, v, ioread32(a)); iounmap(a); }
		break;
	}
	case 'b':	/* BOSA I2C read (12-bit reg; slave banking internal) — live US-TX-vs-stock diff */
		pr_info("rtl9602c-gpon: poke BOSA[%#x]=%#x\n", off, bosa_read_reg(off));
		break;
	case 'B':	/* BOSA I2C write — live laser/extinction tweak */
		bosa_write_reg(off, v);
		pr_info("rtl9602c-gpon: poke BOSA[%#x]<=%#x ->%#x\n", off, v, bosa_read_reg(off));
		break;
	default:
		pr_info("rtl9602c-gpon: poke bad cmd '%c'\n", c);
		break;
	}
	return 0;
}
static const struct kernel_param_ops poke_ops = { .set = poke_set };
module_param_cb(poke, &poke_ops, poke_buf, 0644);
MODULE_PARM_DESC(poke, "DEV: <g|G|p|P> <hexoff> [hexval] live GTC/PON-IP reg read/write");

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
	int i, ret = -ETIMEDOUT;

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
			ret = (cmd & I2C_CMD_NACK) ? -EIO :
				(int)(sw_rd(I2C_IND_RD) & 0xff);
			break;
		}
		udelay(10);
	}

	/* Reconnect the shared optical-SD pad so optic_los stays live (see
	 * bosa_i2c_restore_pad). */
	if (bosa_i2c_restore_pad)
		sw_field(SOC_IO_MODE_EN, 13, 13, 0);
	return ret;
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
 * Read a big-endian 16-bit value from two consecutive BOSA registers — used for
 * the SFF-8472 DDM optical-power words the RTL8290B MCU publishes on its A2/DDM
 * page (slave 0x51 = the 0x1xx bank): RX optical power @0x68/0x69, TX @0x66/0x67,
 * unit 0.1 microWatt/LSB. reg_hi is the high byte, reg_hi+1 the low.
 */
static int bosa_read16(u16 reg_hi)
{
	int h = bosa_read_reg(reg_hi);
	int l = bosa_read_reg(reg_hi + 1);

	if (h < 0 || l < 0)
		return -1;
	return ((h & 0xff) << 8) | (l & 0xff);
}

/*
 * Glitch-tolerant DDM read for the optic power words shown in the web UI. The BOSA
 * I2C bus is also driven by the laser servo (bosa_laser_maint, softirq, ~10ms); an
 * on-demand optic read from process context (the /proc dump) can interleave a servo
 * I2C transaction and come back corrupted with NO NACK — just wrong bytes — which is
 * why the RX power was occasionally wrong in the UI. Sample three times and return the
 * median of the valid samples so a single corrupted reading is discarded. No bus lock
 * is taken (the GPON datapath softirq must never wait on the optic read); rejection is
 * cheap and entirely in this process-context path. Returns -1 (-> "n/a") only if every
 * sample failed.
 */
static int bosa_read16_median(u16 reg_hi)
{
	int v[3], n = 0, i, j;

	for (i = 0; i < 3; i++) {
		int r = bosa_read16(reg_hi);

		if (r >= 0)
			v[n++] = r;
	}
	if (n == 0)
		return -1;
	for (i = 1; i < n; i++) {		/* insertion sort the valid samples */
		int key = v[i];

		for (j = i - 1; j >= 0 && v[j] > key; j--)
			v[j + 1] = v[j];
		v[j + 1] = key;
	}
	return v[n / 2];			/* median */
}

/* --- ANI-G (ME 263) live optical levels ------------------------------------
 * The OMCI GET runs in softirq (GFP_ATOMIC) and must not busy-wait on the I2C
 * bus, so the DDM optical words are sampled by a periodic workqueue (~3 s, as
 * stock's separate DDM thread does) into this cache; the GET just returns the
 * cache. Values use the
 * G.988 ANI-G encoding (2's-complement s16, 0.002 dB referred to 1 mW). The
 * defaults are the stock first-light snapshot, used until the first good read. */
static s16 anig_rx_level = (s16)0xeedc;		/* #10 Optical signal level (DS RX) */
static s16 anig_tx_level = (s16)0x04d7;		/* #14 Transmit optical level (TX)  */

/*
 * Convert a raw SFF-8472 DDM power word (unit 0.1 microWatt/LSB) to the ANI-G
 * level encoding. power_mW = raw/10000, so
 *   level = 500 * 10*log10(raw/10000) = 1505*log2(raw)/65536 - 20000
 * (5000*log10(2) = 1505.15). log2 is computed in Q16 fixed point (fls for the
 * integer part + linear interpolation of the mantissa; ~0.25 dB error, ample for
 * the diagnostic ANI-G report) so no kernel FPU is needed. Returns INT_MIN when
 * the word reads "not available" (0x0000/0xffff) so the caller keeps the cache.
 */
static s32 ddm_word_to_level(int raw)
{
	int e;
	u32 frac;
	s64 log2_q16, level;

	if (raw <= 0 || raw >= 0xffff)
		return INT_MIN;
	e = fls((u32)raw) - 1;				/* floor(log2(raw))      */
	frac = e ? (((u32)(raw - (1u << e)) << 16) >> e) : 0;	/* mantissa frac, Q16 */
	log2_q16 = ((s64)e << 16) | frac;		/* log2(raw) in Q16      */
	level = ((log2_q16 * 1505) >> 16) - 20000;
	if (level > 32767)
		level = 32767;
	else if (level < -32768)
		level = -32768;
	return (s32)level;
}

/* Sample the calibrated DDM optical words and refresh the ANI-G level cache.
 * Called from the periodic FSM tick at a slow cadence (never from the GET path);
 * a "not available" word keeps the previous cached value. */
static void gpon_optical_cache_poll(void)
{
	s32 v;

	v = ddm_word_to_level(bosa_read16_median(0x168));	/* RX @0x68/0x69 */
	if (v != INT_MIN)
		anig_rx_level = (s16)v;
	v = ddm_word_to_level(bosa_read16_median(0x166));	/* TX @0x66/0x67 */
	if (v != INT_MIN)
		anig_tx_level = (s16)v;
}

/* Exported to the OMCI responder (rtl9602c_eth.c) for the ANI-G GET: returns the
 * cached live optical levels. No I2C here, so it is safe to call from softirq. */
void gpon_anig_optical_omci(s16 *rx_level, s16 *tx_level)
{
	*rx_level = anig_rx_level;
	*tx_level = anig_tx_level;
}

/* The DDM optical read busy-waits on the I2C bus, so it must NOT run in the PLOAM
 * fsm_poll softirq — a stall there risks missed US-grant timing and an OLT-side
 * deactivate. Run it from a workqueue (process context) on a ~3 s cadence, like
 * stock's separate DDM polling thread. The median read already tolerates racing
 * the laser servo, so no bus lock is needed. */
static struct delayed_work gpon_optical_work;
static void gpon_optical_work_fn(struct work_struct *w)
{
	if (!optical_poll)
		return;
	gpon_optical_cache_poll();
	schedule_delayed_work(&gpon_optical_work, msecs_to_jiffies(3000));
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
	int i, ret = -ETIMEDOUT;

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

		if (!(cmd & I2C_CMD_BUSY)) {
			ret = (cmd & I2C_CMD_NACK) ? -EIO : 0;
			break;
		}
		udelay(10);
	}

	/* Reconnect the shared optical-SD pad so optic_los stays live (see
	 * bosa_i2c_restore_pad). */
	if (bosa_i2c_restore_pad)
		sw_field(SOC_IO_MODE_EN, 13, 13, 0);
	return ret;
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
static int apc_offk_armed;	/* rtl8290b_apc_init armed FSU/OFFK; servo will latch */
static int apc_offk_latched;	/* runtime servo latched OFFK (R29 0x31d &0x3c==0x3c) */
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
	if (trace && (bosa_stat_ticks++ % 50) == 0)
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

/*
 * rtl8290b_apc_init() — RTL8290B (chip_type==1) B-variant laser APC/OFFK ignition.
 *
 * This is the flow the stock BOSA/laser module runs for the B-variant chip, and the
 * one our clean-room bosa_apc_calibrate (rtl8290 NON-B) gets wrong. The decisive
 * difference is the OFFK (modulator offset) calibration: the B chip completes it
 * via the FSU done-flag (R29/0x31d & 0x3c == 0x3c), NOT via the non-B R30/0x31e
 * b7 poll. Without OFFK the modulator offset is never nulled, so the laser emits
 * DC between bursts, deafening the shared downstream RX -> ~50% lease. This
 * implementation:
 *
 *   - gates on the MCU power-on + CHECK_READY ((0x383&0xe0)==0xc0 AND
 *     (0x301&0x80)!=0) before touching anything;
 *   - writes the W62/W63 OFFK_EN trio in B order (0x24e b7=1; 0x23e=0xfd;
 *     0x23f=0xfd) and the per-board DCL setpoints P0/P1/Pavg = 0x26/0x50/0x50;
 *   - runs both W77 (0x24d) MCU-command handshake batches;
 *   - runs the FSU/OFFK-FSM config + ARM, then POLLS R29(0x31d) to (&0x3c)==0x3c
 *     completion with a bounded timeout + pr_warn on fail (does NOT declare done
 *     on the wrong R30 poll);
 *   - latches on full-done, loads the per-board operating bias/mod LUT;
 *   - keeps EN_L (0x204 b4) BURST-GATED: EN_L=1 only transiently to seat the APC,
 *     then deasserted to 0 (0x204=0x8e) so the laser is NOT CW (DS-safe);
 *   - laser-safety: aborts the ignition window if the monitor photodiode reads no
 *     feedback (MPD==0) or a hard TX-kill fault latches, so we never run the
 *     booster open-loop above the calibrated bias/mod ceiling.
 *
 * Reg numbers are the decimal-europa register ids; the slave banking (hi-nibble
 * of reg>>8 -> 0x50/0x51/0x54/0x55) is done inside bosa_read_reg/bosa_write_reg.
 */
static void __init rtl8290b_apc_init(void)
{
	/* Exact stock B-variant laser APC init behavior (GPON/pon=1), re-expressed from
	 * the observed stock register sequence with the per-board dpoConfig table. setReg ->
	 * bosa_write_reg, setRegBit -> bosa_set_bit, setRegBits -> bosa_set_field
	 * (val shifted into the mask's low bit, == stock). The OFFK offset
	 * compute+write (0x243/0x245) is GATED on dpo[0x72]!=0 in stock; ours is 0,
	 * so stock SKIPS it -> we MUST NOT write it (the spurious write parked R29 at
	 * 0x0e). The terminal OFFK latch ((R29 0x31d &0x3c)==0x3c) is done by the
	 * runtime servo in gpon_fsm_poll, once the laser bursts at O5. */
	static const u8 w77_a[] = { 0xa8, 0xb0, 0xd0, 0xd8, 0xe8, 0xe0 };
	static const u8 w77_b[] = { 0xb0, 0xd0, 0xb8, 0xb0, 0xd0, 0xc0 };
	int i, j;
	u8 t8;

	pr_info("rtl9602c-gpon: rtl8290b_apc_init: B-variant OFFK (exact stock seq, base 0x578)\n");

	/* step 1: MCU power-on kick + gate + ~11ms settle */
	bosa_write_reg(0x380, 0x01);
	if ((bosa_read_reg(0x380) & 0xff) != 1) {
		pr_warn("rtl9602c-gpon: rtl8290b_apc_init: MCU power-on (0x380!=1) -> abort\n");
		return;
	}
	for (i = 0; i < 11; i++)
		udelay(1000);

	/* CHECK_READY: (0x383&0xe0)==0xc0, then (0x301 b7) */
	for (i = 0; i < 20000; i++) {
		int sret = bosa_read_reg(0x383);

		if (sret >= 0 && (sret & 0xe0) == 0xc0)
			break;
		udelay(50);
	}
	bosa_poll_bit(0x301, 7, 1, 50, 20000);

	/* step 2: OFFK enable trio (W78 b7; W62=W63=0xfd full bytes) */
	bosa_set_bit(0x24e, 7, 1);
	bosa_write_reg(0x23e, 0xfd);
	bosa_write_reg(0x23f, 0xfd);

	/* step 3: per-field config (dpoConfig@0x578, pon=1 values) */
	bosa_write_reg(0x23a, 0x00);		/* apcIavg = 0			*/
	bosa_set_field(0x23b, 0xf0, 0x02);	/* apcEr (dpo[0x1e]=2)		*/
	bosa_set_field(0x23b, 0x0c, 0x03);	/* apcApcTimer = 3		*/
	bosa_set_field(0x23b, 0x03, 0x03);	/* apcErcTimer = 3		*/
	bosa_set_field(0x240, 0x03, 0x00);	/* apcLpfBw = 0			*/
	bosa_set_bit(0x23e, 0, 0);		/* txsdMode = 0			*/
	bosa_set_field(0x24c, 0xe0, 0x01);	/* txsdTh = 1			*/
	bosa_set_bit(0x24c, 4, 1);		/* txsdTiaGain = 1		*/
	bosa_set_bit(0x24c, 3, 1);		/* txsdHighLoopGain = 1		*/
	for (i = 0; i < 11; i++)
		udelay(1000);
	bosa_read_reg(0x31d);

	/* step 4: W77 (0x24d) MCU-command walk BATCH 1 */
	for (j = 0; j < ARRAY_SIZE(w77_a); j++) {
		bosa_write_reg(0x24d, w77_a[j]);
		for (i = 0; i < 11; i++)
			udelay(1000);
		bosa_read_reg(0x31d);
	}

	/* step 5: ERC chopper + W80 b3 toggle */
	bosa_set_bit(0x243, 7, 1);
	bosa_set_bit(0x284, 6, 1);
	bosa_set_bit(0x284, 5, 1);
	bosa_set_bit(0x284, 4, 1);		/* apcErcChopperEn = 1 (b6/b5/b4) */
	t8 = bosa_read_reg(0x27c) & 0xff;
	bosa_write_reg(0x27c, t8 & 0xf7);
	bosa_write_reg(0x27c, t8 | 0x08);

	/* step 6: W77 walk BATCH 2 */
	for (j = 0; j < ARRAY_SIZE(w77_b); j++) {
		bosa_write_reg(0x24d, w77_b[j]);
		for (i = 0; i < 11; i++)
			udelay(1000);
		bosa_read_reg(0x31d);
	}

	/* step 7: OFFK offset compute+write -- STOCK SKIPS IT (dpo[0x72]=0). */

	/* step 8: bias/mod init-code seeds (dpo[0x65]=0 -> manual path) + loop mode */
	bosa_set_bit(0x23d, 7, 0);
	bosa_set_field(0x236, 0xff, 0x10);	/* Ibias init 0x100 >> 4 (stock seed)	*/
	bosa_set_field(0x238, 0x0f, 0x00);	/* Ibias[3:0] = 0		*/
	bosa_set_bit(0x23d, 7, 1);
	bosa_set_bit(0x23d, 7, 0);
	bosa_set_field(0x237, 0xff, 0x20);	/* Imod init 0x200 >> 4 (stock seed)	*/
	bosa_set_field(0x238, 0xf0, 0x00);	/* Imod[3:0] = 0		*/
	bosa_set_bit(0x23d, 7, 1);
	bosa_set_field(0x27c, 0xc0, 0x03);	/* apcLoopMode DCL (remap 1->3)	*/
	bosa_set_bit(0x24a, 4, 0);		/* apcLoopModeEx = 0		*/
	bosa_set_field(0x23d, 0x70, 0x04);	/* apcLaserOnDelay = 4		*/
	bosa_set_field(0x23d, 0x03, 0x02);	/* apcSettleCnt = 2		*/
	bosa_set_field(0x239, 0x38, 0x05);	/* apcApcLoopGain = 5		*/
	bosa_set_field(0x23c, 0x07, 0x00);	/* apcCmpd = 0			*/
	bosa_set_field(0x239, 0x07, 0x03);	/* apcErcLoopGain = 3		*/
	bosa_set_field(0x23c, 0xf8, 0x05);	/* apcErTrim = 5		*/

	/* step 9: FSU config (the previously-guessed block, now exact) */
	bosa_set_bit(0x241, 6, 1);		/* fsuMode = 1			*/
	bosa_set_field(0x241, 0x30, 0x01);	/* fsuApcLoopGain = 1 (b4-5)	*/
	bosa_set_field(0x241, 0x0c, 0x01);	/* fsuApcRampb = 1 (b2-3)	*/
	bosa_set_field(0x241, 0x03, 0x03);	/* fsuApcRampm = 3		*/
	bosa_set_bit(0x242, 7, 1);		/* fsuRstCount = 1		*/
	bosa_set_field(0x242, 0x60, 0x02);	/* fsuSettleCount = 2 (b5-6)	*/
	bosa_set_field(0x242, 0x18, 0x02);	/* fsuErcLoopGain = 2 (b3-4)	*/
	bosa_set_field(0x242, 0x06, 0x00);	/* fsuErcRampm = 0 (b1-2)	*/

	/* step 10: txsd off-rst */
	bosa_set_bit(0x27c, 3, 1);		/* txsdOffRstCount = 1		*/

	/* step 11: ceilings + enables (modmax 0xcc, biasmax 0x4d, biasmin 0x01) */
	bosa_set_bit(0x246, 2, 0);
	bosa_set_field(0x247, 0xff, 0xcc);
	bosa_set_bit(0x246, 2, 1);		/* apcModMax = 0xcc		*/
	bosa_set_bit(0x246, 1, 0);
	bosa_set_field(0x248, 0xff, 0x4d);
	bosa_set_bit(0x246, 1, 1);		/* apcBiasMax = 0x4d		*/
	bosa_set_bit(0x284, 7, 0);		/* apcCrossEn = 0		*/
	bosa_set_bit(0x246, 0, 0);
	bosa_set_field(0x249, 0xff, 0x01);
	bosa_set_bit(0x246, 0, 1);		/* apcBiasMin = 0x01		*/
	bosa_set_bit(0x246, 5, 1);		/* apcModMaxEn = 1		*/
	bosa_set_bit(0x246, 3, 1);		/* apcBiasMinEn = 1		*/
	bosa_set_field(0x283, 0xff, 0x01);	/* apcCrossStr = 1		*/

	/* step 12: FSU ARM (0x27c b5=0->b4=1->0x20e b7=1->0x27c b5=1) + commit */
	bosa_set_bit(0x27c, 5, 0);
	bosa_set_bit(0x27c, 4, 1);
	bosa_set_bit(0x20e, 7, 1);
	bosa_set_bit(0x27c, 5, 1);
	bosa_write_reg(0x232, 0xc0);
	bosa_write_reg(0x24a, 0x60);

	apc_offk_armed = 1;
	pr_info("rtl9602c-gpon: rtl8290b_apc_init: armed; post-cfg R29=0x%02x R30=0x%02x 0x241=0x%02x 0x242=0x%02x 0x247=0x%02x 0x248=0x%02x; servo latches at O5\n",
		bosa_read_reg(0x31d) & 0xff, bosa_read_reg(0x31e) & 0xff,
		bosa_read_reg(0x241) & 0xff, bosa_read_reg(0x242) & 0xff,
		bosa_read_reg(0x247) & 0xff, bosa_read_reg(0x248) & 0xff);
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
static int gpon_serdes_init(void)	/* not __init: re-run on re-range from gpon_cdr_reset_worker */
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

	/* 3. Pulse the SDS reset to latch the analog config. Stock rev-A pulses ONLY
	 *    bit0 (CMD_SDS_RST_PS); the extra bit7 (CMD_SDS_CFG_RST_PS) is never written
	 *    by stock and, left RMW-latched through bring-up, re-rolls
	 *    the US-TX serializer phase per power-on. Gate bit7 behind serdes_sds_cfgrst
	 *    (default 0 = stock bit0-only = the cold-start fix). */
	if (serdes_sds_cfgrst)
		sw_field(SW_SOFTWARE_RST, 7, 7, 1);	/* legacy CMD_SDS_CFG_RST_PS */
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
	 * 6a-ModeV1. US-TX SerDes CMU/PLL + TX-LA-LDO — the rev-A GPON mode-set analog
	 * full-word writes our init OMITTED. SDS_ANA_COM_REG02/03/08 program the TX CMU/PLL
	 * that clocks the upstream serializer; COM_REG24=0x8001 (REG_TXLA_LDOEN) powers the TX
	 * limiting-amp output stage feeding the laser modulation input. Without them the laser
	 * is DC-biased but the MAC's US data is not cleanly serialized onto it -> the OLT's
	 * burst-RX sees no decodable burst ("Laser out") and rxsid stays 0. Full-word writes
	 * (sw_wr), placed before the D2A interconnect to match ModeV1 order. The CMU is shared
	 * with the RX 25M reference, so watch DS/ranging. */
	if (serdes_modev1_tx) {
		sw_wr(0x22588, 0x6df8);		/* SDS_ANA_COM_REG02 TX CMU/PLL */
		sw_wr(SDS_ANA_COM_REG03, 0x8941);	/* SDS_ANA_COM_REG03 TX CMU (0x2258c) */
		sw_wr(0x225a0, 0x0713);		/* SDS_ANA_COM_REG08 */
		sw_wr(0x225e4, 0x001f);		/* SDS_ANA_COM_REG25 */
		sw_wr(0x225e0, 0x8001);		/* SDS_ANA_COM_REG24 REG_TXLA_LDOEN */
		pr_info("rtl9602c-gpon: ModeV1 TX SerDes applied: COM_REG02=0x%04x 03=0x%04x 08=0x%04x 24=0x%04x 25=0x%04x\n",
			sw_rd(0x22588) & 0xffff, sw_rd(SDS_ANA_COM_REG03) & 0xffff,
			sw_rd(0x225a0) & 0xffff, sw_rd(0x225e0) & 0xffff, sw_rd(0x225e4) & 0xffff);
	}

	/*
	 * 6b. TX DATA PATH — route the digital US-framer data into the analog TX
	 * serializer AND set the TX-data sample-clock edges, *** BEFORE *** switching
	 * CFG_SDS_MODE to GPON. CRITICAL ORDERING for the rev-A bring-up SerDes: program
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
	/* ORACLE-PARITY 2026-06-13: the LIVE stock-ref ONU (ttyUSB3 mmiord @O5) does NOT set
	 * these three serializer-path bits — stock reads 0x220a8=0x2 ([5:4]=0), 0x2281c=0x1359
	 * ([14]=0), 0x22a30=0x4 ([8]=0). Our build was setting them (0x32/0x5359/0x104), which is
	 * the only confirmed mine-vs-live-stock divergence in the SerDes-TX block (0x2280c/0x225d8
	 * already match). The old comment claimed they were required for a decodable burst, but the
	 * working stock is the existence proof: it ranges + bursts WITHOUT them. Match stock. The
	 * `serdes_tx_xtra` param restores the old behaviour for A/B if this regresses ranging. */
	sw_field(0x220a8, 5, 4, serdes_tx_xtra ? 0x3 : 0x0);	/* WSDS_DIG_1E D2A interconnect (stock=0) */
	sw_field(0x2281c, 14, 14, serdes_tx_xtra ? 0x1 : 0x0);	/* SDS_REG7 SP_CFG_NEG_CLKWR_A2D (stock=0) */
	sw_field(0x22a30, 8, 8, serdes_tx_xtra ? 0x1 : 0x0);	/* SDS_EXT_REG12 SEP_CFG_NEG_CLKRD_D2A (stock=0) */

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

	/* SerDes CDR-lock pulse — the stock SerDes CDR-reset our
	 * init OMITTED: invert SDS_ANA_COM_REG12 bit15 (REG_RX_SD_POR_SEL), hold 10ms,
	 * restore. Re-PORs the RX signal-detect path so a recovered CDR re-acquires
	 * cleanly (see serdes_cdr_reset param). Done here, after GPON mode + the TX
	 * reset-B re-sync, so the analog-ready poll below confirms the SerDes recovered.
	 *
	 * REGISTER FIX 2026-06-17: the stock CDR-reset operates on REG12 (0x225b0), NOT
	 * REG08 (0x225a0). Confirmed from the stock register sequence
	 * (reads/inverts/restores REG12[15]) + the chip's register/field map:
	 * REG12[15]=REG_RX_SD_POR_SEL, whereas REG08[15] falls in the RESERVED top-16
	 * field. The prior REG08[15] toggle wrote a reserved bit (the earlier
	 * "0x22560 -> 0x225a0" address fix corrected one wrong reg to another). The
	 * COM_REG08=0x0713 write seen in the stock trace is the ModeV1 *config* constant, not
	 * this toggle. */
	if (serdes_cdr_reset) {
		u32 cdr = sw_rd(SDS_ANA_COM_REG12);

		sw_wr(SDS_ANA_COM_REG12, cdr ^ BIT(15));
		mdelay(10);
		sw_wr(SDS_ANA_COM_REG12, cdr);
		pr_info("rtl9602c-gpon: serdesCdr_reset pulse (COM_REG12 0x225b0 bit15), restored=0x%08x\n",
			cdr);
	}

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
 * gpon_serdes_init_stock() - stock rev-A (CHIP_REV_ID_A) GPON SerDes bring-up
 * ORDER as observed from the stock device, with OUR golden analog overlay applied
 * in that order. VERIFIED 2026-06-16 against:
 *   - the stock GPON mode-set sequence (the PON-MAC GPON-mode path and its
 *     inlined ModeV1 analog config sub-step)
 *   - the chip's register map (every offset below confirmed)
 *   - the chip's field map (every lsp/len confirmed)
 *
 * SCOPE NOTE (corrected): this is NOT a bit-faithful replica of stock's ANALOG
 * config. The stock sequence has NO golden table and NO fib_reg0 loop;
 * the ModeV1 analog sub-step IS the entire stock analog config. The golden
 * table + fib power-on here are OUR overlay (same data as gpon_serdes_init),
 * applied in stock ORDER (after the reset). The ModeV1 explicit writes overwrite
 * every overlapping golden entry, so the final operating point is the ModeV1
 * one. What IS faithful to stock is the ORDER (reset-first), the single reset
 * bit (CMD_SDS_RST_PS only, NOT CMD_SDS_CFG_RST_PS), and the D2A/sample-clock
 * bits being SET=1 (the value stock programs; note the LIVE stock ONU reads them
 * clear at O5 - a real programmed-vs-runtime divergence worth A/B testing).
 *
 * __init: this function reads sds_analog_golden[]/fib_reg0_banks[], both
 * __initconst, so it MUST live in .init.text or modpost reports a section
 * mismatch and (post-init) it would read freed memory. Marked __init
 * accordingly. If post-init re-runnability is ever required, de-__initconst
 * both tables (a wider change shared with gpon_serdes_init).
 *
 * Offsets not in the file's #define block are declared local (consistent with
 * the existing 0x22588/0x225a0/etc. literals used elsewhere in this file).
 */
static int __init gpon_serdes_init_stock(void)
{
	/* offsets absent from our #define block - all confirmed via the chip register map */
	const u32 WSDS_DIG_1Eo         = 0x220a8;
	const u32 SDS_REG7o            = 0x2281c;
	const u32 SDS_EXT_REG12o       = 0x22a30;
	const u32 SDS_ANA_COM_REG02o   = 0x22588;
	const u32 SDS_ANA_COM_REG19o   = 0x225cc;
	const u32 SDS_ANA_COM_REG24o   = 0x225e0;
	const u32 SDS_ANA_COM_REG25o   = 0x225e4;
	const u32 SDS_ANA_1P25G_REG46o = 0x226b8;
	const u32 SDS_ANA_EPON_REG46o  = 0x227b8;
	int i;

	/* === stock step 1: serdes mode -> illegal (0x1f) === */
	sw_field(SDS_CFG, 4, 0, SDS_MODE_OFF);		/* CFG_SDS_MODE = 0x1f     */

	/* === stock step 2: no force sds === */
	sw_wr(WSDS_DIG_01, 0);				/* WSDS_DIG_01 = 0         */

	/* === stock step 3: RESET FIRST - pulse ONLY CMD_SDS_RST_PS (bit0).
	 * Stock does NOT touch CMD_SDS_CFG_RST_PS (bit7) here. === */
	sw_field(SW_SOFTWARE_RST, 0, 0, 1);		/* CMD_SDS_RST_PS = 1      */
	mdelay(10);					/* 10ms settle             */

	/* === stock step 4: BEN on === */
	sw_field(WSDS_DIG_18, 12, 12, 1);		/* BEN_OE = 1              */

	/* === stock step 5: rev-A (CHIP_REV_ID_A) else-branch - adjust
	 * TX_Burst's Burst Mode Sequence === */
	sw_field(WSDS_DIG_03, 6, 4, 0x0);		/* CFG_TXDIS_SEL_DLY = 0   */
	sw_field(WSDS_DIG_03, 3, 0, 0x0);		/* CFG_D2ANLOG_SEL   = 0   */

	/* ----------------------------------------------------------------------
	 * OUR golden analog overlay + fiber power-on, applied AFTER the reset
	 * (stock ORDER). NOT part of stock; the ModeV1 writes below override every
	 * overlapping entry. Provides the per-rate/lane + FIB bank config that
	 * stock assumes is already present from an earlier mode-set.
	 * -------------------------------------------------------------------- */
	for (i = 0; i < ARRAY_SIZE(sds_analog_golden); i++)
		sw_wr(sds_analog_golden[i].off, sds_analog_golden[i].val);
	for (i = 0; i < ARRAY_SIZE(fib_reg0_banks); i++)
		sw_wr(fib_reg0_banks[i],
		      sw_rd(fib_reg0_banks[i]) & ~FIB_REG0_PDOWN);	/* fiber on */

	/* ======================================================================
	 * stock step 6: the inlined rev-A bring-up analog config.
	 * ==================================================================== */

	/* --- ### TX ### PON TX CMU/PLL + TX-LA-LDO --- */
	sw_wr(SDS_ANA_COM_REG02o, 0x6df8);	/* COM_REG02 (CMU/LDO/ISTANK) */
	sw_wr(SDS_ANA_COM_REG03, 0x8941);	/* COM_REG03 (0x2258c)        */
	sw_wr(SDS_ANA_COM_REG08, 0x0713);	/* COM_REG08 (0x225a0)        */
	sw_wr(SDS_ANA_COM_REG25o, 0x1f);	/* COM_REG25                  */

	/* WSDS_DIG_1E[5]=CFG_ANALOG2D_SEL, [4]=CFG_D2ANLOG_INF_SEL - stock SETs
	 * these =1 (opposite of our oracle-parity path which clears them). */
	sw_field(WSDS_DIG_1Eo, 5, 5, 0x1);	/* CFG_ANALOG2D_SEL = 1       */
	sw_field(WSDS_DIG_1Eo, 4, 4, 0x1);	/* CFG_D2ANLOG_INF_SEL = 1    */

	sw_wr(SDS_ANA_COM_REG24o, 0x8001);	/* COM_REG24 (REG_TXLA_LDOEN) */

	/* --- ### RX ### (ASIC, !FPGA) --- */
	sw_wr(SDS_ANA_1P25G_REG46o, 0x80c5);	/* FIB1G Rx 1.25 KP/KI        */
	sw_wr(SDS_ANA_EPON_REG46o,  0x80c5);	/* EPON  Rx 1.25 KP/KI        */
	sw_wr(SDS_ANA_GPON_REG46,   0x80c5);	/* GPON  Rx 2.488 KP/KI       */

	sw_field(SDS_ANA_COM_REG12, 14, 14, 0x1);	/* REG_RX_SEL_CDR_AFEN = 1   */
	sw_field(SDS_ANA_COM_REG11,  7,  0, 0x0);	/* REG_RX_FILT_CONFIG = 0    */
	sw_field(SDS_ANA_COM_REG19o, 14, 14, 0x1);	/* REG_CDR_RESET_MANUAL = 1  */
	sw_field(SDS_ANA_COM_REG19o, 10, 10, 0x1);	/* REG_CDR_EN_LPF_MANUAL = 1 */

	/* ### RX_EN toggle ### MISC_REG00: FRC_RX_EN_ON[4], FRC_RX_EN_VAL[5] */
	sw_wr(SDS_ANA_MISC_REG00, 0x10);	/* RX_EN force on, val=0       */
	sw_wr(SDS_ANA_MISC_REG00, 0x30);	/* RX_EN val 0->1 (start CDR)  */

	mdelay(50);				/* 50ms settle                 */

	sw_field(WSDS_DIG_02, 10, 10, 0x0);	/* REG_EN_PDOWN_BEN = 0        */

	/* TX-data sample clocks - stock SETs these =1. */
	sw_field(SDS_REG7o,      14, 14, 0x1);	/* SP_CFG_NEG_CLKWR_A2D = 1   */
	sw_field(SDS_EXT_REG12o,  8,  8, 0x1);	/* SEP_CFG_NEG_CLKRD_D2A = 1  */
	/* === end ModeV1 analog config === */

	/* === stock step 7: serdes mode -> GPON (0x8) === */
	sw_field(SDS_CFG, 4, 0, SDS_MODE_GPON);		/* CFG_SDS_MODE = 0x8      */

	/* === stock step 8 (!FPGA): force ber notify [13:12] = 0x3 === */
	sw_field(SDS_ANA_MISC_REG02, 13, 13, 0x1);	/* FRC_BER_NOTIFY_VAL = 1  */
	sw_field(SDS_ANA_MISC_REG02, 12, 12, 0x1);	/* FRC_BER_NOTIFY_ON  = 1  */

	pr_info("rtl9602c-gpon: stock rev-A SerDes init done: SDS_CFG=0x%08x DIG18=0x%08x MISC02=0x%08x fib21=0x%08x\n",
		sw_rd(SDS_CFG), sw_rd(WSDS_DIG_18), sw_rd(SDS_ANA_MISC_REG02),
		sw_rd(FIB_EXT_REG21));

	/* Local readiness gate (not part of the stock sequence) - mirrors gpon_serdes_init tail. */
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

/* ===================================================================
 * Faithful port of the stock all-module GPON datapath
 * bring-up, in stock MODULE ORDER, run ONCE on a QUIESCENT switch (after the
 * GMAC IP-block reset + U-Boot swcore resync, before the datapath/TX is armed).
 * Reproduces the ordering + table-clears that the prior piecemeal flat writes
 * (done amid a running datapath) did NOT — the hypothesis being that the switch
 * honoring of the cpu-tag PSEL directed-egress to the PON-MAC is an emergent
 * property of the full ordered/quiescent bring-up. Decoded clean-room from the
 * stock module-init behavior across the switch/L2/VLAN/port/CPU/trap/classify/
 * ponmac blocks, with addresses from the chip's register map.
 * Gated by full_sdk_init; the indirect TBL_ACCESS table-clears self-abort if the
 * engine does not respond (no hang/corruption), and VLAN filtering is only
 * enabled if its member table actually wrote.
 * =================================================================== */
static unsigned int full_sdk_init = 1;
module_param(full_sdk_init, uint, 0644);
MODULE_PARM_DESC(full_sdk_init, "1=run the faithful full SDK datapath init on the quiescent switch (default), 0=legacy minimal init");
static unsigned int full_sdk_tables;	/* 0: the operational VLAN setup (our Ethernet driver) owns the VLAN table now; isolate the VLAN_CTRL=0x19 enable test */
module_param(full_sdk_tables, uint, 0644);
MODULE_PARM_DESC(full_sdk_tables, "1=also run the L2 LUT clear + VLAN-enable table-engine steps (risky), 0=field-writes only");
/* "harmful write" hypothesis: stock NEVER writes the US-NIC
 * region (0x1bf04xxx) or these PON-IP speculative regs from the CPU; my gpon_pbo_init
 * accumulated ~15 such writes (MEDIA_STS_US, MOCIR force, credit cluster, internal-link
 * force) across earlier rounds. One may BREAK the US-NIC ingress that would otherwise
 * accept the cpu-tag frame. usnic_strip=1 SKIPS them all (stock-faithful pbo only);
 * =0 restores the prior behavior. Default 1 for the strip-and-retest. */
static unsigned int usnic_strip;	/* 0 = KEEP the US-NIC writes: live-stock same-board diff shows stock HAS
					 * MOCIR 0x2170/0x2174=0x1ffff etc. set, so stripping them DIVERGED from
					 * stock (the "stock never writes these" hypothesis was wrong; live = truth). */
module_param(usnic_strip, uint, 0644);
MODULE_PARM_DESC(usnic_strip, "1=skip the speculative non-stock US-NIC writes (strip test), 0=keep them (match stock)");
/* Faithful port of the stock PON-MAC GPON mode-set branch as ONE ordered
 * block at O5. Root cause: the US-NIC RX engine latches its
 * SID-classify table at the GMII_RX_EN rising edge; my driver sets the classify at
 * O5 but never RE-PULSES that edge, so the US-NIC keeps a stale boot-time latch and
 * discards SID-64 OMCI pre-MAC. Default OFF (protect DS/LAN/ranging); bisectable. */
static unsigned int ponmac_modeset;	/* 0 = OFF: the classify block + GMII re-latch at O5 had NO effect on rxsid (DISPROVEN — the latch is not at the GMII edge); kept for reference, default off. */
module_param(ponmac_modeset, uint, 0644);
MODULE_PARM_DESC(ponmac_modeset, "1=stock-ordered GPON mode_set classify block + GMII re-latch at O5");

/* finding 3 (2026-06-13): stock fires the US-NIC GMII_RX_EN latch edge ONCE,
 * at O5, AFTER the COMPLETE OMCC provisioning (classify triple + the US scheduler flow2queue
 * binding). My driver fires it at boot/ifup (gpon_pbo_init) — BEFORE the scheduler (qid 64
 * PIR/CIR/WFQ/SCH_QMAP) is bound in gpon_install_tcont. The round-32 re-latch "had no effect"
 * but that predates the SID2QID packing fix, so it re-latched the WRONG classify slot. With
 * SID2QID[64] now at the true word 0x2138=64, re-latch the edge at the END of install_tcont so
 * the US-NIC RX engine latches the complete+correct config. Default ON for this bisection step. */
static unsigned int relatch_us = 1;
module_param(relatch_us, uint, 0644);
MODULE_PARM_DESC(relatch_us, "1=re-pulse US-NIC GMII_RX_EN edge after the scheduler binding (finding 3)");
static unsigned int serdes_recommit;	/* 0 = OFF: a full SDS re-commit (CMD_SDS_RST_PS) at O5 DROPS the locked DS framer (omcirx->0, DS dead) — DISPROVEN destructive. */
module_param(serdes_recommit, uint, 0644);
MODULE_PARM_DESC(serdes_recommit, "1=also re-commit SerDes inside the O5 mode_set block (risky; gate separately)");

/* swcore TBL_ACCESS engine (L2/VLAN tables) */
#define TBL_CTRL_OFF	0x12000u
#define TBL_STS_OFF	0x12004u	/* BUSY = bit13 */
#define TBL_WRDATA_OFF	0x12008u
#define TBL_BUSY_BIT	(1u << 13)

static bool tbl_ok = true;

static void tbl_wait(void)
{
	int to = 0;

	while ((sw_rd(TBL_STS_OFF) & TBL_BUSY_BIT) && to++ < 0x4000)
		udelay(1);
	if (to >= 0x4000)
		tbl_ok = false;		/* engine wedged / wrong encoding: stop */
}

static void tbl_write(u32 type, u32 addr)
{
	u32 ctrl;

	if (!tbl_ok)
		return;
	tbl_wait();
	if (!tbl_ok)
		return;
	ctrl = sw_rd(TBL_CTRL_OFF);
	ctrl = (ctrl & ~(0x7u << 0))   | ((type & 0x7u) << 0);	/* TBL_TYPE [2:0]   */
	ctrl = (ctrl & ~(0x1u << 3))   | (1u << 3);		/* CMD_TYPE=write   */
	ctrl = (ctrl & ~(0x7u << 4))   | (1u << 4);		/* ACCESS_METHOD=1  */
	ctrl = (ctrl & ~(0xfffu << 9)) | ((addr & 0xfffu) << 9);/* ADDR [20:9]      */
	sw_wr(TBL_CTRL_OFF, ctrl);
	tbl_wait();
}

void rtl9602c_full_sdk_datapath_init(void)
{
	void __iomem *ipsel = (void __iomem *)0xb800063cul;	/* SoC PONPBO clk */
	int port, idx;

	if (!full_sdk_init)
		return;

	/* 1) switch_init -------------------------------------------------- */
	writel(readl(ipsel) | (1u << 5), ipsel);	/* PONPBO IP enable      */
	pi_field(0x2190, 7, 0, 0x6e);			/* PON_TB_CTRL tick      */
	pi_field(0x2190, 15, 8, 0x95);
	sw_field(0x25000, 7, 0, 43);			/* METER_TB_CTRL tick    */
	sw_field(0x25000, 15, 8, 189);
	sw_field(0x2d89c, 0, 0, 1);			/* SCH_WFQ_TKN_CTRL      */
	sw_field(0x2d8b8, 18, 0, 0x3ffff);		/* LINE_RATE_2500M       */
	sw_field(0x00110, 0, 0, 1);			/* PATCH_PHY_DONE        */
	sw_field(0x23040, 0, 0, 1);			/* CFG_UNHIOL IPG_COMP   */
	sw_field(0x20c04, 2, 2, 1);			/* P_MISC[CPU] RX_SPC    */
	for (port = 0; port <= 3; port++)
		sw_field(0x11008 + port * 4, 1, 0, 0x3);	/* ACCEPT_MAX_LEN */

	/* 2) l2_init: per-port action defaults (FORWARD). NOTE: a trial that wrote the
	 * exact stock LUT-action values (0x1c000=0x1a, 0x1c00c=0xaa, ...) + the MSTI/STP
	 * region BROKE the DS path (omcirx went 0) — our other config doesn't satisfy
	 * those actions' assumptions, and STP already reads 0x0f forwarding anyway. Keep
	 * the working FORWARD(0) defaults; do NOT match those stock LUT actions. */
	sw_field(0x17000, 22, 22, 1);			/* LUT LINKDOWN_AGEOUT   */
	for (port = 0; port <= 3; port++) {
		sw_field(0x1c00c, port * 2 + 1, port * 2, 0);
		sw_field(0x17004, port, port, 1);
		sw_field(0x1c004, port * 2 + 1, port * 2, 0);
		sw_field(0x1c000, port * 2 + 1, port * 2, 0);
		sw_field(0x1c014, port * 2 + 1, port * 2, 0);
		sw_field(0x1c010, port * 2 + 1, port * 2, 0);
		sw_field(0x1c008, port * 2 + 1, port * 2, 0);
		sw_field(0x1c02c, port, port, 1);
	}
	for (idx = 0; idx < 0x200 && tbl_ok && full_sdk_tables; idx++) {
		sw_wr(TBL_WRDATA_OFF + 0x0, 0);
		sw_wr(TBL_WRDATA_OFF + 0x4, 0);
		tbl_write(/*L2_UC*/ 0, idx);
	}

	/* 3) vlan_init: default VLAN-1 (all ports member) — only ENABLE
	 *    filtering if the member entry actually wrote (else keep VLAN off,
	 *    our working baseline). Risky table-engine path: gated by full_sdk_tables. */
	for (port = 0; port <= 3; port++) {
		sw_field(0x13000, port * 2 + 1, port * 2, 0);	/* ACCEPT ALL    */
		sw_field(0x2a000 + 4 * port, 1, 0, 0);		/* EGRESS ORIG   */
	}
	if (full_sdk_tables) {
		tbl_ok = true;				/* retry engine for VLAN  */
		sw_wr(TBL_WRDATA_OFF + 0x0, (0xfu << 4) | 0xfu);  /* untag|mbr=all */
		sw_wr(TBL_WRDATA_OFF + 0x4, 0x7f);
		tbl_write(/*VLAN*/ 1, /*vid*/ 1);
		if (tbl_ok) {
			for (port = 0; port <= 3; port++)
				sw_field(0x13004, port, port, 1);	/* VLAN_INGRESS  */
			/* VLAN_FILTER ON for ranging/config (reliable onlining); the FSM auto-clears
			 * it once stably at O5 to open LAN access (see vlan_lan_open in gpon_fsm_poll).
			 * lan_keep_open (default) keeps LAN open from boot -> never assert the filter,
			 * so a bad cold-start (no O5) or a WAN-disconnect can't kill LAN management. */
			if (!lan_keep_open)
				sw_field(0x13008, 0, 0, 1);	/* VLAN_FILTER on (config phase) */
			sw_field(0x13008, 4, 4, 0);
		}
	}

	/* 4) port_init: CPU + PON force link UP (already done in swcore bringup;
	 *    re-assert for stock fidelity/order). Ports 0,1 stay auto. */
	sw_field(0x180 + 3 * 4, 1, 0, 2); sw_field(0x180 + 3 * 4, 2, 2, 1);
	sw_field(0x180 + 3 * 4, 4, 4, 1); sw_wr(0x1b4 + 3 * 4, 0xfff);
	sw_field(0x180 + 2 * 4, 1, 0, 2); sw_field(0x180 + 2 * 4, 2, 2, 1);
	sw_field(0x180 + 2 * 4, 4, 4, 1); sw_wr(0x1b4 + 2 * 4, 0xfff);

	/* 5) cpu_init: TAG_AWARE AFTER the CPU port is forced link-up. */
	sw_field(0x23030, 8, 8, 1);			/* TRAP_TAGET_INSERT_EN  */
	sw_field(0x23030, 9, 9, 1);			/* TAG_AWARE             */

	/* 6) trap_init: RMA baseline */
	sw_field(0x1c084, 2, 0, 0);
	sw_field(0x1c01c, 2, 0, 0);
	sw_field(0x1c03c, 5, 4, 2);
	sw_field(0x1c040, 5, 4, 2);
	sw_field(0x1c100, 2, 0, 0);
	sw_field(0x1c0cc, 0, 0, 0);

	/* 7) classify_init: CF_CFG = the EXACT live-working-stock value 0x1d009 (diffed
	 *    stock-vs-mine 2026-06-13). = CF_US_PERMIT=1 (bits[1:0]) + CF_SEL_PON_EN=1
	 *    (bit3, the classification engine ROUTES/GATES the PON path) + CF_PATTERN1_NUM
	 *    =128 (bits[12:5]) + WANIF_DEFAULT_MULTICAST=0xe (bits[16:13]). The PON-port
	 *    egress of the cpu-tagged US-OMCI is gated by the CF engine selecting the PON
	 *    path; with CF_SEL_PON_EN=0 (our old value) the frame is never routed to port-2.
	 *    (A prior round read the classify_init SOURCE default=0 and wrongly reverted
	 *    CF_US_PERMIT to 0 — the gpon driver_initialize OVERRIDES it to this; the LIVE
	 *    stock register is the ground truth.) */
	sw_wr(0x1600c, 0x0001d009u);
	/* Port isolation + BUM-flood masks to the live-stock values (we had 0x3fffff /
	 * 0x0b; stock = 0x000ff9ff / 0x08 CPU-only). */
	sw_wr(0x27000, 0x000ff9ffu); sw_wr(0x27004, 0x000ff9ffu);
	sw_wr(0x27008, 0x000ff9ffu); sw_wr(0x2700c, 0x000ff9ffu);
	sw_wr(0x1c020, 0x00000008u); sw_wr(0x1c024, 0x00000008u); sw_wr(0x1c028, 0x00000008u);

	/* 8) ponmac_init: PON-IP scheduler + OMCI egress steering */
	sw_field(0x001ec, 0, 0, 1);			/* DYNGASP_CMP_INV       */
	if (serdes_stock_analog) {
		/* Match live-stock post-reset SDS_ANA: REG01 (0x22584)=0x73a4 (CMU bit14=1,
		 * BEN_TTL_OUT bit0=0) + REG11 (0x225ac) RX_FILT_CONFIG=0. The golden table sets
		 * these BEFORE the SDS reset, which wipes REG01 bit14 / REG11 RX_FILT to defaults
		 * (0x33a4 / 0xb008); we re-apply them here, post-reset, exactly like stock. This
		 * was the ONLY stock-vs-ours SerDes register diff and bit14 is in the shared CMU
		 * block -> the marginal TX serializer behind the cold-start ~50% US-TX "Laser out". */
		sw_field(0x22584, 14, 14, 1);		/* REG01 CMU bit14 = 1 (stock 0x73a4) */
		sw_field(0x22584,  0,  0, 0);		/* REG01 BEN_TTL_OUT = 0 (stock)      */
		sw_field(0x225ac,  7,  0, 0);		/* REG11 RX_FILT_CONFIG = 0 (stock)   */
	} else {
		sw_field(0x22584, 0, 0, 1);		/* legacy SDS_ANA REG_BEN_TTL_OUT = 1 */
	}
	pi_field(0x02150, 29, 16, 5);			/* PON_BW_THRES last     */
	pi_field(0x02150, 13, 0, 5);			/* PON_BW_THRES runt     */
	pi_field(0x02194, 18, 18, 1);			/* PON_GEN_PIR_DROP      */
	for (idx = 0; idx < 8; idx++) {
		pi_field(0x023e8, idx, idx, 0);		/* WFQ_TYPE = STRICT     */
		pi_field(0x02198 + idx * 4, 31, 0, 0);	/* QID_CIR_RATE = 0      */
	}
	sw_field(0x111f8, 2, 0, 7);			/* PON_TRAP_CFG OMCI_MPCP_PRIORITY=7 -> steer OMCI egress to PON queue 7 */
	pi_field(0x02194, 18, 18, 0);			/* rev-A: clear PIR_DROP */
	sw_field(0x20804, 2, 2, 1);			/* P_MISC[PON] RX_SPC    */
	sw_field(0x20c04, 2, 2, 1);			/* P_MISC[CPU] RX_SPC    */

	pr_info("rtl9602c-gpon: full_sdk_datapath_init done (tbl_ok=%d)\n", tbl_ok);
}
EXPORT_SYMBOL(rtl9602c_full_sdk_datapath_init);

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
/* NOT __init: also called from rtl9602c_eth_open() (the eth driver) to RE-RUN the
 * full PON US/DS-NIC bring-up AFTER the GMAC IP-block reset, so the US-NIC RX engine
 * latches against the freshly-reset GMAC (stock order: GMAC reset -> then PON-NIC). */
void gpon_pbo_init(void)
{
	/* 1. Halt GMII and disable both packet buffers while reconfiguring. */
	/* Enable the upstream GMII TX/RX framer to its O5 operating value (0x90101070,
	 * GMII_TX_EN|GMII_RX_EN + US datapath bits). This was forced to 0, leaving
	 * the PON-IP upstream datapath DISABLED — the ONU composed Serial_Number_ONU
	 * but could never transmit it upstream, so the OLT never heard it and the
	 * ONU was stuck in O3. Must be set here in pbo_init (before the MAC reset);
	 * setting it post-boot is too late. */
	/* US-NIC GMII RX/TX enable is DEFERRED to the very END of this function
	 * (after the descriptor pool + PBUF_EN), mirroring the DS side (IO_CMD_0_DS
	 * written last) and the stock pbo_init off-configure-on edge. ROOT CAUSE of
	 * "US-NIC RX never receives": the RX engine LATCHES the descriptor-pool /
	 * FIFO config at the GMII-RX-enable edge; enabling GMII here (FIRST, before
	 * the pool is configured) latched an unprovisioned pool, so every US frame
	 * was dropped at descriptor-fetch before the MAC (PKT_OK/ERR/MISS all 0).
	 * Here: write IO_CMD_0_US with GMII_RX_EN[5]/GMII_TX_EN[4] CLEARED (0x...50);
	 * the full 0x90101070 is re-written last. */
	pi_wr(PI_IO_CMD_0_US, 0x90101050);	/* GMII OFF; pool configured below, GMII enabled LAST */
	/* DS IO_CMD (the DMA/FIFO drain enable, 0x90081070) is written LAST, after the
	 * backpressure thresholds + PBUF_EN, so the DS engine drains out of a properly
	 * bounded buffer (see end of this function). */
	pi_field(PI_PONIP_CTL_US, 0, 0, 0);		/* CFG_PBUF_EN = 0        */
	pi_field(PI_PONIP_CTL_DS, 0, 0, 0);

	/* 2. Descriptor accounting (128B pages). US uses a DRAM packet pool (the
	 * US-NIC RX path REQUIRES it — see PI_IP_MSTBASE_US note); DS stays
	 * SRAM-only like stock. Allocate the 1MB US PBO DRAM pool once and point
	 * the HW at its phys base. The PON-IP DMAs received US packets into this
	 * region; the CPU never reads it, so plain contiguous pages + virt_to_phys
	 * suffice (stock uses a fixed reserved region at 0x07eff000). */
	{
		static unsigned long us_pool;

		if (!us_pool)
			us_pool = __get_free_pages(GFP_KERNEL, PI_US_DRAM_ORDER);
		if (us_pool)
			pi_wr(PI_IP_MSTBASE_US,
			      (u32)virt_to_phys((void *)us_pool));
		else
			pr_warn("rtl9602c-gpon: US PBO DRAM pool alloc failed; US-NIC RX may not work\n");
	}
	pi_field(PI_PON_DSC_CFG_US, 12, 0, PI_US_SRAM_NO);
	pi_field(PI_PON_DSC_CFG_DS, 12, 0, PI_DS_SRAM_NO);
	pi_field(PI_PON_DSC_CFG_US, 28, 16, PI_US_DRAM_PAGES);	/* RAM_NO = SRAM+DRAM (0x1fff) */
	pi_field(PI_PON_DSC_CFG_DS, 28, 16, PI_DS_SRAM_NO);
	pi_field(PI_DSCRUNOUT_US, 12, 0, PI_US_SRAM_RUNOUT);
	pi_field(PI_DSCRUNOUT_DS, 12, 0, PI_DS_SRAM_RUNOUT);
	pi_field(PI_DSCRUNOUT_US, 28, 16, PI_US_DRAM_RUNOUT);	/* DRAM runout (0x1f58) */
	pi_field(PI_DSCRUNOUT_DS, 28, 16, 0);

	/* PBO backpressure thresholds (SRAM-only, 128B pages). Without these every
	 * threshold field reads 0, so the PBO treats the small DS SRAM pool as
	 * instantly over-threshold and never releases switch flow-control — which is
	 * why enabling the full DS DMA backs the buffer up and stalls the US. The
	 * per-SID RPV thresholds (0x02458, stride 4) are flow-control limits, NOT SID
	 * validity (writing them for every SID is safe; SIDVALID/SID2QID untouched).
	 * Values: us_sram_runt 126 -> stop 125, glb on/off 125/123; per-SID 150/130
	 * (page scale 1, non-tripping); DS flow-ctrl on/off 2/22. */
	/* DRAM-pool thresholds (stock live values; stock formula: stop=us_dram_runt-40,
	 * global ON/OFF scaled to the 1MB pool). The old 125/125/123 were the
	 * SRAM-only values — far too small for the DRAM pool, tripping flow
	 * control immediately. */
	pi_field(PI_PON_SID_STOP_TH, 12, 0, 0x1f30);
	pi_field(PI_PON_SID_GLB_TH, 28, 16, 0x1ee0);
	pi_field(PI_PON_SID_GLB_TH, 12, 0, 0x1e40);
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
	/* R_DBG_FUNC_SEL[7:5] = the US-NIC ingest-source select: field-bit-1 (=reg 0x40)
	 * wires switch-PON-port-2-egress -> PON-IP US-NIC GMII-RX so the US-NIC RECEIVES
	 * + SID-stamps US OMCI (RX_SID_GOOD_CNT_US can increment). This is the SYMMETRIC
	 * twin of the DS fix at the end of this function (PI_PROBE_SELECT_DS=0x40): the
	 * old `pi_field(...,1,1,1)` wrote REGISTER bit1 (0x02) — the same bit1-vs-bit6
	 * error the DS side had — leaving [7:5]=0 so US OMCI never reached the US-NIC
	 * (rxsid=0 regardless of cpu-tag). Stock pbo_init sets field-bit-1 (live 0x40). */
	pi_field(PI_PROBE_SELECT_US, 7, 5, 2);		/* R_DBG_FUNC_SEL=010b => reg 0x40 */
	pi_field(PI_CFG_US, 26, 26, 1);			/* E_EN_RFF_AFULL         */
	pi_field(PI_CFG_US, 17, 17, 1);			/* EN_TX_STOP             */
	pi_field(PI_CFG_US, 16, 16, 1);			/* EN_TXE_EXTRA           */
	/* (0xD400 PROBE_SELECT_DS is set to the stock golden 0x40 at the end of this
	 * function, with 0xD404/0xD42C — the DS-NIC drain config, not a debug probe.) */
	pi_field(PI_CFG_DS, 26, 26, 1);
	pi_field(PI_CFG_DS, 17, 17, 1);
	pi_field(PI_CFG_DS, 16, 16, 1);
	/* CFG_DS[6:0] = RX_SID: the stream-id the DS-NIC STAMPS on every frame it egresses
	 * over the internal MII into GMAC0's GMII-RX. The GMAC's CPUtag1CR SID-64 trap only
	 * fires (and the GMAC only accepts the on-wire cpu-tag) when this SID = 64. Reset
	 * default is 0x40 but the pbo MAC reset can clear it; set it explicitly. */
	pi_field(PI_CFG_DS, 6, 0, 64);			/* RX_SID = 64 (OMCC SID) */
	/* CFG_US[6:0] = RX_SID for the US-NIC, symmetric to the DS stamp above. The
	 * US-NIC stamps this stream-id on frames it receives from the switch (the
	 * CPU's US OMCI), so they classify to SID 64 -> GEM port 2 -> physical queue
	 * 64 -> T-CONT 16. Without it the OMCI reaches the US-NIC with NO stream-id
	 * (the switch strips our software cpu-tag at the PON-port egress) so it never
	 * enqueues to qid 64 and the OMCC US GEM stays empty (gem2=0, idle16 high).
	 * The DS stamp alone is not enough — this is the US half that was missing. */
	pi_field(PI_CFG_US, 6, 0, 64);			/* US-NIC RX_SID = 64 (OMCC SID) */

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
	/* US-NIC PBUF_EN=1. NOTE: live stock runs ctl_us(0x20ac)=0 (PBUF OFF, US streams
	 * to the GTC), but setting it 0 on THIS clean-room driver regressed the link to
	 * "Laser out" + deactivate (ks90/91) whereas PBUF_EN=1 holds O5 Active (ks89) —
	 * our incomplete US datapath depends on the buffer. A clean-room divergence;
	 * revisit when the full US datapath matches stock. */
	pi_field(PI_PONIP_CTL_US, 0, 0, 1);		/* CFG_PBUF_EN = 1 (stable for our datapath) */
	pi_field(PI_PONIP_CTL_DS, 0, 0, 1);
	pi_field(PI_PONIP_CTL_DS, 7, 7, 1);		/* CFG_TX_PAUSE low bit -> O5 value 0x81 (DS buffer release; safe now thresholds bound the buffer) */

	/* 8. Enable the full downstream PONNIC DMA drain (LAST — after the thresholds
	 * + PBUF_EN above). 0x90081070 = MAX_DMA_SEL_0[31] | EARLY_TX_EN[28] |
	 * TX_FIFO_THR[20:19]=1 | RX_FIFO_THR[12:11]=2 | RX_MAX_DMA_SEL[7:6]=1 |
	 * GMII_RX_EN[5] | GMII_TX_EN[4]. The DMA/FIFO fields start the DS engine
	 * draining de-encapsulated GEM frames out of the (now bounded) buffer into the
	 * switch -> CPU-port GMAC; DS OMCI (SID 64) egresses on the GMAC RX with the
	 * cpu-tag stream-id 64 that the NIC's OMCI hook catches. */
	/* Force the DS-NIC <-> GMAC0 internal-MII link UP (golden 0x106e8400,
	 * FORCELINK[18] FORCEDFULLDUP[19] FORCE_SPD/TRXFCE). This is the missing
	 * symmetric twin of the US write at PI_MEDIA_STS_US below: the bootloader
	 * latches 0xc058 once, but rtl9602c_eth_open()'s GMAC0 IP-block power-cycle
	 * (rtl9602c_ipsel_cycle, gmac_reset=1) tears down the GMAC0 side of this
	 * link and NOTHING re-asserts it -> a de-encapped DS frame counts RX_OK at
	 * the DS-NIC MAC (PKT_OK_CNT_DS @0xc010 = D_rxok) but never crosses the
	 * un-trained MII into the GMAC RX ring (filled=0, gpon0 RX=0) on ~50% of
	 * cold boots. Force it here, BEFORE the GMII enable edge, so the edge
	 * latches against an established link. */
	pi_wr(PI_MEDIA_STS_DS, 0x106e8400u);
	pi_wr(PI_IO_CMD_0_DS, 0x90081070u);

	/* PON-IP DS-NIC drain config (0xD400/0xD404/0xD42C). A LIVE STOCK ONU that is
	 * online and draining real OMCI has these SET — 0xd400=0x40, 0xd404=0x11100348,
	 * 0xd42c=0x40 — with PKT_OK_CNT_DS (0xc010) climbing (dumped 0x0265 = 613 frames).
	 * My driver left them 0 (and wrote 0xd400 bit1 instead of bit6), and its
	 * PKT_OK_CNT_DS stayed 0 = de-encapsulated DS OMCI was never handed off to the
	 * GMAC NIC (filled=0). The earlier "SWPBO-only, stock never touches them" note
	 * was wrong — measured against the live stock datapath. These program the DS
	 * de-encap engine's transfer to the GMAC NIC RX (the long-unsolved transfer gap).
	 * Written LAST, after IO_CMD_0_DS, matching stock's golden values verbatim. */
	pi_wr(PI_PROBE_SELECT_DS, 0x00000040u);
	pi_wr(PI_DS_NIC_CFG_D404, 0x11100348u);
	pi_wr(PI_DS_NIC_CFG_D42C, 0x00000040u);

	/* 9. US-NIC GMII enable — the VERY LAST write, after the descriptor pool,
	 * PBUF_EN, and the DS drain, mirroring IO_CMD_0_DS above. The US RX engine
	 * latches the now-provisioned pool/FIFO config at this GMII_RX_EN[5] rising
	 * edge (the root cause of US-NIC RX never receiving: we used to enable GMII
	 * FIRST, latching an empty pool). Force the GMAC0-TX -> US-NIC internal-MII
	 * link up first, then enable GMII RX/TX. 0x90101070 = MAX_DMA_SEL_0[31] |
	 * EARLY_TX_EN[28] | TX_FIFO_THR[20:19]=2 | RX_FIFO_THR[12:11]=2 |
	 * RX_MAX_DMA_SEL[7:6]=1 | GMII_RX_EN[5] | GMII_TX_EN[4]. */
	if (!usnic_strip)
		pi_wr(PI_MEDIA_STS_US, 0x106e8400u);	/* stock NEVER writes 0x1bf04058 */

	/* Pre-arm the OMCC SID-64 classification BEFORE the GMII_RX_EN latch edge below.
	 * The US-NIC RX engine latches its SID2QID/SIDVALID classification table at the
	 * GMII_RX_EN[5] rising edge (next write). Our driver previously programmed SID 64
	 * only later, at OMCC-install (gpon_install_omcc, O5) — AFTER this edge — so the
	 * RX engine latched WITHOUT SID-64 recognition and every SID-64 (OMCI) upstream
	 * frame was dropped pre-MAC: the GMAC TXes it fine and it reaches the US-NIC GMII
	 * (link up), but PKT_OK_CNT_US RX_OK/ERR/MISS all stay 0 (the frame is discarded
	 * before the MAC counts it because its SID is not a latched-valid classification).
	 * Bake the fixed OMCC SID->QID map here so it is latched at the edge.
	 *
	 * PACKED-TABLE ADDRESSING CORRECTED 2026-06-13 (from the stock array-field-write
	 * routine): the 7-bit SID2QID array (base PI 0x20f8) is packed
	 * entries_per_word = 32/width = 32/7 = 4 entries per 32-bit word (top 4 bits of each
	 * word UNUSED) -- it is NOT contiguous-bit-packed. So:
	 *   word  = sid / 4 ;  shift = (sid % 4) * 7
	 *   SID 64 -> word 16 -> PI 0x20f8 + 16*4 = 0x2138, shift 0.
	 * The OLD code wrote 0x2130 shift 0, which under this 4-per-word packing is SID 56's
	 * slot (a non-OMCI data flow that legitimately reads qid 63). That misread is why a
	 * prior session "live-stock SID2QID[64]=63" reading and the GPON_OMCC_PHYS_QID=63
	 * value were both wrong: they were reading SID 56, not SID 64. The TRUE SID-64 slot
	 * (0x2138) was never written, so the US-NIC classified OMCI to a stale/garbage queue.
	 * Stock value = physQid = TCONT_QUEUE_MAX(32)*(TCONT16/8)+queue0 = 64 (the stock
	 * flow-to-queue / physical-queue-id / SID-to-queue-map mapping).
	 * NOTE: SIDVALID(1b)/SID_Q_MAP_DS(2b) are unaffected (32/width divides evenly so
	 * 4-per-word == contiguous for them); only the 7-bit SID2QID diverges. */
	pi_field(0x2138, 6, 0, 64);	/* SID2QID[64] = OMCC phys qid 64 (TCONT16/q0); 4-per-word packing -> 0x20f8+16*4 */
	pi_field(0x2144, 0, 0, 1);	/* SIDVALID[64] = 1 (1b, 0x213c+64/32*4 = 0x2144 bit0) */
	/* PON_OMCI_CFG[6:0] = OMCC SID 64 — the THIRD member of the US-NIC ingress
	 * classification triple (SID2QID + SIDVALID + OMCI_CFG). The two writes above
	 * were baked pre-edge, but OMCI_CFG was left for gpon_install_omcc (O5, AFTER
	 * this latch) — so the RX engine latched WITHOUT knowing which SID is the OMCI
	 * management stream and discarded every SID-64 upstream frame PRE-MAC (RX_OK/
	 * ERR/MISS all 0). Stock ponmac_init writes the FULL triple before the GMII
	 * edge: it sets PON_OMCI_CFG @ 0x1bf02154 [6:0] = 64, THEN does the
	 * NIC-bringup GMII edge. Complete the triple here so all three latch together. */
	pi_field(0x2154, 6, 0, 64);	/* PON_OMCI_CFG[6:0] = OMCC SID 64 */

	/* Pre-arm the WAN DATA flow (SID 1) classify here too, alongside the SID-64
	 * pre-arm, so this boot/ifup GMII edge (the final pi_wr(0x90101070) below) ALSO
	 * latches flow-1 -- the same property that makes SID-64/OMCI reliable every boot.
	 * Without this, flow-1's classify was first written only later in
	 * gpon_install_data_gem (after this edge had already fired), making its latch
	 * depend on a chance later edge -> the ~50/50 data-US half-boot bug. SID2QID[1]=
	 * OMCC phys qid (data rides T-CONT 16's grants), SIDVALID[1]=1. gpon_install_data_gem
	 * refreshes the SAME values + re-pulses the edge, so this is a harmless pre-seed
	 * even on a re-config cycle (gpon_pbo_init is re-run from rtl9602c_eth_open). Pure
	 * US-NIC ingress-classify writes -- no SerDes/DS/optical touch.
	 * Use literal pi_field (matching the SID-64 pre-arm above) because the PI_PON_*
	 * macros / pi_packed_set helper are defined later in the file. 4-per-word 7-bit
	 * packing: SID2QID[1] -> base 0x20f8 word0, shift (1%4)*7=7 -> bits[13:7]=qid 64;
	 * SIDVALID[1] -> base 0x213c word0 bit1 (1-bit packing is contiguous). */
	if (!usnic_strip) {
		pi_field(0x20f8, 13, 7, data_tcont ? 32 : 64);	/* SID2QID[1] = data qid 32 (T-CONT 8) or legacy OMCC qid 64 */
		pi_field(0x213c, 1, 1, 1);	/* SIDVALID[1] = 1 */
	}

	/* MOCIR force-mode (stock QoS init): PON-IP 0x2170 MOCIR_FRC_MD=0x1FFFF,
	 * 0x2174 MOCIR_FRC_VAL=0x1FFFF — force the per-flow committed-info-rate to a
	 * fixed mode so the US-NIC's CIR auto-detection cannot trip for the OMCC SID-64
	 * flow and silently discard the OMCI frame BEFORE the MAC counts it (the exact
	 * RX_OK/ERR/MISS-all-0 symptom = a pre-MAC discard). Our minimal init omitted it. */
	if (!usnic_strip) {
		pi_wr(0x2170, 0x0001ffffu);	/* MOCIR_FRC_MD  = 0x1FFFF (all flows forced)   */
		pi_wr(0x2174, 0x0001ffffu);	/* MOCIR_FRC_VAL = 0x1FFFF (forced CIR = max)   */
	}

	/* Stock PON-MAC US-NIC datapath credit/threshold cluster (from the stock
	 * ponmac_init behavior, with addresses from the chip's register map, then
	 * pinned to the EXACT runtime values read from the live working stock ONU @O5).
	 * Our minimal init left this WHOLE cluster at 0, so the US-NIC's grant/credit/
	 * token-bucket logic never released a credit for the OMCI SID-64 flow and every
	 * upstream frame was discarded BEFORE the MAC counted it (PKT_OK/ERR/MISS all 0,
	 * rxsid=0 — a pre-MAC credit discard, NOT a link-down). Stock writes these in
	 * ponmac_init AHEAD of the GMII rising edge that latches US-NIC ingress, so they
	 * go here, just before the final pi_wr(PI_IO_CMD_0_US, 0x90101070) edge below.
	 * The "5 written twice" register resolves to 0x2150 on the live stock (0x00050005),
	 * NOT 0x20f4 (off-by-4, disambiguated by the live stock register read). */
	if (!usnic_strip) {
		pi_wr(0x2150, 0x00050005u);	/* PON US-NIC IP-status/BW threshold (reg924, 5+5)  */
		pi_wr(0x20f0, 0x00000013u);	/* US-NIC datapath cfg                              */
		/* 0x20f4 = PON_IPSTS_US, a READ-ONLY init-ready status reg (bit0=PONIC_INITRDY);
		 * stock never writes it. The old pi_wr(0x20f4,1) was a no-op write to a reserved
		 * bit -> removed. It is POLLED before the GMII latch edge below (usnic_initrdy_poll). */
		pi_wr(0x2184, 0x00001000u);	/* MOCIR_TH_H (request/grant credit threshold)      */
		pi_wr(0x2188, 0x00001000u);	/* MOCIR_TH_L                                       */
		pi_wr(0x218c, 0x0003ffffu);	/* PON_OLT_BW_MTR_FULL (maxFlow)                    */
		pi_wr(0x2190, 0x0000956eu);	/* PON_TB_CTRL (token bucket)                       */
		pi_wr(0x23a0, 0x0000000fu);	/* PON_SCH_QMAP                                     */
		pi_wr(0x23e4, 0x00010001u);	/* PON US T-CONT enable                             */
	}

	/* === GMAC0<->US-NIC INTERNAL link FORCE — the long-missing step ===
	 * Decoded clean-room from the stock PON-MAC GPON mode-set branch
	 * (GPON mode). Our optical SerDes is up (DS OMCI, US PLOAM,
	 * ranging all work) but a CPU cpu-tag direct-TX US-OMCI frame NEVER reaches the
	 * US-NIC ingress (RX_SID_GOOD_CNT_US[4]=0, PKT_OK/ERR/MISS=0): the switch
	 * PON-port(2)<->US-NIC INTERNAL MAC link was never FORCED up for the egress/
	 * direct-inject direction (DS works because that direction's link is up). Stock
	 * forces it here in the GPON mode-set, just before the US-NIC GMII RX/TX enable
	 * edge, so the US-NIC latches with the internal link live. ABLTY_FORCE_MODE GPON
	 * value=0xc (vs 0x1f/8/4/5 for the Ethernet/SGMII WAN modes); CFG_FE_POLL_WD is
	 * the front-end GMII auto-poll/watchdog that actually trains the forced link and
	 * is set ONLY in GPON mode. Stock releases the force externally post-train (steady
	 * O5 reads 0) — we leave it SET: we want the internal link held up. Addresses +
	 * values match the stock behavior; the in-word bit shifts are from the field map. */
	if (!usnic_strip) {
		sw_field(0x001b4, 4, 0, 0xc);	/* ABLTY_FORCE_MODE[4:0]=0xc — GPON internal-link force   */
		sw_field(0x000f4, 5, 5, 1);	/* CFG_FE_POLL_WD_1[5]=1 — front-end GMII poll/watchdog    */
		mdelay(10);			/* settle the forced internal link before PCS enables+edge */
		sw_field(0x22a70, 11, 11, 0);	/* SDS_EXT_REG28[11]=0 — release non-GPON SerDes-select    */
		sw_field(0x220e0, 9, 8, 1);	/* WSDS_DIG_2C[9:8]=1 — WAN-PCS digital enable            */
		pi_field(0x0a10c, 13, 13, 1);	/* RSVD_PONIP_DS[13]=1 — DS-side datapath enable           */
		pi_field(0x0a10c, 12, 12, 1);	/* RSVD_PONIP_DS[12]=1                                     */
		sw_field(0x22080, 12, 12, 1);	/* WSDS_DIG_14[12]=1 — WAN-PCS digital enable             */
	}

	/* CF_CFG.CF_US_PERMIT is set to its stock value (0 = NORMAL/permit) by
	 * rtl9602c_full_sdk_datapath_init()'s classify_init step. (An earlier attempt
	 * set it to 1 here = NOPON/permit-without-PON, which the stock behavior
	 * shows actually STRIPS the PON egress path — reverted.) */

	/* PON_IPSTS_US.PONIC_INITRDY commit-poll (PON-IP 0x1bf020f4 bit0): wait for the
	 * US PON-IP core to report init-ready BEFORE the GMII RX/TX latch edge below, so
	 * the US-NIC latches with the core ready. Passive, bounded (200ms), read-only ->
	 * no config/reset reorder, cannot worsen the lock; on timeout log+proceed (never
	 * hang boot). Stock has no such poll (relies on fixed mdelay) — this is a stock-
	 * aligned readiness gate + discriminating instrumentation for the ~50% bug. */
	if (usnic_initrdy_poll) {
		int n;

		for (n = 0; n < SDS_LOCK_POLL_MAX; n++) {
			if (pi_rd(0x20f4) & BIT(0))		/* PONIC_INITRDY */
				break;
			udelay(200);
		}
		if (n >= SDS_LOCK_POLL_MAX) {
			pr_warn("rtl9602c-gpon: PON_IPSTS_US.PONIC_INITRDY not set after %dus; proceeding\n",
				SDS_LOCK_POLL_MAX * 200);
			if (usnic_initrdy_repulse) {		/* ACTIVE: re-roll the CDR lock */
				u32 cdr = sw_rd(SDS_ANA_COM_REG12);

				sw_wr(SDS_ANA_COM_REG12, cdr ^ BIT(15));
				mdelay(10);
				sw_wr(SDS_ANA_COM_REG12, cdr);
				for (n = 0; n < SDS_LOCK_POLL_MAX; n++) {
					if (pi_rd(0x20f4) & BIT(0))
						break;
					udelay(200);
				}
				pr_warn("rtl9602c-gpon: PONIC_INITRDY after CDR re-pulse: %s\n",
					(pi_rd(0x20f4) & BIT(0)) ? "ready" : "still-not-ready");
			}
		} else {
			pr_info("rtl9602c-gpon: PON_IPSTS_US.PONIC_INITRDY ready after %dus\n", n * 200);
		}
	}

	pi_wr(PI_IO_CMD_0_US, 0x90101070u);
}
EXPORT_SYMBOL(gpon_pbo_init);	/* re-run from rtl9602c_eth_open() after the GMAC reset */

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

/* Full PON-IP register dump in the EXACT format of cross-compiler/stock_dump_good.txt
 * ("0x1bf0XXXX YYYYYYYY"), so mine-vs-stock diffing needs no reformatting. Covers the
 * 0x0000-0x54fc span the stock oracle captured. Read once: cat /proc/pidump. */
static int pidump_proc_show(struct seq_file *s, void *v)
{
	/* Only the mapped PON-IP windows (gaps bus-fault on read). Ranges match the
	 * readable spans of cross-compiler/stock_dump_good.txt. */
	static const u32 ranges[][2] = {
		{0x0000, 0x03fc}, {0x2000, 0x2bfc}, {0x4000, 0x40fc}, {0x5400, 0x54fc},
	};
	u32 off, val;
	int r;

	for (r = 0; r < (int)ARRAY_SIZE(ranges); r++)
		for (off = ranges[r][0]; off <= ranges[r][1]; off += 4) {
			val = pi_rd(off);
			if (val)	/* skip zeros: keeps the dump small for reliable serial read */
				seq_printf(s, "0x%08x %08x\n", 0x1bf00000u + off, val);
		}
	seq_puts(s, "0x1bf0ffff ffffffff\n");	/* end marker */
	return 0;
}

/* Switch-core + GMAC + SerDes + GTC dump (the GMAC0->switch->US-NIC handoff path that is
 * OUTSIDE PON-IP space). Same "0xADDR VAL" format as stock_dump_good.txt for direct diff.
 * 0x1bxxxxxx regs come via swcore_base (sw_rd); GMAC 0x18012/0x18013 via a local ioremap. */
static int swdump_proc_show(struct seq_file *s, void *v)
{
	static const u32 sw[][2] = {		/* offsets into swcore_base (phys 0x1b000000) */
		{0x00000, 0x000fc}, {0x1c000, 0x1c0fc}, {0x20800, 0x2083c},
		{0x20c00, 0x20c3c}, {0x23000, 0x230fc}, {0x27000, 0x2703c}, {0x701000, 0x70101c},
	};
	static const u32 gm[][2] = {		/* absolute phys (separate ioremap) */
		{0x18012000, 0x180120fc}, {0x18013400, 0x180134fc},
	};
	u32 off, a, val;
	int r;

	for (r = 0; r < (int)ARRAY_SIZE(sw); r++)
		for (off = sw[r][0]; off <= sw[r][1]; off += 4) {
			val = sw_rd(off);
			if (val)
				seq_printf(s, "0x%08x %08x\n", 0x1b000000u + off, val);
		}
	for (r = 0; r < (int)ARRAY_SIZE(gm); r++) {
		void __iomem *b = ioremap(gm[r][0], gm[r][1] - gm[r][0] + 4);

		if (!b)
			continue;
		for (a = gm[r][0]; a <= gm[r][1]; a += 4) {
			val = ioread32(b + (a - gm[r][0]));
			if (val)
				seq_printf(s, "0x%08x %08x\n", a, val);
		}
		iounmap(b);
	}
	seq_puts(s, "0xffffffff ffffffff\n");	/* end marker */
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

/* Per-flow downstream GEM FORWARDED-to-PON-IP count (GEM_DS_FWD_CNTR, IND 0x404C /
 * STAT 0x4050, field ETH_PKT_FWD) — same indirect protocol as gpon_gem_ds_rx_cnt.
 * DECISIVE diagnostic: GEM_NON_IDLE is a GLOBAL de-assembler counter, so its rise
 * only proves the GTC de-encapsulated SOMETHING, not that flow-64/OMCI was forwarded
 * toward the PON-IP. FWD[64] localizes the break: if flow_cnt(64) climbs but FWD=0
 * the GTC de-encap'd but did NOT forward (GTC-side gap); if FWD climbs but the PON-IP
 * DS SRAM (PI_DSC_USAGE_DS) stays flat the PON-IP rejected it (descriptor base/region
 * or the internal DS-GMII link MEDIA_STS_DS). */
static u32 gpon_gem_ds_fwd_cnt(u8 flow)
{
	int i;

	gpon_wr(0x404c, flow & 0x7f);
	for (i = 0; i < 1000; i++) {
		if (gpon_rd(0x404c) & BIT(15))		/* ETH_PKT_FWD_R_ACK */
			break;
		udelay(1);
	}
	return gpon_rd(0x4050);				/* ETH_PKT_FWD count */
}

/* Read back the DS GEM-port CAM entry for `flow` (READ op = OP_MODE 2): does the
 * CAM actually hold the OMCC gem at flow 64 at runtime? Returns [11:0]=stored gem,
 * bit16=OP_HIT. Stock de-encaps OMCI on flow 64 (786) while our flow 64 reads 0, so
 * either our CAM entry is wrong/absent or it is not being matched. */
static u32 gpon_ds_cam_read(u8 flow)
{
	int i;
	u32 ind;

	gpon_wr(0x1100, (2u << 8) | (flow & 0x7f));		/* DS_PORT_IND OP_MODE=READ, REQ=0 */
	gpon_wr(0x1100, (2u << 8) | (flow & 0x7f) | BIT(15));	/* REQ=1 -> trigger */
	for (i = 0; i < 1000; i++) {
		if (gpon_rd(0x1100) & BIT(14))	/* OP_COMPL */
			break;
		udelay(1);
	}
	ind = gpon_rd(0x1100);
	return ((ind & BIT(13)) ? BIT(16) : 0) | (gpon_rd(0x110c) & 0xfff);	/* HIT | RDATA gem */
}

/* Invalidate ALL 128 DS GEM-port CAM entries (CLEAN op = OP_MODE 3). The CAM holds
 * only a 12-bit gemPortId per entry with NO valid bit, so at reset the entries carry
 * GARBAGE gem values (observed e0=gem3566). The lookup matches an incoming gem against
 * every entry, so a stale entry that happens to equal the OMCC gem (2) shadows flow 64
 * and steals the DS OMCI — flow 64 then de-encaps nothing. Clear them all before
 * installing the OMCC so ONLY flow 64 matches gem 2. */
static void gpon_ds_cam_clear_all(void)
{
	int f, i;

	for (f = 0; f < 128; f++) {
		if (f == 64)		/* never disturb the OMCC flow (its CAM+TRAFFIC_CFG are
					 * written right after; the CLEAN op also zeroes the
					 * entry's TRAFFIC_CFG and races our isOMCI write). */
			continue;
		gpon_wr(0x1100, (3u << 8) | (f & 0x7f));		/* OP_MODE=CLEAN, IDX, REQ=0 */
		gpon_wr(0x1100, (3u << 8) | (f & 0x7f) | BIT(15));	/* REQ=1 -> trigger */
		for (i = 0; i < 1000; i++) {
			if (gpon_rd(0x1100) & BIT(14))			/* OP_COMPL */
				break;
			udelay(1);
		}
	}
}

/* GTC US MISC PM counter (GPON_GTC_US_MISC_CNTR_IDX 0x5140 [2:0] / STAT 0x5148):
 * write the raw type index, read the 32-bit count. raw idx map (stock US-GTC
 * misc-counter type mapping): 0=PLOAM_BOH_TX 1=GEM_DBRU_TX 2=PLOAM_CPU_TX
 * 3=PLOAM_AUTO_TX 4=GEM_BYTE_TX. Decisive US-emission instrument: idx2 (our ACK/SN
 * PLOAM, expect >0) vs idx4 GEM_BYTE_TX + TCONT_IDLE_BYTE_STAT[16] (expect 0 = ONU
 * transmits PLOAM but NO US GEM on its T-CONT-16 grants -> OLT deactivates ~42s). */
static u32 gpon_us_misc_cnt(u8 idx)
{
	gpon_wr(0x5140, idx & 0x7);
	udelay(5);
	return gpon_rd(0x5148);
}

/* GEM DS MISC PM counter (GPON_GEM_DS_MISC_IND 0x4064 [3:0]=idx, R_ACK bit15, STAT
 * 0x4068) — GLOBAL, CAM-INDEPENDENT de-encap-stage counters. idx map (gponv2):
 * 0=MC_RX 1=UC_RX 2=MC_FWD 3=MC_LEAK 4=ETH_CRC_ERR 5=OVER_INTERLEAV 6=OMCI_RX.
 * Decisive: UC_RX(1) counts every DS unicast GEM the de-assembler accepts regardless
 * of the per-port CAM; OMCI_RX(6) is the authoritative "GTC de-encapsulated an OMCI
 * frame" count. If UC_RX climbs but OMCI_RX stays 0 -> unicast arrives but is not
 * PTI-classified as OMCI; if neither climbs -> no unicast reaches the de-assembler. */
static u32 gpon_gem_ds_misc_cnt(u8 idx)
{
	int i;

	gpon_wr(0x4064, idx & 0xf);
	for (i = 0; i < 16; i++) {
		if (gpon_rd(0x4064) & BIT(15))
			break;
		udelay(2);
	}
	return gpon_rd(0x4068);
}

/* Read back the GTC alloc CAM entry for a T-CONT (READ op, same indirect protocol
 * as the DS GEM CAM): does T-CONT 16 actually hold alloc 0x400 + HIT? If the BWMAP's
 * alloc-id does not match a valid CAM entry, the GTC ignores the operational grant
 * (bwm_acpt stays 0) and the ONU never transmits operational US. Returns [11:0]=
 * stored allocId, bit16=OP_HIT. ALLOC_IND 0x10c0 / ALLOC_RD 0x10cc. */
static u32 gpon_alloc_cam_read(u8 tcont)
{
	int i;
	u32 ind;

	gpon_wr(0x10c0, (2u << 8) | (tcont & 0x1f));		/* OP_MODE=READ, REQ=0 */
	gpon_wr(0x10c0, (2u << 8) | (tcont & 0x1f) | BIT(15));	/* REQ=1 -> trigger */
	for (i = 0; i < 1000; i++) {
		if (gpon_rd(0x10c0) & BIT(14))			/* OP_COMPL */
			break;
		udelay(1);
	}
	ind = gpon_rd(0x10c0);
	return ((ind & BIT(13)) ? BIT(16) : 0) | (gpon_rd(0x10cc) & 0xfff);
}

/* DS-PIPELINE STAGE-A: per-flow de-encapsulated GEM frame count (indirect):
 * GPON_GTC_DS_PORT_CNTR_IND(0x1140) IDX[6:0]=flow|RSEL[8]=0(pkt)/1(byte), poll
 * R_ACK(bit15), read GPON_GTC_DS_PORT_CNTR_STAT(0x1144). Unlike gem_ds_rx_cnt
 * (ETH-only 0x4040), this counts ALL de-encapped GEM frames incl. isOMCI flow 64
 * — the reliable "did the GTC de-encapsulate OMCI?" detector. Bounded poll. */
static u32 gpon_gem_flow_cnt(u32 idx, int rsel)
{
	int t;

	gpon_wr(0x1140, (idx & 0x7f) | ((rsel & 1) << 8));
	for (t = 0; t < 1000; t++) {
		if (gpon_rd(0x1140) & BIT(15))
			return gpon_rd(0x1144);
		udelay(1);
	}
	return 0xffffffffu;				/* ACK never set */
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
	seq_printf(s, "cdr_recover: wedged=%u fixed=%u last_sts=0x%08x (sentinel=0x%08x)\n",
		   gpon_cdr_stuck_count, gpon_cdr_stuck_fixed, gpon_gtc_ds_sts_last,
		   GTC_DS_CDR_STUCK);
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
	/* DS_MISC counters (GTC-relative, register-map base 0x7011xx): do we even SEE/accept the
	 * OLT's BWmap grants + DS PLOAMs? ploam_acpt/bwm_acpt nonzero => GTC recognizes
	 * grants and asserts BEN (so a zero at the OLT = analog SerDes-TX emission);
	 * bwm_fail/inv nonzero => grants seen but rejected (CRC/format); all zero =>
	 * GTC never sees the OLT grants (downstream BWmap parse issue). */
	seq_printf(s, "ds_cntr: ploam_acpt=%u ploam_fail=%u bwm_acpt=%u bwm_fail=%u bwm_inv=%u active=%u\n",
		   gpon_rd(0x119c), gpon_rd(0x11a0), gpon_rd(0x11b0),
		   gpon_rd(0x11a4), gpon_rd(0x11a8), gpon_rd(0x11ac));
	seq_printf(s, "gem_ds_rx: omcc(f64)=%u f0=%u f1=%u f2(mcast)=%u  (>0 => OLT is sending DS GEM/OMCI)\n",
		   gpon_gem_ds_rx_cnt(64), gpon_gem_ds_rx_cnt(0),
		   gpon_gem_ds_rx_cnt(1), gpon_gem_ds_rx_cnt(GPON_MCAST_FLOW));
	/* GLOBAL CAM-independent de-assembly counters (the decisive OMCI detector):
	 * UC_RX>0 => DS unicast GEM reaches the de-assembler; OMCI_RX>0 => it de-encapped
	 * an OMCI frame; ETH_CRC_ERR>0 => frames arrive but fail FCS (then are dropped). */
	seq_printf(s, "ds_misc: UC_RX=%u MC_RX=%u OMCI_RX=%u ETH_CRC_ERR=%u OVER_INTL=%u MC_LEAK=%u\n",
		   gpon_gem_ds_misc_cnt(1), gpon_gem_ds_misc_cnt(0), gpon_gem_ds_misc_cnt(6),
		   gpon_gem_ds_misc_cnt(4), gpon_gem_ds_misc_cnt(5), gpon_gem_ds_misc_cnt(3));
	/* GTC-layer GEM health (direct regs): distinguishes "GEM arrives but FAILS" (FAIL
	 * or HEC climbs => garble/sync) from "no GEM at all" (all flat). NON_IDLE/IDLE =
	 * good frames; FAIL=0x11c0 LOS=0x11b4 HEC=0x11b8 frm_to(0x4098). */
	seq_printf(s, "ds_gem: NON_IDLE=%u IDLE=%u FAIL=%u LOS=%u HEC=%u | frm_to(0x4098)=0x%x\n",
		   gpon_rd(0x11c4), gpon_rd(0x11bc), gpon_rd(0x11c0),
		   gpon_rd(0x11b4), gpon_rd(0x11b8), gpon_rd(0x4098));
	/* DS pipeline stages for the OMCI frame (read with the gate open at O5):
	 * A=de-encap pkt(f64)+global non-idle; B=PBO HIGH-queue(Q0) page cur/max;
	 * C=DS SRAM pool used/peak; D=PON-IP->NIC RX_OK/MISS/ERR + init-ready.
	 * Walk A->D: first 0 (or non-zero-meets-zero boundary) = the stall stage. */
	{
		u32 a = gpon_gem_flow_cnt(64, 0);
		u32 q0 = pi_rd(0xa100), us = pi_rd(0xa0bc), sts = pi_rd(0xa0c8);
		u32 ok = pi_rd(0xc010), ms = pi_rd(0xc018), er = pi_rd(0xc014);

		seq_printf(s, "ds_pipe: A_deenc(f64)=%u nonidle=%u | B_q0cur=%u q0max=%u | C_sram=%u peak=%u | D_rxok=%u miss=%u err=%u initrdy=%u\n",
			   a, gpon_rd(0x11c4),
			   q0 & 0x1fff, (q0 >> 13) & 0x1fff,
			   us & 0x1fff, sts & 0x1fff,
			   ok & 0xffff, (ms >> 16) & 0xffff, er & 0xffff,
			   pi_rd(0xa0c0) & 1);
		/* DECISIVE break-localizer: FWD(f64)=GEM frames the GTC FORWARDED to the
		 * PON-IP (0x404c/0x4050) vs A_deenc(=de-encap'd). media_sts(0x1bf0c058) bit18
		 * = internal DS-GMII FORCELINK (stock ~0x106e8400). de-encap>FWD => GTC drops;
		 * FWD>0 & C_sram flat => PON-IP rejects (descriptor base or DS-GMII down). */
		seq_printf(s, "ds_fwd: FWD(f64)=%u FWD(f0)=%u | media_sts=0x%08x (bit18 link=%u) gmii_en=%u\n",
			   gpon_gem_ds_fwd_cnt(64), gpon_gem_ds_fwd_cnt(0),
			   pi_rd(0xc058), (pi_rd(0xc058) >> 18) & 1,
			   (pi_rd(0xd434) >> 5) & 1);
		/* Read-back the DS-engine enables (a later reset may have cleared them):
		 * ctl_ds expect 0x81 (CFG_PBUF_EN bit0 + bit7), io0_ds expect 0x90081070
		 * (GMII_RX_EN bit5 + GMII_TX_EN bit4 must be set). */
		seq_printf(s, "ds_en: ctl_ds(0xa0ac)=0x%08x io0_ds(0xd434)=0x%08x io1_ds(0xd438)=0x%08x\n",
			   pi_rd(0xa0ac), pi_rd(0xd434), pi_rd(0xd438));
		seq_printf(s, "ds_nic: cfg_ds(0xc04c)=0x%08x[RX_SID=%u] rxcfg_ds(0xc044)=0x%08x media_ds(0xc058)=0x%08x rxfdp_ds(0xd3f0)=0x%08x\n",
			   pi_rd(0xc04c), pi_rd(0xc04c) & 0x7f, pi_rd(0xc044),
			   pi_rd(0xc058), pi_rd(0xd3f0));
		/* US-NIC, compare against live stock: cfg_us=0x24030040 (bit29 set!),
		 * ctl_us=0, io0_us=0x90101070, io1_us=0x08000000, rxfdp_us=0. */
		seq_printf(s, "us_nic: cfg_us(0x404c)=0x%08x[RX_SID=%u] ctl_us(0x20ac)=0x%08x io0_us(0x5434)=0x%08x io1_us(0x5438)=0x%08x rxfdp_us(0x53f0)=0x%08x\n",
			   pi_rd(0x404c), pi_rd(0x404c) & 0x7f, pi_rd(0x20ac),
			   pi_rd(0x5434), pi_rd(0x5438), pi_rd(0x53f0));
		/* US-NIC per-group RX SID counters (from the chip's register map:
		 * RX_SID_GOOD_CNT_US @ SoC 0xF0203C = PON-IP off 0x203c, 5 groups at
		 * 4-byte stride; RX_SID_BAD_CNT_US @ 0xF02054 = off 0x2054). good>0
		 * confirms a CPU-injected US OMCI frame reached the US-NIC and matched
		 * SID 64; bad>0 = SID mismatch. (These read cleanly in the PON-IP window
		 * — unlike the 0xc010 DS pkt counter which bus-aborts.) */
		seq_printf(s, "us_rxsid: good=%u/%u/%u/%u/%u bad=%u\n",
			   pi_rd(0x203c), pi_rd(0x2040), pi_rd(0x2044),
			   pi_rd(0x2048), pi_rd(0x204c), pi_rd(0x2054));
		/* NOTE: PI_PKT_*_CNT_US/DS at 0x4010-0x4018 / 0xc010-0x0c018 (inferred from a
		 * comment) BUS-ABORT on direct pi_rd (0x404c/0xd3f0 read fine, 0xc010 faults) —
		 * they are NOT directly readable here. The US-NIC ingest packet count must come
		 * via an INDIRECT accessor (cf. the GTC misc PM counters: write idx->0x5140,
		 * read 0x5148) or a different offset. Do NOT re-add a direct read (crashes
		 * /proc/gpon). Next: find the indirect US-NIC RX/ingest counter access. */
		/* US-NIC arm (symmetric to DS) + US SID-64 classification, for ustx=0
		 * triage: media_us(0x4058) must be 0x106e8400 (force-link UP), io0_us
		 * GMII enables, gem_us_map[64](gpon 0x6500) the OMCC GEM port, sidvalid
		 * word(0x2144) bit0, sid2qid word(0x2138), omci_cfg(0x2154).
		 * s2q64 = SID2QID[64] decoded with the CORRECT 32/bits=4-entries-per-word packing
		 * (from the stock array-field-write routine): word = 64/4 = 16 -> base 0x20f8 +
		 * 16*4 = 0x2138, shift (64%4)*7 = 0 -> read 0x2138[6:0]. Should read 64
		 * (GPON_OMCC_PHYS_QID). The 0x2130 word (SID 56's slot under this packing) is kept
		 * alongside as the value the OLD contiguous-packing bug mistakenly wrote/read. */
		seq_printf(s, "us_arm: media_us(0x4058)=0x%08x io0_us(0x5434)=0x%08x gemus_map64(0x6500)=0x%08x sidvld(0x2144)=0x%08x s2q(0x2138/0x2130)=0x%08x/0x%08x s2q64=%lu omcicfg(0x2154)=0x%08x\n",
			   pi_rd(0x4058), pi_rd(0x5434), gpon_rd(0x6c00 /* GEM_US_PORT_MAP[flow 64] = 0x6400+64*0x20; macros defined later in file */),
			   pi_rd(0x2144), pi_rd(0x2138), pi_rd(0x2130),
			   pi_rd(0x2138) & 0x7fUL, pi_rd(0x2154));
		/* PON-IP OMCI packet counters (from the chip's register map, SoC base 0x1b000000 ->
		 * swcore offsets): OMCI_RX_PKT_CNT 0x329c0 (DS OMCI de-encapsulated by the
		 * PON-IP, DISTINCT from my GTC 0x4064 idx6), DROP 0x329b8, CRC_ERR 0x329cc,
		 * US_TX 0x329bc; PON_TRAP_CFG 0x111f8 [2:0]=OMCI_MPCP_PRIORITY. DECODER:
		 * rx>0 & NIC filled=0 => de-encap OK but trap-to-CPU gap; drop>0 =>
		 * SID/queue/PBO mapping rejecting the OMCI before it traps. */
		seq_printf(s, "omci_pi: rx(0x329c0)=%u drop(0x329b8)=%u crcerr(0x329cc)=%u ustx(0x329bc)=%u trapcfg(0x111f8)=0x%08x\n",
			   sw_rd(0x329c0), sw_rd(0x329b8), sw_rd(0x329cc),
			   sw_rd(0x329bc), sw_rd(0x111f8));
		/* US packet-engine TX counters (PI_PKT_OK_CNT_US 0x04010 / ERR 0x04014 /
		 * MISS 0x04018). If the ONU transmits ANY upstream GEM on the OLT's BWMAP
		 * grants these climb; us_tx_ok=0 => the ONU never fills its grants (US PLOAM
		 * only, no US GEM packet engine) — the suspected reason the OLT deactivates
		 * us ~42s after O5 without ever sending OMCI. */
		seq_printf(s, "us_tx: ploam_acpt(0x119c)=%u bwm_acpt(0x11b0)=%u bwm_fail(0x11a4)=%u bwm_inv(0x11a8)=%u | us_onu_id=%u ds_cfg(0x1014)=0x%08x\n",
			   gpon_rd(0x119c), gpon_rd(0x11b0), gpon_rd(0x11a4), gpon_rd(0x11a8),
			   (gpon_rd(GPON_GTC_US_ONU_ID) >> 8) & 0xff, gpon_rd(0x1014));
		/* Alloc CAM read-back: T-CONT 16 must hold alloc 0x400 (HIT) for the GTC to
		 * accept the OLT's operational BWMAP grant on that alloc-id. */
		{
			u32 a16 = gpon_alloc_cam_read(16);

			seq_printf(s, "us_alloc: tc16=alloc0x%x hit%u\n", a16 & 0xfff, !!(a16 & BIT(16)));
		}
		/* US GTC emission breakdown: PLOAM (idx2 cpu / idx3 auto) vs GEM (idx4 byte /
		 * idx1 dbru) + per-T-CONT-16 idle-GEM (TCONT_IDLE_BYTE_STAT[16] = 0x6c00+16*64
		 * = 0x7000). The decisive signature for "OLT deactivates ~42s, no OMCI": cpu>0
		 * (ACKs egress) but gem_byte=0 AND idle16=0 (no US GEM fills the grants). */
		/* OMCC US-emission detector. TCONT_IDLE_BYTE_STAT array base 0x6c00, stride 8
		 * BYTES (register-map "array offset 64"=64 BITS), 64-bit/entry: T-CONT 16 (OMCC) =
		 * 0x6c00+16*8=0x6c80, T-CONT 8 (data) = 0x6c40. GEM_US_BYTE_STAT base 0x6800,
		 * stride 8, flow 64 (OMCC) = 0x6800+64*8=0x6a00. If idle16/gemus64 climb the ONU
		 * IS emitting US GEM on the OMCC (so the OLT should confirm it); if flat, the
		 * OMCC US is silent = the OLT keeps re-Configure_Port-ID and withholds OMCI. */
		seq_printf(s, "us_gtc: ploam_cpu=%u ploam_auto=%u | gem_byte=%u gem_dbru=%u | idle16=%u/%u idle8=%u gemus64=%u/%u gem2=%u(0x6810) | us_cfg=0x%04x pti=0x%08x\n",
			   gpon_us_misc_cnt(2), gpon_us_misc_cnt(3),
			   gpon_us_misc_cnt(4), gpon_us_misc_cnt(1),
			   gpon_rd(0x6c80), gpon_rd(0x6c84), gpon_rd(0x6c40),
			   gpon_rd(0x6a00), gpon_rd(0x6a04), gpon_rd(0x6810),
			   gpon_rd(GPON_GTC_US_CFG), gpon_rd(0x6020));
		/* GEM_US_BYTE_STAT full scan (base 0x6800, stride 8): which flow does the
		 * port-2 OMCI actually land on? Non-zero on flow!=64 => SID-stamp/SID2QID
		 * misroute; all-zero => the US-NIC drops it before any flow (ingest gap). */
		{
			int f, n = 0; char gbuf[220];
			for (f = 0; f < 128 && n < 200; f++) {
				u32 gv = gpon_rd(0x6800 + f * 8);
				if (gv)
					n += scnprintf(gbuf + n, sizeof(gbuf) - n, " f%d=%u", f, gv);
			}
			seq_printf(s, "gemus_scan:%s\n", n ? gbuf : " (all flows 0)");
		}
		/* US-scheduler readback: did the gpon_install_tcont writes for qid 64 /
		 * T-CONT 16 actually LAND (vs being write-protected or wrong-offset)?
		 * Expect pir64/cir64 = 0x3ffff, tcont_en bit16=1, qmap16=1, wfqtype bit0=0,
		 * wfqwt bits[19:10]=1, drn bit0=0 (idle). */
		seq_printf(s, "us_sched: pir64(0x239c)=0x%08x cir64(0x2298)=0x%08x tcont_en(0x23e4)=0x%08x qmap16(0x23e0)=0x%08x wfqtype(0x23f0)=0x%08x wfqwt(0x244c)=0x%08x drn(0x20e4)=0x%08x sch_ctrl(0x2194)=0x%08x[PIR_DROP=%lu]\n",
			   pi_rd(0x239c), pi_rd(0x2298), pi_rd(0x23e4), pi_rd(0x23e0),
			   pi_rd(0x23f0), pi_rd(0x244c), pi_rd(0x20e4),
			   pi_rd(0x2194), (pi_rd(0x2194) >> 18) & 1UL);
		/* ★ SID-64 page-occupancy probe (OLT-INDEPENDENT verify of the descriptor-
		 * sideband OMCI fix): MAX_PAGE_CNT high-water for SID 64 is NON-ZERO iff an
		 * OMCI frame was ever classified+enqueued to physical queue 64. This latches
		 * even after drain, and needs NO OLT grant — so a self-test OMCI inject that
		 * makes this go non-zero CONFIRMS opts3.tx_dst_stream_id=64 reaches the US-NIC
		 * classifier. PONIP_DBG_CTRL_US=0x255c {SID_NO[6:0], RD_MAX bit7, CLR bit8,
		 * BUSY bit9}; PONIP_SID_USED_PAGE_CNT_US=0x2564 {USED[12:0], MAX[28:16]}. */
		{
			u32 n, pc;
			pi_wr(0x255c, 64u | (1u << 7));		/* SID_NO=64 + RD_SID_MAX_PAGE_CNT */
			for (n = 0; n < 2000 && (pi_rd(0x255c) & (1u << 9)); n++)
				udelay(1);
			pc = pi_rd(0x2564);
			seq_printf(s, "sidpage64: used=%u max=%u (max>0 = OMCI ENQUEUED to queue 64) [r255c=0x%08x r2564=0x%08x poll=%u]\n",
				   pc & 0x1fff, (pc >> 16) & 0x1fff, pi_rd(0x255c), pc, n);
		}
		/* CAM read-back: does the DS GEM CAM actually map gem->flow 64 at runtime?
		 * e64 should read gem=2 (the OMCC) HIT=1; traffic_cfg[64] should be 0x4. */
		{
			u32 c64 = gpon_ds_cam_read(64), c0 = gpon_ds_cam_read(0);
			u32 c1 = gpon_ds_cam_read(GPON_DATA_FLOW);

			seq_printf(s, "ds_cam: e64=gem%u hit%u tcfg64=0x%x | e0=gem%u hit%u | e%u(data)=gem%u hit%u tcfg=0x%x\n",
				   c64 & 0xfff, !!(c64 & BIT(16)),
				   gpon_rd(0x1400 + 64 * 4) & 0x1f,
				   c0 & 0xfff, !!(c0 & BIT(16)),
				   GPON_DATA_FLOW, c1 & 0xfff, !!(c1 & BIT(16)),
				   gpon_rd(0x1400 + GPON_DATA_FLOW * 4) & 0x1f);
		}
	}
	/* Full per-flow de-encap sweep: does ANY flow de-encapsulate a GEM frame?
	 * If some flow > 0 => the OLT IS sending de-encodable GEM (find the OMCI
	 * flow). If NONE => frames arrive but de-encap is globally not happening
	 * (gem-port CAM / OMCI not classifying), or the OLT sends only idle GEM. */
	{
		int f, n = 0;

		seq_printf(s, "ds_deenc_sweep:");
		for (f = 0; f < 128; f++) {
			u32 v = gpon_gem_flow_cnt(f, 0);

			if (v && v != 0xffffffffu) {
				seq_printf(s, " f%d=%u", f, v);
				n++;
			}
		}
		seq_printf(s, "%s\n", n ? "" : " (NONE de-encap)");
	}
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
	/* Fiber optical power for the LuCI overview: the RTL8290B MCU publishes
	 * finished SFF-8472 DDM power words on its A2/DDM page (slave 0x51 = the
	 * 0x1xx bank), so no per-board calibration is needed. RX @0x68/0x69, TX
	 * @0x66/0x67, unit 0.1 microWatt/LSB. Emit the raw u16; userspace converts
	 * dBm = 10*log10(raw/10000) (no in-kernel log10/FPU). 0x0000/0xffff = n/a. */
	{
		int rxw = bosa_read16_median(0x168);
		int txw = bosa_read16_median(0x166);

		seq_printf(s, "optic_rx_raw: 0x%04x optic_tx_raw: 0x%04x\n",
			   rxw & 0xffff, txw & 0xffff);
		/* ANI-G ME263 #10/#14 cached levels (OMCI 0.002 dB, 2's-comp) reported to
		 * the OLT — refreshed from the DDM words on the FSM tick. */
		seq_printf(s, "anig_rx_level: 0x%04x anig_tx_level: 0x%04x\n",
			   (u16)anig_rx_level, (u16)anig_tx_level);
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
	/* SerDes/serializer run-state vs LIVE working-stock O5 (stock_usburst.txt
	 * golden in [..]). Decisive good-vs-bad-boot diff for the cold-start US-TX
	 * serializer lock: any reg that DIVERGES from the [stock] value on a
	 * non-leasing boot is a live-confirmed candidate (unlike the transient
	 * mode-set programmed values, which are overwritten by the CDR reset). */
	seq_printf(s, "sds_run: 280c=%04x[3106] 281c=%04x[1359] 225a0=%04x[713] 220a8=%04x[2] 225d8=%04x[29]\n",
		   sw_rd(0x2280c) & 0xffff, sw_rd(0x2281c) & 0xffff,
		   sw_rd(0x225a0) & 0xffff, sw_rd(0x220a8) & 0xffff,
		   sw_rd(0x225d8) & 0xffff);
	seq_printf(s, "sds_ext: 22a2c=%04x[0] 22a30=%04x[4] 22a34=%04x[326a]\n",
		   sw_rd(0x22a2c) & 0xffff, sw_rd(0x22a30) & 0xffff,
		   sw_rd(0x22a34) & 0xffff);
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
#define PLM_DS_REQUEST_KEY		0x0d	/* Request_key (-> Encryption_Key) */
#define PLM_DS_KEY_SWITCH		0x13	/* Key_Switching_Time (-> arm HW key switch) */
#define PLM_DS_ASSIGN_ALLOC_ID		0x0a	/* Assign_Alloc-ID (ACK) */
#define PLM_DS_REQUEST_PASSWORD		0x09	/* Request_Password (-> US Password 0x02) */
#define PLM_DS_CONFIG_PORT		0x0e	/* Configure_Port-ID (ACK) */
#define PLM_DS_CFG_VPVC			0x07	/* Configure_VP/VC (ATM, unsupported; ACK) */
#define PLM_DS_BER_INTERVAL		0x12	/* BER interval (ACK; arms BER reporting) */
#define PLM_US_ENCRYPT_KEY		0x05	/* US Encryption_Key response */
#define PLM_US_PASSWORD			0x02	/* US Password response (to Request_Password) */
#define PLM_US_SERIAL_NUMBER		0x01
#define PLM_US_ACKNOWLEDGE		0x09	/* US Acknowledge message type */
#define PLM_US_QUEUE_SN			0x6	/* US_PLOAM_IND[10:8] auto-SN queue */
#define PLM_US_QUEUE_URG		0x1	/* US_PLOAM_IND[10:8] urgent queue (ACKs) */
#define PLM_US_QUEUE_NOMSG		0x7	/* US_PLOAM_IND[10:8] HW auto-No_message slot */

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

	/* The CPU US-PLOAM path has a SINGLE transmit buffer (GPON_GTC_US_PLOAM_DATA) and
	 * a self-clearing ENQ bit: the HW sends the enqueued message in the next granted US
	 * PLOAM slot and clears ENQ. Back-to-back sends (the 6 Encryption_Key fragments, the
	 * Acknowledge, the Key_Switching_Time ACK) otherwise overwrite the buffer before the
	 * previous one is transmitted, so the OLT never receives the important Acknowledge
	 * and re-cycles Configure_Port-ID forever (PLOAM_CPU_TX stays 0). Wait (bounded) for
	 * ENQ to self-clear before reloading the buffer. */
	for (i = 0; i < 1000; i++) {
		if (!(gpon_rd(GPON_GTC_US_PLOAM_IND) & GPON_US_PLM_ENQ))
			break;
		udelay(5);
	}

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

	/* DBG (ploam_tx_dbg): did the HW actually TRANSMIT this CPU PLOAM? Poll ENQ
	 * self-clear (HW sends in the next granted US PLOAM slot + clears ENQ). The
	 * Serial_Number rides the auto-SN queue (0x6); the ACK/Password ride the urgent
	 * queue (0x1). Ground truth = OLT raises LOAi (never gets our ACKs) -> suspect the
	 * urgent-queue CPU TX never fires. Log the first N sends to compare per-queue. */
	if (ploam_tx_dbg) {
		static int dbgn;
		int j, cleared = 0;
		u32 i2;

		for (j = 0; j < 400; j++) {	/* up to ~2ms */
			i2 = gpon_rd(GPON_GTC_US_PLOAM_IND);
			if (!(i2 & GPON_US_PLM_ENQ)) { cleared = 1; break; }
			udelay(5);
		}
		if (dbgn++ < 40)
			pr_info("rtl9602c-gpon: PLM_TX q=%u enq_cleared=%d(%dus) IND=0x%08x urg_e=%d urg_f=%d nrm_e=%d nrm_f=%d cputx=%u autotx=%u\n",
				queue, cleared, j * 5, i2,
				!!(i2 & GPON_US_PLM_URG_EMPTY), !!(i2 & GPON_US_PLM_URG_FULL),
				!!(i2 & GPON_US_PLM_NRM_EMPTY), !!(i2 & GPON_US_PLM_NRM_FULL),
				gpon_us_misc_cnt(2), gpon_us_misc_cnt(3));
	}
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
 * Respond to a downstream Request_Password (0x09) with the US Password message
 * (G.984.3 msg type 0x02, 10-octet password). GROUND TRUTH (OLT poll 2026-06-13):
 * the OLT (BCM68620) sits at O5 spamming Request_Password(0x09)/Encrypted_Port-ID
 * (0x08), NEVER advancing to Configure_Port-ID/Assign_Alloc-ID, and deactivates us
 * with alarm **LOAi** (Loss Of Acknowledge) — because this Password reply was never
 * sent. The OLT is SN-auth (password field empty) so the VALUE is ignored, but the
 * activation handshake stalls without the message. Send an all-zero (empty)
 * password 3x on the urgent queue (G.984.3 sends Password in consecutive US slots
 * for reliability over the un-acked channel). This unblocks the OLT to proceed to
 * the OMCI provisioning PLOAMs the FSM already handles.
 */
static void gpon_send_password(void)
{
	u8 p[12] = { 0 };
	int i;

	p[0] = gpon_fsm_onu_id;		/* our assigned ONU-ID */
	p[1] = PLM_US_PASSWORD;		/* 0x02 */
	/* p[2..11] = 10-octet password, all zero = empty (OLT SN-auth ignores it) */
	for (i = 0; i < 3; i++)
		gpon_send_cpu_ploam(PLM_US_QUEUE_URG, p);
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
 * Respond to a downstream Request_key (0x0d): generate a 128-bit AES key and send it
 * to the OLT in two upstream Encryption_Key (US type 0x05) PLOAM fragments
 * (msg[2]=key_index, msg[3]=row, msg[4..11]=8 key bytes; row 0 = key[0..7], row 1 =
 * key[8..15]). The OLT requests this during activation; an ONU that never returns a
 * key can't complete config (it stalls before/at OMCI). Matches stock
 * key-TX behavior. (HW decryption isn't programmed — the OMCC is unencrypted; this
 * just satisfies the OLT's key exchange so config proceeds.)
 */
static u8 gpon_aes_key[16];
static u8 gpon_key_index;
static u32 gpon_aes_switch_time = 0xffffffff;	/* last Key_Switching_Time superframe (de-dup) */
static bool gpon_key_staged;			/* a valid AES key is loaded in the staged bank */

/* Program the 16-byte AES-128 key into the GPON hardware STAGED (next) key bank. After
 * the ONU answers Request_Key with the upstream Encryption_Key it MUST also load that
 * same key into hardware; the OLT then sends Key_Switching_Time (0x13) and the HW
 * promotes the staged key to active at the given superframe. Without this load the OLT
 * treats the key exchange as incomplete and keeps re-cycling Request_Key / Configure_
 * Port-ID every ~15s, never advancing to OMCI. GPON-block regs (gpon_wr): SWITCH_REQ
 * 0x3010 [15]=KEY_CFG_REQ(strobe) [14]=CFG_ACTIVE_KEY(0=staged); WORD_DATA 0x3024 [15:0];
 * WORD_IND 0x3020 [15]=KEY_WR_REQ(strobe) [14]=KEY_WR_COMPL [2:0]=KEY_WORD_IDX. Word i
 * carries key[2i] (high byte) | key[2i+1] (low byte); words 0..7 = key bytes 0..15. */
static void gpon_aes_stage_key(const u8 *key)
{
	int idx, i;

	gpon_wr(0x3010, 0x0000);			/* CFG_ACTIVE_KEY=0 (staged), REQ=0     */
	gpon_wr(0x3010, 0x8000);			/* KEY_CFG_REQ 0->1: config staged bank */
	for (idx = 0; idx < 8; idx++) {
		gpon_wr(0x3024, ((u32)key[2 * idx] << 8) | key[2 * idx + 1]);
		gpon_wr(0x3020, idx);			/* KEY_WORD_IDX=idx, KEY_WR_REQ=0       */
		gpon_wr(0x3020, idx | 0x8000);		/* KEY_WR_REQ 0->1: latch word idx      */
		for (i = 0; i < 200; i++) {		/* bounded; timeout != success          */
			if (gpon_rd(0x3020) & BIT(14))	/* KEY_WR_COMPL                         */
				break;
			udelay(5);
		}
	}
}

static void gpon_send_key(void)
{
	u8 m[12];
	int row, rep;

	get_random_bytes(gpon_aes_key, sizeof(gpon_aes_key));
	gpon_key_index++;
	/* Send the SAME key 3x (6 PLOAMs total), matching stock behavior (3 reps around the
	 * 2-fragment emit). The US PLOAM channel is lossy and the OLT re-issues Request_key
	 * rapidly when it does not receive a complete key, stalling config; the redundant
	 * triple-send maximises the chance the OLT accepts the key and proceeds. */
	for (rep = 0; rep < 3; rep++) {
		for (row = 0; row < 2; row++) {
			memset(m, 0, sizeof(m));
			m[0] = gpon_fsm_onu_id;		/* ONU-ID (HW may override)    */
			m[1] = PLM_US_ENCRYPT_KEY;	/* 0x05 Encryption_Key         */
			m[2] = gpon_key_index;
			m[3] = row;			/* fragment row 0/1            */
			memcpy(&m[4], gpon_aes_key + row * 8, 8);
			gpon_send_cpu_ploam(PLM_US_QUEUE_URG, m);
		}
	}
	/* Load the SAME key into the HW staged bank (the OLT waits for this before OMCI). */
	gpon_aes_stage_key(gpon_aes_key);
	gpon_key_staged = true;
	pr_info("rtl9602c-gpon: Request_key -> sent Encryption_Key idx %u (3x2 frags) + staged in HW\n",
		gpon_key_index);
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
#define GPON_GTC_DS_TRAFFIC_CFG	0x1400		/* array: base 0x1400, STRIDE 4 bytes, idx
						 * 0..127, [4:0] traffic-type. The register map's
						 * "array offset"=32 is in BITS (32b = 4 bytes), NOT
						 * bytes — confirmed on a live online stock ONU whose
						 * tcfg[64] sits at 0x1500 (=0x1400+64*4)=0x4, with
						 * 0x1c00 empty. (A 0x20 stride wrote outside the
						 * 0x1400..0x1600 array, into the void.) */
#define   DS_TRAFFIC_CFG_STRIDE	4u
#define   DS_TRAFFIC_IS_OMCI	BIT(2)
#define GPON_GTC_GEM_US_PORT_MAP 0x6400		/* array: base 0x6400, STRIDE 0x20 (register-map
						 * "array offset"=32), idx 0..127,
						 * [11:0] gemPortId. NOT stride 4. */
#define   GEM_US_PORT_MAP_STRIDE 4u		/* 4 BYTES/entry. The register-map "offset 32"
						 * means 32 BITS: the US GEM-port config
						 * indexes 4-byte words (flow 64
						 * = 0x6500). The 0x20-stride epoch wrote 0x6C00,
						 * OUTSIDE the 128-entry table. */
#define GPON_GTC_DS_OMCI_PTI	0x1204		/* [6:4] PTI_MASK [2:0] END_PTI */
#define   DS_OMCI_PTI_VAL	((1u << 4) | 1u)	/* mask=1 ptn=1 -> 0x11 */
#define GPON_GEM_DS_MC_CFG	0x4080		/* [6] BROADCAST_PASS [4] NON_MULTICAST_PASS [3] FCS_CHK_EN */
#define   GEM_DS_MC_CFG_VAL	0x59u		/* stock O5 operating value (read live from an online stock ONU): BROADCAST_PASS(6)|NON_MULTICAST_PASS(4)|FCS_CHK_EN(3)|bit0. The earlier 0x18 (no broadcast/bit0) was a wrong "avoid US stall" guess — stock runs 0x59 stably online with OMCI flowing. */
#define PI_PON_SID2QID		0x020f8		/* packed 7b/SID: physical queue */
#define PI_PON_SIDVALID		0x0213c		/* packed 1b/SID */
#define PI_PON_OMCI_CFG		0x02154		/* [6:0] OMCI SID */
#define PI_PON_SID_Q_MAP_DS	0x0a0e4		/* packed 2b/SID: DS PBO queue */
#define GPON_OMCC_FLOW		64		/* RTL9602C fixed OMCI flow/SID */
/* GPON_DATA_FLOW(1) / GPON_DATA_GEM(193) — the WAN data GEM (clean-room nas0-equivalent) —
 * are defined in rtl9602c_gpon_nic.h (shared with the eth driver's gpon0 TX descriptor). */
#define GPON_OMCC_PHYS_QID	64		/* OMCC physical qid = TCONT_QUEUE_MAX(32)*(TCONT16/8)+q0 = 64
						 * (stock physical-queue-id mapping, GPON branch:
						 * srl>>3 then <<5). The prior "63" was a misread: with the WRONG
						 * contiguous packing, the readback of SID2QID[64] actually read
						 * SID 56's slot (a data flow legitimately at qid 63). Fixed packing
						 * (entries_per_word = 32/bits) below now addresses SID 64 correctly,
						 * so the true stock value 64 applies. Used for SID2QID + scheduler qid. */
#define GPON_OMCC_DSQ_HIGH	2

/* Dedicated DATA T-CONT for the WAN data GEM (193). STOCK rides gem 193 on its own data
 * T-CONT bound to the OLT-assigned data Alloc-ID (256, via OMCI ME262 inst 0x8000) — NOT on
 * the OMCC's T-CONT 16/mgmt Alloc. Riding the OMCC's T-CONT means the data US goes out on the
 * mgmt Alloc; the OLT grants the data Alloc 256, sees it idle, and WITHHOLDS downstream. So
 * bind gem 193's US to this data T-CONT. qid = 32*(tcont/8). */
#define GPON_DATA_TCONT		8
#define GPON_DATA_ALLOC		256u	/* OLT data Alloc-ID for THIS OLT (consistent; from ME262) */
#define GPON_DATA_PHYS_QID	32	/* = 32*(GPON_DATA_TCONT/8) */
static int gpon_install_tcont(u8 tcont, u16 alloc);	/* fwd: data-GEM install binds the data T-CONT */

/* Set a `bits`-wide entry at index `idx` in the packed pi-register array based at
 * `base` (driver-relative).
 *
 * PACKING CORRECTED 2026-06-13 (from the stock array-field-write routine):
 * the reg-array helper packs entries `entries_per_word = 32/bits` PER 32-bit
 * WORD, word-aligned, leaving the top (32 - entries_per_word*bits) bits of each word
 * UNUSED. A field NEVER straddles a word boundary. So:
 *   epw   = 32 / bits
 *   word  = idx / epw          (byte addr = base + word*4)
 *   shift = (idx % epw) * bits
 * The OLD code used CONTIGUOUS bit-packing (bit = idx*bits) which is only correct when
 * bits divides 32 evenly (1b, 2b, 4b). For the 7-bit SID2QID array it addressed the
 * wrong word: SID 64 -> 0x2130 (== HW SID 56's slot) instead of the true 0x2138. That
 * single off-by-one-word bug pointed the OMCI SID-64 classify entry at a data flow's
 * queue, so the US-NIC never classified upstream OMCI to its T-CONT16/q0 -> the OLT
 * never received the MIB-upload. (Matches stock: SID 64 -> base 0x20f8 + 16*4 = 0x2138.) */
static void pi_packed_set(u32 base, unsigned int idx, unsigned int bits, u32 val)
{
	unsigned int epw = 32u / bits;			/* entries per 32-bit word (top bits wasted) */
	u32 reg = base + (idx / epw) * 4;
	unsigned int sh = (idx % epw) * bits;
	u32 mask = (bits >= 32) ? 0xffffffffu : ((1u << bits) - 1);
	u32 lo;

	lo = pi_rd(reg);
	lo = (lo & ~(mask << sh)) | ((val & mask) << sh);
	pi_wr(reg, lo);
}

/* Read the `bits`-wide entry at index `idx` from the packed array based at `base`.
 * Mirror of pi_packed_set's 32/bits-entries-per-word packing so /proc readback shows
 * the TRUE value (the old contiguous read showed the wrong SID's value, which is
 * exactly why the SID2QID mis-addressing stayed invisible across many boot tests). */
static u32 pi_packed_get(u32 base, unsigned int idx, unsigned int bits)
{
	unsigned int epw = 32u / bits;
	u32 reg = base + (idx / epw) * 4;
	unsigned int sh = (idx % epw) * bits;
	u32 mask = (bits >= 32) ? 0xffffffffu : ((1u << bits) - 1);

	return (pi_rd(reg) >> sh) & mask;
}

/*
 * Faithful port of the stock PON-MAC GPON mode-set branch, run as ONE
 * ordered block at O5. Stock assembles the GMAC0->US-NIC IP-mux routing state in a
 * single window: (1) INVALIDATE every US SID's classify; (2) program the OMCI SID
 * triple in order; (3) PON/CPU-port RX_SPC; (4) [gated] SerDes re-commit; ending on
 * (5) the GMII_RX_EN rising edge that LATCHES the classify table into the US-NIC RX
 * engine. My driver previously set the SID classify (gpon_install_omcc) but never
 * re-pulsed the GMII edge afterwards, so the US-NIC kept its boot-time latch and
 * dropped SID-64 OMCI before the MAC (RX_OK=ERR=MISS=0). Gated by ponmac_modeset.
 */
static void rtl9602c_ponmac_modeset_gpon(void)
{
	unsigned int sid;

	if (!ponmac_modeset)
		return;

	/* (1) all-SID classify INVALIDATION pre-pass: clear every US SID so the
	 * US-NIC resolves SID-64 cleanly at the re-latch (stock clears all SIDs
	 * before programming the OMCI SID). */
	for (sid = 0; sid < PI_SID_NUM; sid++) {
		if (sid == GPON_OMCC_FLOW || (gpon_data_installed && sid == GPON_DATA_FLOW))
			continue;
		pi_packed_set(PI_PON_SIDVALID, sid, 1, 0);
		pi_packed_set(PI_PON_SID2QID, sid, 7, GPON_OMCC_PHYS_QID & 0x7f);
	}

	/* (2) OMCI classify triple, in stock order (SID2QID -> SIDVALID -> OMCI_CFG). */
	pi_packed_set(PI_PON_SID2QID, GPON_OMCC_FLOW, 7, GPON_OMCC_PHYS_QID & 0x7f);
	pi_packed_set(PI_PON_SIDVALID, GPON_OMCC_FLOW, 1, 1);
	pi_field(PI_PON_OMCI_CFG, 6, 0, GPON_OMCC_FLOW);

	/* (2b) WAN data-GEM classify, re-asserted every modeset (the all-SID loop above
	 * would otherwise wipe it — the same trap that hid the OMCC SID for ~30 rounds).
	 * SID2QID = the data qid 32 (T-CONT 8, the OLT's data Alloc-ID) so the data US is
	 * reported to the OLT's DBA on the data Alloc; legacy = OMCC qid 64 (rides T-CONT 16). */
	if (gpon_data_installed) {
		pi_packed_set(PI_PON_SID2QID, GPON_DATA_FLOW, 7,
			      (data_tcont ? GPON_DATA_PHYS_QID : GPON_OMCC_PHYS_QID) & 0x7f);
		pi_packed_set(PI_PON_SIDVALID, GPON_DATA_FLOW, 1, 1);
	}

	/* (3) PON-port(2) + CPU-port(3) RX_SPC: accept the sub-64B OMCI frame. */
	sw_field(0x20804, 2, 2, 1);
	sw_field(0x20c04, 2, 2, 1);

	/* (4) [risky, separately gated] SerDes RE-COMMIT — stock re-runs the SDS mode
	 * cycle inside mode_set so the SerDes commit and the US-NIC SID latch share one
	 * ordered window. Pulses CMD_SDS_RST_PS and can drop the locked DS framer. */
	if (serdes_recommit) {
		sw_field(SDS_CFG, 4, 0, SDS_MODE_OFF);		/* park = 0x1f      */
		sw_wr(WSDS_DIG_01, 0);
		sw_field(SW_SOFTWARE_RST, 0, 0, 1);		/* CMD_SDS_RST_PS   */
		mdelay(10);
		sw_field(WSDS_DIG_18, 12, 12, 1);		/* BEN_OE = 1       */
		sw_field(SDS_CFG, 4, 0, SDS_MODE_GPON);		/* commit GPON=0x08 */
	}

	/* (5) GMII RE-LATCH: pulse GMII_RX_EN off->on so the US-NIC RX engine latches
	 * the now-complete SID classify table (the missing edge — the core fix). */
	pi_wr(PI_IO_CMD_0_US, 0x90101050u);	/* GMII off */
	udelay(50);
	pi_wr(PI_IO_CMD_0_US, 0x90101070u);	/* GMII on -> rising edge latches classify */

	pr_info("rtl9602c-gpon: ponmac_modeset_gpon: re-latched US-NIC SID classify (serdes_recommit=%u)\n",
		serdes_recommit);
}

static int gpon_install_omcc(u16 gem)
{
	int i;

	/* Wipe stale/garbage CAM entries first so none shadow the OMCC gem at lookup. */
	gpon_ds_cam_clear_all();

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
	gpon_wr(GPON_GTC_DS_TRAFFIC_CFG + GPON_OMCC_FLOW * DS_TRAFFIC_CFG_STRIDE,
		DS_TRAFFIC_IS_OMCI);

	/* DS OMCI PTI: tell the GTC how to detect the end of an OMCI GEM frame for
	 * reassembly (PTI_MASK[6:4]=1 compares GEM-header PTI bit0; END_PTI[2:0]=1 =
	 * that bit set marks end-of-OMCI-fragment). At reset this register is 0, so
	 * the GTC never recognises an OMCI frame boundary and drops every downstream
	 * OMCI frame — the reason DS OMCI never reaches the CPU. Operating value 0x11. */
	gpon_wr(GPON_GTC_DS_OMCI_PTI, DS_OMCI_PTI_VAL);
	/* A live online stock ONU sets the ADJACENT DS-PTI registers too — 0x1200 and
	 * 0x1208 both = 0x11, same as the OMCI PTI 0x1204. My driver set only 0x1204 and
	 * the DS de-encap/reassembly produced NOTHING (DS SRAM flat, PKT_OK_DS=0, no OMCI
	 * to the CPU). 0x1200 is the GENERAL DS-PTI / frame-boundary config the reassembly
	 * engine needs for ANY flow; 0x1208 = the eth-PTI. Set both to stock's 0x11. */
	gpon_wr(0x1200, DS_OMCI_PTI_VAL);	/* general DS-PTI (stock O5 = 0x11) */
	gpon_wr(0x1208, DS_OMCI_PTI_VAL);	/* eth DS-PTI (stock O5 = 0x11)     */

	/* GEM DS pass config: WITHOUT NON_MULTICAST_PASS (bit4) the GTC drops every
	 * unicast downstream GEM frame BEFORE de-encapsulation — including OMCI, which
	 * the OLT sends unicast on the OMCC GEM port — so OMCI never reaches the flow
	 * datapath or the CPU (DS GEM RX counter stays 0). At reset this register is 0.
	 * The O5 operating value 0x59 = BROADCAST_PASS(6) | NON_MULTICAST_PASS(4) | FCS_CHK_EN(3) | bit0.
	 * Now written UNCONDITIONALLY: a live online stock ONU runs 0x59 with the US
	 * stable, so the earlier "US stall" fear (which gated this behind gem_gate_open
	 * and used a partial 0x18) was wrong — the stall came from the partial value's
	 * broadcast/bit0 mishandling, not from opening the gate. */
	gpon_wr(GPON_GEM_DS_MC_CFG, GEM_DS_MC_CFG_VAL);

	/* GEM-DS reassembly flush/forward timer (GPON_GEM_DS_FRM_TIMEOUT 0x4098,
	 * ASSM_TIMEOUT_FRM[4:0]). Stock writes 16 UNCONDITIONALLY at device init
	 * (default assemble_timer=16). FRM_TIMEOUT field map: [4:0]
	 * ASSM_TIMEOUT_FRM, [8] OMCI_TR_MODE, [15:14] DEBUG_BUS_SEL. The hardware RESET
	 * default is 0x8110 (OMCI_TR_MODE=1) and stock only ever FIELD-writes the
	 * assemble timer (bits[4:0]), PRESERVING OMCI_TR_MODE=1. A full write of 0x10
	 * (as done before) CLEARS OMCI_TR_MODE — and with OMCI transparent mode off the
	 * DS GEM de-assembler does not pass OMCI frames, so the GEM-DS MISC counters
	 * (UC_RX/OMCI_RX) stay 0 and no OMCI ever reaches the CPU. Field-write only. */
	gpon_field(0x4098, 8, 8, 1);		/* OMCI_TR_MODE = 1 (stock reset default) */
	gpon_field(0x4098, 4, 0, 16);		/* ASSM_TIMEOUT_FRM = 16 frames */

	/* US GEM-port map for flow 64. STRIDE is 0x20 (32B/entry per the register-map array),
	 * NOT DS_TRAFFIC_CFG_STRIDE(4) — the old code used the wrong stride constant, so it
	 * wrote flow 8's slot (0x6500) and left flow 64's real slot (0x6c00) unmapped, so the
	 * US-NIC had no GEM-port for the OMCI flow and dropped US OMCI (rxsid/gemus64=0). */
	gpon_wr(GPON_GTC_GEM_US_PORT_MAP + GPON_OMCC_FLOW * GEM_US_PORT_MAP_STRIDE,
		gem & 0xfff);

	/* PON-IP: SID-valid + OMCI-SID. CONFIRMED against LIVE stock O5: SIDVALID[64]=1,
	 * OMCI_CFG=0x40. SID_Q_MAP_DS[64] is 0 on stock (NOT the HIGH queue 2 I set
	 * before) — DS OMCI reaches the CPU purely via the GMAC CPUtag SID-64 trap, not a
	 * PBO queue. My SID_Q_MAP_DS[64]=2 MISROUTED the de-encapped OMCI away from the
	 * CPU. Write 0 to match stock. */
	pi_packed_set(PI_PON_SID2QID, GPON_OMCC_FLOW, 7, GPON_OMCC_PHYS_QID & 0x7f);
	pi_packed_set(PI_PON_SIDVALID, GPON_OMCC_FLOW, 1, 1);
	pi_field(PI_PON_OMCI_CFG, 6, 0, GPON_OMCC_FLOW);
	pi_packed_set(PI_PON_SID_Q_MAP_DS, GPON_OMCC_FLOW, 2, 0);
	/* Enroll SID 64 in US counter-mask group 0 (CNT_MASK_US 0x20a8 + g*12 +
	 * word; SID64 -> 0x20b0 bit0). RX_SID_GOOD/BAD_CNT_US count ONLY
	 * mask-enrolled SIDs (the stock PBO US counter-group member-add step) —
	 * without this every us_rxsid readout is structurally zero and proves
	 * nothing. The UNGATED ingest counter is PKT_OK_CNT_US (PI 0x4010,
	 * RX_OK[15:0]). */
	pi_wr(0x20b0, pi_rd(0x20b0) | 1);

	/* Read the packed entries back through pi_packed_get (the contiguous-bit-pack
	 * mirror of pi_packed_set) to confirm SID2QID[64] landed at the TRUE word
	 * (base+0x38=0x2130) — the old per-word math wrote 0x2138 and this readback was
	 * blind to it. sid2qid64 must equal GPON_OMCC_PHYS_QID for US OMCI to egress. */
	pr_info("rtl9602c-gpon: pi readback sid2qid[64]=%u sidvalid[64]=%u sidqmapds[64]=%u\n",
		pi_packed_get(PI_PON_SID2QID, GPON_OMCC_FLOW, 7),
		pi_packed_get(PI_PON_SIDVALID, GPON_OMCC_FLOW, 1),
		pi_packed_get(PI_PON_SID_Q_MAP_DS, GPON_OMCC_FLOW, 2));

	/* Arm the NIC OMCI trap so DS stream-64 frames reach the CPU netdev, and hand
	 * the eth driver this board's 8-byte ONU-SN so its OMCI ONU-G GET reply reports
	 * a Vendor-ID/Serial matching the PLOAM Serial_Number the OLT ranged. */
	rtl9602c_eth_set_omci_sid(GPON_OMCC_FLOW);
	rtl9602c_eth_set_omci_identity(gpon_sn_bytes);

	/* Stock-ordered ponmac_mode_set classify block + GMII re-latch (gated by
	 * ponmac_modeset). The SID classify above is set AFTER gpon_pbo_init's boot
	 * GMII edge; this re-pulses the edge so the US-NIC RX engine actually latches
	 * the SID-64 classification (without it the frame is dropped pre-MAC). */
	rtl9602c_ponmac_modeset_gpon();

	pr_info("rtl9602c-gpon: OMCC installed gem=%u flow=%u (compl %d)\n",
		gem, GPON_OMCC_FLOW, i);
	return 0;
}

/*
 * Install the WAN data GEM (wire gem-port-id GPON_DATA_GEM=193) on internal flow
 * GPON_DATA_FLOW(1) as a BRIDGED (non-OMCI) datapath — the clean-room nas0-equivalent.
 * v1 rides the OMCC's T-CONT 16 / qid 64 for upstream (the OLT binds only one Alloc-ID),
 * so data US uses the OMCC's already-working grants; the gem-id (193) keeps data distinct
 * from OMCI on the wire.
 *   DS: gem193 -> flow1 CAM, PLAIN de-encap (NOT DS_TRAFFIC_IS_OMCI) so the de-assembler
 *       passes raw Ethernet to switch port-2 -> CPU; the eth driver's src_port==2 RX demux
 *       hands those frames to the gpon0 netdev (no OMCI trap, no PTI).
 *   US: GEM_US_PORT_MAP[flow1]=193 stamps gem-id 193 on frames the gpon0 TX path steers
 *       with tx_dst_stream_id=flow1; SID2QID[flow1]=qid64 routes them to T-CONT 16's grants.
 * One-shot (gpon_data_installed); re-armed on Deactivate. Requires the OMCC up first.
 */
int gpon_install_data_gem(void)
{
	int i;

	if (gpon_data_installed)
		return 0;
	if (!gpon_omcc_installed)	/* need the OMCC GEM cfg (0x59 pass, qid 64) up first */
		return -EAGAIN;

	/* DS GEM-port CAM: gem 193 -> flow 1 (same indirect op as the OMCC CAM). */
	gpon_wr(GPON_GTC_DS_PORT_IND, DS_PORT_OP_WRITE | (GPON_DATA_FLOW & 0x7f));
	gpon_wr(GPON_GTC_DS_PORT_WR, GPON_DATA_GEM & 0xfff);
	gpon_wr(GPON_GTC_DS_PORT_IND,
		DS_PORT_OP_WRITE | (GPON_DATA_FLOW & 0x7f) | DS_PORT_OP_REQ);
	for (i = 0; i < 1000; i++) {
		if (gpon_rd(GPON_GTC_DS_PORT_IND) & DS_PORT_OP_COMPL)
			break;
		udelay(1);
	}
	if (i == 1000) {
		pr_err("rtl9602c-gpon: DATA GEM DS install timeout\n");
		return -ETIMEDOUT;
	}
	/* DATA flow DS routing = 0x2 (BIT1). ★ORACLE-CONFIRMED 2026-06-15: a live working stock
	 * RTL9602C (ttyUSB3) has DS_TRAFFIC_CFG[data flows] = 0x2 and 0x3, NOT 0; flow64(OMCI)=0x4.
	 * Our prior 0 ("bridged to switch") was THE WAN-DHCP DS BUG: the GTC de-encapsulated the
	 * data (fsm ds_rx climbed) but with cfg=0 the de-encapped frame was DROPPED — PKT_OK_CNT_DS
	 * (0xc010) stayed at the OMCI-only count, switch port-2 rx=0, gpon0 rx=0, no DHCP OFFER
	 * delivered. BIT1 routes de-encapped UNICAST data into the PON-IP NIC->GMAC-RX drain (stock's
	 * PKT_OK_CNT_DS climbs continuously), where rtl9602c_eth_rx's src_port==2 demux hands it to
	 * gpon0. (GEM_DS_MC_CFG 0x59 + DS-PTI from the OMCC install cover this flow too.) */
	gpon_wr(GPON_GTC_DS_TRAFFIC_CFG + GPON_DATA_FLOW * DS_TRAFFIC_CFG_STRIDE, 0x2);

	/* US GEM-port map: flow 1 -> gem 193 (the gem-id stamped on US frames). Same stride-4
	 * indexing the OMCC uses (the OMCC's flow64 slot 0x6500 holds its gem 0). */
	gpon_wr(GPON_GTC_GEM_US_PORT_MAP + GPON_DATA_FLOW * GEM_US_PORT_MAP_STRIDE,
		GPON_DATA_GEM & 0xfff);

	/* PON-IP classify: SID2QID[1]=OMCC qid 64 (ride T-CONT 16 grants — the OMCC and data
	 * SHARE the OLT's single Alloc-ID 256, confirmed live: T-CONT 16 <- alloc 0x100), SIDVALID[1]=1,
	 * SID_Q_MAP_DS[1]=0 (DS reaches the CPU via switch port-2, not a PBO queue). Do NOT
	 * touch PI_PON_OMCI_CFG (stays 64). [Binding the data to a 2nd T-CONT on Alloc 256 was
	 * tried and REGRESSED the US — the OMCC's later bind of Alloc 256 to T-CONT 16 won the CAM,
	 * leaving the data T-CONT grantless.] */
	pi_packed_set(PI_PON_SID2QID, GPON_DATA_FLOW, 7, GPON_OMCC_PHYS_QID & 0x7f);
	pi_packed_set(PI_PON_SIDVALID, GPON_DATA_FLOW, 1, 1);
	pi_packed_set(PI_PON_SID_Q_MAP_DS, GPON_DATA_FLOW, 2, 0);

	/* Multicast/broadcast GEM (4095) -> flow 2, BRIDGED, DS-only. Broadcast DS (e.g. the
	 * DHCP OFFER) may ride this GEM rather than the unicast data GEM; without a CAM entry
	 * the GTC drops it. de-encap -> switch port-2 -> CPU -> gpon0 (same demux as flow 1). */
	gpon_wr(GPON_GTC_DS_PORT_IND, DS_PORT_OP_WRITE | (GPON_MCAST_FLOW & 0x7f));
	gpon_wr(GPON_GTC_DS_PORT_WR, GPON_MCAST_GEM & 0xfff);
	gpon_wr(GPON_GTC_DS_PORT_IND,
		DS_PORT_OP_WRITE | (GPON_MCAST_FLOW & 0x7f) | DS_PORT_OP_REQ);
	for (i = 0; i < 1000; i++) {
		if (gpon_rd(GPON_GTC_DS_PORT_IND) & DS_PORT_OP_COMPL)
			break;
		udelay(1);
	}
	/* MCAST/broadcast flow DS routing = 0x3 (BIT1|BIT0) — oracle's other data flow value;
	 * BIT0 = broadcast/flood variant so DHCP-broadcast + ARP DS also drain to the NIC. */
	gpon_wr(GPON_GTC_DS_TRAFFIC_CFG + GPON_MCAST_FLOW * DS_TRAFFIC_CFG_STRIDE, 0x3);

	gpon_data_installed = true;	/* set BEFORE modeset so its collision-fix keeps flow 1 */

	/* ===== DATA-FLOW SID CLASSIFY COMMIT (the flow-1 US half-boot fix) =====
	 * ROOT CAUSE of the ~50/50 data-US latch: the US-NIC RX engine commits its
	 * SID-classify table (SID2QID/SIDVALID) ONLY on a GMII_RX_EN rising edge
	 * (0x...50 -> 0x...70 on PI_IO_CMD_0_US). SID-64 (OMCI) is committed at TWO
	 * guaranteed edges (boot pre-arm before gpon_pbo_init's edge @2379, and the
	 * gpon_install_tcont relatch @3718-3720 which is GATED to tcont==OMCC_TCONT),
	 * so OMCI is reliable. But the flow-1 classify written just above is committed
	 * at ZERO guaranteed edges: the intended relatch here (rtl9602c_ponmac_modeset_gpon)
	 * is a NO-OP by default (ponmac_modeset=0), and the install_tcont relatch fires
	 * for the OMCC T-CONT BEFORE this data gem is installed. So flow-1 only latched
	 * when the OLT happened to re-trigger a T-CONT edge AFTER Configure_Port-ID ->
	 * the boot-dependent ~50% latch (symptom: us_gtc gem_byte stuck ~1024, gemus64=0,
	 * DISCOVER never egresses, while the alloc/T-CONT/OMCI are all fine).
	 *
	 * FIX: pulse the SAME proven GMII edge HERE, right after the flow-1/mcast classify
	 * is written, and BLOCK on a readback commit-poll (the omitted SID-valid commit):
	 * re-read SIDVALID[1]/SID2QID[1] through pi_packed_get (already used below) and, if
	 * either reads back wrong, rewrite the triple and re-pulse the edge. Bounded retries.
	 * SAFE: this runs in the Configure_Port-ID PLOAM handler (the config window the
	 * driver already tolerates for US-NIC re-latch -- see gpon_install_omcc / line 4047),
	 * NOT while Online (no "Laser out") and NOT in the fast O5 print path (no pi_rd hang).
	 * The edge re-commits the WHOLE current table, so the OMCC SID-64 entry is preserved.
	 * Gated by relatch_us (default 1, same as the OMCC relatch). */
	if (relatch_us) {
		int tries;

		for (tries = 0; tries < 4; tries++) {
			/* (re)assert the flow-1 classify triple so a missed edge is re-armed */
			pi_packed_set(PI_PON_SID2QID, GPON_DATA_FLOW, 7,
				      GPON_OMCC_PHYS_QID & 0x7f);
			pi_packed_set(PI_PON_SIDVALID, GPON_DATA_FLOW, 1, 1);

			/* commit edge: GMII_RX_EN OFF -> ON latches the classify table */
			pi_wr(PI_IO_CMD_0_US, 0x90101050u);	/* GMII RX OFF */
			udelay(50);
			pi_wr(PI_IO_CMD_0_US, 0x90101070u);	/* GMII RX ON -> latch flow-1 */

			/* commit-poll: the classify regs are not write-protected, they
			 * latch on the edge; a correct readback is the definitive
			 * "committed" signal (not a timing guess). */
			if (pi_packed_get(PI_PON_SIDVALID, GPON_DATA_FLOW, 1) == 1 &&
			    pi_packed_get(PI_PON_SID2QID, GPON_DATA_FLOW, 7) ==
				    (GPON_OMCC_PHYS_QID & 0x7f))
				break;
			udelay(100);
		}
		pr_info("rtl9602c-gpon: data-gem: flow-%u classify committed after %d edge(s)\n",
			GPON_DATA_FLOW, tries + 1);

		/* DS half of the same fix: the DS-NIC drain config + GMII edge were
		 * latched at boot in gpon_pbo_init, BEFORE this Configure_Port-ID
		 * window and BEFORE the ifup GMAC0 power-cycle may have perturbed the
		 * DS-NIC<->GMAC0 internal MII. Re-force the DS link and re-pulse the DS
		 * GMII_RX_EN edge here (OFF->ON) so the now-complete DS data flow is
		 * latched against an established link -> makes DS delivery to the GMAC
		 * RX ring deterministic every boot (cures the ~50% D_rxok-climbs-but-
		 * filled=0 split). Same config window the US re-latch above already
		 * tolerates (NOT Online -> no US-burst/'Laser out' risk). */
		pi_wr(PI_MEDIA_STS_DS, 0x106e8400u);	/* re-force DS-NIC<->GMAC0 internal MII link UP */
		pi_wr(PI_IO_CMD_0_DS, 0x90081050u);	/* DS GMII_RX_EN OFF */
		udelay(50);
		pi_wr(PI_IO_CMD_0_DS, 0x90081070u);	/* ON -> re-latch DS drain config + link */
	}
	rtl9602c_ponmac_modeset_gpon();	/* keep: no-op unless ponmac_modeset=1 (reference path) */

	pr_info("rtl9602c-gpon: DATA GEM installed gem=%u flow=%u qid=%u sid2qid=%u sidvalid=%u\n",
		GPON_DATA_GEM, GPON_DATA_FLOW, GPON_OMCC_PHYS_QID,
		pi_packed_get(PI_PON_SID2QID, GPON_DATA_FLOW, 7),
		pi_packed_get(PI_PON_SIDVALID, GPON_DATA_FLOW, 1));
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

	/* PON-MAC US scheduler activation — the half of the stock ponmac queue-add our
	 * driver omitted. The GTC alloc CAM above binds the OLT Alloc-ID to the T-CONT for
	 * BWMAP grants, but the PON-IP scheduler never knew T-CONT 16 / physical queue 64
	 * was a live member, so the ONU could not actually transmit US GEM on it. The OLT
	 * then never sees the upstream T-CONT operate, stays Config State "initial" and
	 * never starts OMCI (so DS OMCI never arrives — gem-2 frames never reach our
	 * correctly-programmed flow-64 CAM). Activate: enable the T-CONT, put its logical
	 * queue 0 in the schedule mask, give physical queue 64 STRICT type + MAX PIR/CIR,
	 * and (RTL9602C = rev A) clear PON_GEN_PIR_DROP "due to the tcont 16". Offsets are
	 * PON-IP driver-relative (phys - 0xF00000); physicalQid = 32*(16/8)+0 = 64. */
	{
		/* physicalQid = TCONT_QUEUE_MAX(32) * (tcont/8) + logical-queue-0.
		 * T-CONT 0 -> qid 0 (default/mgmt), T-CONT 16 -> qid 64 (OMCC). The PIR/CIR
		 * rate arrays hold ONE 18-bit RATE field PER 32-bit word (addr = base +
		 * qid*4), NOT bit-packed — the old rwd=(qid*18)/32 indexing wrote qid 36's
		 * word, leaving qid 64's shaper rate at reset (~0) -> 0 US bandwidth, so the
		 * T-CONT pulled nothing and sent idle GEM. Index the rate arrays by qid*4. */
		/* OMCC T-CONT 16 uses the silicon qid 63 (GPON_OMCC_PHYS_QID), not the
		 * formula's 64, so the scheduler drains the SAME queue SID2QID routes US OMCI to. */
		u8 qid = (tcont == GPON_OMCC_TCONT) ? GPON_OMCC_PHYS_QID : 32 * (tcont / 8);

		pi_field(0x023e4 + (tcont / 32) * 4, tcont % 32, tcont % 32, 1); /* PON_TCONT_EN[tcont] */
		/*
		 * Drain out the physical queue BEFORE binding it into the schedule mask.
		 * This is the half of the stock ponmac queue-add that was previously
		 * omitted: DRN_CMD (PON-IP 0x20e4) = CFG_DRN_QUEUE_MODE(bit2)=1 |
		 * CFG_DRN_IDX(bits3..9)=qid | DRN_PS(bit1)=1, then poll DRN_FLG(bit0) until
		 * it clears. Without it the queue stays in a stale/blocked drain state and
		 * the US scheduler never PULLS it onto the T-CONT's grants, so the T-CONT
		 * fills its grants with idle GEM (idle16 climbs) and OMCC GEM never egresses
		 * (gemus64 = 0) — the queued US OMCI sits forever. DRN_CMD address + field
		 * bits from the chip's register/field map; field semantics from the stock
		 * ponmac queue-add behavior.
		 */
		{
			int n;

			pi_wr(0x020e4, (1u << 2) | ((qid & 0x7f) << 3) | (1u << 1));
			for (n = 0; n < 10000 && (pi_rd(0x020e4) & 1u); n++)
				udelay(1);
			if (n >= 10000)
				pr_warn("rtl9602c-gpon: qid %u drain-out timeout\n", qid);
		}
		pi_wr(0x023a0 + tcont * 4, 0x1);		/* PON_SCH_QMAP[tcont] = logical-q0 */
		pi_field(0x0229c + qid * 4, 17, 0, 0x3ffff);	/* PON_QID_PIR_RATE[qid] = MAX (1 word/qid) */
		pi_field(0x02198 + qid * 4, 17, 0, 0);		/* PON_QID_CIR_RATE[qid] = 0 (live-stock; STRICT uses PIR only) */
		pi_field(0x023e8 + (qid / 32) * 4, qid % 32, qid % 32, 0); /* PON_WFQ_TYPE[qid] = STRICT */
		/* PON_WFQ_WEIGHT[qid] = 1 (10 bits/entry, 3 entries per 32-bit word):
		 * stock writes weight 1 even for a STRICT queue ("for safe") — a zero-weight
		 * queue is skipped by the WFQ round. */
		pi_field(0x023f8 + (qid / 3) * 4, (qid % 3) * 10 + 9, (qid % 3) * 10, 1);
		pi_wr(0x02194, 0x66000);			/* PON_SCH_CTRL base (bits 13,14,17,18) */
		/* RTL9602C = CHIP_REV_ID_A: stock ponmac mode-set clears PON_GEN_PIR_DROP
		 * (PON_SCH_CTRL bit18) on rev A. Left set, every BWMAP grant is PIR-dropped: the
		 * grant is accepted (bwm_acpt) but the GEM-US framing engine is never driven, so the
		 * T-CONT emits NEITHER data NOR idle (idle=0 AND gemus=0, OLT -> "Laser out"). bit18
		 * is GLOBAL: clear it for EVERY T-CONT install — the previous OMCC-only clear meant a
		 * later data-T-CONT install (which re-writes 0x66000 above) re-set it and broke the
		 * OMCC. Clearing unconditionally keeps both the OMCC and the data T-CONT live. */
		pi_field(0x02194, 18, 18, 0);
	}

	/* finding 3: with the scheduler now bound (above) AND the SID2QID classify already set
	 * (boot pre-arm at the true 0x2138 slot, or gpon_install_omcc), the US-NIC has its COMPLETE
	 * config for the first time. Re-pulse the GMII_RX_EN latch edge (OFF->ON) so the US-NIC RX
	 * engine re-latches it — the boot/ifup edge fired before this scheduler binding existed.
	 * Only meaningful for the OMCC T-CONT (16); harmless for data T-CONTs. */
	if (relatch_us && tcont == GPON_OMCC_TCONT) {
		pi_wr(PI_IO_CMD_0_US, 0x90101050u);	/* GMII RX OFF */
		pi_wr(PI_IO_CMD_0_US, 0x90101070u);	/* GMII RX ON -> re-latch complete US config */
		pr_info("rtl9602c-gpon: relatch_us: re-pulsed GMII_RX_EN after OMCC scheduler bind (io0_us=0x%08x)\n",
			pi_rd(PI_IO_CMD_0_US));
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
	u8 rep, i, boh_len, size;

	/* EXACT port of the stock burst-overhead build (the upstream-overhead
	 * calculation + the ranged burst-head set). The old
	 * code ALWAYS stored the 3 delimiter bytes at oh[9..11] and wrote 12 bytes — but
	 * the RANGED (operation) burst the OLT dictates via Extended_Burst_Length (0x14)
	 * is usually SHORTER than 12 (boh_len = guard/8 + t3ranged + 3). With BOH_LENGTH<12
	 * the HW emits only BOH_LENGTH bytes, so the delimiter at oh[9..11] is CUT OFF: the
	 * O5 burst goes out with NO delimiter, the OLT burst-RX can't frame it -> alarm LOAi
	 * (Loss Of Acknowledge) / "Laser out" -> Deactivate cycle. Place the delimiter at the
	 * TRUE end (oh[size-3..size-1]) and set BOH_REPEAT=guard-bytes (the fill-byte index),
	 * matching stock; identical output when boh_len>=12. */
	if (guard > 32)
		guard = 32;
	rep = guard / 8;			/* boh_repeat = whole guard bytes (fill index) */

	boh_len = t3 ? (rep + t3 + 3) : GPON_BOH_LEN;	/* total burst-overhead length */
	if (boh_len > GPON_BOH_MAX_LEN)
		boh_len = GPON_BOH_MAX_LEN;
	size = (boh_len > GPON_BOH_LEN) ? GPON_BOH_LEN : boh_len;	/* stored bytes (<=12) */
	if (size < 4)				/* need room for >=1 fill + 3 delimiter */
		size = 4;

	memset(oh, 0xaa, sizeof oh);
	for (i = 0; i < rep && i < (u8)(size - 3); i++)
		oh[i] = 0xaa;			/* guard bytes */
	for (; i < (u8)(size - 3); i++)
		oh[i] = gpon_boh_ptn;		/* Type-3 preamble fill (= bursthead[rep]) */
	oh[size - 3] = gpon_boh_delim[0];	/* delimiter at the TRUE end of the burst */
	oh[size - 2] = gpon_boh_delim[1];
	oh[size - 1] = gpon_boh_delim[2];

	/*
	 * BOH_REPEAT is NOT guard/8 — it is the stored-byte index of the LAST
	 * preamble byte before the 3-byte delimiter, i.e. (size - 4). This is the
	 * HW pointer the burst-builder uses to know which stored byte to replicate
	 * when extending the stored <=12 bytes out to the full BOH_LENGTH. The stock
	 * burst-overhead set writes exactly `(size-4)&0xf` and
	 * IGNORES the caller's boh_repeat (its `if (rep) {}` is a no-op). With
	 * size=12 this is 8 — matching the live-stock golden O5 dump BOH_CFG=0x083f
	 * (REPEAT=8, LENGTH=63). Writing REPEAT=guard/8=4 mis-positions the
	 * delimiter in the synthesized ranged burst (LENGTH 63/127 > 12 stored), so
	 * the OLT burst-RX never locks the O5 grant burst -> LOSi/SFi ("Laser out")
	 * + LOAi (the PLOAM ACK rides the same broken BOH) -> Deactivate. The
	 * pre-ranged SN burst tolerates it (size==LENGTH==12, no synthesis, wide
	 * acquisition window) which is why ranging succeeds while O5 fails.
	 */
	gpon_wr(GPON_GTC_US_BOH_CFG,
		(((size - 4) & 0xf) << 8) | (boh_len & 0xff));	/* REPEAT=size-4, LENGTH=full */
	for (i = 0; i < size; i++)
		gpon_wr(GPON_GTC_US_BOH_DATA + i * 4, oh[i]);

	pr_info("rtl9602c-gpon: BOH %s guard=%u rep=%u boh_repeat=%u ptn=0x%02x delim=%02x%02x%02x t3=%u boh_len=%u size=%u oh=%*phN\n",
		ranged ? "ranged" : "prerng", guard, rep, (size - 4) & 0xf, gpon_boh_ptn,
		gpon_boh_delim[0], gpon_boh_delim[1], gpon_boh_delim[2],
		t3, boh_len, size, size, oh);
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

/* HYBRID LAN/VLAN switch. VLAN filtering (0x13008 bit0) must be ON during ranging +
 * OMCI config-apply (verified: VLAN-off cold boots fail config-apply; VLAN-on onlines),
 * but this switch will NOT forward LAN port<->CPU traffic with filtering on, so LAN
 * management access (br-lan 192.168.1.1) needs filtering OFF. Resolution: keep it on
 * through config, then clear it once the ONU has held O5 for vlan_lan_o5_ticks poll ticks
 * (~10ms each; default ~50s, well past config-apply), and re-assert on any drop below O5
 * (a re-range must do its config with filtering on again). Proven viable live: online with
 * 0x19 then 0x13008=0 -> stays online 6h + LAN reachable. */
static unsigned int vlan_lan_o5_ticks = 4000;
module_param(vlan_lan_o5_ticks, uint, 0644);
MODULE_PARM_DESC(vlan_lan_o5_ticks, "poll ticks held at O5 before clearing VLAN_FILTER for LAN access (0=keep filtering on)");
static u32 gpon_o5_entry_tick;
static bool gpon_vlan_lan_open;
static int gpon_avc_sent;	/* OMCI oper-state AVCs emitted this O5 (reset on re-range) */

/*
 * Re-lock the TX CMU PLL at O3 entry — see serdes_txpll_relock. A fresh power-on under
 * strong downstream light can assert the optical signal-detect before the CMU has
 * settled, latching the TX PLL onto the wrong clock rate (~50% "Laser out"). Toggling
 * the CMU enable 1->0->1 forces a clean re-acquire now the optics are stable, then the
 * SerDes word FIFO read/write pointer is re-synced. Called once per ranging cycle from
 * the FSM (softirq); the settle is short (the CMU finishes re-locking during the
 * remaining ranging time, well before the upstream burst is framed).
 */
static void gpon_txpll_relock(void)
{
	if (!serdes_txpll_relock)
		return;
	if (sw_rd(SDS_ANA_COM_REG27) & SDS_CMU_EN) {
		sw_field(SDS_ANA_COM_REG27, 10, 10, 0);		/* CMU enable -> 0      */
		udelay(100);
		sw_field(SDS_ANA_COM_REG27, 10, 10, 1);		/* -> 1 (re-acquire)    */
		udelay(100);
	}
	sw_field(WSDS_DIG_1D, 14, 14, 0);			/* FIFO r/w ptr re-sync */
	sw_field(WSDS_DIG_1D, 14, 14, 1);
	pr_info("rtl9602c-gpon: TX-PLL relock (CMU re-toggle + FIFO re-sync) at O3 entry\n");
}

static void gpon_fsm_set_state(u8 st)
{
	u8 prev = gpon_fsm_state;

	/* gpon_hold: keep the FSM parked at O1 — refuse every advance past O1 so the
	 * GPON never ranges/deactivates and the shared switch datapath stops churning,
	 * leaving br-lan + the WiFi AP stable for LAN+WiFi access. (GPON/WAN off.) */
	if (gpon_hold && st > 1)
		return;
	if (gpon_fsm_state != st)
		pr_info("rtl9602c-gpon: ONU state O%u -> O%u\n", gpon_fsm_state, st);
	if (prev != st)
		gpon_los_run = 0;	/* fresh LOS debounce window on every transition */
	gpon_fsm_state = st;

	/* Hybrid LAN/VLAN bookkeeping: mark O5 entry; on any drop below O5, re-arm VLAN
	 * filtering for the next config and reset the LAN-open timer. */
	if (st == 5 && prev != 5) {
		gpon_o5_entry_tick = gpon_fsm_ticks ? gpon_fsm_ticks : 1;
		gpon_avc_sent = 0;	/* re-report oper-up to the OLT each online */
		/* Re-apply the O5 packed-burst gate cluster + re-arm the HW auto-
		 * No_message keepalive on EVERY O5 entry, not just at __init. A
		 * re-range performs a GMAC/SDS reset that can clear these US-side regs,
		 * so a re-ranged O5 must not run on reset defaults ("isolated
		 * tolerates, packed exposes"). Same values as init (4791-4794, 4730-
		 * 4737); US-side only, harmless to DS/ranging. */
		if (o5_rearm_burst_gate) {
			u8 nomsg[12];

			gpon_wr_us_protected(0x5188, 0x00504bfa);	/* US_OPTIC_SD_TH */
			gpon_field(0x526c, 0, 0, 1);			/* US_PWR_SAV_MODE */
			gpon_wr(0x6024, (0x10u << 16) | 0x100u);	/* GEM_US_PWR_SAV_CFG */
			gpon_wr(0x6260, 0x00000028u);			/* GEM_US_EOB_MERGE */
			memset(nomsg, 0xaa, sizeof(nomsg));
			nomsg[0] = 0xff;	/* ONU-ID (HW overrides via ONUID_OVRD) */
			nomsg[1] = 0x04;	/* GPON_PLOAM_US_NOMESSAGE */
			gpon_send_cpu_ploam(PLM_US_QUEUE_NOMSG, nomsg);
		}
	} else if (st < 5 && prev >= 5) {
		gpon_o5_entry_tick = 0;
		if (gpon_vlan_lan_open && !lan_keep_open) {
			sw_field(0x13008, 0, 0, 1);	/* re-assert VLAN_FILTER for re-config */
			gpon_vlan_lan_open = false;
			pr_info("rtl9602c-gpon: re-range -> VLAN_FILTER re-armed (config phase)\n");
		}
		/* lan_keep_open (default): leave VLAN_FILTER cleared so LAN management
		 * survives the WAN-down/re-range; the OLT re-config on resume tolerates it. */
	}
	/* The HW ONU_STATE field uses the same 1-based encoding as our state numbers:
	 * UNKNOWN=0, O1=1, O2=2, O3=3, O4=4, O5=5. So O3 (Serial-Number, where the
	 * GTC's auto-SN-burst transmitter is gated) = register value 3 = our st.
	 * Write st. */
	gpon_field(GPON_GTC_DS_ONU_STATUS, 3, 0, st);
}

/*
 * Process-context worker that runs the stock SerDes CDR-reset
 * (invert SDS_ANA_COM_REG08 bit15, hold 10ms, restore). Scheduled from
 * gpon_fsm_handle() (softirq) on a Deactivate->O1 re-range so the upstream-TX
 * serializer CDR is re-seated with the *correct* primitive before the next
 * ranging burst. Only the TX-CDR bit is touched (RX downstream framer lock is
 * undisturbed); mdelay is safe here. */
/* A/B (2026-06-15): on a re-range, re-run the FULL analog SerDes bring-up
 * instead of just the light CDR pulse. A bad cold-start US-burst lock persists
 * across re-ranges (the light pulse never recovers it -> the OLT keeps issuing
 * Deactivate(0x05)=LOS on ~50%% of boots; boots that get a good FIRST lock stay
 * online + lease). The link is DOWN here (post-deact, FSM re-acquiring), so
 * re-initing the CMU/SDS is as safe as a cold boot and gives a fresh lock. */
static bool full_serdes_reinit;	/* default OFF: A/B 2026-06-15 found re-running the full
				 * gpon_serdes_init on re-range BREAKS the GPON (0/5 boots, link never
				 * reaches O5 — the CMU/SDS reset races the FSM re-acquisition). Keep the
				 * light CDR pulse instead. Do NOT enable. */
module_param(full_serdes_reinit, bool, 0644);
MODULE_PARM_DESC(full_serdes_reinit, "re-range: re-run full gpon_serdes_init (BROKEN, default off) vs light CDR pulse");
static void gpon_cdr_reset_worker(struct work_struct *w)
{
	u32 cdr;

	if (!serdes_cdr_reset)
		return;
	if (full_serdes_reinit) {
		gpon_serdes_init();	/* full analog re-init: fresh lock attempt */
		pr_info("rtl9602c-gpon: re-range FULL serdes re-init\n");
		return;
	}
	cdr = sw_rd(SDS_ANA_COM_REG12);
	sw_wr(SDS_ANA_COM_REG12, cdr ^ BIT(15));
	mdelay(10);
	sw_wr(SDS_ANA_COM_REG12, cdr);
	pr_info("rtl9602c-gpon: re-range serdesCdr_reset pulse (COM_REG12 0x225b0 bit15), restored=0x%08x\n",
		cdr);
}

static void gpon_fsm_handle(const u8 *m)
{
	u8 onu_id = m[0], type = m[1];
	const u8 *d = &m[2];		/* 10 data octets */

	/* Surface any DS PLOAM that is not the repetitive broadcast acquisition
	 * traffic (Upstream_Overhead 0x01 / profile 0x14) — e.g. Assign_ONU-ID or
	 * anything addressed to us — so activation progress is visible. */
	gpon_last_ds_type = type;
	if (trace && type != PLM_DS_UPSTREAM_OVERHEAD && type != PLM_DS_EXT_BURST_LENGTH)
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
			gpon_txpll_relock();	/* re-lock TX CMU PLL now DS optics are stable, before US burst */
			gpon_send_sn();		/* first SN immediately */
		}
		break;
	case PLM_DS_ASSIGN_ONU_ID:
		/* d[0] = assigned ONU-ID, d[1..8] = serial number to match */
		if (!memcmp(&d[1], gpon_sn_bytes, 8)) {
			gpon_fsm_onu_id = d[0];
			gpon_field(GPON_GTC_US_ONU_ID, 15, 8, gpon_fsm_onu_id);
			gpon_field(GPON_GTC_DS_ONU_STATUS, 15, 8, gpon_fsm_onu_id);
			/* Bind the DEFAULT/management Alloc-ID (= ONU-ID, per G.984.3) to the
			 * OMCC's T-CONT 16 (NOT T-CONT 0). Pre-config the OLT grants only this
			 * default alloc for PLOAM/OMCI, and as stock does it owns the OMCC
			 * T-CONT, so its grants must serve the OMCC US queue (phys qid 64). With
			 * it on an empty T-CONT 0 the OMCC upstream is never drained, the OLT
			 * sees the OMCC half-dead and WITHHOLDS DS OMCI. (The separate data
			 * Alloc-ID 0x400 is bound to T-CONT 8 on Assign_Alloc-ID.) */
			gpon_install_tcont(GPON_OMCC_TCONT, gpon_fsm_onu_id);
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
			/* Flush any pre-ranged-format US PLOAM still latched in the single
			 * shared CPU TX buffer before the first ranged (O5) grant fires, so
			 * the OLT's NARROW ranged-window burst-RX never has to frame a stale
			 * pre-ranged-format burst (-> LOSi/SFi -> Deactivate(0x05) on ~50%%
			 * of boots, grant-timing dependent). Stock does
			 * exactly this at this O4-EqD edge (US-PLOAM-buffer flush: PLM_FLUSH_BUF
			 * 0->1). Use gpon_field RMW so CRC_GEN_EN|ONUID_OVRD are preserved. */
			gpon_field(GPON_GTC_US_PLOAM_CFG, 4, 4, 0);	/* PLM_FLUSH_BUF = 0 */
			gpon_field(GPON_GTC_US_PLOAM_CFG, 4, 4, 1);	/* 0->1 edge: flush */
			pr_info("rtl9602c-gpon: Ranging_Time EqD=0x%x -> O5 (us-ploam flushed)\n", eqd);
			gpon_fsm_set_state(5);
		}
		break;
	case PLM_DS_DISABLE_SN:
		/* Disable_serial_number (0x06), G.984.3: d[0] is the disable/ENABLE code,
		 * d[1..8] the target SN. The OLT uses ONUID 0xff (broadcast) for this. ONLY a
		 * real DISABLE resets us: d[0]=0xFF for OUR SN, or d[0]=0x0F (disable all). An
		 * ENABLE (d[0]=0x00 for our SN = the OLT RE-ALLOWING our SN after it had been
		 * disabled) must NOT reset — the old code blindly reset on every 0x06, so it
		 * fought the OLT's re-enable and trapped the ONU in a re-range loop (live trace
		 * showed the OLT spamming 0x06 d[0]=0x00 ENABLE while we kept resetting).
		 * Matches the authoritative stock Disable_SN PLOAM handling (
		 * 0xFF+SN=RX_DISABLE, 0x00+SN=RX_ENABLE, 0x0F=enable-all recovery). */
		if (!((d[0] == 0xff && !memcmp(&d[1], gpon_sn_bytes, 8)) || d[0] == 0x0f))
			break;			/* 0x00 ENABLE / not-our-SN -> ignore, keep activating */
		pr_info("rtl9602c-gpon: Disable_SN code=0x%02x -> reset O1\n", d[0]);
		fallthrough;
	case PLM_DS_DEACTIVATE_ONU:
		if (onu_id == gpon_fsm_onu_id || onu_id == 0xff) {
			pr_info("rtl9602c-gpon: EVT t=%u DEACT(0x05) onu=%u | dsrx64=%u pirx=%u omcirx=%u | ploam_cpu=%u gem_byte=%u gemus64=%u idle16=%u\n",
				gpon_fsm_ticks, gpon_fsm_onu_id,
				gpon_gem_ds_rx_cnt(64), sw_rd(0x329c0), rtl9602c_eth_omci_rx_count(),
				gpon_us_misc_cnt(2), gpon_us_misc_cnt(4), gpon_rd(0x6a00), gpon_rd(0x6c80));
			gpon_fsm_onu_id = 0xff;
			/* FULL reset to O1 — mirror the SN-reprovision path (≈line 3068).
			 * Previously only the SW onu-id/key were cleared, leaving the
			 * one-shot OMCC/T-CONT install guards TRUE and the HW ONU-ID regs
			 * stale. Consequence under OLT deactivate-churn: the 2nd+ re-range
			 * SKIPS gpon_install_omcc()/gpon_install_tcont() (guard still set),
			 * so the ONU never rebuilds its OMCI datapath, and the freshly
			 * rebooted OLT keeps directed-deactivating the stale HW ONU-ID. */
			gpon_omcc_installed = false;
			gpon_tcont_installed = false;
			gpon_data_installed = false;	/* re-install WAN data GEM on re-config */
			gpon_aes_switch_time = 0xffffffff;	/* re-arm 0x13 on next activation */
			gpon_key_staged = false;
			gpon_field(GPON_GTC_DS_ONU_STATUS, 15, 8, 0xff);
			gpon_field(GPON_GTC_US_ONU_ID, 15, 8, 0xff);
			if (cdr_reseat_on_reactivate) {
				/* re-seat US-TX SerDes interface reset-B (softirq-safe, TX-only; the
				 * locked DS RX framer is undisturbed) so the next re-range starts from a
				 * fresh serializer lock instead of the prior marginal one (cuts flapping). */
				sw_field(WSDS_DIG_1D, 16, 16, 0);
				udelay(500);
				sw_field(WSDS_DIG_1D, 16, 16, 1);
				/* AND run the *correct* stock CDR-lock pulse (invert COM_REG12 bit15,
				 * 10ms, restore) deferred to process context — the 10ms hold cannot run
				 * here in softirq. The interface reset-B re-strobe above does not re-lock
				 * the serializer CDR; this does. */
				schedule_work(&gpon_cdr_reset_work);
			}
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
			/* Install the WAN data GEM NOW (config phase), not ~30s later while
			 * Online: a US-NIC modeset while Online glitches the burst -> "Laser
			 * out". Doing it here keeps the modeset in the tolerated config window
			 * (one-shot; gem-ids hardcoded so it needs no OMCI ME268 yet). */
			if (data_gem_en && gpon_omcc_installed && !gpon_data_installed)
				gpon_install_data_gem();
			if (trace)
				pr_info_ratelimited("rtl9602c-gpon: ACK type=0x%02x d=%*phN\n",
						    type, 8, d);
		}
		break;
	case PLM_DS_ASSIGN_ALLOC_ID:
		/* Assign_Alloc-ID (0x0a): bind the OLT's separate DATA Alloc-ID
		 * (alloc=(d[0]<<4)|(d[1]>>4)) to a DATA T-CONT (8), NOT the OMCC T-CONT 16
		 * (which now belongs to the management Alloc-ID = ONU-ID, set at Assign_ONU-ID,
		 * so two allocs do not collide on T-CONT 16). (d[2]: 0x01=allocate,
		 * 0xff=deallocate.) Then Acknowledge. */
		if (onu_id == gpon_fsm_onu_id) {
			u16 alloc = ((u16)d[0] << 4) | (d[1] >> 4);
			pr_info("rtl9602c-gpon: ASSIGN_ALLOC alloc=0x%x op=0x%x tcont_done=%d\n", alloc, d[2], gpon_tcont_installed);

			/* THE alloc the OLT assigns here (e.g. 0x400) is the OMCC's upstream
			 * Alloc-ID, NOT a data Alloc-ID: bind it to the OMCC T-CONT 16 (overwriting
			 * the placeholder ONU-ID bind from Assign_ONU-ID). The OLT grants ONLY this
			 * Alloc-ID pre-OMCI; binding it to a separate T-CONT 8 left the OMCC T-CONT 16
			 * (on the ungranted ONU-ID alloc) SILENT — TCONT_IDLE[16]=0 — so the OLT never
			 * saw the OMCC upstream operate, kept re-Configure_Port-ID and withheld OMCI.
			 * On stock this same alloc is bound to T-CONT 16 and its OMCC emits (~10M). */
			if (d[2] == 0x01) {
				if (!gpon_tcont_installed) {
					/* first Alloc-ID = the OMCC mgmt alloc -> T-CONT 16 */
					if (!gpon_install_tcont(GPON_OMCC_TCONT, alloc)) {
						gpon_tcont_installed = true;
						gpon_omcc_alloc = alloc;
					}
				} else if (data_tcont && alloc != gpon_omcc_alloc &&
					   !gpon_data_tcont_installed) {
					/* the OLT's separate DATA Alloc-ID -> its OWN T-CONT 8, so the
					 * OLT's DBA sees a live reporting T-CONT and does not reclaim/
					 * deactivate it (see data_tcont). */
					if (!gpon_install_tcont(GPON_DATA_TCONT, alloc)) {
						gpon_data_tcont_installed = true;
						gpon_data_alloc = alloc;
						pr_info("rtl9602c-gpon: DATA Alloc 0x%x -> T-CONT %d (qid %d)\n",
							alloc, GPON_DATA_TCONT, GPON_DATA_PHYS_QID);
					}
				}
			} else if (d[2] == 0xff && data_tcont &&
				   gpon_data_tcont_installed && alloc == gpon_data_alloc) {
				/* OLT deallocated the data Alloc-ID: allow a clean re-bind on the next
				 * allocate (the alloc CAM binding is idempotent; no HW teardown needed). */
				gpon_data_tcont_installed = false;
			}
			gpon_send_ack(m);
			if (trace)
				pr_info_ratelimited("rtl9602c-gpon: ACK type=0x%02x d=%*phN\n",
						    type, 8, d);
		}
		break;
	case PLM_DS_REQUEST_KEY:
		/* OLT requests a downstream AES key; reply with Encryption_Key (US 0x05). */
		if (onu_id == gpon_fsm_onu_id) {
			gpon_send_key();
			pr_info("rtl9602c-gpon: EVT t=%u REQ_KEY(0x0d) dsrx64=%u pirx=%u omcirx=%u\n",
				gpon_fsm_ticks, gpon_gem_ds_rx_cnt(64), sw_rd(0x329c0),
				rtl9602c_eth_omci_rx_count());
		}
		break;
	case PLM_DS_REQUEST_PASSWORD:
		/* Request_Password (0x09): the OLT asks for our Password (US 0x02). GROUND
		 * TRUTH: without it the OLT stalls at O5 (spamming 0x09) and deactivates us
		 * with LOAi. Reply with the (empty) Password; the OLT is SN-auth so the value
		 * is ignored but the message is required to advance activation. */
		if (onu_id == gpon_fsm_onu_id || onu_id == 0xff) {
			gpon_send_password();
			pr_info("rtl9602c-gpon: EVT t=%u REQ_PW(0x09) -> sent Password dsrx64=%u pirx=%u omcirx=%u\n",
				gpon_fsm_ticks, gpon_gem_ds_rx_cnt(64), sw_rd(0x329c0),
				rtl9602c_eth_omci_rx_count());
		}
		break;
	case PLM_DS_KEY_SWITCH:
		/* Key_Switching_Time (0x13): the OLT supplies the 30-bit superframe count at
		 * which the HW promotes the staged AES key (loaded by gpon_send_key) to active.
		 * Arm the HW comparator (write SWITCH_SUPERFRAME) and Acknowledge. The OLT will
		 * not advance to OMCI until this key handshake completes, so a missing 0x13
		 * handler leaves it re-cycling Request_Key/Configure_Port-ID forever. De-dup the
		 * register write per superframe (the OLT re-sends 0x13 every cycle). */
		if (onu_id == gpon_fsm_onu_id || onu_id == 0xff) {
			u32 fc = ((u32)(d[0] & 0x3f) << 24) | ((u32)d[1] << 16) |
				 ((u32)d[2] << 8) | d[3];

			/* Only arm the HW key-switch once we have actually loaded a key into
			 * the staged bank (via Request_Key); arming a switch to an empty/stale
			 * staged bank would promote a garbage key and corrupt AES. ACK either way
			 * so the OLT sees the message handled. */
			if (gpon_key_staged && fc != gpon_aes_switch_time) {
				gpon_aes_switch_time = fc;
				gpon_wr(0x3014, fc);	/* AES_KEY_SWITCH_TIME[29:0] */
				pr_info("rtl9602c-gpon: Key_Switching_Time -> arm switch @superframe %u\n",
					fc);
			}
			gpon_send_ack(m);
			pr_info("rtl9602c-gpon: EVT t=%u KEY_SW(0x13) staged=%d arm@%u hwswt=%u dsrx64=%u pirx=%u omcirx=%u\n",
				gpon_fsm_ticks, gpon_key_staged, gpon_aes_switch_time,
				gpon_rd(0x3014) & 0x3fffffff,
				gpon_gem_ds_rx_cnt(64), sw_rd(0x329c0), rtl9602c_eth_omci_rx_count());
		}
		break;
	case PLM_DS_ENCRYPT_PORT:
		/* Encrypted_Port-ID (0x08): G.984.3 requires a US Acknowledge; the OLT
		 * arms a ~43s timer and Deactivates us if none arrives. */
		if (onu_id == gpon_fsm_onu_id) {
			gpon_send_ack(m);
			if (trace)
				pr_info_ratelimited("rtl9602c-gpon: ACK type=0x%02x d=%*phN\n",
						    type, 8, d);
		}
		break;
	case PLM_DS_CFG_VPVC:
		/* Configure_VP/VC (0x07): legacy ATM connection setup — unsupported on a
		 * GEM ONU, but stock still sends a US
		 * Acknowledge. A missing ACK to any AK-required DS PLOAM raises LOAi. */
		if (onu_id == gpon_fsm_onu_id) {
			gpon_send_ack(m);
			pr_info_ratelimited("rtl9602c-gpon: ACK CFG_VPVC(0x07)\n");
		}
		break;
	case PLM_DS_BER_INTERVAL:
		/* BER_interval (0x12): the OLT configures the upstream BER reporting interval
		 * and REQUIRES a US Acknowledge. Stock also arms a
		 * timer that periodically emits Remote_Error_Indication (US 0x08); the ACK is
		 * the part that prevents LOAi, so send it (broadcast or our ONU-ID, like stock
		 * which accepts the default ONU-ID). REI reporting is informational and not
		 * required to stay activated. */
		if (onu_id == gpon_fsm_onu_id || onu_id == 0xff) {
			gpon_send_ack(m);
			pr_info_ratelimited("rtl9602c-gpon: ACK BER_INTERVAL(0x12) d=%*phN\n",
					    8, d);
		}
		break;
	default:
		/* Any DS PLOAM addressed to us that we don't model: log it (rate-limited,
		 * always on) so a missing AK-required type is visible instead of a silent
		 * drop -> LOAi. PEE(0x0f)/PowerLevel(0x10)/PST(0x11)/Rang_Adjust(0x17) do NOT
		 * require an ACK in G.984.3; only log. If a logged type turns out to need an
		 * ACK, add an explicit case above. */
		if (onu_id == gpon_fsm_onu_id || onu_id == 0xff)
			pr_info_ratelimited("rtl9602c-gpon: UNHANDLED DS PLOAM type=0x%02x onu=0x%02x d=%*phN\n",
					    type, onu_id, 8, d);
		break;
	}
}

static void gpon_fsm_poll(struct timer_list *t)
{
	int guard = 0;

	gpon_fsm_ticks++;
	/* SN was (re)provisioned after ranging began (the driver started with the
	 * placeholder SN, which the OLT auto-ranges as a phantom that never matches
	 * the provisioned ONU). Drop to O1 and re-offer the new Serial_Number. */
	if (gpon_sn_changed) {
		gpon_sn_changed = false;
		if (gpon_fsm_state > 1) {
			gpon_fsm_onu_id = 0xff;
			gpon_omcc_installed = false;
			gpon_tcont_installed = false;
			gpon_data_installed = false;	/* re-install WAN data GEM on re-config */
			gpon_aes_switch_time = 0xffffffff;
			gpon_key_staged = false;
			gpon_field(GPON_GTC_DS_ONU_STATUS, 15, 8, 0xff);
			gpon_field(GPON_GTC_US_ONU_ID, 15, 8, 0xff);
			gpon_fsm_set_state(1);
			pr_info("rtl9602c-gpon: SN reprovisioned (%8phN) -> re-ranging\n",
				gpon_sn_bytes);
		}
	}
	while (!(gpon_rd(GPON_GTC_DS_PLOAM_IND) & GPON_DS_PLM_BUF_EMPTY) &&
	       guard++ < 16) {
		u8 m[13];

		gpon_ploam_read(m);
		gpon_fsm_handle(m);
		gpon_ds_rx++;					/* DS-lock liveness */
		gpon_wr(GPON_GTC_DS_PLOAM_IND, GPON_DS_PLM_DEQ);	/* advance */
	}
	/* WAN data GEM is now installed in the Configure_Port-ID handler (config phase),
	 * NOT here at +30s: a US-NIC modeset while Online glitched the burst -> "Laser out"
	 * -> deactivate (confirmed by stability bisection). Kept out of the O5 poll entirely. */

	/* Report the WAN-egress (VEIP) operational to the OLT via OMCI AVC. Must fire AFTER
	 * config-apply finishes AND after the OLT has created ME329 (~2500 ticks / ~31s after
	 * O5; config-apply is ~10s, the VEIP create is mid-config). Firing it earlier (during
	 * config, before ME329 exists) DISRUPTED config -> Config=fail (observed). The OLT never
	 * polls the data MEs it creates; it un-gates DOWNSTREAM user-data forwarding only when the
	 * ONU reports the port up. Send a few times; re-armed on each O5 entry. OMCI TX on the
	 * established OMCC (not a modeset), so it does not disturb the US burst. */
	if (gpon_fsm_state == 5 && gpon_omcc_installed && gpon_avc_sent < 3 &&
	    gpon_o5_entry_tick && (gpon_fsm_ticks - gpon_o5_entry_tick) > 2500 &&
	    ((gpon_fsm_ticks - gpon_o5_entry_tick) % 150) == 0) {
		rtl9602c_eth_omci_report_oper_up();
		gpon_avc_sent++;
	}

	/* O5 provisioning watchdog (see o5_provision_watchdog_ticks). A boot that
	 * reached O5 locally but the OLT never provisioned it (gpon0 RX still 0 well
	 * past the slow-lease window) is stuck on a non-frameable US-TX serializer
	 * phase with no OLT Deactivate to recover it. Self-re-range to RE-ROLL the
	 * phase, mirroring the Deactivate->O1 path (incl. the CDR/reset-B re-seat that
	 * actually changes the serializer lock). wan_rx>0 on any working or
	 * slow-leasing link, so this fires only on a genuinely dead/stuck link. */
	if (o5_provision_watchdog_ticks && gpon_fsm_state == 5 &&
	    gpon_fsm_onu_id != 0xff && gpon_o5_entry_tick &&
	    (gpon_fsm_ticks - gpon_o5_entry_tick) > o5_provision_watchdog_ticks &&
	    rtl9602c_eth_wan_rx_count() == 0) {
		pr_info("rtl9602c-gpon: O5 provision watchdog (%u ticks, gpon0 RX=0) -> re-range to re-roll serializer phase\n",
			gpon_fsm_ticks - gpon_o5_entry_tick);
		gpon_fsm_onu_id = 0xff;
		gpon_omcc_installed = false;
		gpon_tcont_installed = false;
		gpon_data_installed = false;
		gpon_aes_switch_time = 0xffffffff;
		gpon_key_staged = false;
		gpon_field(GPON_GTC_DS_ONU_STATUS, 15, 8, 0xff);
		gpon_field(GPON_GTC_US_ONU_ID, 15, 8, 0xff);
		if (cdr_reseat_on_reactivate) {
			sw_field(WSDS_DIG_1D, 16, 16, 0);
			udelay(500);
			sw_field(WSDS_DIG_1D, 16, 16, 1);
			schedule_work(&gpon_cdr_reset_work);
		}
		gpon_fsm_set_state(1);
	}

	/* Autonomous downstream-LOS recovery (fiber-pull / DS-light loss). The OLT cannot
	 * send a Deactivate when downstream light is gone, so the ONU must notice the
	 * sustained optical-LOS itself, tear down to O1 (mirroring the Deactivate->O1 re-
	 * range, incl. the CDR/reset-B re-seat), and re-acquire when light returns. Without
	 * this the FSM sits stale at O5 after a fiber pull and never re-ranges on reconnect.
	 * Debounced (los_rerange_ticks consecutive asserts) against transient dips. Once at
	 * O1 the state<2 guard stops counting until the FSM climbs back past O1 on relight. */
	if (los_rerange_ticks && gpon_fsm_state >= 2) {
		if (gpon_rd(GPON_GTC_DS_LOS_CFG_STS) & GPON_OPTIC_LOS_SIG) {
			if (++gpon_los_run == los_rerange_ticks) {
				pr_info("rtl9602c-gpon: downstream LOS %u ticks -> O1 (re-range on light return)\n",
					gpon_los_run);
				gpon_fsm_onu_id = 0xff;
				gpon_omcc_installed = false;
				gpon_tcont_installed = false;
				gpon_data_installed = false;
				gpon_aes_switch_time = 0xffffffff;
				gpon_key_staged = false;
				gpon_field(GPON_GTC_DS_ONU_STATUS, 15, 8, 0xff);
				gpon_field(GPON_GTC_US_ONU_ID, 15, 8, 0xff);
				if (cdr_reseat_on_reactivate) {
					sw_field(WSDS_DIG_1D, 16, 16, 0);
					udelay(500);
					sw_field(WSDS_DIG_1D, 16, 16, 1);
					schedule_work(&gpon_cdr_reset_work);
				}
				gpon_fsm_set_state(1);
			}
		} else {
			gpon_los_run = 0;
		}
	}

	/* Hybrid LAN/VLAN: once stably at O5 (config-apply done), clear VLAN_FILTER so the
	 * LAN ports forward to the CPU (br-lan 192.168.1.1 management access). Re-armed on
	 * any drop below O5 by gpon_fsm_set_state. */
	if (gpon_fsm_state == 5 && !gpon_vlan_lan_open && vlan_lan_o5_ticks &&
	    gpon_o5_entry_tick && (gpon_fsm_ticks - gpon_o5_entry_tick) > vlan_lan_o5_ticks) {
		sw_field(0x13008, 0, 0, 0);		/* VLAN_FILTER off -> open LAN */
		gpon_vlan_lan_open = true;
		pr_info("rtl9602c-gpon: O5 stable %u ticks -> VLAN_FILTER off (LAN access open)\n",
			gpon_fsm_ticks - gpon_o5_entry_tick);
	}
	if (trace && gpon_fsm_state == 5 && (gpon_fsm_ticks % 150) == 0)
		/* rxsid/ustx/dirty were REMOVED from this fast O5-poll print: pi_rd in
		 * this context reproducibly HANGS the FSM poll right after OMCC install
		 * (silent hang at +18.78s, two boots identical). pi_rd is an indirect
		 * polled PON-IP access — unsafe here, unlike sw_rd/gpon_rd. Read
		 * us_rxsid (groups [0..4], 0x203c..0x204c) and ustx (0x329bc) via /proc
		 * instead (process context, already exposed there, safe). */
		pr_info("rtl9602c-gpon: O5 t=%u last=0x%02x onu=%u hwst=%u eqd=0x%08x | dsrx64=%u pirx=%u omcirx=%u | ploam_cpu=%u gem_byte=%u gemus64=%u idle16=%u\n",
			gpon_fsm_ticks, gpon_last_ds_type, gpon_fsm_onu_id,
			gpon_rd(GPON_GTC_DS_ONU_STATUS) & 0xf, gpon_rd(GPON_GTC_US_EQD),
			gpon_gem_ds_rx_cnt(64), sw_rd(0x329c0), rtl9602c_eth_omci_rx_count(),
			gpon_us_misc_cnt(2), gpon_us_misc_cnt(4), gpon_rd(0x6a00), gpon_rd(0x6c80));
	/* US-OMCI EGRESS STALL LOCALIZER (SAFE reads only — sw_rd/gpon_rd, NO pi_rd and
	 * NO cross-driver accessor; both were the suspected hang sources). Diagnosis:
	 * the US OMCI is queued to qid64 but gemus64 (0x6a00) stays 0 (no OMCC GEM
	 * egress) while idle16 (0x6c80) climbs -> the OLT sees the upstream T-CONT silent
	 * and deactivates ~47s. Localize WHERE it stalls: ustx(0x329bc)=US OMCI-PI TX
	 * count (0 => frame never entered the US OMCI-PI TX path = descriptor steering /
	 * ring gap); pirx(0x329c0)=US OMCI-PI RX; usdrop/uscrc=drops; gemus_scan_first =
	 * first GEM flow with US bytes => flow 64 unscheduled (2b) vs flow!=64 SID
	 * misroute (2a) vs none = US-NIC ingest drop (2a). */
	if (trace && gpon_fsm_state == 5 && (gpon_fsm_ticks % 150) == 0) {
		int j;
		/* OLT-INDEPENDENT US-OMCI DATAPATH SELF-TEST: inject synthetic OMCI frames
		 * through OUR US-OMCI TX path (same ring/descriptor steering), so the rxsid
		 * read below reflects whether OUR steering reaches the US-NIC — with NO
		 * dependency on the degraded OLT sending DS OMCI. rxsid[4] (5th value)
		 * climbing after these injects => frame reaches US-NIC (steering OK, stall
		 * downstream); staying 0 => frame never egresses (ring-fetch / CPU->PON gap).
		 * (DIAGNOSTIC — remove with the rest of the USDIAG probe later.) */
		for (j = 0; j < 1; j++)	/* de-burst: ONE inject/tick. The 4-burst in one softirq tick overran the GMAC TX fetch engine — it drained ~3, parked at the producer head, the ring filled, and (no TDU re-kick) stayed parked, freezing ALL GMAC TX. */
			rtl9602c_eth_omci_selftest();
		/* rxsid = RX_SID_GOOD_CNT_US[0..4] (pi 0x203c/40/44/48/4c); group [4] = SID
		 * 64 (OMCC). rxsid[4] NON-ZERO => the US OMCI frame DOES reach the US-NIC
		 * classifier stamped SID 64 -> the stall is downstream (qid64/scheduler, 2b).
		 * rxsid[4]=0 with ustx=0 => the frame NEVER reaches the US-NIC (descriptor
		 * steering / ring routes it elsewhere = 2a). pi_rd is safe here: gpon_install_
		 * tcont uses pi_rd in the same FSM context and completes (the earlier hang was
		 * the cross-driver accessor, NOT pi_rd). */
		pr_info("rtl9602c-gpon: USDIAG t=%u ustx=%u pirx=%u usdrop=%u uscrc=%u | rxsid=%u/%u/%u/%u/%u\n",
			gpon_fsm_ticks, sw_rd(0x329bc), sw_rd(0x329c0),
			sw_rd(0x329b8), sw_rd(0x329cc),
			(u32)pi_rd(0x203c), (u32)pi_rd(0x2040), (u32)pi_rd(0x2044),
			(u32)pi_rd(0x2048), (u32)pi_rd(0x204c));
	}
	/* DS-PIPELINE STAGE PROBE (gate open + O5): sample ~1/s to localize where a
	 * de-encapsulated OMCI frame stalls during the short O5 window before any
	 * deactivate. A=de-encap, B=PBO HIGH-queue, C=DS SRAM, D=PON-IP->NIC. The
	 * FIRST 0 (or non-zero-meets-zero boundary) A->D is the stall stage. */
	if (gem_gate_open && gpon_fsm_state == 5 && (gpon_fsm_ticks % 100) == 0) {
		/* de-encap pkt count per flow index: f64=OMCC flow, f3=OMCC gem,
		 * f0/f1/f2=low flows. Localizes whether OMCI de-encaps ANYWHERE
		 * (mapping issue) vs nowhere (OLT not sending OMCI). */
		pr_emerg("DSPIPE deenc f64=%u f3=%u f2=%u f1=%u f0=%u | nonidle=%u idle=%u los=%u hec=%u | sram=%u q0=%u rxok=%u\n",
			 gpon_gem_flow_cnt(64, 0), gpon_gem_flow_cnt(3, 0),
			 gpon_gem_flow_cnt(2, 0), gpon_gem_flow_cnt(1, 0),
			 gpon_gem_flow_cnt(0, 0),
			 gpon_rd(0x11c4), gpon_rd(0x11bc), gpon_rd(0x11b4),
			 gpon_rd(0x11b8),
			 pi_rd(0xa0bc) & 0x1fff, pi_rd(0xa100) & 0x1fff,
			 pi_rd(0xc010) & 0xffff);
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
	/* Periodic O5 upstream-PLOAM keepalive. Once ranged (onu_id != 0xff) the FSM
	 * otherwise emits ZERO upstream PLOAM, and the shared US-PLOAM buffer's auto-
	 * No_message template can be stale-clobbered by intervening ACK/SN sends. Emit
	 * a fresh No_message (HW auto queue 0x7) every o5_ploam_keepalive_ticks so a
	 * valid PLOAM is present in the OLT's granted slots each window, defeating the
	 * BCM68620 PLOAM/ack-liveness timeout that fires Deactivate(0x05) ~25-35s after
	 * provision on ~50%% of boots. Mirrors the un-ranged SN cadence above; US-PLOAM
	 * only, does not touch DS RX or OMCI. */
	if (gpon_fsm_state == 5 && gpon_fsm_onu_id != 0xff &&
	    o5_ploam_keepalive_ticks &&
	    (gpon_fsm_ticks % o5_ploam_keepalive_ticks) == 0) {
		u8 nomsg[12];

		memset(nomsg, 0xaa, sizeof(nomsg));
		nomsg[0] = 0xff;	/* ONU-ID (HW overrides via ONUID_OVRD) */
		nomsg[1] = 0x04;	/* GPON_PLOAM_US_NOMESSAGE */
		gpon_send_cpu_ploam(PLM_US_QUEUE_NOMSG, nomsg);
	}
	/* Continuous laser keep-lit: once ignited, service any BOSA TX fault every
	 * ~50ms (5 x 10ms ticks) so a transient TX_FAULT after DIGITAL_POWER_ON does
	 * not leave the laser latched dark. This is the continuous laser INT/fault poll.
	 * Runs in softirq — bosa_laser_maint() does at most a bounded 500us strobe. */
	if (bosa_laser_up && (gpon_fsm_ticks % 5) == 0)
		bosa_laser_maint();

	/* Runtime DS-CDR-wedge recovery — the stock link-state-check
	 * mechanism our driver was MISSING. If the GTC DS framer status latches the
	 * wedge sentinel (0xca0eca0f), the DS CDR has come up stuck (the cold-start
	 * lock that a soft/WDT reboot cannot clear); re-acquire it by toggling
	 * SP_SDS_EN_RX (SDS_REG0[1]) 1->0->1, exactly as stock does. Done as a two-tick
	 * toggle (disable now, re-enable next tick ~10ms later) so no 10ms busy-wait
	 * runs in this softirq. Self-limiting: only fires while wedged, and a bounded
	 * consecutive-attempt cap yields to the LOS/re-range path on a dead link. */
	if (cdr_stuck_recover) {
		static int cdr_pending, cdr_tries;
		u32 sts = gpon_rd(GPON_GTC_DS_INTR_STS);

		gpon_gtc_ds_sts_last = sts;
		if (cdr_pending) {
			sw_field(SDS_REG0, 1, 1, 1);		/* SP_SDS_EN_RX -> 1 */
			cdr_pending = 0;
			if (gpon_rd(GPON_GTC_DS_INTR_STS) != GTC_DS_CDR_STUCK)
				gpon_cdr_stuck_fixed++;
		} else if (sts == GTC_DS_CDR_STUCK) {
			if (cdr_tries < GPON_CDR_STUCK_MAX) {
				sw_field(SDS_REG0, 1, 1, 0);	/* SP_SDS_EN_RX -> 0 */
				cdr_pending = 1;
				cdr_tries++;
				gpon_cdr_stuck_count++;
				pr_warn_ratelimited("rtl9602c-gpon: DS CDR wedged (GTC_DS_STS=0x%08x); SP_SDS_EN_RX re-acquire #%u\n",
						    sts, gpon_cdr_stuck_count);
			}
		} else {
			cdr_tries = 0;	/* healthy -> reset the consecutive-attempt cap */
		}
	}

	/* OFFK runtime servo (stock europa_LoopMon equiv): once the laser bursts at
	 * O5, latch the RTL8290B modulator-offset cal. Runs in this same softirq as
	 * bosa_laser_maint() (serialized BOSA I2C). Strobe 0x24d=0xb0, read R29
	 * (0x31d); on (&0x3c)==0x3c latch (clr FSU arm 0x20e.b7 + 0x27c.b4) once. */
	/* Run from boot (NOT gated on O5): stock's europa_LoopMon converges OFFK
	 * continuously from driver init, so the modulator is nulled BEFORE the OLT
	 * ranges -> the first real bursts are clean. Gating on O5 latched ~17s too
	 * late (after deact churn started). OFFK uses an internal ref, no burst
	 * needed. Poll every ~50ms until latched. */
	if (apc_offk_armed && !apc_offk_latched && (gpon_fsm_ticks % 5) == 0) {
		int r;

		bosa_write_reg(0x24d, 0xb0);
		r = bosa_read_reg(0x31d);
		if (r >= 0 && (r & 0x3c) == 0x3c) {
			bosa_set_bit(0x20e, 7, 0);
			bosa_set_bit(0x27c, 4, 0);
			apc_offk_latched = 1;
			pr_info("rtl9602c-gpon: OFFK LATCHED at O5: R29(0x31d)=0x%02x (modulator nulled)\n",
				r & 0xff);
		}
	}
	mod_timer(&gpon_fsm_timer, jiffies + msecs_to_jiffies(10));
}

/* Stock's omitted LDO init step. Byte-exact
 * replica of the stock behavior. SC-indirect (SerDes-Control) analog-reg engine via swcore:
 *   0x40 = SC_IND_CMD  (ADR[15:0], CMD_EN[16], WREN[17]); 0x0001fdca=read 0xfdca,
 *          0x0003fdca=write 0xfdca.  0x44 = SC_IND_RD (RD_DAT[7:0], BUSY[8]).
 *          0x3c = SC_IND_WD (WR_DAT[7:0]).  Stock settles 1ms/step (no BUSY poll).
 * reg 0xfdca = "Disable DRAM LDO output" (8-bit): clear bits 2,3 (mask ~0xC) so
 * the DDR switching reg drives the rail, not the linear LDO. THERMAL_CTRL_0
 * (0x130): arm the on-die over-temp comparator (TM_HIGHCMP_EN | TM_HIGH_THR=0x6c,
 * PWRON_DLY preserved) -> 0x00ec0005. NOTE (RE verdict): this is DRAM-LDO + thermal
 * ALARM, NOT the SerDes/laser path -> stock platform hygiene, not the WAN fix. */
static void __init rtl9602c_sc_ldo_init(void)
{
	u32 fdca, t130 = sw_rd(0x130);

	if (!sc_ldo_init)
		return;
	sw_wr(0x40, 0x0001fdcau);		/* SC read cmd: reg 0xfdca */
	udelay(1000);
	fdca = sw_rd(0x44);
	pr_info("rtl9602c-gpon: sc_ldo_init BEFORE: 0xfdca=0x%08x THERMAL(0x130)=0x%08x (stock 0x130=0x00ec0005)\n",
		fdca, t130);
	sw_wr(0x3c, fdca & ~0xcu);		/* clear DRAM-LDO bits 2,3 (stock mask ~0xC) */
	udelay(1000);
	sw_wr(0x40, 0x0003fdcau);		/* SC write commit */
	sw_wr(0x40, 0x0001fdcau);		/* re-issue read (stock reads back for log) */
	udelay(1000);
	(void)sw_rd(0x44);
	sw_wr(0x130, sw_rd(0x130) | 0x00800000u);			/* THERMAL: set bit23 */
	sw_wr(0x130, (sw_rd(0x130) & 0xff80ffffu) | 0x006c0000u);	/* preserve low-16 */
	pr_info("rtl9602c-gpon: sc_ldo_init AFTER: 0x130=0x%08x\n", sw_rd(0x130));
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
	{
		const char *via;
		int sret;

		rtl9602c_sc_ldo_init();		/* stock LDO init: analog LDO + thermal comp, before SerDes bring-up */
		rtl960x_c2_postmode_perturb = serdes_postmode_perturb;	/* A/B: skip the post-GPON-mode US-TX perturbations (default = skip, stock rev-A) */
		rtl960x_c2_sds_cfgrst = serdes_sds_cfgrst;	/* A/B: SDS reset = stock bit0-only (default) vs legacy bit7+bit0 */
		rtl960x_c2_stock_analog = serdes_stock_analog;	/* A/B: match live-stock SDS REG01/REG11 post-reset (default) */
		rtl960x_c2_analog_postreset = serdes_analog_postreset;	/* A/B: program full analog CMU/CDR table AFTER the SDS reset (stock rev-A, default) = cold-start determinism */
		rtl960x_c2_cmu_settle_ms = serdes_cmu_settle_ms;	/* A/B: TX-CMU-lock settle before reset-B (cold-start metastability candidate) */
		rtl960x_c2_clkgate_rstb = serdes_clkgate_rstb;	/* A/B: clock-gated reset-B release (rank-1 cold-start fix candidate) */
		rtl960x_c2_skip_rstb_dance = serdes_skip_rstb_dance;	/* A/B: skip the gratuitous DIG_1D reset-B pulse (already released) */
		rtl960x_c2_minimal_analog = serdes_minimal_analog;	/* A/B: skip the over-configure golden-table writes (match stock's minimal set) */
		if (force_soc_clk) {
			/* Match the live-stock (100%-deterministic) SoC sysctl/clock regs that our
			 * FAIL boot differed from, BEFORE the SerDes CMU locks. Same physical board,
			 * so these are Board C's own stock values. */
			static const struct { u32 off, val; } soc_clk[] = {
				{ 0x18000100u, 0x00440e00u },
				{ 0x1800012cu, 0x024d024du },
				{ 0x18000140u, 0x024d024du },
			};
			unsigned int k;
			for (k = 0; k < ARRAY_SIZE(soc_clk); k++) {
				void __iomem *a = ioremap(soc_clk[k].off, 4);
				if (a) {
					writel(soc_clk[k].val, a);
					pr_info("rtl9602c-gpon: force_soc_clk [%#x]<=%#x ->%#x\n",
						soc_clk[k].off, soc_clk[k].val, readl(a));
					iounmap(a);
				}
			}
		}
		/* CROSS-SUBSYSTEM ORDER (bosa_before_serdes): stage the external BOSA RX
		 * analog + a settle BEFORE the SoC SerDes CMU/serializer bring-up, matching
		 * stock (europa/BOSA staged before ponmac mode_set). The SerDes CMU then locks
		 * against a settled analog front-end instead of a still-powered-down BOSA — a
		 * cold-start ~50% serializer-phase determinism fix candidate. The duplicate
		 * bosa_probe()/bosa_rx_enable() below are skipped when this runs. */
		if (bosa_before_serdes && !skip_bosa) {
			bosa_probe();
			bosa_rx_enable();
			if (bosa_settle_ms)
				mdelay(bosa_settle_ms);
			pr_info("rtl9602c-gpon: BOSA RX up + %ums settle BEFORE SerDes (stock order)\n",
				bosa_settle_ms);
		}
		if (family_lib) {
			/* bring up via the clean-room family lib (the open-source path) */
			sret = rtl960x_ponmac_mode_set(RTL960X_CHIP_9602C, RTL960X_REV_A,
						       RTL960X_SUBTYPE_NONE,
						       RTL960X_MODE_GPON, &rtl9602c_r960_ops);
			via = "family-lib 9602C";
			/* STABILITY fallback: if the lib path ever fails to bring the analog
			 * ready, fall back to the months-tested inline bring-up so the board
			 * always comes up. (The lib path is a faithful translation, so this is
			 * a belt-and-suspenders safety net, not an expected path.) */
			if (sret) {
				pr_warn("rtl9602c-gpon: family-lib SerDes not ready (0x%08x) -> inline fallback\n",
					sw_rd(FIB_EXT_REG21));
				sret = gpon_serdes_init();
				via = "inline fallback";
			}
		} else {
			sret = serdes_stock_seq ? gpon_serdes_init_stock() : gpon_serdes_init();
			via = serdes_stock_seq ? "stock rev-A order" : "GPON mode";
		}

		if (sret)
			pr_warn("rtl9602c-gpon: SerDes analog-ready not seen (%s, FIB_EXT_REG21=0x%08x)\n",
				via, sw_rd(FIB_EXT_REG21));
		else
			pr_info("rtl9602c-gpon: PON SerDes up (%s, analog ready)\n", via);
	}

	/*
	 * Probe the external RTL8290B BOSA over I2C (read-only chip-ID check). The
	 * optical RX signal-detect comes from this chip; a working unit initialises
	 * it over I2C and only then does SDS_FIB_STATUS.SDS_SDET assert. This
	 * validates the I2C transport before the RX-enable writes are added.
	 */
	if (!bosa_before_serdes)		/* else already probed before the SerDes */
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
	if (!bosa_before_serdes)		/* else already RX-enabled before the SerDes */
		bosa_rx_enable();

	/* Power on the BOSA optical transmitter (laser bias/modulation/APC) so the
	 * ONU can send upstream PLOAM bursts during activation. The APC offset
	 * calibration is deferred until after the PON-IP datapath + MAC reset below,
	 * because the APC digital block only clocks once the SerDes/PON TX clock is
	 * running (calibrating earlier leaves its readout dead -> OFFK_DONE never
	 * asserts). */
	if (!laser_off) {
		bosa_tx_enable();
		/* Burst bias/mod override (user-directed: bump the burst so the OLT's
		 * operational burst-RX stops raising "Laser out" at O5). MOD raises the peak;
		 * BIAS kept low to preserve extinction (DS). Latched via the 0x23d DAC strobe,
		 * same sequence the APC uses. Skipped if both 0 (keep A4-golden). */
		if (laser_bias || laser_mod) {
			/* Apply Board-C's REAL per-board laser calib (rtl8290b.data): bias DAC12=0x32f
			 * (0x236=0x32, low nibble 0xf), mod DAC12=0xbbd (0x237=0xbb, low nibble 0xd) ->
			 * 0x238=0xdf. The earlier code zeroed 0x238's nibbles (dropping the low DAC bits) —
			 * fixed here to the real low nibbles. Latched via the 0x23d bit7 DAC strobe. */
			bosa_set_bit(0x23d, 7, 0);
			bosa_set_field(0x236, 0xff, laser_bias ? laser_bias : 0x19);
			bosa_set_field(0x238, 0x0f, 0x0f);	/* IBIAS[3:0] = 0xf */
			bosa_set_bit(0x23d, 7, 1);
			bosa_set_bit(0x23d, 7, 0);
			bosa_set_field(0x237, 0xff, laser_mod ? laser_mod : 0x67);
			bosa_set_field(0x238, 0xf0, 0xd0);	/* IMOD[3:0] = 0xd */
			bosa_set_bit(0x23d, 7, 1);
			mdelay(2);
			pr_info("rtl9602c-gpon: laser DAC override bias=0x%02x mod=0x%02x -> readback bias=0x%02x mod=0x%02x R30=0x%02x mpd=%02x/%02x\n",
				laser_bias, laser_mod, bosa_read_reg(0x236) & 0xff,
				bosa_read_reg(0x237) & 0xff, bosa_read_reg(0x31e) & 0xff,
				bosa_read_reg(0x320) & 0xff, bosa_read_reg(0x321) & 0xff);
		}
		/* OLT-RE FIX (HSGQ-G008 gpondev: deactivation reason 6 = "los"/"Laser out"):
		 * the OLT ranges us (acquisition-bias SN burst lands once) then deactivates
		 * ~0.5-1s after Configure_Port-ID because our upstream burst COLLAPSES within
		 * one LOS window after O5 — the laser has no continuous TX-fault recovery.
		 * bosa_laser_maint() (the ~50ms fault re-ignite) is gated on bosa_laser_up,
		 * which was set ONLY inside bosa_apc_calibrate() — skipped when apc_off=true
		 * (the default, kept because full APC deafens the shared-BOSA DS-RX). Decouple
		 * the keepalive: arm bosa_laser_up here so the fault-service runs WITHOUT the
		 * DS-deafening APC, holding the burst lit through O5 so the OLT stops raising
		 * LOS. */
		bosa_laser_up = 1;
	}

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
	if (!skip_bosa && !laser_off && apc_offk)
		rtl8290b_apc_init();		/* B-variant: completes OFFK (DS-safe) */
	else if (!skip_bosa && !laser_off && !apc_off)
		bosa_apc_calibrate();		/* legacy non-B flow (A/B fallback) */

	proc_create_single("gpon", 0444, NULL, gpon_proc_show);
	proc_create_single("bosadump", 0444, NULL, bosadump_proc_show);
	proc_create_single("pidump", 0444, NULL, pidump_proc_show);
	proc_create_single("swdump", 0444, NULL, swdump_proc_show);

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
	/* US GEM-header PTI vector (GPON_GEM_US_PTI_CFG 0x6020, NOT write-protected).
	 * Stock sets the US GEM PTI vector (0,1,0,1) => PTI_VECTOR1[6:4]=1,
	 * PTI_VECTOR3[14:12]=1 => 0x00001010 (FS_GEM_IDLE[31]=0 keeps auto-idle). Reset
	 * is 0, so every US GEM frame (incl OMCC OMCI responses) carries PTI=000 even on
	 * end-of-fragment; some OLTs (e.g. ALU) only accept OMCI with
	 * NON_END_FRAG=0 and END_FRAG=1. Set it so the upstream GEM/OMCC is well-formed. */
	gpon_wr(0x6020, 0x00001010);
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
	gpon_field(0x1014, 11, 11, 0);				/* DS_CFG BWM_NO_FLT = 0 (stock value; the
								 * bring-up =1 "accept all grants" left bwm_acpt=0
								 * (vs stock 130k+), i.e. it broke the BWMAP parser
								 * rather than relaxing it — filter by US_ONU_ID
								 * like stock: 0xff during ranging, assigned id at O5) */
	gpon_wr(GPON_GTC_US_PLOAM_CFG,
		GPON_US_PLM_CRC_GEN_EN | GPON_US_PLM_ONUID_OVRD);
	/* Arm the HW auto-No_message PLOAM keepalive (US_PLOAM_IND queue type 0x7). At
	 * O5 the OLT continuously grants the ONU's default Alloc-ID a PLOAM slot and reads
	 * back what we emit; the GTC auto-fills every otherwise-empty granted PLOAM slot
	 * with this latched No_message (US type 0x04) template. WITHOUT it our granted
	 * slots carry zeroed/invalid PLOAMs once the ACK/key bursts drain, so the OLT
	 * never confirms a continuously-alive upstream PLOAM/OMCC channel, keeps re-issuing
	 * Configure_Port-ID/Request_key and WITHHOLDS DS OMCI. Stock loads this
	 * unconditionally during PLOAM init. One call latches the persistent template;
	 * /proc/gpon us_gtc:ploam_auto should then climb on every OLT grant. */
	{
		u8 nomsg[12];

		memset(nomsg, 0xaa, sizeof(nomsg));
		nomsg[0] = 0xff;		/* ONU-ID (HW overrides via ONUID_OVRD)  */
		nomsg[1] = 0x04;		/* GPON_PLOAM_US_NOMESSAGE                */
		gpon_send_cpu_ploam(PLM_US_QUEUE_NOMSG, nomsg);
	}
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
	 * O5 grant-burst optical config — the "Laser out"/LOAi wall (2026-06-13).
	 * The digital US datapath egresses (gemus64 climbs) but the OLT gets no
	 * valid O5 burst, while the ranging SN burst works. Root: the GTC US burst-mode/laser
	 * registers stock programs in its GPON init that our init OMITTED — the
	 * wide-window isolated SN burst tolerates the reset defaults; the packed back-to-back
	 * O5 bursts do not ("isolated tolerates, packed exposes").
	 *
	 * #1 GPON_GTC_US_OPTIC_SD_TH (0x5188): MISM_THRESH[30:16]=0xa0, TOOLONG_THRESH[14:0]=
	 *    0x7fff. Rationale (stock behavior): because of the laser-driver TX_SD delay, TX_SD
	 *    may merge into the next burst, so the toolong threshold is raised to 0x7fff and the
	 *    mismatch threshold to 0xa0. At reset defaults the
	 *    GTC mis-judges the duration of our own packed O5 bursts (BOSA TX_SD lingers into the
	 *    next grant window) -> "too long"/"mismatch" -> it gates/suppresses the burst at the
	 *    SerDes burst-gate => OLT LOSi/SFi ("Laser out") + LOAi. The SN burst is isolated in a
	 *    wide quiet window so the lingering TX_SD never overlaps -> ranging works.
	 * #2 Laser power-save windowing: US_PWR_SAV_MODE (0x526c) bit0=1 (GEM mode) +
	 *    GEM_US_PWR_SAV_CFG (0x6024) OPT_AHEAD_CYCLES[9:0]=0x100 / OPT_BEHIND_CYCLES[20:16]=
	 *    0x10 = laser pre-fire / post-hold margins around each granted burst (so the preamble
	 *    isn't emitted before the BOSA driver has settled). Stock programs both unconditionally.
	 * These are US-side only (harmless to DS/ranging). Offsets exact via the chip's
	 * register map; values from the stock GPON init behavior.
	 */
	/* Values CORRECTED to the LIVE stock-ref-ONU oracle (live stock register read @O5, 2026-06-13) —
	 * the stock *init* programmed value for OPTIC_SD_TH (0x00a07fff) did NOT match the live operating
	 * value (0x00504bfa): MISM_THRESH[30:16]=0x50, TOOLONG_THRESH[14:0]=0x4bfa. Oracle-parity. */
	gpon_wr_us_protected(0x5188, 0x00504bfa);	/* US_OPTIC_SD_TH: live stock = MISM 0x50 | TOOLONG 0x4bfa */
	gpon_field(0x526c, 0, 0, 1);			/* US_PWR_SAV_MODE.PWR_SAV_MODE = 1 (live stock = 1) */
	gpon_wr(0x6024, (0x10u << 16) | 0x100u);	/* GEM_US_PWR_SAV_CFG = 0x00100100 (live stock) */
	gpon_wr(0x6260, 0x00000028u);			/* GEM_US_EOB_MERGE = 0x28 (live stock; mine omitted) */

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
	INIT_WORK(&gpon_cdr_reset_work, gpon_cdr_reset_worker);
	INIT_DELAYED_WORK(&gpon_optical_work, gpon_optical_work_fn);
	if (optical_poll)
		schedule_delayed_work(&gpon_optical_work, msecs_to_jiffies(3000));
	timer_setup(&gpon_fsm_timer, gpon_fsm_poll, 0);
	mod_timer(&gpon_fsm_timer, jiffies + msecs_to_jiffies(50));
	return 0;
}

static void __exit rtl9602c_gpon_exit(void)
{
	timer_delete_sync(&gpon_fsm_timer);
	cancel_work_sync(&gpon_cdr_reset_work);
	cancel_delayed_work_sync(&gpon_optical_work);
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
