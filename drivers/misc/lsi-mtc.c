/*
 *  Copyright (C) 2013 LSI Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  Device driver for LSI Master Test Controller (MTC), which is a test
 *  generator that is fully compliant with IEEE 1149.1 and can run JTAG test
 *  sequences on external devices. The device is accessed via a character
 *  device (/dev/mtc) through which test sequences can be loaded and exececuted
 *  by the controller.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include <linux/atomic.h>
#include <linux/io.h>
#include <linux/string.h>
#include "linux/lsi_mtc_ioctl.h"


/*
   device tree node:

   mtc@2010098000 {
	   compatible = "lsi,mtc";
	   reg = <0x20 0x10098000 0 0x3000>;
	   interrupts = <0 45 4>;
   };

 */

/* MTC registers */
struct mtc_regs {
	u32 params;		/* 0x00 */
	u32 scratch0;		/* 0x04 */
	u32 scratch1;		/* 0x08 */
	u32 reserved_0c[1];	/* 0x0c */
	u32 config0;		/* 0x10 */
	u32 config1;		/* 0x14 */
	u32 reserved_18[2];	/* 0x18..0x1c */
	u32 status1;		/* 0x20 */
	u32 status2;		/* 0x24 */
	u32 reserved_28[2];	/* 0x28..0x2c */
	u32 execute;		/* 0x30 */
	u32 reserved_34[3];	/* 0x34..0x3c */
	u32 mem_init;		/* 0x40 */
	u32 ecc_disable;	/* 0x44 */
	u32 ecc_invert_en;	/* 0x48 */
	u32 ecc_invert;		/* 0x4c */
	u32 int_status;		/* 0x50 */
	u32 int_enable;		/* 0x54 */
	u32 int_force;		/* 0x58 */
	u32 ecc_int_status;	/* 0x5c */
	u32 ecc_int_enable;	/* 0x60 */
	u32 ecc_int_force;	/* 0x64 */
	u32 reserved_68[6];	/* 0x68..0x7c */
	u32 debug0;		/* 0x80 */
	u32 debug1;		/* 0x84 */
	u32 debug2;		/* 0x88 */
	u32 debug3;		/* 0x8c */
	u32 debug4;		/* 0x90 */
	u32 debug5;		/* 0x94 */
};

#define MTC_PRGMEM_OFFSET 0x1000
#define MTC_TDOMEM_OFFSET 0x2000
#define MTC_PRGMEM_SIZE 256	/* program memory size in words */
#define MTC_TDOMEM_SIZE 256	/* tdo memory size in words */

#ifdef __MTC_SIMULATION
struct mtc_regs _mtc_regs;
u32 _mtc_tdomem[256] = { 0x11110000, 0x22221111, 0x33332222 };
u32 *_mtc_prgmem = _mtc_tdomem;
#endif

/******************************************************/
/* register definitions generated from RDL */

/******************************************************/

   /* NODE 0x15d , TARGET 0xffffffff */

#define     NCP_AXIS_MTC_MTC_INST_PARAMS0_REG_ADDR              (0x00000000)
#define     NCP_AXIS_MTC_MTC_SCRATCH0_REG_ADDR                  (0x00000004)
#define     NCP_AXIS_MTC_MTC_SCRATCH1_REG_ADDR                  (0x00000008)
#define     NCP_AXIS_MTC_MTC_CONFIG0_REG_ADDR                   (0x00000010)
#define     NCP_AXIS_MTC_MTC_CONFIG1_REG_ADDR                   (0x00000014)
#define     NCP_AXIS_MTC_MTC_STATUS1_REG_ADDR                   (0x00000020)
#define     NCP_AXIS_MTC_MTC_STATUS2_REG_ADDR                   (0x00000024)
#define     NCP_AXIS_MTC_MTC_EXECUTE1_REG_ADDR                  (0x00000030)
#define     NCP_AXIS_MTC_MTC_MEM_INIT_REG_ADDR                  (0x00000040)
#define     NCP_AXIS_MTC_MTC_ECC_DISABLE_REG_ADDR               (0x00000044)
#define     NCP_AXIS_MTC_MTC_ECC_INVERT_EN_REG_ADDR             (0x00000048)
#define     NCP_AXIS_MTC_MTC_ECC_INVERT_REG_ADDR                (0x0000004c)
#define     NCP_AXIS_MTC_MTC_DEBUG0_REG_ADDR                    (0x00000080)
#define     NCP_AXIS_MTC_MTC_DEBUG1_REG_ADDR                    (0x00000084)
#define     NCP_AXIS_MTC_MTC_DEBUG2_REG_ADDR                    (0x00000088)
#define     NCP_AXIS_MTC_MTC_DEBUG3_REG_ADDR                    (0x0000008c)
#define     NCP_AXIS_MTC_MTC_DEBUG4_REG_ADDR                    (0x00000090)
#define     NCP_AXIS_MTC_MTC_DEBUG5_REG_ADDR                    (0x00000094)

#define     NCP_AXIS_MTC_MTC_PRGM_MEM_START_ADDR_BASE           (0x00001000)
#define     NCP_AXIS_MTC_MTC_PRGM_MEM_START_ADDR(n)      (0x00001000 + (4*(n)))
#define     NCP_AXIS_MTC_MTC_PRGM_MEM_START_ADDR_COUNT          (0x00000100)
#define     NCP_AXIS_MTC_MTC_PRGM_MEM_START_ADDR_IDX(addr) \
	(((addr) - NCP_AXIS_MTC_MTC_PRGM_MEM_START_ADDR_BASE) / 4)
#define     NCP_AXIS_MTC_MTC_PRGM_MEM_START_ADDR_END            (0x00001400)
#define     NCP_AXIS_MTC_MTC_PRGM_MEM_START_ADDR_RANGE(addr) \
	(((addr) >= NCP_AXIS_MTC_MTC_PRGM_MEM_START_ADDR_BASE) && \
	((addr) < NCP_AXIS_MTC_MTC_PRGM_MEM_START_ADDR_END))

#define     NCP_AXIS_MTC_MTC_TDO_CAPTURE_MEM_START_ADDR_BASE    (0x00002000)
#define   NCP_AXIS_MTC_MTC_TDO_CAPTURE_MEM_START_ADDR(n) (0x00002000 + (4*(n)))
#define     NCP_AXIS_MTC_MTC_TDO_CAPTURE_MEM_START_ADDR_COUNT   (0x00000100)
#define     NCP_AXIS_MTC_MTC_TDO_CAPTURE_MEM_START_ADDR_IDX(addr) \
	(((addr) - NCP_AXIS_MTC_MTC_TDO_CAPTURE_MEM_START_ADDR_BASE) / 4)
#define     NCP_AXIS_MTC_MTC_TDO_CAPTURE_MEM_START_ADDR_END     (0x00002400)
#define     NCP_AXIS_MTC_MTC_TDO_CAPTURE_MEM_START_ADDR_RANGE(addr) \
	(((addr) >= NCP_AXIS_MTC_MTC_TDO_CAPTURE_MEM_START_ADDR_BASE) && \
	((addr) < NCP_AXIS_MTC_MTC_TDO_CAPTURE_MEM_START_ADDR_END))

#define     NCP_AXIS_MTC_MTC_TSTGEN_INT_STATUS                  (0x00000050)
#define     NCP_AXIS_MTC_MTC_TSTGEN_INT_STATUS_ALARM_MASK       (0x0000007f)
#define     NCP_AXIS_MTC_MTC_TSTGEN_INT_EN                      (0x00000054)
#define     NCP_AXIS_MTC_MTC_TSTGEN_INT_EN_ALARM_MASK           (0x0000007f)
#define     NCP_AXIS_MTC_MTC_TSTGEN_INT_FRC                     (0x00000058)
#define     NCP_AXIS_MTC_MTC_TSTGEN_INT_FRC_ALARM_MASK          (0x0000007f)
#define     NCP_AXIS_MTC_MTC_ECC_INT_STATUS                     (0x0000005c)
#define     NCP_AXIS_MTC_MTC_ECC_INT_STATUS_ALARM_MASK          (0x0000000f)
#define     NCP_AXIS_MTC_MTC_ECC_INT_EN                         (0x00000060)
#define     NCP_AXIS_MTC_MTC_ECC_INT_EN_ALARM_MASK              (0x0000000f)
#define     NCP_AXIS_MTC_MTC_ECC_INT_FRC                        (0x00000064)
#define     NCP_AXIS_MTC_MTC_ECC_INT_FRC_ALARM_MASK             (0x0000000f)

/*! @struct ncp_axis_mtc_MTC_INST_PARAMS0_REG_ADDR_r_t
 *  @brief MTC Parameter Register
 *  @details null
 *  <table width="70%" align="center">
 *  <tr>
 *   <td colspan="4" align="center">
 *    struct ncp_axis_mtc_MTC_INST_PARAMS0_REG_ADDR_r_t\n
 *   </td>
 *  </tr>
 *  <tr>
 *     <td width="30%"><b> Name </b></td>
 *     <td width="20%" align="center"><b> Width </b></td>
 *     <td width="20%" align="center"><b> Start Offset </b></td>
 *     <td width="30%"><b> Description </b></td>
 *  </tr>
 *   <tr>
 *     <td width="30%"> @param reserved0 </td>
 *     <td width="20%" align="center"> 6 </td>
 *     <td width="20%" align="center"> 26 </td>
 *   <td width="30%"> Reserved for future use </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param tdo_memory_size </td>
 *     <td width="20%" align="center"> 10 </td>
 *     <td width="20%" align="center"> 16 </td>
 *   <td width="30%"> Size of TDO record memory in bytes. </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param reserved1 </td>
 *     <td width="20%" align="center"> 6 </td>
 *     <td width="20%" align="center"> 10 </td>
 *   <td width="30%"> Reserved for future use </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param tst_prgm_memory_size </td>
 *     <td width="20%" align="center"> 10 </td>
 *     <td width="20%" align="center"> 0 </td>
 *   <td width="30%"> Size of Test Program memory in bytes. </td>
 * </tr>
 * </table>
 * Applies to: 5500 --> ncp_axis_mtc_MTC_INST_PARAMS0_REG_ADDR_r_t
 *
 */

struct ncp_axis_mtc_MTC_INST_PARAMS0_REG_ADDR_r_t {
#ifdef NCP_BIG_ENDIAN
	unsigned reserved0:6;
	unsigned tdo_memory_size:10;
	unsigned reserved1:6;
	unsigned tst_prgm_memory_size:10;
#else	/* Little Endian */
	unsigned tst_prgm_memory_size:10;
	unsigned reserved1:6;
	unsigned tdo_memory_size:10;
	unsigned reserved0:6;
#endif
};

/*! @struct ncp_axis_mtc_MTC_SCRATCH0_REG_ADDR_r_t
 *  @brief MTC Scratch 0 Register
 *  @details null
 *  <table width="70%" align="center">
 *  <tr>
 *   <td colspan="4" align="center">
 *    struct ncp_axis_mtc_MTC_SCRATCH0_REG_ADDR_r_t\n
 *   </td>
 *  </tr>
 *  <tr>
 *     <td width="30%"><b> Name </b></td>
 *     <td width="20%" align="center"><b> Width </b></td>
 *     <td width="20%" align="center"><b> Start Offset </b></td>
 *     <td width="30%"><b> Description </b></td>
 *  </tr>
 *   <tr>
 *     <td width="30%"> @param scratchpad0 </td>
 *     <td width="20%" align="center"> 32 </td>
 *     <td width="20%" align="center"> 0 </td>
 *   <td width="30%"> SW read/write register - Not used by MTC hardware. </td>
 * </tr>
 * </table>
 * Applies to: 5500 --> ncp_axis_mtc_MTC_SCRATCH0_REG_ADDR_r_t
 *
 */

struct ncp_axis_mtc_MTC_SCRATCH0_REG_ADDR_r_t {
	unsigned int scratchpad0;
};

/*! @struct ncp_axis_mtc_MTC_SCRATCH1_REG_ADDR_r_t
 *  @brief MTC Scratch 1 Register
 *  @details null
 *  <table width="70%" align="center">
 *  <tr>
 *   <td colspan="4" align="center">
 *    struct ncp_axis_mtc_MTC_SCRATCH1_REG_ADDR_r_t\n
 *   </td>
 *  </tr>
 *  <tr>
 *     <td width="30%"><b> Name </b></td>
 *     <td width="20%" align="center"><b> Width </b></td>
 *     <td width="20%" align="center"><b> Start Offset </b></td>
 *     <td width="30%"><b> Description </b></td>
 *  </tr>
 *   <tr>
 *     <td width="30%"> @param scratchpad1 </td>
 *     <td width="20%" align="center"> 32 </td>
 *     <td width="20%" align="center"> 0 </td>
 *   <td width="30%"> SW read/write register - Not used by MTC hardware. </td>
 * </tr>
 * </table>
 * Applies to: 5500 --> ncp_axis_mtc_MTC_SCRATCH1_REG_ADDR_r_t
 *
 */

struct ncp_axis_mtc_MTC_SCRATCH1_REG_ADDR_r_t {
	unsigned int scratchpad1;
};

/*! @struct ncp_axis_mtc_MTC_CONFIG0_REG_ADDR_r_t
 *  @brief MTC Config 0 Register
 *  @details Configuration/Control Registers for the MTC block
 *  <table width="70%" align="center">
 *  <tr>
 *   <td colspan="4" align="center">
 *    struct ncp_axis_mtc_MTC_CONFIG0_REG_ADDR_r_t\n
 *   </td>
 *  </tr>
 *  <tr>
 *     <td width="30%"><b> Name </b></td>
 *     <td width="20%" align="center"><b> Width </b></td>
 *     <td width="20%" align="center"><b> Start Offset </b></td>
 *     <td width="30%"><b> Description </b></td>
 *  </tr>
 *   <tr>
 *     <td width="30%"> @param reserved0 </td>
 *     <td width="20%" align="center"> 16 </td>
 *     <td width="20%" align="center"> 16 </td>
 *   <td width="30%"> Reserved for future use </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param mtc_mpu_tdo_inactive_en </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 15 </td>
 *   <td width="30%"> 1) When Config mode Includes the AXM in the scan chain.
 *         1'b0 = MTC_TDO_ENB = 1'b1; TDO Output buffer always
 *        enabled.  1'b1 = MTC_TDO_ENB = JTC_TDO_ENB; TDO
 *        Output buffer follows the JTC control signal.  2)
 *        When Config mode Excludes the AXM in the scan chain.
 *        1'b0 = MTC_TDO_ENB = 1'b1; TDO Output buffer always
 *        enabled. 1'b1 = MTC_TDO_ENB = Active (1'b1) when in
 *        shift-DR or shift-IR states and inactive (1'b0) in
 *        all other states (tri-state capable).
 *   </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param loop_en </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 14 </td>
 *   <td width="30%"> 1'b1 = When an End-of-test command is received the
 *        test program will continue to execute from location
 *        0 of the memory instead of halting.   This assumes
 *        the test program fits within the 256 x 32 bit memory.
 *        1'b0 = In normal operation the MTC_TESTGEN block
 *        will loop through the 256 location program memory
 *        until an end-of-test command is detected.  Once
 *        detected the program will stop execution in its current
 *        state (Run/Test Idle, pause-IR, or pause-DR).
 *   </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param single_step_en </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 13 </td>
 *   <td width="30%"> 1'b1 = Single step mode is enabled;  1'b0 = Single
 *        step mode is disabled (ignore writes to single_step
 *        one-shot register  Note: In single step mode the
 *        TACP state machine cannot stop in the Select-DR-Scan
 *        state waitng for the user to write the single_step
 *        one-shot register.  Therefore, the end-state-bit[28]
 *        value is ignored in this mode and always pauses in
 *        run-test/idle state.
 *   </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param start_stopn </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 12 </td>
 *   <td width="30%"> 1'b1 = Run/Resume  1'b0 = Stop in current task </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param reserved1 </td>
 *     <td width="20%" align="center"> 3 </td>
 *     <td width="20%" align="center"> 9 </td>
 *   <td width="30%"> Reserved for future use </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param rate_sel </td>
 *     <td width="20%" align="center"> 5 </td>
 *     <td width="20%" align="center"> 4 </td>
 *   <td width="30%"> TCK clock rate based off the internal clk_per clock
 *        rate (nominal value 200 MHz)  5'd0, others = TCK
 *        = Disable  5'd1 = TCK = 20.00 MHz (/10)  5'd2
 *        = TCK = 16.67 MHz (/12)  5'd3 = TCK = 14.28 MHz
 *        (/14)  5'd4 = TCK = 12.50 MHz (/16)  5'd5 =
 *        TCK = 11.11 MHz (/18)  5'd6 = TCK = 10.00 MHz (/20)
 *         5'd7 = TCK = 9.09 MHz (/22)  5'd8 = TCK = 8.33
 *        MHz (/24)  5'd9 = TCK = 7.14 MHz (/28)  5'd10
 *        = TCK = 6.25 MHz (/32)  5'd11 = TCK = 5.00 MHz
 *        (/40)  5'd12 = TCK = 4.00 MHz (/50)  5'd13 =
 *        TCK = 3.00 MHz (/66)  5'd14 = TCK = 2.00 MHz (/100)
 *         5'd15 = TCK = 1.00 MHz (/200)  5'd16 = TCK
 *        = 0.50 MHz (/400)
 *   </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param reserved2 </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 3 </td>
 *   <td width="30%"> Reserved for future use </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param cfg_config_ctl </td>
 *     <td width="20%" align="center"> 3 </td>
 *     <td width="20%" align="center"> 0 </td>
 *   <td width="30%"> 0,others,illegal=Board Test Mode  1=MTC Internal
 *        Test Mode - Not Supported in AXM  2=MTC External
 *        Mode (AXM Included) - Not Supported in AXM  3=MTC
 *        External Mode (AXM NOT Included)  4=MTC System Test
 *        Mode (AXM Included) - Not Supported in AXM  5=MTC
 *        System Test Mode (AXM NOT Included)  Note: The MTC
 *        hardware inhibits entering modes identifed as Not
 *        Supported when mtc_axm_selftst_disable control bit
 *        = 1.  If a Not Supported mode is programmed the
 *        device will default to the Board Test Mode.
 *   </td>
 * </tr>
 * </table>
 * Applies to: 5500 --> ncp_axis_mtc_MTC_CONFIG0_REG_ADDR_r_t
 *
 */

struct ncp_axis_mtc_MTC_CONFIG0_REG_ADDR_r_t {
#ifdef NCP_BIG_ENDIAN
	unsigned reserved0:16;
	unsigned mtc_mpu_tdo_inactive_en:1;
	unsigned loop_en:1;
	unsigned single_step_en:1;
	unsigned start_stopn:1;
	unsigned reserved1:3;
	unsigned rate_sel:5;
	unsigned reserved2:1;
	unsigned cfg_config_ctl:3;
#else				/* Little Endian */
	unsigned cfg_config_ctl:3;
	unsigned reserved2:1;
	unsigned rate_sel:5;
	unsigned reserved1:3;
	unsigned start_stopn:1;
	unsigned single_step_en:1;
	unsigned loop_en:1;
	unsigned mtc_mpu_tdo_inactive_en:1;
	unsigned reserved0:16;
#endif
};

/*! @struct ncp_axis_mtc_MTC_CONFIG1_REG_ADDR_r_t
 *  @brief MTC Config 1 Register
 *  @details TDO Capture and TCK Gapped Clock Control Signals
 *  <table width="70%" align="center">
 *  <tr>
 *   <td colspan="4" align="center">
 *    struct ncp_axis_mtc_MTC_CONFIG1_REG_ADDR_r_t\n
 *   </td>
 *  </tr>
 *  <tr>
 *     <td width="30%"><b> Name </b></td>
 *     <td width="20%" align="center"><b> Width </b></td>
 *     <td width="20%" align="center"><b> Start Offset </b></td>
 *     <td width="30%"><b> Description </b></td>
 *  </tr>
 *   <tr>
 *     <td width="30%"> @param reserved0 </td>
 *     <td width="20%" align="center"> 26 </td>
 *     <td width="20%" align="center"> 6 </td>
 *   <td width="30%"> Reserved for future use </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param record_tdo_in_shift_ir_state </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 5 </td>
 *   <td width="30%"> 1'b1=Capture TDI Input data when in shift-IR state,
 *         1'b0=Don't Capture data in shift-IR state.
 *   </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param record_tdo_in_shift_dr_state </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 4 </td>
 *   <td width="30%"> 1'b1=Capture TDI Input data when in shift-DR state,
 *         1'b0=Don't Capture data in shift-DR state.
 *   </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param reserved1 </td>
 *     <td width="20%" align="center"> 2 </td>
 *     <td width="20%" align="center"> 2 </td>
 *   <td width="30%"> Reserved for future use </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param sw_gate_tck_test_logic_reset </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 1 </td>
 *   <td width="30%"> 1'b1=Gap TCK output clock in Test-Logic-Reset State
 *        and start_stopn = STOP(1'b0) Only;  1'b0=Don't Gap
 *        TCK clock
 *   </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param sw_gate_tck </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 0 </td>
 *   <td width="30%"> 1'b1=Gap TCK output clock in all TAPC states;  1'b0=Don't
 *        Gap TCK clock
 *   </td>
 * </tr>
 * </table>
 * Applies to: 5500 --> ncp_axis_mtc_MTC_CONFIG1_REG_ADDR_r_t
 *
 */

struct ncp_axis_mtc_MTC_CONFIG1_REG_ADDR_r_t {
#ifdef NCP_BIG_ENDIAN
	unsigned reserved0:26;
	unsigned record_tdo_in_shift_ir_state:1;
	unsigned record_tdo_in_shift_dr_state:1;
	unsigned reserved1:2;
	unsigned sw_gate_tck_test_logic_reset:1;
	unsigned sw_gate_tck:1;
#else				/* Little Endian */
	unsigned sw_gate_tck:1;
	unsigned sw_gate_tck_test_logic_reset:1;
	unsigned reserved1:2;
	unsigned record_tdo_in_shift_dr_state:1;
	unsigned record_tdo_in_shift_ir_state:1;
	unsigned reserved0:26;
#endif
};

/*! @struct ncp_axis_mtc_MTC_STATUS1_REG_ADDR_r_t
 *  @brief MTC Status 1 Register
 *  @details null
 *  <table width="70%" align="center">
 *  <tr>
 *   <td colspan="4" align="center">
 *    struct ncp_axis_mtc_MTC_STATUS1_REG_ADDR_r_t\n
 *   </td>
 *  </tr>
 *  <tr>
 *     <td width="30%"><b> Name </b></td>
 *     <td width="20%" align="center"><b> Width </b></td>
 *     <td width="20%" align="center"><b> Start Offset </b></td>
 *     <td width="30%"><b> Description </b></td>
 *  </tr>
 *   <tr>
 *     <td width="30%"> @param reserved0 </td>
 *     <td width="20%" align="center"> 2 </td>
 *     <td width="20%" align="center"> 30 </td>
 *   <td width="30%"> Reserved for future use </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param tdo_record_ram_bit_counter </td>
 *     <td width="20%" align="center"> 14 </td>
 *     <td width="20%" align="center"> 16 </td>
 *   <td width="30%"> Number of Valid Bits in the TDO capture memory </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param tdo_record_ram_last_addr </td>
 *     <td width="20%" align="center"> 8 </td>
 *     <td width="20%" align="center"> 8 </td>
 *   <td width="30%"> Last address within the TDO capture memory that contains
 *        valid data.  Addresses are filled started at bit
 *        location 0 (MSB) -> 31(LSB); address 1, bit 0 -> 31,
 *        ...
 *   </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param prgm_mem_rd_addr </td>
 *     <td width="20%" align="center"> 8 </td>
 *     <td width="20%" align="center"> 0 </td>
 *   <td width="30%"> Next Address to be read from program memory  In
 *        the pause state this address should be written with
 *        the next task to be processed
 *   </td>
 * </tr>
 * </table>
 * Applies to: 5500 --> ncp_axis_mtc_MTC_STATUS1_REG_ADDR_r_t
 *
 */

struct ncp_axis_mtc_MTC_STATUS1_REG_ADDR_r_t {
#ifdef NCP_BIG_ENDIAN
	unsigned reserved0:2;
	unsigned tdo_record_ram_bit_counter:14;
	unsigned tdo_record_ram_last_addr:8;
	unsigned prgm_mem_rd_addr:8;
#else				/* Little Endian */
	unsigned prgm_mem_rd_addr:8;
	unsigned tdo_record_ram_last_addr:8;
	unsigned tdo_record_ram_bit_counter:14;
	unsigned reserved0:2;
#endif
};

/*! @struct ncp_axis_mtc_MTC_STATUS2_REG_ADDR_r_t
 *  @brief MTC Status 2 Register
 *  @details null
 *  <table width="70%" align="center">
 *  <tr>
 *   <td colspan="4" align="center">
 *    struct ncp_axis_mtc_MTC_STATUS2_REG_ADDR_r_t\n
 *   </td>
 *  </tr>
 *  <tr>
 *     <td width="30%"><b> Name </b></td>
 *     <td width="20%" align="center"><b> Width </b></td>
 *     <td width="20%" align="center"><b> Start Offset </b></td>
 *     <td width="30%"><b> Description </b></td>
 *  </tr>
 *   <tr>
 *     <td width="30%"> @param mtc_cmpl_enablen </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 31 </td>
 *   <td width="30%"> 1'b0=Disable MTC Block from Driving JTAG primary IO,
 *         1'b1=Allow cfg_config_ctl[2:0] register to select
 *        the mode of the MTC block.  Note: Control signal
 *        from primary device input pin.
 *   </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param mtc_axm_selftst_disable </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 30 </td>
 *   <td width="30%"> 1'b1=Disable all MTC configuration modes that allow
 *        access to the internal JTC controller.  1'b0=Allow
 *        all modes.  Note: Control signal from internal EFUSE
 *        module.
 *   </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param reserved0 </td>
 *     <td width="20%" align="center"> 12 </td>
 *     <td width="20%" align="center"> 18 </td>
 *   <td width="30%"> Reserved for future use </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param nxt_task_error </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 17 </td>
 *   <td width="30%"> Fatal Error Detected while Processing a Task resulting
 *        in the MTC block halting.  Once the program is fixed
 *        a SW reset is needed to restart the block.
 *   </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param tst_gen_state_error </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 16 </td>
 *   <td width="30%"> Fatal Error Detected while Processing a Task resulting
 *        in the MTC block halting.  Once the program is fixed
 *        a SW reset is needed to restart the block.
 *   </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param reserved1 </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 15 </td>
 *   <td width="30%"> Reserved for future use </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param pause_in_shiftir_tck_off </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 14 </td>
 *   <td width="30%"> TAPC paused in pause-IR state with the TCK clock off
 *        to allow reloading of TESTGEN memory with more instruction
 *        data. The one-shot cont_after_pause, single_step or
 *        the sw_reset register must be written to exit this
 * state. pause_in_shiftir
 *               = (curr_task_load_jtag_instr_reg_and_pause_in_pause_ir
 * | curr_task_load_jtag_instr_reg_continue_from_pause_ir
 * | curr_task_load_jtag_instr_reg_continue_from_pause_ir_and_stop_in_pause_ir)
 *        && curr_tapc_pause_IR) & curr_task_gap_tck_en;
 *   </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param pause_in_shiftdr_tck_off </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 13 </td>
 *   <td width="30%"> TAPC paused in pause-DR state with the TCK clock off
 *        to allow reloading of TESTGEN memory with more data.
 *        The one-shot cont_after_pause, single_step or the
 *        sw_reset register must be written to exit this state.
 *        pause_in_shiftdr = (curr_task_load_jtag_data_reg_and_pause_in_pause_dr
 *        | curr_task_load_jtag_data_reg_continue_from_pause_dr
 *  | curr_task_load_jtag_data_reg_continue_from_pause_dr_and_stop_in_pause_dr)
 *        && curr_tapc_pause_DR) & curr_task_gap_tck_en;
 *   </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param pause_in_run_test_idle_tck_off </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 12 </td>
 *   <td width="30%"> TESTGEN State Machine is Paused in the Run-Test/Idle
 *        state because TCK clock is gapped.
 *   </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param pause_in_test_logic_reset_tck_off </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 11 </td>
 *   <td width="30%"> TESTGEN State Machine is Paused in the Test-Logic-Reset
 *        state because TCK clock is gapped.
 *   </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param testgen_state_machine_paused </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 10 </td>
 *   <td width="30%"> TESTGEN state machine is paused - valid in single
 *        step mode and pause modes
 *   </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param pause_in_shiftir </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 9 </td>
 *   <td width="30%"> TAPC paused in pause-IR state to allow reloading of
 *        TESTGEN memory with more instruction data.  The
 *        one-shot cont_after_pause, single_step or the sw_reset
 *        register must be written to exit this state.  pause_in_shiftir
 *        =  (curr_task_load_jtag_instr_reg_and_pause_in_pause_ir
 *        | curr_task_load_jtag_instr_reg_continue_from_pause_ir
 * | curr_task_load_jtag_instr_reg_continue_from_pause_ir_and_stop_in_pause_ir)
 *        && curr_tapc_pause_IR);
 *   </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param pause_in_shiftdr </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 8 </td>
 *   <td width="30%"> TAPC paused in pause-DR state to allow reloading of
 *        TESTGEN memory with more data.  The one-shot cont_after_pause,
 *        single_step or the sw_reset register must be written
 *   to exit this state.  pause_in_shiftdr
*           =  (curr_task_load_jtag_data_reg_and_pause_in_pause_dr
 *        | curr_task_load_jtag_data_reg_continue_from_pause_dr
 *  | curr_task_load_jtag_data_reg_continue_from_pause_dr_and_stop_in_pause_dr)
 *        && curr_tapc_pause_DR);
 *   </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param curr_task_inst_i </td>
 *     <td width="20%" align="center"> 4 </td>
 *     <td width="20%" align="center"> 4 </td>
 *   <td width="30%"> Current task value. </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param reserved2 </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 3 </td>
 *   <td width="30%"> Reserved for future use </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param curr_task_end_of_test </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 2 </td>
 *   <td width="30%"> Test Finished </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param curr_task_pause_in_run_test_idle_state </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 1 </td>
 *   <td width="30%"> Pause Bit </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param start_stopn </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 0 </td>
 *   <td width="30%"> Busy Bit </td>
 * </tr>
 * </table>
 * Applies to: 5500 --> ncp_axis_mtc_MTC_STATUS2_REG_ADDR_r_t
 *
 */

struct ncp_axis_mtc_MTC_STATUS2_REG_ADDR_r_t {
#ifdef NCP_BIG_ENDIAN
	unsigned mtc_cmpl_enablen:1;
	unsigned mtc_axm_selftst_disable:1;
	unsigned reserved0:12;
	unsigned nxt_task_error:1;
	unsigned tst_gen_state_error:1;
	unsigned reserved1:1;
	unsigned pause_in_shiftir_tck_off:1;
	unsigned pause_in_shiftdr_tck_off:1;
	unsigned pause_in_run_test_idle_tck_off:1;
	unsigned pause_in_test_logic_reset_tck_off:1;
	unsigned testgen_state_machine_paused:1;
	unsigned pause_in_shiftir:1;
	unsigned pause_in_shiftdr:1;
	unsigned curr_task_inst_i:4;
	unsigned reserved2:1;
	unsigned curr_task_end_of_test:1;
	unsigned curr_task_pause_in_run_test_idle_state:1;
	unsigned start_stopn:1;
#else				/* Little Endian */
	unsigned start_stopn:1;
	unsigned curr_task_pause_in_run_test_idle_state:1;
	unsigned curr_task_end_of_test:1;
	unsigned reserved2:1;
	unsigned curr_task_inst_i:4;
	unsigned pause_in_shiftdr:1;
	unsigned pause_in_shiftir:1;
	unsigned testgen_state_machine_paused:1;
	unsigned pause_in_test_logic_reset_tck_off:1;
	unsigned pause_in_run_test_idle_tck_off:1;
	unsigned pause_in_shiftdr_tck_off:1;
	unsigned pause_in_shiftir_tck_off:1;
	unsigned reserved1:1;
	unsigned tst_gen_state_error:1;
	unsigned nxt_task_error:1;
	unsigned reserved0:12;
	unsigned mtc_axm_selftst_disable:1;
	unsigned mtc_cmpl_enablen:1;
#endif
};

/*! @struct ncp_axis_mtc_MTC_EXECUTE1_REG_ADDR_r_t
 *  @brief MTC EXECUTE Register - One-Shot Registers
 *  @details Self Clearing Bits
 *  <table width="70%" align="center">
 *  <tr>
 *   <td colspan="4" align="center">
 *    struct ncp_axis_mtc_MTC_EXECUTE1_REG_ADDR_r_t\n
 *   </td>
 *  </tr>
 *  <tr>
 *     <td width="30%"><b> Name </b></td>
 *     <td width="20%" align="center"><b> Width </b></td>
 *     <td width="20%" align="center"><b> Start Offset </b></td>
 *     <td width="30%"><b> Description </b></td>
 *  </tr>
 *   <tr>
 *     <td width="30%"> @param reserved0 </td>
 *     <td width="20%" align="center"> 27 </td>
 *     <td width="20%" align="center"> 5 </td>
 *   <td width="30%"> Reserved for future use </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param tdo_flush_capture_data </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 4 </td>
 *   <td width="30%"> Write to 1 to force the MTC TDO CAPTURE state machine
 *        to write any remaining data to memory. This signal
 *        would be written before the tdo_capture_reset bit
 *        is set.  Self clearing bit.
 *   </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param tdo_capture_reset </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 3 </td>
 *   <td width="30%"> Write to 1 to reset the MTC TDO CAPTURE state machine.
 *        All variables set to zero (memory write address, tdo
 *        capture bit count, etc).  Self clearing bit.
 *   </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param sw_reset </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 2 </td>
 *   <td width="30%"> Write to 1 to reset the MTC TESTGEN, MTC TDO CAPTURE
 *        state machines and to recover from the state machine
 *        haulting due to a command error.  Self clearing
 *        bit.
 *   </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param cont_after_pause </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 1 </td>
 *   <td width="30%"> Write to 1 to allow the MTC TESTGEN state machine
 *        to continue from the pause-IR or pause-DR states.
 *         Self clearing bit.
 *   </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param single_step </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 0 </td>
 *   <td width="30%"> Write to 1 to allow the MTC_TESTGEN state machine
 *        to process the next command.  In the single step
 *        mode the state machine will stop after each task is
 *        processed.  Self clearing bit.
 *   </td>
 * </tr>
 * </table>
 * Applies to: 5500 --> ncp_axis_mtc_MTC_EXECUTE1_REG_ADDR_r_t
 *
 */

struct ncp_axis_mtc_MTC_EXECUTE1_REG_ADDR_r_t {
#ifdef NCP_BIG_ENDIAN
	unsigned reserved0:27;
	unsigned tdo_flush_capture_data:1;
	unsigned tdo_capture_reset:1;
	unsigned sw_reset:1;
	unsigned cont_after_pause:1;
	unsigned single_step:1;
#else				/* Little Endian */
	unsigned single_step:1;
	unsigned cont_after_pause:1;
	unsigned sw_reset:1;
	unsigned tdo_capture_reset:1;
	unsigned tdo_flush_capture_data:1;
	unsigned reserved0:27;
#endif
};

/*! @struct ncp_axis_mtc_MTC_MEM_INIT_REG_ADDR_r_t
 *  @brief MTC MEM Init Register
 *  @details Registers to Control Initializing the TDO Capture and TESTPROG memories
 *  <table width="70%" align="center">
 *  <tr>
 *   <td colspan="4" align="center">
 *    struct ncp_axis_mtc_MTC_MEM_INIT_REG_ADDR_r_t\n
 *   </td>
 *  </tr>
 *  <tr>
 *     <td width="30%"><b> Name </b></td>
 *     <td width="20%" align="center"><b> Width </b></td>
 *     <td width="20%" align="center"><b> Start Offset </b></td>
 *     <td width="30%"><b> Description </b></td>
 *  </tr>
 *   <tr>
 *     <td width="30%"> @param reserved0 </td>
 *     <td width="20%" align="center"> 21 </td>
 *     <td width="20%" align="center"> 11 </td>
 *   <td width="30%"> Reserved for future use </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param tdo_capture_mem_ini_value </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 10 </td>
 *   <td width="30%"> Memory init value for capture memory </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param tdo_capture_mem_do_mem_init </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 9 </td>
 *   <td width="30%"> Do memory initialization. When set, the capture memory
 *        is initialized to either zero or one depending
 *        on tdo_capture_mem_ini_value value by hardware. This
 *        bit clears itself one cycle after being set
 *   </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param tdo_capture_mem_init_done </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 8 </td>
 *   <td width="30%"> Memory initialization done. When set, the program
 *        memory has been initialized to zero or one by hardware
 *        following setting the tst_prgm_mem_do_mem_init bit
 *        and tst_prgm_mem_ini_value of this register.
 *   </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param reserved1 </td>
 *     <td width="20%" align="center"> 5 </td>
 *     <td width="20%" align="center"> 3 </td>
 *   <td width="30%"> Reserved for future use </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param tst_prgm_mem_ini_value </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 2 </td>
 *   <td width="30%"> Memory init value for program memory </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param tst_prgm_mem_do_mem_init </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 1 </td>
 *   <td width="30%"> Do memory initialization. When set, the program memory
 *        is initialized to either all zero or ones depending
 *        on tst_prgm_mem_ini_value value by hardware. This
 *        bit clears itself one cycle after being set
 *   </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param tst_prgm_mem_init_done </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 0 </td>
 *   <td width="30%"> Memory initialization done. When set, the program
 *        memory has been initialized to zero or one by hardware
 *        following setting  the tst_prgm_mem_do_mem_init
 *        bit and tst_prgm_mem_ini_value of this register.
 *   </td>
 * </tr>
 * </table>
 * Applies to: 5500 --> ncp_axis_mtc_MTC_MEM_INIT_REG_ADDR_r_t
 *
 */

struct ncp_axis_mtc_MTC_MEM_INIT_REG_ADDR_r_t {
#ifdef NCP_BIG_ENDIAN
	unsigned reserved0:21;
	unsigned tdo_capture_mem_ini_value:1;
	unsigned tdo_capture_mem_do_mem_init:1;
	unsigned tdo_capture_mem_init_done:1;
	unsigned reserved1:5;
	unsigned tst_prgm_mem_ini_value:1;
	unsigned tst_prgm_mem_do_mem_init:1;
	unsigned tst_prgm_mem_init_done:1;
#else				/* Little Endian */
	unsigned tst_prgm_mem_init_done:1;
	unsigned tst_prgm_mem_do_mem_init:1;
	unsigned tst_prgm_mem_ini_value:1;
	unsigned reserved1:5;
	unsigned tdo_capture_mem_init_done:1;
	unsigned tdo_capture_mem_do_mem_init:1;
	unsigned tdo_capture_mem_ini_value:1;
	unsigned reserved0:21;
#endif
};

/*! @struct ncp_axis_mtc_MTC_ECC_DISABLE_REG_ADDR_r_t
 *  @brief MTC MEM Init Register
 *  @details null
 *  <table width="70%" align="center">
 *  <tr>
 *   <td colspan="4" align="center">
 *    struct ncp_axis_mtc_MTC_ECC_DISABLE_REG_ADDR_r_t\n
 *   </td>
 *  </tr>
 *  <tr>
 *     <td width="30%"><b> Name </b></td>
 *     <td width="20%" align="center"><b> Width </b></td>
 *     <td width="20%" align="center"><b> Start Offset </b></td>
 *     <td width="30%"><b> Description </b></td>
 *  </tr>
 *   <tr>
 *     <td width="30%"> @param reserved0 </td>
 *     <td width="20%" align="center"> 30 </td>
 *     <td width="20%" align="center"> 2 </td>
 *   <td width="30%"> Reserved for future use </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param disable_ecc_mtc_tst_prgm_mem </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 1 </td>
 *   <td width="30%"> This bit disables the ECC correction for mtc_tst_prgm_mem
 *        memory.
 *   </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param disable_ecc_mtc_tdo_record_mem </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 0 </td>
 * <td width="30%"> This bit disables the ECC correction for mtc_tdo_record_mem
 *        memory.
 *   </td>
 * </tr>
 * </table>
 * Applies to: 5500 --> ncp_axis_mtc_MTC_ECC_DISABLE_REG_ADDR_r_t
 *
 */

struct ncp_axis_mtc_MTC_ECC_DISABLE_REG_ADDR_r_t {

#ifdef NCP_BIG_ENDIAN
	unsigned reserved0:30;
	unsigned disable_ecc_mtc_tst_prgm_mem:1;
	unsigned disable_ecc_mtc_tdo_record_mem:1;
#else				/* Little Endian */
	unsigned disable_ecc_mtc_tdo_record_mem:1;
	unsigned disable_ecc_mtc_tst_prgm_mem:1;
	unsigned reserved0:30;
#endif
};

/*! @struct ncp_axis_mtc_MTC_ECC_INVERT_EN_REG_ADDR_r_t
 *  @brief MTC MEM Invert Enable ECC Register
 *  @details null
 *  <table width="70%" align="center">
 *  <tr>
 *   <td colspan="4" align="center">
 *    struct ncp_axis_mtc_MTC_ECC_INVERT_EN_REG_ADDR_r_t\n
 *   </td>
 *  </tr>
 *  <tr>
 *     <td width="30%"><b> Name </b></td>
 *     <td width="20%" align="center"><b> Width </b></td>
 *     <td width="20%" align="center"><b> Start Offset </b></td>
 *     <td width="30%"><b> Description </b></td>
 *  </tr>
 *   <tr>
 *     <td width="30%"> @param reserved0 </td>
 *     <td width="20%" align="center"> 30 </td>
 *     <td width="20%" align="center"> 2 </td>
 *   <td width="30%"> Reserved for future use </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param ecc_invert_en_mtc_tst_prgm_mem </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 1 </td>
 *   <td width="30%"> ECC invert enable for mtc_tst_prgm_mem memory. </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param ecc_invert_en_tc_tdo_record_mem </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 0 </td>
 *   <td width="30%"> ECC invert enable for mtc_tdo_record memory. </td>
 * </tr>
 * </table>
 * Applies to: 5500 --> ncp_axis_mtc_MTC_ECC_INVERT_EN_REG_ADDR_r_t
 *
 */

struct ncp_axis_mtc_MTC_ECC_INVERT_EN_REG_ADDR_r_t {
#ifdef NCP_BIG_ENDIAN
	unsigned reserved0:30;
	unsigned ecc_invert_en_mtc_tst_prgm_mem:1;
	unsigned ecc_invert_en_tc_tdo_record_mem:1;
#else				/* Little Endian */
	unsigned ecc_invert_en_tc_tdo_record_mem:1;
	unsigned ecc_invert_en_mtc_tst_prgm_mem:1;
	unsigned reserved0:30;
#endif
};

/*! @struct ncp_axis_mtc_MTC_ECC_INVERT_REG_ADDR_r_t
 *  @brief MTC MEM Invert ECC Register
 *  @details null
 *  <table width="70%" align="center">
 *  <tr>
 *   <td colspan="4" align="center">
 *    struct ncp_axis_mtc_MTC_ECC_INVERT_REG_ADDR_r_t\n
 *   </td>
 *  </tr>
 *  <tr>
 *     <td width="30%"><b> Name </b></td>
 *     <td width="20%" align="center"><b> Width </b></td>
 *     <td width="20%" align="center"><b> Start Offset </b></td>
 *     <td width="30%"><b> Description </b></td>
 *  </tr>
 *   <tr>
 *     <td width="30%"> @param reserved0 </td>
 *     <td width="20%" align="center"> 25 </td>
 *     <td width="20%" align="center"> 7 </td>
 *   <td width="30%"> Reserved for future use </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param ecc_invert_reg </td>
 *     <td width="20%" align="center"> 7 </td>
 *     <td width="20%" align="center"> 0 </td>
 *   <td width="30%"> Each memory type uses appropriate number of bits in
 *        this field starting from right to left.   Set one
 *        bit to cause a single bit error, two bits to cause
 *        a double bit error.
 *   </td>
 * </tr>
 * </table>
 * Applies to: 5500 --> ncp_axis_mtc_MTC_ECC_INVERT_REG_ADDR_r_t
 *
 */

struct ncp_axis_mtc_MTC_ECC_INVERT_REG_ADDR_r_t {
#ifdef NCP_BIG_ENDIAN
	unsigned reserved0:25;
	unsigned ecc_invert_reg:7;
#else				/* Little Endian */
	unsigned ecc_invert_reg:7;
	unsigned reserved0:25;
#endif
};

/*! @struct ncp_axis_mtc_MTC_DEBUG0_REG_ADDR_r_t
 *  @brief MTC Debug0 Register
 *  @details DEBUG0: Current Task Values
 *  <table width="70%" align="center">
 *  <tr>
 *   <td colspan="4" align="center">
 *    struct ncp_axis_mtc_MTC_DEBUG0_REG_ADDR_r_t\n
 *   </td>
 *  </tr>
 *  <tr>
 *     <td width="30%"><b> Name </b></td>
 *     <td width="20%" align="center"><b> Width </b></td>
 *     <td width="20%" align="center"><b> Start Offset </b></td>
 *     <td width="30%"><b> Description </b></td>
 *  </tr>
 *   <tr>
 *     <td width="30%"> @param mtc_testgen_tdo_inactive_enb </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 31 </td>
 *   <td width="30%"> When mtc_mpu_tdo_inactive_en control signal is active
 *        (1'b1) this signal is used to control the TDO output
 *        buffer tri-state control input.  1'b1 = TDO Output
 *        Buffer Active;  1'b0 = TDO Output Buffer Inactive
 *   </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param curr_task_gap_tck_en </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 30 </td>
 *   <td width="30%"> 1'b1 = Gap TCK clock in end state;  1'b0 = Don't
 *        Gap TCK clock in end state
 *   </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param curr_task_tck_ctl_i </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 29 </td>
 *   <td width="30%"> Current TASK TCK Control Value </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param curr_task_end_state_ctl_i </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 28 </td>
 *   <td width="30%"> 1'b1 = Start Next Command from the Select-DR-Scan
 *        State (Clock cannot be stopped)  1'b0 = Go to Run-test/Idle
 *        State - Pause only if Gap TCK Clock active
 *   </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param reserved0 </td>
 *     <td width="20%" align="center"> 2 </td>
 *     <td width="20%" align="center"> 26 </td>
 *   <td width="30%"> Reserved for future use </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param curr_task_ctl_i </td>
 *     <td width="20%" align="center"> 2 </td>
 *     <td width="20%" align="center"> 24 </td>
 *   <td width="30%"> Current Task control values </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param reserved1 </td>
 *     <td width="20%" align="center"> 3 </td>
 *     <td width="20%" align="center"> 21 </td>
 *   <td width="30%"> Reserved for future use </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param nxt_tstgen_tdi_cnt30 </td>
 *     <td width="20%" align="center"> 5 </td>
 *     <td width="20%" align="center"> 16 </td>
 *   <td width="30%"> Next Shift Cnt value </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param reserved2 </td>
 *     <td width="20%" align="center"> 4 </td>
 *     <td width="20%" align="center"> 12 </td>
 *   <td width="30%"> Reserved for future use </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param task_sub_state </td>
 *     <td width="20%" align="center"> 4 </td>
 *     <td width="20%" align="center"> 8 </td>
 *   <td width="30%"> TESTGEN state Machine subtask counter </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param reserved3 </td>
 *     <td width="20%" align="center"> 4 </td>
 *     <td width="20%" align="center"> 4 </td>
 *   <td width="30%"> Reserved for future use </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param nxt_task_inst_i </td>
 *     <td width="20%" align="center"> 4 </td>
 *     <td width="20%" align="center"> 0 </td>
 *   <td width="30%"> Next Task Command Value </td>
 * </tr>
 * </table>
 * Applies to: 5500 --> ncp_axis_mtc_MTC_DEBUG0_REG_ADDR_r_t
 *
 */

struct ncp_axis_mtc_MTC_DEBUG0_REG_ADDR_r_t {

#ifdef NCP_BIG_ENDIAN
	unsigned mtc_testgen_tdo_inactive_enb:1;
	unsigned curr_task_gap_tck_en:1;
	unsigned curr_task_tck_ctl_i:1;
	unsigned curr_task_end_state_ctl_i:1;
	unsigned reserved0:2;
	unsigned curr_task_ctl_i:2;
	unsigned reserved1:3;
	unsigned nxt_tstgen_tdi_cnt30:5;
	unsigned reserved2:4;
	unsigned task_sub_state:4;
	unsigned reserved3:4;
	unsigned nxt_task_inst_i:4;
#else				/* Little Endian */
	unsigned nxt_task_inst_i:4;
	unsigned reserved3:4;
	unsigned task_sub_state:4;
	unsigned reserved2:4;
	unsigned nxt_tstgen_tdi_cnt30:5;
	unsigned reserved1:3;
	unsigned curr_task_ctl_i:2;
	unsigned reserved0:2;
	unsigned curr_task_end_state_ctl_i:1;
	unsigned curr_task_tck_ctl_i:1;
	unsigned curr_task_gap_tck_en:1;
	unsigned mtc_testgen_tdo_inactive_enb:1;
#endif
};

/*! @struct ncp_axis_mtc_MTC_DEBUG1_REG_ADDR_r_t
 *  @brief MTC Debug1 Register
 *  @details DEBUG1: Current TASK Being Processed
 *  <table width="70%" align="center">
 *  <tr>
 *   <td colspan="4" align="center">
 *    struct ncp_axis_mtc_MTC_DEBUG1_REG_ADDR_r_t\n
 *   </td>
 *  </tr>
 *  <tr>
 *     <td width="30%"><b> Name </b></td>
 *     <td width="20%" align="center"><b> Width </b></td>
 *     <td width="20%" align="center"><b> Start Offset </b></td>
 *     <td width="30%"><b> Description </b></td>
 *  </tr>
 *   <tr>
 *     <td width="30%"> @param curr_task_no_operation </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 31 </td>
 *<td width="30%"> Current task being processed is No Operation(4'b0000) </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param curr_task_test_reset_1 </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 30 </td>
 *   <td width="30%"> Current task being processed is Test reset 1 (TRSTZ
 *        low for x TCK cycles - 4'b0001)
 *   </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param curr_task_test_reset_2 </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 29 </td>
 *   <td width="30%"> Current task being processed is Test reset 2 (TMS
 *        high for x TCK cycles) - 4'b0010
 *   </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param curr_task_load_jtag_inst_reg </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 28 </td>
 *   <td width="30%"> Current task being processed is Load JTAG instruction
 *        register
 *   </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param curr_task_load_jtag_data_reg </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 27 </td>
 *<td width="30%"> Current task being processed is Load JTAG data register </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param curr_task_mtc_reserved_task_id_5 </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 26 </td>
 *   <td width="30%"> Reserved Task value - Illegal - 4'b0101 </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param curr_task_mtc_reserved_task_id_6 </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 25 </td>
 *   <td width="30%"> Reserved Task value - Illegal - 4'b0110 </td>
 * </tr>
 *   <tr>
 * <td width="30%">
 * @param curr_task_wait_in_run_test_idle_state_xtck_cycles </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 24 </td>
 *   <td width="30%"> Current task being processed is Wait in Run-Test-Idle
 *        state a number of TCK cycles - 4'b1010
 *   </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param curr_task_pause_in_run_test_idle_state </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 23 </td>
 *   <td width="30%"> Current task being processed is Pause in Run-Test-Idle
 *        State - 4'b1011
 *   </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param curr_task_end_of_test </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 22 </td>
 *<td width="30%"> Current task being processed is End of Test - 4'b1100 </td>
 * </tr>
 *   <tr>
 *<td width="30%"> @param
 *   curr_task_load_jtag_data_reg_and_pause_in_pause_dr </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 21 </td>
 *   <td width="30%"> Current task being processed is Load JTAG Data Register
 *        and pause in pause-DR state - 4'b0111
 *   </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param
 *  curr_task_load_jtag_data_reg_continue_from_pause_dr </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 20 </td>
 *   <td width="30%"> Current task being processed is Load JTAG Data Register
 *        (continue from pause-DR state) - 4'b1000
 *   </td>
 * </tr>
 *   <tr>
 * <td width="30%"> @param
*curr_task_load_jtag_data_reg_continue_from_pause_dr_and_stop_in_pause_dr </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 19 </td>
 *   <td width="30%"> Current task being processed is Load JTAG Data Register
 *        (continue from pause-DR state and end in pause-DR
 *        state - 4'b1001
 *   </td>
 * </tr>
 *   <tr>
 *<td width="30%"> @param
 *  curr_task_load_jtag_inst_reg_and_pause_in_pause_ir </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 18 </td>
 *   <td width="30%"> Current task being processed is Load JTAG Instruction
 *        Register and pause in pause-IR state - 4'b1101
 *   </td>
 * </tr>
 *   <tr>
 * <td width="30%"> @param
 *  curr_task_load_jtag_inst_reg_continue_from_pause_ir </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 17 </td>
 *   <td width="30%"> Current task being processed is Load JTAG Instruction
 *        Register (continue from pause-IR state) - 4'b1110
 *   </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param
*curr_task_load_jtag_inst_reg_continue_from_pause_ir_and_stop_in_pause_ir </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 16 </td>
 *   <td width="30%"> Current task being processed is Load JTAG Instruction
 *        Register (continue from pause-IR state and end in
 *        pause-IR state - 4'b1111
 *   </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param reserved0 </td>
 *     <td width="20%" align="center"> 4 </td>
 *     <td width="20%" align="center"> 12 </td>
 *   <td width="30%"> Reserved for future use </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param curr_task_inst_i </td>
 *     <td width="20%" align="center"> 4 </td>
 *     <td width="20%" align="center"> 8 </td>
 *   <td width="30%"> Value of current task being processed </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param prgm_mem_rd_addr </td>
 *     <td width="20%" align="center"> 8 </td>
 *     <td width="20%" align="center"> 0 </td>
 *   <td width="30%"> Address of memory location to be read from program
 *        memory
 *   </td>
 * </tr>
 * </table>
 * Applies to: 5500 --> ncp_axis_mtc_MTC_DEBUG1_REG_ADDR_r_t
 *
 */

struct ncp_axis_mtc_MTC_DEBUG1_REG_ADDR_r_t {
#ifdef NCP_BIG_ENDIAN
	unsigned curr_task_no_operation:1;
	unsigned curr_task_test_reset_1:1;
	unsigned curr_task_test_reset_2:1;
	unsigned curr_task_load_jtag_inst_reg:1;
	unsigned curr_task_load_jtag_data_reg:1;
	unsigned curr_task_mtc_reserved_task_id_5:1;
	unsigned curr_task_mtc_reserved_task_id_6:1;
	unsigned curr_task_wait_in_run_test_idle_state_xtck_cycles:1;
	unsigned curr_task_pause_in_run_test_idle_state:1;
	unsigned curr_task_end_of_test:1;
	unsigned curr_task_load_jtag_data_reg_and_pause_in_pause_dr:1;
	unsigned curr_task_load_jtag_data_reg_continue_from_pause_dr:1;
	unsigned
	 curr_task_load_jtag_data_reg_continue_from_pause_dr_and_stop_in_pause_dr:1;
	unsigned curr_task_load_jtag_inst_reg_and_pause_in_pause_ir:1;
	unsigned curr_task_load_jtag_inst_reg_continue_from_pause_ir:1;
	unsigned
	 curr_task_load_jtag_inst_reg_continue_from_pause_ir_and_stop_in_pause_ir:1;
	unsigned reserved0:4;
	unsigned curr_task_inst_i:4;
	unsigned prgm_mem_rd_addr:8;
#else				/* Little Endian */
	unsigned prgm_mem_rd_addr:8;
	unsigned curr_task_inst_i:4;
	unsigned reserved0:4;
	unsigned
	 curr_task_load_jtag_inst_reg_continue_from_pause_ir_and_stop_in_pause_ir:1;
	unsigned curr_task_load_jtag_inst_reg_continue_from_pause_ir:1;
	unsigned curr_task_load_jtag_inst_reg_and_pause_in_pause_ir:1;
	unsigned
	 curr_task_load_jtag_data_reg_continue_from_pause_dr_and_stop_in_pause_dr:1;
	unsigned
	 curr_task_load_jtag_data_reg_continue_from_pause_dr:1;
	unsigned curr_task_load_jtag_data_reg_and_pause_in_pause_dr:1;
	unsigned curr_task_end_of_test:1;
	unsigned curr_task_pause_in_run_test_idle_state:1;
	unsigned curr_task_wait_in_run_test_idle_state_xtck_cycles:1;
	unsigned curr_task_mtc_reserved_task_id_6:1;
	unsigned curr_task_mtc_reserved_task_id_5:1;
	unsigned curr_task_load_jtag_data_reg:1;
	unsigned curr_task_load_jtag_inst_reg:1;
	unsigned curr_task_test_reset_2:1;
	unsigned curr_task_test_reset_1:1;
	unsigned curr_task_no_operation:1;
#endif
};

/*! @struct ncp_axis_mtc_MTC_DEBUG2_REG_ADDR_r_t
 *  @brief MTC Debug2 Register
 *  @details Current TAPC
 *  <table width="70%" align="center">
 *  <tr>
 *   <td colspan="4" align="center">
 *    struct ncp_axis_mtc_MTC_DEBUG2_REG_ADDR_r_t\n
 *   </td>
 *  </tr>
 *  <tr>
 *     <td width="30%"><b> Name </b></td>
 *     <td width="20%" align="center"><b> Width </b></td>
 *     <td width="20%" align="center"><b> Start Offset </b></td>
 *     <td width="30%"><b> Description </b></td>
 *  </tr>
 *   <tr>
 *     <td width="30%"> @param curr_tapc_update_ir </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 31 </td>
 *   <td width="30%"> Current TAPC State is UPDATE-IR </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param curr_tapc_exit2_ir </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 30 </td>
 *   <td width="30%"> Current TAPC State is EXIT2-IR </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param curr_tapc_pause_ir </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 29 </td>
 *   <td width="30%"> Current TAPC State is PAUSE-IR </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param curr_tapc_exit1_ir </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 28 </td>
 *   <td width="30%"> Current TAPC State is EXIT1-IR </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param curr_tapc_shift_ir </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 27 </td>
 *   <td width="30%"> Current TAPC State is SHIFT-IR </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param curr_tapc_capture_ir </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 26 </td>
 *   <td width="30%"> Current TAPC State is CAPTURE-IR </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param curr_tapc_update_dr </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 25 </td>
 *   <td width="30%"> Current TAPC State is UPDATE-DR </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param curr_tapc_exit2_dr </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 24 </td>
 *   <td width="30%"> Current TAPC State is EXIT2-DR </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param curr_tapc_pause_dr </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 23 </td>
 *   <td width="30%"> Current TAPC State is PAUSE-DR </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param curr_tapc_exit1_dr </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 22 </td>
 *   <td width="30%"> Current TAPC State is EXIT1-DR </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param curr_tapc_shift_dr </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 21 </td>
 *   <td width="30%"> Current TAPC State is SHIFT-DR </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param curr_tapc_capture_dr </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 20 </td>
 *   <td width="30%"> Current TAPC State is CAPTURE-DR </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param curr_tapc_select_ir_scan </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 19 </td>
 *   <td width="30%"> Current TAPC State is SELECT-IR-SCAN </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param curr_tapc_select_dr_scan </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 18 </td>
 *   <td width="30%"> Current TAPC State is SELECT-DR-SCAN </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param curr_tapc_run_test_idle </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 17 </td>
 *   <td width="30%"> Current TAPC State is RUN-TEST_IDLE </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param curr_tapc_test_logic_reset </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 16 </td>
 *   <td width="30%"> Current TAPC State is TEST-LOGIC-RESET </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param reserved0 </td>
 *     <td width="20%" align="center"> 12 </td>
 *     <td width="20%" align="center"> 4 </td>
 *   <td width="30%"> Reserved for future use </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param tapc_state </td>
 *     <td width="20%" align="center"> 4 </td>
 *     <td width="20%" align="center"> 0 </td>
 *   <td width="30%"> Current TAPC state value </td>
 * </tr>
 * </table>
 * Applies to: 5500 --> ncp_axis_mtc_MTC_DEBUG2_REG_ADDR_r_t
 *
 */

struct ncp_axis_mtc_MTC_DEBUG2_REG_ADDR_r_t {
#ifdef NCP_BIG_ENDIAN
	unsigned curr_tapc_update_ir:1;
	unsigned curr_tapc_exit2_ir:1;
	unsigned curr_tapc_pause_ir:1;
	unsigned curr_tapc_exit1_ir:1;
	unsigned curr_tapc_shift_ir:1;
	unsigned curr_tapc_capture_ir:1;
	unsigned curr_tapc_update_dr:1;
	unsigned curr_tapc_exit2_dr:1;
	unsigned curr_tapc_pause_dr:1;
	unsigned curr_tapc_exit1_dr:1;
	unsigned curr_tapc_shift_dr:1;
	unsigned curr_tapc_capture_dr:1;
	unsigned curr_tapc_select_ir_scan:1;
	unsigned curr_tapc_select_dr_scan:1;
	unsigned curr_tapc_run_test_idle:1;
	unsigned curr_tapc_test_logic_reset:1;
	unsigned reserved0:12;
	unsigned tapc_state:4;
#else				/* Little Endian */
	unsigned tapc_state:4;
	unsigned reserved0:12;
	unsigned curr_tapc_test_logic_reset:1;
	unsigned curr_tapc_run_test_idle:1;
	unsigned curr_tapc_select_dr_scan:1;
	unsigned curr_tapc_select_ir_scan:1;
	unsigned curr_tapc_capture_dr:1;
	unsigned curr_tapc_shift_dr:1;
	unsigned curr_tapc_exit1_dr:1;
	unsigned curr_tapc_pause_dr:1;
	unsigned curr_tapc_exit2_dr:1;
	unsigned curr_tapc_update_dr:1;
	unsigned curr_tapc_capture_ir:1;
	unsigned curr_tapc_shift_ir:1;
	unsigned curr_tapc_exit1_ir:1;
	unsigned curr_tapc_pause_ir:1;
	unsigned curr_tapc_exit2_ir:1;
	unsigned curr_tapc_update_ir:1;
#endif
};

/*! @struct ncp_axis_mtc_MTC_DEBUG3_REG_ADDR_r_t
 *  @brief MTC Debug3 Register
 *  @details TDO Data
 *  <table width="70%" align="center">
 *  <tr>
 *   <td colspan="4" align="center">
 *    struct ncp_axis_mtc_MTC_DEBUG3_REG_ADDR_r_t\n
 *   </td>
 *  </tr>
 *  <tr>
 *     <td width="30%"><b> Name </b></td>
 *     <td width="20%" align="center"><b> Width </b></td>
 *     <td width="20%" align="center"><b> Start Offset </b></td>
 *     <td width="30%"><b> Description </b></td>
 *  </tr>
 *   <tr>
 *     <td width="30%"> @param nxt_tdi_shift_data_d </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 31 </td>
 *   <td width="30%"> TDO value output on the next falling edge of the TCK
 *        clock - nxt_tdi_shift_data_d[0]
 *   </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param reserved0 </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 30 </td>
 *   <td width="30%"> Reserved for future use </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param nxt_tdi_shift_data </td>
 *     <td width="20%" align="center"> 30 </td>
 *     <td width="20%" align="center"> 0 </td>
 *   <td width="30%"> Next Data to be shifted out </td>
 * </tr>
 * </table>
 * Applies to: 5500 --> ncp_axis_mtc_MTC_DEBUG3_REG_ADDR_r_t
 *
 */

struct ncp_axis_mtc_MTC_DEBUG3_REG_ADDR_r_t {
#ifdef NCP_BIG_ENDIAN
	unsigned nxt_tdi_shift_data_d:1;
	unsigned reserved0:1;
	unsigned nxt_tdi_shift_data:30;
#else				/* Little Endian */
	unsigned nxt_tdi_shift_data:30;
	unsigned reserved0:1;
	unsigned nxt_tdi_shift_data_d:1;
#endif
};

/*! @struct ncp_axis_mtc_MTC_DEBUG4_REG_ADDR_r_t
 *  @brief MTC Debug4 Register
 *  @details Next Task Value
 *  <table width="70%" align="center">
 *  <tr>
 *   <td colspan="4" align="center">
 *    struct ncp_axis_mtc_MTC_DEBUG4_REG_ADDR_r_t\n
 *   </td>
 *  </tr>
 *  <tr>
 *     <td width="30%"><b> Name </b></td>
 *     <td width="20%" align="center"><b> Width </b></td>
 *     <td width="20%" align="center"><b> Start Offset </b></td>
 *     <td width="30%"><b> Description </b></td>
 *  </tr>
 *   <tr>
 *     <td width="30%"> @param nxt_task_inst_i </td>
 *     <td width="20%" align="center"> 32 </td>
 *     <td width="20%" align="center"> 0 </td>
 *   <td width="30%"> Next TASK to be processed </td>
 * </tr>
 * </table>
 * Applies to: 5500 --> ncp_axis_mtc_MTC_DEBUG4_REG_ADDR_r_t
 *
 */

struct ncp_axis_mtc_MTC_DEBUG4_REG_ADDR_r_t {
	unsigned int nxt_task_inst_i;
};

/*! @struct ncp_axis_mtc_MTC_DEBUG5_REG_ADDR_r_t
 *  @brief MTC Debug5 Register
 *  @details Shift Count Value
 *  <table width="70%" align="center">
 *  <tr>
 *   <td colspan="4" align="center">
 *    struct ncp_axis_mtc_MTC_DEBUG5_REG_ADDR_r_t\n
 *   </td>
 *  </tr>
 *  <tr>
 *     <td width="30%"><b> Name </b></td>
 *     <td width="20%" align="center"><b> Width </b></td>
 *     <td width="20%" align="center"><b> Start Offset </b></td>
 *     <td width="30%"><b> Description </b></td>
 *  </tr>
 *   <tr>
 *     <td width="30%"> @param reserved0 </td>
 *     <td width="20%" align="center"> 6 </td>
 *     <td width="20%" align="center"> 26 </td>
 *   <td width="30%"> Reserved for future use </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param delay_shift_cnt </td>
 *     <td width="20%" align="center"> 26 </td>
 *     <td width="20%" align="center"> 0 </td>
 *   <td width="30%"> Data shift value </td>
 * </tr>
 * </table>
 * Applies to: 5500 --> ncp_axis_mtc_MTC_DEBUG5_REG_ADDR_r_t
 *
 */

struct ncp_axis_mtc_MTC_DEBUG5_REG_ADDR_r_t {
#ifdef NCP_BIG_ENDIAN
	unsigned reserved0:6;
	unsigned delay_shift_cnt:26;
#else				/* Little Endian */
	unsigned delay_shift_cnt:26;
	unsigned reserved0:6;
#endif
};

/*! @struct ncp_axis_mtc_MTC_PRGM_MEM_START_ADDR_r_t
 *  @brief MTC Test Program Memory
 *  @details null
 *  <table width="70%" align="center">
 *  <tr>
 *   <td colspan="4" align="center">
 *    struct ncp_axis_mtc_MTC_PRGM_MEM_START_ADDR_r_t\n
 *   </td>
 *  </tr>
 *  <tr>
 *     <td width="30%"><b> Name </b></td>
 *     <td width="20%" align="center"><b> Width </b></td>
 *     <td width="20%" align="center"><b> Start Offset </b></td>
 *     <td width="30%"><b> Description </b></td>
 *  </tr>
 *   <tr>
 *     <td width="30%"> @param tst_prgm </td>
 *     <td width="20%" align="center"> 32 </td>
 *     <td width="20%" align="center"> 0 </td>
 *   <td width="30%"> TESTGEN program memory  Task1 Format: [31:30]=2'b10,
 *        [29]=TCK Action, [28]=0, [27:4]=Unused, [3:0]=Inst
 *        Code  Task2 Format: [31:30]=2'b10, [29:16]=Not Used,
 *        [15:4]=TCK Cycles., [3:0]=Inst Code  Task3 Format:
 *        [31:30]=2'b10, [29:4]=TCK Cycles., [3:0]=Inst Code
 *         Task4 Format_1: [31:30]=2'b11, [29]=TCK Action,
 *        [28]=End State, [27:16]=Data Shift,  [15:4]=Inst
 *        Shift, [3:0]=Inst Code  Task4 Format_2: [31:30]=2'b01,
 *        [29:0]=TDI Data (more data to Follow).  Task4 Format_3:
 *        [31:30]=2'b00, [29:0]=TDI Data (LAST data).
 *        TASK defintions:Value:Description:TCK [29]:End-State
 *        [28]:Task Format:One-Shot used to release from PAUSE
 *        state  4'b0000:	No operation - Should not be used
 *        in programs.:NA:NA:1:NO  4'b0001:	Generate test
 *        reset 1 (TRSTZ low for x TCK cycles):NA:NA:2:NO
 *        4'b0010:	Generate test reset 2 (TMS high for x TCK
 *        cycles):NA:NA:2:NO  4'b0011:	Load JTAG instruction
 *        register:YES:YES:4:YES  4'b0100:	Load JTAG data
 *        register:YES:YES:4:YES  4'b0101:	Reserved State:-:-:-:-
 *         4'b0110:	Reserved State:-:-:-:-  4'b0111:	Load
 *        JTAG data register and pause in pause-dr:YES:NA:4:YES
 *         4'b1000:	Load JTAG data register (continue from
 *        pause-dr):YES:YES:4:YES  4'b1001:	Load JTAG data
 *        register (continue from pause-dr and stop in pause-dr):
 *                  YES:YES:4:YES
 *         4'b1010:	Wait in Run-Test-Idle state a number of
 *        TCK cycles:NA:NA:3:NO  4'b1011:	Pause in Run-Test-Idle
 *        state:YES:0:1:YES  4'b1100:	End of test:YES:0:1:NO
 *         4'b1101:	Load JTAG instruction register and pause
 *        in pause-ir:YES:NA:4:YES  4'b1110:	Load JTAG instruction
 *        register (continue from pause-ir):YES:YES:4:YES
 *        4'b1111:	Load JTAG instruction register (continue
 *        from pause-ir and stop in pause-ir):YES:YES:4:YES
 *   </td>
 * </tr>
 * </table>
 * Applies to: 5500 --> ncp_axis_mtc_MTC_PRGM_MEM_START_ADDR_r_t
 *
 */

struct ncp_axis_mtc_MTC_PRGM_MEM_START_ADDR_r_t {
	unsigned int tst_prgm;
};

/*! @struct ncp_axis_mtc_MTC_TDO_CAPTURE_MEM_START_ADDR_r_t
 *  @brief MTC TDO capture Memory Program Memory
 *  @details null
 *  <table width="70%" align="center">
 *  <tr>
 *   <td colspan="4" align="center">
 *    struct ncp_axis_mtc_MTC_TDO_CAPTURE_MEM_START_ADDR_r_t\n
 *   </td>
 *  </tr>
 *  <tr>
 *     <td width="30%"><b> Name </b></td>
 *     <td width="20%" align="center"><b> Width </b></td>
 *     <td width="20%" align="center"><b> Start Offset </b></td>
 *     <td width="30%"><b> Description </b></td>
 *  </tr>
 *   <tr>
 *     <td width="30%"> @param tdo_capture </td>
 *     <td width="20%" align="center"> 32 </td>
 *     <td width="20%" align="center"> 0 </td>
 *   <td width="30%"> TDI capture memory - Data is right justified when
 *        written to the memory
 *   </td>
 * </tr>
 * </table>
 * Applies to: 5500 --> ncp_axis_mtc_MTC_TDO_CAPTURE_MEM_START_ADDR_r_t
 *
 */

struct ncp_axis_mtc_MTC_TDO_CAPTURE_MEM_START_ADDR_r_t {

	unsigned int tdo_capture;
};

/*! @struct ncp_axis_mtc_mtc_tstgen_int_status_r_t
 *  @brief Interrupt Status Register
 *  @details This register holds interrupt status
 *  <table width="70%" align="center">
 *  <tr>
 *   <td colspan="4" align="center">
 *    struct ncp_axis_mtc_mtc_tstgen_int_status_r_t\n
 *   </td>(
 *  </tr>
 *  <tr>
 *     <td width="30%"><b> Name </b></td>
 *     <td width="20%" align="center"><b> Width </b></td>
 *     <td width="20%" align="center"><b> Start Offset </b></td>
 *     <td width="30%"><b> Description </b></td>
 *  </tr>
 *   <tr>
 *     <td width="30%"> @param reserved0 </td>
 *     <td width="20%" align="center"> 25 </td>
 *     <td width="20%" align="center"> 7 </td>
 *   <td width="30%"> Reserved for future use </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param tdo_record_ram_addr_overflow_os </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 6 </td>
 *   <td width="30%"> The TDO Capture memory overflowed. Data is lost when
 *        this alarm occurs. Status
 *   </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param nxt_task_error_os </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 5 </td>
 *   <td width="30%"> Fatal Error Occured while processing a TASK. TESTGEN
 *        state machine is halted at that failure.  The errored
 *        task must be fixed and the sw_reset signal written
 *        to restart the test. Status
 *   </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param tst_gen_state_error_os </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 4 </td>
 *   <td width="30%"> Fatal Error Occured while processing a TASK. TESTGEN
 *        state machine is halted at the failure.  The errored
 *        task must be fixed and the sw_reset signal written
 *        to restart the test. Status
 *   </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param cont_after_pause_os </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 3 </td>
 *   <td width="30%"> Active anytime the TESTGEN state machine is paused.
 *        A write to cont_after_pause or single_step one-shot
 *        registers will release the state machine from the
 *        paused state. Status
 *   </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param pause_in_shiftir_os </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 2 </td>
 *   <td width="30%"> Paused in Shift-IR state Status </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param pause_in_shiftdr_os </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 1 </td>
 *   <td width="30%"> Paused in Shift-DR state Status </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param curr_task_end_of_test_os </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 0 </td>
 *   <td width="30%"> In End-of-Test Task Status </td>
 * </tr>
 * </table>
 * Applies to: 5500 --> ncp_axis_mtc_mtc_tstgen_int_status_r_t
 *
 */

struct ncp_axis_mtc_mtc_tstgen_int_status_r_t {
#ifdef NCP_BIG_ENDIAN
	unsigned reserved0:25;
	unsigned tdo_record_ram_addr_overflow_os:1;
	unsigned nxt_task_error_os:1;
	unsigned tst_gen_state_error_os:1;
	unsigned cont_after_pause_os:1;
	unsigned pause_in_shiftir_os:1;
	unsigned pause_in_shiftdr_os:1;
	unsigned curr_task_end_of_test_os:1;
#else				/* Little Endian */
	unsigned curr_task_end_of_test_os:1;
	unsigned pause_in_shiftdr_os:1;
	unsigned pause_in_shiftir_os:1;
	unsigned cont_after_pause_os:1;
	unsigned tst_gen_state_error_os:1;
	unsigned nxt_task_error_os:1;
	unsigned tdo_record_ram_addr_overflow_os:1;
	unsigned reserved0:25;
#endif
};

/*! @struct ncp_axis_mtc_mtc_tstgen_int_en_r_t
 *  @brief Interrupt Enable Register
 *  @details This register enables interrupts
 *  <table width="70%" align="center">
 *  <tr>
 *   <td colspan="4" align="center">
 *    struct ncp_axis_mtc_mtc_tstgen_int_en_r_t\n
 *   </td>
 *  </tr>
 *  <tr>
 *     <td width="30%"><b> Name </b></td>
 *     <td width="20%" align="center"><b> Width </b></td>
 *     <td width="20%" align="center"><b> Start Offset </b></td>
 *     <td width="30%"><b> Description </b></td>
 *  </tr>
 *   <tr>
 *     <td width="30%"> @param reserved0 </td>
 *     <td width="20%" align="center"> 25 </td>
 *     <td width="20%" align="center"> 7 </td>
 *   <td width="30%"> Reserved for future use </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param tdo_record_ram_addr_overflow_os </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 6 </td>
 *   <td width="30%"> The TDO Capture memory overflowed. Data is lost when
 *        this alarm occurs. Enable
 *   </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param nxt_task_error_os </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 5 </td>
 *   <td width="30%"> Fatal Error Occured while processing a TASK. TESTGEN
 *        state machine is halted at that failure.  The errored
 *        task must be fixed and the sw_reset signal written
 *        to restart the test. Enable
 *   </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param tst_gen_state_error_os </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 4 </td>
 *   <td width="30%"> Fatal Error Occured while processing a TASK. TESTGEN
 *        state machine is halted at the failure.  The errored
 *        task must be fixed and the sw_reset signal written
 *        to restart the test. Enable
 *   </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param cont_after_pause_os </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 3 </td>
 *   <td width="30%"> Active anytime the TESTGEN state machine is paused.
 *        A write to cont_after_pause or single_step one-shot
 *        registers will release the state machine from the
 *        paused state. Enable
 *   </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param pause_in_shiftir_os </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 2 </td>
 *   <td width="30%"> Paused in Shift-IR state Enable </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param pause_in_shiftdr_os </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 1 </td>
 *   <td width="30%"> Paused in Shift-DR state Enable </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param curr_task_end_of_test_os </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 0 </td>
 *   <td width="30%"> In End-of-Test Task Enable </td>
 * </tr>
 * </table>
 * Applies to: 5500 --> ncp_axis_mtc_mtc_tstgen_int_en_r_t
 *
 */

struct ncp_axis_mtc_mtc_tstgen_int_en_r_t {
#ifdef NCP_BIG_ENDIAN
	unsigned reserved0:25;
	unsigned tdo_record_ram_addr_overflow_os:1;
	unsigned nxt_task_error_os:1;
	unsigned tst_gen_state_error_os:1;
	unsigned cont_after_pause_os:1;
	unsigned pause_in_shiftir_os:1;
	unsigned pause_in_shiftdr_os:1;
	unsigned curr_task_end_of_test_os:1;
#else				/* Little Endian */
	unsigned curr_task_end_of_test_os:1;
	unsigned pause_in_shiftdr_os:1;
	unsigned pause_in_shiftir_os:1;
	unsigned cont_after_pause_os:1;
	unsigned tst_gen_state_error_os:1;
	unsigned nxt_task_error_os:1;
	unsigned tdo_record_ram_addr_overflow_os:1;
	unsigned reserved0:25;
#endif
};

/*! @struct ncp_axis_mtc_mtc_tstgen_int_force_r_t
 *  @brief Interrupt Force Register
 *  @details This address is an alias for the Interrupt Status register
      that allows normal CFG writes (as opposed to the Clear-On-Write-One
      behavior if the Interrupt Status register address is used).  This
     allows CFG to set interrupt bits for testing purposes.  Reading this
    address returns the current value of the Interrupt Status Register.
 *  <table width="70%" align="center">
 *  <tr>
 *   <td colspan="4" align="center">
 *    struct ncp_axis_mtc_mtc_tstgen_int_force_r_t\n
 *   </td>
 *  </tr>
 *  <tr>
 *     <td width="30%"><b> Name </b></td>
 *     <td width="20%" align="center"><b> Width </b></td>
 *     <td width="20%" align="center"><b> Start Offset </b></td>
 *     <td width="30%"><b> Description </b></td>
 *  </tr>
 *   <tr>
 *     <td width="30%"> @param reserved0 </td>
 *     <td width="20%" align="center"> 25 </td>
 *     <td width="20%" align="center"> 7 </td>
 *   <td width="30%"> Reserved for future use </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param tdo_record_ram_addr_overflow_os </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 6 </td>
 *   <td width="30%"> The TDO Capture memory overflowed. Data is lost when
 *        this alarm occurs. Force
 *   </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param nxt_task_error_os </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 5 </td>
 *   <td width="30%"> Fatal Error Occured while processing a TASK. TESTGEN
 *        state machine is halted at that failure.  The errored
 *        task must be fixed and the sw_reset signal written
 *        to restart the test. Force
 *   </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param tst_gen_state_error_os </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 4 </td>
 *   <td width="30%"> Fatal Error Occured while processing a TASK. TESTGEN
 *        state machine is halted at the failure.  The errored
 *        task must be fixed and the sw_reset signal written
 *        to restart the test. Force
 *   </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param cont_after_pause_os </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 3 </td>
 *   <td width="30%"> Active anytime the TESTGEN state machine is paused.
 *        A write to cont_after_pause or single_step one-shot
 *        registers will release the state machine from the
 *        paused state. Force
 *   </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param pause_in_shiftir_os </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 2 </td>
 *   <td width="30%"> Paused in Shift-IR state Force </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param pause_in_shiftdr_os </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 1 </td>
 *   <td width="30%"> Paused in Shift-DR state Force </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param curr_task_end_of_test_os </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 0 </td>
 *   <td width="30%"> In End-of-Test Task Force </td>
 * </tr>
 * </table>
 * Applies to: 5500 --> ncp_axis_mtc_mtc_tstgen_int_force_r_t
 *
 */

struct ncp_axis_mtc_mtc_tstgen_int_force_r_t {
#ifdef NCP_BIG_ENDIAN
	unsigned reserved0:25;
	unsigned tdo_record_ram_addr_overflow_os:1;
	unsigned nxt_task_error_os:1;
	unsigned tst_gen_state_error_os:1;
	unsigned cont_after_pause_os:1;
	unsigned pause_in_shiftir_os:1;
	unsigned pause_in_shiftdr_os:1;
	unsigned curr_task_end_of_test_os:1;
#else				/* Little Endian */
	unsigned curr_task_end_of_test_os:1;
	unsigned pause_in_shiftdr_os:1;
	unsigned pause_in_shiftir_os:1;
	unsigned cont_after_pause_os:1;
	unsigned tst_gen_state_error_os:1;
	unsigned nxt_task_error_os:1;
	unsigned tdo_record_ram_addr_overflow_os:1;
	unsigned reserved0:25;
#endif
};

/*! @struct ncp_axis_mtc_mtc_ecc_int_status_r_t
 *  @brief Interrupt Status Register
 *  @details This register holds interrupt status
 *  <table width="70%" align="center">
 *  <tr>
 *   <td colspan="4" align="center">
 *    struct ncp_axis_mtc_mtc_ecc_int_status_r_t\n
 *   </td>
 *  </tr>
 *  <tr>
 *     <td width="30%"><b> Name </b></td>
 *     <td width="20%" align="center"><b> Width </b></td>
 *     <td width="20%" align="center"><b> Start Offset </b></td>
 *     <td width="30%"><b> Description </b></td>
 *  </tr>
 *   <tr>
 *     <td width="30%"> @param reserved0 </td>
 *     <td width="20%" align="center"> 28 </td>
 *     <td width="20%" align="center"> 4 </td>
 *   <td width="30%"> Reserved for future use </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param ecc_mult_tstgen_prgm_mem_os </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 3 </td>
 *   <td width="30%"> Multi-bit ECC Error in TESTGEN Program Memory Status </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param ecc_mult_tdo_capture_mem_os </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 2 </td>
 *   <td width="30%"> Multi-bit ECC Error in TDO Capture Memory Status </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param ecc_single_tstgen_prgm_mem_os </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 1 </td>
 * <td width="30%"> Single-bit ECC Error in TESTGEN Program Memory Status </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param ecc_single_tdo_capture_mem_os </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 0 </td>
 *   <td width="30%"> Single-bit ECC Error in TDO Capture Memory Status </td>
 * </tr>
 * </table>
 * Applies to: 5500 --> ncp_axis_mtc_mtc_ecc_int_status_r_t
 *
 */

struct ncp_axis_mtc_mtc_ecc_int_status_r_t {

#ifdef NCP_BIG_ENDIAN
	unsigned reserved0:28;
	unsigned ecc_mult_tstgen_prgm_mem_os:1;
	unsigned ecc_mult_tdo_capture_mem_os:1;
	unsigned ecc_single_tstgen_prgm_mem_os:1;
	unsigned ecc_single_tdo_capture_mem_os:1;
#else				/* Little Endian */
	unsigned ecc_single_tdo_capture_mem_os:1;
	unsigned ecc_single_tstgen_prgm_mem_os:1;
	unsigned ecc_mult_tdo_capture_mem_os:1;
	unsigned ecc_mult_tstgen_prgm_mem_os:1;
	unsigned reserved0:28;
#endif
};

/*! @struct ncp_axis_mtc_mtc_ecc_int_en_r_t
 *  @brief Interrupt Enable Register
 *  @details This register enables interrupts
 *  <table width="70%" align="center">
 *  <tr>
 *   <td colspan="4" align="center">
 *    struct ncp_axis_mtc_mtc_ecc_int_en_r_t\n
 *   </td>
 *  </tr>
 *  <tr>
 *     <td width="30%"><b> Name </b></td>
 *     <td width="20%" align="center"><b> Width </b></td>
 *     <td width="20%" align="center"><b> Start Offset </b></td>
 *     <td width="30%"><b> Description </b></td>
 *  </tr>
 *   <tr>
 *     <td width="30%"> @param reserved0 </td>
 *     <td width="20%" align="center"> 28 </td>
 *     <td width="20%" align="center"> 4 </td>
 *   <td width="30%"> Reserved for future use </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param ecc_mult_tstgen_prgm_mem_os </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 3 </td>
 *   <td width="30%"> Multi-bit ECC Error in TESTGEN Program Memory Enable </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param ecc_mult_tdo_capture_mem_os </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 2 </td>
 *   <td width="30%"> Multi-bit ECC Error in TDO Capture Memory Enable </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param ecc_single_tstgen_prgm_mem_os </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 1 </td>
 * <td width="30%"> Single-bit ECC Error in TESTGEN Program Memory Enable </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param ecc_single_tdo_capture_mem_os </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 0 </td>
 *   <td width="30%"> Single-bit ECC Error in TDO Capture Memory Enable </td>
 * </tr>
 * </table>
 * Applies to: 5500 --> ncp_axis_mtc_mtc_ecc_int_en_r_t
 *
 */

struct ncp_axis_mtc_mtc_ecc_int_en_r_t {
#ifdef NCP_BIG_ENDIAN
	unsigned reserved0:28;
	unsigned ecc_mult_tstgen_prgm_mem_os:1;
	unsigned ecc_mult_tdo_capture_mem_os:1;
	unsigned ecc_single_tstgen_prgm_mem_os:1;
	unsigned ecc_single_tdo_capture_mem_os:1;
#else				/* Little Endian */
	unsigned ecc_single_tdo_capture_mem_os:1;
	unsigned ecc_single_tstgen_prgm_mem_os:1;
	unsigned ecc_mult_tdo_capture_mem_os:1;
	unsigned ecc_mult_tstgen_prgm_mem_os:1;
	unsigned reserved0:28;
#endif
};

/*! @struct ncp_axis_mtc_mtc_ecc_int_force_r_t
 *  @brief Interrupt Force Register
 *  @details This address is an alias for the Interrupt Status register
       that allows normal CFG writes (as opposed to the Clear-On-Write-One
	behavior if the Interrupt Status register address is used).  This
	allows CFG to set interrupt bits for testing purposes.  Reading this
	address returns the current value of the Interrupt Status Register.
 *  <table width="70%" align="center">
 *  <tr>
 *   <td colspan="4" align="center">
 *    struct ncp_axis_mtc_mtc_ecc_int_force_r_t\n
 *   </td>
 *  </tr>
 *  <tr>
 *     <td width="30%"><b> Name </b></td>
 *     <td width="20%" align="center"><b> Width </b></td>
 *     <td width="20%" align="center"><b> Start Offset </b></td>
 *     <td width="30%"><b> Description </b></td>
 *  </tr>
 *   <tr>
 *     <td width="30%"> @param reserved0 </td>
 *     <td width="20%" align="center"> 28 </td>
 *     <td width="20%" align="center"> 4 </td>
 *   <td width="30%"> Reserved for future use </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param ecc_mult_tstgen_prgm_mem_os </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 3 </td>
 *   <td width="30%"> Multi-bit ECC Error in TESTGEN Program Memory Force </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param ecc_mult_tdo_capture_mem_os </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 2 </td>
 *   <td width="30%"> Multi-bit ECC Error in TDO Capture Memory Force </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @param ecc_single_tstgen_prgm_mem_os </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 1 </td>
 *   <td width="30%"> Single-bit ECC Error in TESTGEN Program Memory Force </td>
 * </tr>
 *   <tr>
 *     <td width="30%"> @:param ecc_single_tdo_capture_mem_os </td>
 *     <td width="20%" align="center"> 1 </td>
 *     <td width="20%" align="center"> 0 </td>
 *   <td width="30%"> Single-bit ECC Error in TDO Capture Memory Force </td>
 * </tr>
 * </table>
 * Applies to: 5500 --> ncp_axis_mtc_mtc_ecc_int_force_r_t
 *
 */

struct ncp_axis_mtc_mtc_ecc_int_force_r_t {
#ifdef NCP_BIG_ENDIAN
	unsigned reserved0:28;
	unsigned ecc_mult_tstgen_prgm_mem_os:1;
	unsigned ecc_mult_tdo_capture_mem_os:1;
	unsigned ecc_single_tstgen_prgm_mem_os:1;
	unsigned ecc_single_tdo_capture_mem_os:1;
#else				/* Little Endian */
	unsigned ecc_single_tdo_capture_mem_os:1;
	unsigned ecc_single_tstgen_prgm_mem_os:1;
	unsigned ecc_mult_tdo_capture_mem_os:1;
	unsigned ecc_mult_tstgen_prgm_mem_os:1;
	unsigned reserved0:28;
#endif
};

/******************************************************/
/* end of RDL register definitions */
/******************************************************/

struct mtc_device {
	struct kref ref;
	struct platform_device *pdev;
	unsigned long flags;
#define FLAG_REGISTERED        0	/* Misc device registered */
	struct mtc_regs __iomem *regs;
	u32 __iomem *prgmem;
	u32 __iomem *tdomem;
	unsigned int irq;
	struct miscdevice char_device;
};

#define miscdev_to_mtc(mdev) container_of(mdev, struct mtc_device, char_device)

/* Called when removed and last reference is released */
static void mtc_destroy(struct kref *ref);

/**
 * mtc_dev_open
 *
 * Returns 0 for success or negative errno.
 */
static int mtc_dev_open(struct inode *inode, struct file *filp)
{
	struct miscdevice *misc = filp->private_data;
	struct mtc_device *dev = miscdev_to_mtc(misc);
	pr_debug("mtc_dev_open(%p)\n", dev);
	kref_get(&dev->ref);
	return 0;
}

/**
 * mtc_dev_release
 */
static int mtc_dev_release(struct inode *inode, struct file *filp)
{
	struct miscdevice *misc = filp->private_data;
	struct mtc_device *dev = miscdev_to_mtc(misc);
	pr_debug("mtc_dev_release(%p)\n", dev);
	kref_put(&dev->ref, mtc_destroy);
	return 0;
}

/**
 * mtc_dev_read
 *
 * Returns number of valid bits read or negative errno.
 */
static ssize_t
mtc_dev_read(struct file *filp, char __user *data, size_t len, loff_t *ppose)
{
	struct miscdevice *misc = filp->private_data;
	struct mtc_device *dev = miscdev_to_mtc(misc);
	u32 __iomem *ptdo;
	u32 tdo_size_word, tdo_size_bit;	/* data to be read in words */
	struct ncp_axis_mtc_MTC_STATUS1_REG_ADDR_r_t status1Reg = { 0 };
	struct ncp_axis_mtc_MTC_EXECUTE1_REG_ADDR_r_t exec1Reg = { 0 };

	pr_debug("mtc_dev_read(%u @ %llu)\n", len, *ppose);
	ptdo = dev->tdomem;

	/* flush tdo buffer */
	exec1Reg.tdo_flush_capture_data = 1;
	dev->regs->execute = *((u32 *) &exec1Reg);

	/* read status 1register */
	status1Reg =
	    *((struct ncp_axis_mtc_MTC_STATUS1_REG_ADDR_r_t *)
	      &(dev->regs->status1));
	tdo_size_bit = status1Reg.tdo_record_ram_bit_counter;

	/* test code for mtc memory sim only */
	/* tdo_size_bit = 8192; */
	/* end of test code */

	tdo_size_word = (tdo_size_bit + 31) / 32;
#ifdef DEBUG
	pr_debug("mtc_dev_read(), tdo_bit=%d tdo_word=%d\n",
		 tdo_size_bit, tdo_size_word);
#endif
	/* copy tdo data to user space, always read from location 0
	   because we reset tdomem after each read */
	if (copy_to_user(data, ptdo, tdo_size_word * 4))
		return -EFAULT;

	/* data sent to user, reset tdo capture buffer */
	memset(&exec1Reg, 0, sizeof(exec1Reg));
	exec1Reg.tdo_capture_reset = 1;
	dev->regs->execute = *((u32 *) &exec1Reg);

	/* return # of bits */
	return tdo_size_bit;
}

/**
 * mtc_dev_write
 *
 * Returns number of bytes written or negative errno.
 */

static ssize_t
mtc_dev_write(struct file *filp,
	      const char __user *data,
	      size_t len,
	      loff_t *ppose)
{

	struct miscdevice *misc = filp->private_data;
	struct mtc_device *dev = miscdev_to_mtc(misc);
	u32 __iomem *pprg;
	u32 mtc_buf[256];	/* max 256 words */
	u32 size, size1, isWraparound = 0, i;	/* size in word */
	struct ncp_axis_mtc_MTC_STATUS1_REG_ADDR_r_t status1Reg = { 0 };

	pr_debug("mtc_dev_write(%u @ %llu)\n", len, *ppose);

	if (len > 1024)
		return -EINVAL;

	size = len / 4;
	size1 = size;

	/* copy to a lcoal buffer */
	memset(mtc_buf, 0, 1024);
	if (copy_from_user((void *)mtc_buf, (void *)data, len)) {
		pr_debug("MTC Error write\n");
		return -EFAULT;
	}

	/* read status 1register, find the starting write offset */
	status1Reg =
	    *((struct ncp_axis_mtc_MTC_STATUS1_REG_ADDR_r_t *)
	      &(dev->regs->status1));

	/* TEST CODE when mtc sim is used */
	/* status1Reg.prgm_mem_rd_addr = 253; */
	/* pr_debug("buf offset=%d\n",status1Reg.prgm_mem_rd_addr ); */
	/*END OF TEST CODE */

	/* find starting location of the write */
	pprg = dev->prgmem + status1Reg.prgm_mem_rd_addr;

	/* determine if wrap around is needed */
	if (status1Reg.prgm_mem_rd_addr + size > 256) {
		isWraparound = 1;
		/* number of words load from starting location */
		size1 = 256 - status1Reg.prgm_mem_rd_addr;
		/*number of words load from location 0 will be size-size1 */
#ifdef DEBUG
		pr_debug("Wraparound size=%d, size1=%d size-size1=%d\n", size,
			 size1, size - size1);
#endif
	}
#ifdef DEBUG
	pr_debug("Appending buff size1=%d\n", size1);
#endif

	for (i = 0; i < size1; i++) {
#ifdef DEBUG
		pr_debug("i=%d mtc_buf[i]=%d pprg=0x%x\n",
			 i, mtc_buf[i], (u32) pprg);
#endif
		*pprg = mtc_buf[i];
		pprg++;
	}

	/* wraparound, copy the 2nd half */
	if (isWraparound) {
		pprg = dev->prgmem;	/* reset write pointer to location 0 */
#ifdef DEBUG
		pr_debug("\n\nWraparound buff size=%d\n",
			 size - size1);
#endif
		for (i = 0; i < (size - size1); i++) {
#ifdef DEBUG
			pr_debug("i=%d mtc_buf[size1+i]=%d pprg=0x%x\n", i,
				 mtc_buf[size1 + i], (u32) pprg);
#endif
			*pprg = mtc_buf[size1 + i];
			pprg++;
		}
	}

	return len;
}

/**
 * mtc_dev_ioctl
 *
 */

static long _mtc_config(struct mtc_device *dev, struct lsi_mtc_cfg_t *pMTCCfg);

static long
mtc_dev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct miscdevice *misc = filp->private_data;
	struct mtc_device *dev = miscdev_to_mtc(misc);
	long ret = 0;
	u32 addr, tmp2;
	unsigned long numByteCopied;

	pr_debug("mtc_dev_ioctl(%#x, %#lx)\n", cmd, arg);

	switch (cmd) {

	case MTC_DEBUG_OP:
		numByteCopied =
		    copy_from_user((void *)&addr, (void *)arg,
				   sizeof(unsigned int));
		if (numByteCopied) {
			pr_debug("MTC Error ioctl\n");
			return -EFAULT;
		}
		tmp2 = *((u32 *) dev->regs + addr / 4);

		if (copy_to_user((void *)arg, &tmp2, sizeof(unsigned int)))
			return -EFAULT;

		break;

	case MTC_CFG:
		{
			struct lsi_mtc_cfg_t mtc_cfg;
			if (copy_from_user
			    ((void *)&mtc_cfg, (void *)arg, sizeof(mtc_cfg))) {
				pr_debug("MTC Error ioctl\n");
				return -EFAULT;
			}

			ret = _mtc_config(dev, &mtc_cfg);
		}
		break;

	case MTC_SINGLESTEP_ENABLE:
		{
			struct ncp_axis_mtc_MTC_CONFIG0_REG_ADDR_r_t cfg0 = {0};
			int single_step;

			if (copy_from_user
			    ((void *)&single_step, (void *)arg, sizeof(int))) {
				pr_debug("MTC Error ioctl\n");
				return -EFAULT;
			}

			if ((single_step != 0) && (single_step != 1))
				return -EINVAL;

			cfg0 =
			    *((struct ncp_axis_mtc_MTC_CONFIG0_REG_ADDR_r_t *)
			      &(dev->regs->config0));
			cfg0.single_step_en = single_step;
			dev->regs->config0 = *((u32 *) &cfg0);
#ifdef DEBUG
			pr_debug(
			    "MTC_SINGLESTEP_ENABLE: dev->regs->config0=0x%x\n",
			    dev->regs->config0);
#endif
		}

		break;

	case MTC_LOOPMODE_ENABLE:
		{
			struct ncp_axis_mtc_MTC_CONFIG0_REG_ADDR_r_t cfg0 = {0};
			int loop_mode;
			if (copy_from_user
			    ((void *)&loop_mode, (void *)arg, sizeof(int))) {
				pr_debug("MTC Error ioctl\n");
				return -EFAULT;
			}

			if ((loop_mode != 0) && (loop_mode != 1))
				return -EINVAL;

			cfg0 =
			    *((struct ncp_axis_mtc_MTC_CONFIG0_REG_ADDR_r_t *)
			      &(dev->regs->config0));
			cfg0.loop_en = loop_mode;
			dev->regs->config0 = *((u32 *) &cfg0);
#ifdef DEBUG
			pr_debug(
			       "MTC_LOOPMODE_ENABLE dev->regs->config0=0x%x\n",
			       dev->regs->config0);
#endif
		}

		break;

	case MTC_RESET:
		{
			struct
			 ncp_axis_mtc_MTC_EXECUTE1_REG_ADDR_r_t exec1 = {0};
			exec1.sw_reset = 1;
			dev->regs->execute = *((u32 *) &exec1);
#ifdef DEBUG
			pr_debug("dev->regs->execute=0x%x\n",
				 dev->regs->execute);
#endif
		}

		break;

	case MTC_TCKCLK_GATE:
		{
			struct lsi_mtc_tckclk_gate_t tckGate;
			struct ncp_axis_mtc_MTC_CONFIG1_REG_ADDR_r_t cfg1 = {0};

			if (copy_from_user
			    ((void *)&tckGate, (void *)arg, sizeof(tckGate))) {
				pr_debug("MTC Error ioctl\n");
				return -EFAULT;
			}

			if (((tckGate.gate_tck_test_logic_reset != 0)
			     && (tckGate.gate_tck_test_logic_reset != 1))
			    || ((tckGate.gate_tck != 0)
				&& (tckGate.gate_tck != 1)))
				return -EINVAL;

			cfg1 =
			    *((struct ncp_axis_mtc_MTC_CONFIG1_REG_ADDR_r_t *)
			      &(dev->regs->config1));
			cfg1.sw_gate_tck_test_logic_reset =
			    tckGate.gate_tck_test_logic_reset;
			cfg1.sw_gate_tck = tckGate.gate_tck;
			dev->regs->config1 = *((u32 *) &cfg1);
#ifdef DEBUG
			pr_debug("dev->regs->config1=0x%x\n",
				 dev->regs->config1);
#endif
		}

		break;

	case MTC_STARTSTOP_EXEC:
		{
			struct ncp_axis_mtc_MTC_CONFIG0_REG_ADDR_r_t cfg0 = {0};
			int start_stop;
			if (copy_from_user
			    ((void *)&start_stop, (void *)arg, sizeof(int))) {
				pr_debug("MTC Error ioctl\n");
				return -EFAULT;
			}

			if ((start_stop != 0) && (start_stop != 1))
				return -EINVAL;

			cfg0 =
			    *((struct ncp_axis_mtc_MTC_CONFIG0_REG_ADDR_r_t *)
			      &(dev->regs->config0));
			cfg0.start_stopn = start_stop;
			dev->regs->config0 = *((u32 *) &cfg0);
#ifdef DEBUG
			pr_debug("dev->regs->config0=0x%x\n",
				 dev->regs->config0);
#endif
		}

		break;

	case MTC_SINGLESTEP_EXEC:
		{
			struct
			 ncp_axis_mtc_MTC_EXECUTE1_REG_ADDR_r_t exec1 = {0};
			exec1.single_step = 1;
			dev->regs->execute = *((u32 *) &exec1);
			pr_debug("dev->regs->execute=0x%x\n",
				 dev->regs->execute);

		}
		break;

	case MTC_CONTINUE_EXEC:
		{
			struct
			 ncp_axis_mtc_MTC_EXECUTE1_REG_ADDR_r_t exec1 = {0};
			exec1.cont_after_pause = 1;
			dev->regs->execute = *((u32 *) &exec1);
			pr_debug("dev->regs->execute=0x%x\n",
				 dev->regs->execute);

		}
		break;

	case MTC_READ_STATS:
		{
			struct lsi_mtc_stats_regs_t stats;

			stats.statsReg1 = dev->regs->status1;
			stats.statsReg2 = dev->regs->status2;

			if (copy_to_user((void *)arg, &stats, sizeof(stats)))
				return -EFAULT;

		}
		break;

	case MTC_READ_DEBUG:
		{
			struct lsi_mtc_debug_regs_t debug;

			debug.debugReg0 = dev->regs->debug0;
			debug.debugReg1 = dev->regs->debug1;
			debug.debugReg2 = dev->regs->debug2;
			debug.debugReg3 = dev->regs->debug3;
			debug.debugReg4 = dev->regs->debug4;
			debug.debugReg5 = dev->regs->debug5;

			if (copy_to_user((void *)arg, &debug, sizeof(debug)))
				return -EFAULT;

		}

		break;

	default:
		pr_debug("Invalid ioctl cmd=%d MTC_DEBUG_OP=%d\n",
			 cmd, MTC_DEBUG_OP);
		ret = -EINVAL;

	}

	return ret;
}

static const struct file_operations mtc_char_ops = {
	.owner = THIS_MODULE,
	.open = mtc_dev_open,
	.release = mtc_dev_release,
	.llseek = generic_file_llseek,
	.read = mtc_dev_read,
	.write = mtc_dev_write,
	.unlocked_ioctl = mtc_dev_ioctl
};

static irqreturn_t mtc_isr(int irq_no, void *arg)
{
	struct mtc_device *priv = arg;
	u32 status = readl(&priv->regs->int_status);

	pr_debug("mtc: int status %#x\n", status);

	/* Handle interrupt */
	/* ... */

	/* Write bits to clear interrupt status */
	writel(status, &priv->regs->int_status);

	return IRQ_HANDLED;
}

/**
 * mtc_probe
 *
 * Initialize device.
 */
static int mtc_probe(struct platform_device *pdev)
{
	static struct mtc_device *dev;
	void __iomem *regs;
	int rc;
	u32 *pRegs;

	pr_debug("!!!!MTC: mtc_probe()\n");
	/* Allocate space for device private data */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		rc = -ENOMEM;
		goto err;
	}
	dev_set_drvdata(&pdev->dev, &dev);
	kref_init(&dev->ref);
	dev->pdev = pdev;

	/* Map hardware registers */
	regs = of_iomap(pdev->dev.of_node, 0);
	if (!regs) {
		rc = -EINVAL;
		goto err;
	}
	pRegs = (u32 *) regs;
#ifdef __MTC_SIMULATION
	dev->regs = &_mtc_regs;
	dev->prgmem = _mtc_prgmem;
	dev->tdomem = _mtc_tdomem;

#else
	dev->regs = regs;
	dev->prgmem =  pRegs + MTC_PRGMEM_OFFSET/4;
	dev->tdomem =  pRegs + MTC_TDOMEM_OFFSET/4;
#endif
	/* Attach to IRQ */
	dev->irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	rc = request_irq(dev->irq, mtc_isr, 0, "mtc", &dev);
	if (rc)
		goto err;

	/* Initialize hardware */
	/* ... */

	/*
	 * Register device client interface
	 */

	dev->char_device.minor = MISC_DYNAMIC_MINOR;
	dev->char_device.name = "mtc";
	dev->char_device.fops = &mtc_char_ops;

	rc = misc_register(&dev->char_device);
	if (rc)
		goto err;
	set_bit(FLAG_REGISTERED, &dev->flags);

	return 0;

 err:
	if (dev)
		kref_put(&dev->ref, mtc_destroy);
	dev_err(&pdev->dev, "Failed to probe device (%d)\n", rc);
	return rc;
}

/**
 * mtc_remove
 */
static int mtc_remove(struct platform_device *pdev)
{
	struct mtc_device *dev = dev_get_drvdata(&pdev->dev);
	kref_put(&dev->ref, mtc_destroy);
	return 0;
}

/*
 * mtc_destroy
 *
 * Called when refcount reaches zero to unregister device and free resources.
 */
static void mtc_destroy(struct kref *ref)
{
	struct mtc_device *dev = container_of(ref, struct mtc_device, ref);

	dev_set_drvdata(&dev->pdev->dev, NULL);
	if (test_and_clear_bit(FLAG_REGISTERED, &dev->flags))
		misc_deregister(&dev->char_device);
	if (dev->irq)
		free_irq(dev->irq, &dev);
	iounmap(dev->regs);
	kfree(dev);
}

#ifdef CONFIG_PM
static int mtc_suspend(struct platform_device *pdev, pm_message_t state)
{
	return -ENOSYS;
}

static int mtc_resume(struct platform_device *pdev)
{
	return -ENOSYS;
}
#else
#define mtc_suspend	NULL
#define mtc_resume	NULL
#endif

static const struct of_device_id mtc_of_ids[] = {
	{.compatible = "lsi,mtc"},
	{}
};

static struct platform_driver mtc_driver = {
	.driver = {
		   .name = "mtc",
		   .owner = THIS_MODULE,
		   .of_match_table = mtc_of_ids,
		   },
	.probe = mtc_probe,
	.remove = mtc_remove,
	.suspend = mtc_suspend,
	.resume = mtc_resume
};

module_platform_driver(mtc_driver);

MODULE_AUTHOR("LSI Corporation");
MODULE_DESCRIPTION("Master Test Controller driver");
MODULE_LICENSE("GPL");

/* MTC operating mode. */
#define  LSI_MTC_BOARDTEST_MODE 0  /* MTC Board Test Mode.  */
#define  LSI_MTC_EXTTEST_MODE   1  /* MTC External Test Mode DBC3 excluded. */
#define  LSI_MTC_SYSTTEST_MODE  2  /* MTC System Test Mode DBC3 excluded. */

/* Test data output recording mode. */
/* Do not save TDO data during shiftir and shiftdr. */
#define   LSI_MTC_TDO_NOREC 0
  /* Do not save TDO data during shiftir, but save TDO during shiftdr. */
#define   LSI_MTC_TDO_NOREC_SHIFTIR 1
  /* Do not save TDO data during shiftdr, but save TDO during shiftir. */
#define  LSI_MTC_TDO_NOREC_SHIFTDR 2
  /* Save TDO data during shiftdr and shiftir. */
#define   ACELL_MTCI_TDO_REC_ALL  3

/* Test data output buffer mode. */
/* TDO output buffer always enabled. */
#define  LSI_MTC_TDOBUF_ENABLED 0
/* TDO output buffer active  when in shift-DR or shift-IR states
   and inactive in all other states. */
#define  LSI_MTC_TDOBUF_TRISTATE_CAPABLE 1

/* config MTC hardware */
static long _mtc_config(struct mtc_device *dev, struct lsi_mtc_cfg_t *pMTCCfg)
{

	struct ncp_axis_mtc_MTC_CONFIG0_REG_ADDR_r_t cfg0 = { 0 };
	struct ncp_axis_mtc_MTC_CONFIG1_REG_ADDR_r_t cfg1 = { 0 };
	struct ncp_axis_mtc_MTC_EXECUTE1_REG_ADDR_r_t exec1 = { 0 };

	if ((!pMTCCfg) || (!dev))
		return -EINVAL;

	/* 1. stop testgen state machine */
	cfg0 =
	    *((struct ncp_axis_mtc_MTC_CONFIG0_REG_ADDR_r_t *)
	      &(dev->regs->config0));
	cfg0.start_stopn = 0;
	dev->regs->config0 = *((u32 *) &cfg0);

	/* 2. reset testgen, and init mem */
	exec1.sw_reset = 1;
	dev->regs->execute = *((u32 *) &exec1);
	dev->regs->mem_init = 0x202;

	/* 3. config MTC */
	cfg0 =
	    *((struct ncp_axis_mtc_MTC_CONFIG0_REG_ADDR_r_t *)
	      &(dev->regs->config0));

	cfg1 =
	    *((struct ncp_axis_mtc_MTC_CONFIG1_REG_ADDR_r_t *)
	      &(dev->regs->config1));

	/*set MTC mode */
	if (pMTCCfg->opMode == LSI_MTC_BOARDTEST_MODE) {
		/* board testing mode */
		cfg0.cfg_config_ctl = 0;
	} else if (pMTCCfg->opMode == LSI_MTC_EXTTEST_MODE) {
		/* external testing mode */
		cfg0.cfg_config_ctl = 3;
	} else {
		/* system testing mode */
		cfg0.cfg_config_ctl = 5;

	}

	/* set clock rate */
	cfg0.rate_sel = pMTCCfg->clkSpeed;

	/* set TDO buffer mode */
	cfg0.mtc_mpu_tdo_inactive_en = pMTCCfg->buffMode;

	/* set TDO recording mode */
	if (pMTCCfg->recMode == LSI_MTC_TDO_NOREC) {
		cfg1.record_tdo_in_shift_ir_state = 0;
		cfg1.record_tdo_in_shift_dr_state = 0;
	} else if (pMTCCfg->recMode == LSI_MTC_TDO_NOREC_SHIFTIR) {
		cfg1.record_tdo_in_shift_ir_state = 0;
		cfg1.record_tdo_in_shift_dr_state = 1;
	} else if (pMTCCfg->recMode == LSI_MTC_TDO_NOREC_SHIFTDR) {
		cfg1.record_tdo_in_shift_ir_state = 0;
		cfg1.record_tdo_in_shift_dr_state = 1;
	} else {
		/* recMode == ACELL_MTCI_TDO_REC_ALL */
		cfg1.record_tdo_in_shift_ir_state = 1;
		cfg1.record_tdo_in_shift_dr_state = 1;
	}

	dev->regs->config0 = *((u32 *) &cfg0);
	dev->regs->config1 = *((u32 *) &cfg1);

#ifdef DEBUG
	pr_debug("buffmode=%d,rate=%d dev->regs->config0 =0x%x\n",
		 pMTCCfg->buffMode, pMTCCfg->clkSpeed, dev->regs->config0);

	pr_debug("dev->regs->config1 =0x%x, dev->regs-> execute=0x%x\n",
		 dev->regs->config1, dev->regs->execute);
#endif
	/* test */
	return 0;
}
