#ifndef __MIS4001_CMOS_PARAM_H_
#define __MIS4001_CMOS_PARAM_H_

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif

#include <linux/cvi_type.h>
#include "cvi_sns_ctrl.h"
#include "mis4001_cmos_ex.h"

static const MIS4001_MODE_S g_astMIS4001_mode[MIS4001_MODE_NUM] = {
	[MIS4001_MODE_2560X1440P30] = {
		.name = "2560x1440p30",
		.astImg[0] = {
			.stSnsSize = {
				.u32Width = 2576,
				.u32Height = 1456,
			},
			.stWndRect = {
				.s32X = 8,
				.s32Y = 8,
				.u32Width = 2560,
				.u32Height = 1440,
			},
			.stMaxSize = {
				.u32Width = 2576,
				.u32Height = 1456,
			},
		},
		.f32MaxFps = 30,
		.f32MinFps = 2.75,
		.u32HtsDef = 3000,
		.u32VtsDef = 1500,
		.stExp[0] = {
			.u32Min = 2,
			.u32Max = 1500 - 4,
			.u32Def = 0x0100,
			.u32Step = 1,
		},
		.stAgain[0] = {
			.u32Min = 1024,
			.u32Max = 16128,
			.u32Def = 1024,
			.u32Step = 1,
		},
		.stDgain[0] = {
			.u32Min = 1024,
			.u32Max = 16376,
			.u32Def = 1024,
			.u32Step = 1,
		},
	},
};

static ISP_CMOS_BLACK_LEVEL_S g_stIspBlcCalibratio = {
	.bUpdate = CVI_TRUE,
	.blcAttr = {
		.Enable = 1,
		.enOpType = OP_TYPE_AUTO,
		.stManual = {124, 124, 124, 124, 0, 0, 0, 0},
		.stAuto = {
			{124, 124, 124, 124, 124, 124, 124, 124, 124, 124, 124, 124, 124, 124, 124, 124},
			{124, 124, 124, 124, 124, 124, 124, 124, 124, 124, 124, 124, 124, 124, 124, 124},
			{124, 124, 124, 124, 124, 124, 124, 124, 124, 124, 124, 124, 124, 124, 124, 124},
			{124, 124, 124, 124, 124, 124, 124, 124, 124, 124, 124, 124, 124, 124, 124, 124},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		},
	},
};

struct combo_dev_attr_s mis4001_rx_attr = {
	.input_mode = INPUT_MODE_MIPI,
	.mac_clk = RX_MAC_CLK_200M,
	.mipi_attr = {
		.raw_data_type = RAW_DATA_10BIT,
		.lane_id = {2, 0, 1, -1, -1},
		.pn_swap = {0, 0, 0, 0, 0},
		.wdr_mode = CVI_MIPI_WDR_MODE_NONE,
		.dphy = {
			.enable = 1,
			.hs_settle = 8,
		},
	},
	.mclk = {
		.cam = 0,
		.freq = CAMPLL_FREQ_27M,
	},
	.devno = 0,
};

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif

#endif /* __MIS4001_CMOS_PARAM_H_ */
