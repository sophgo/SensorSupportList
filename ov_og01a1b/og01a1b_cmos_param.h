#ifndef __OG01A1B_CMOS_PARAM_H_
#define __OG01A1B_CMOS_PARAM_H_

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif

#include <cvi_comm_cif.h>
#include <cvi_type.h>
#include "cvi_sns_ctrl.h"
#include "og01a1b_cmos_ex.h"

static const OG01A1B_MODE_S g_astOg01a1b_mode[OG01A1B_MODE_NUM] = {
	[OG01A1B_MODE_640P50_8bit] = {
		.name = "640p50_1lane_8bit",
		.astImg[0] = {
			.stSnsSize = {
				.u32Width = 640,
				.u32Height = 400,
			},
			.stWndRect = {
				.s32X = 0,
				.s32Y = 0,
				.u32Width = 640,
				.u32Height = 400,
			},
			.stMaxSize = {
				.u32Width = 640,
				.u32Height = 400,
			},
		},
		.f32MaxFps = 50,
		.f32MinFps = 0.46, /* u32VtsDef * f32MaxFps / 0xFFFF */
		.u32HtsDef = 800,
		.u32VtsDef = 1538, //0x0602
		.stExp[0] = {
			.u16Min = 1,
			.u16Max = 1524,      /* u32VtsDef - 14 */
			.u16Def = 627,
			.u16Step = 1,
		},
		.stAgain[0] = {
			.u32Min = 1024,
			.u32Max = 15872,
			.u32Def = 1024,
			.u32Step = 1,
		},
		.stDgain[0] = {
			.u32Min = 1024,
			.u32Max = 16320,
			.u32Def = 1024,
			.u32Step = 1,
		},
	},
};

static ISP_CMOS_BLACK_LEVEL_S g_stIspBlcCalibration = {
	.bUpdate = CVI_TRUE,
	.blcAttr = {
		.Enable = 1,
		.enOpType = OP_TYPE_AUTO,
		.stManual = {252, 252, 252, 252, 0, 0, 0, 0
		},
		.stAuto = {
			{252, 252, 252, 252, 252, 252, 252, 252, /*8*/252, 252, 252, 252, 252, 252, 252, 252},
			{252, 252, 252, 252, 252, 252, 252, 252, /*8*/252, 252, 252, 252, 252, 252, 252, 252},
			{252, 252, 252, 252, 252, 252, 252, 252, /*8*/252, 252, 252, 252, 252, 252, 252, 252},
			{252, 252, 252, 252, 252, 252, 252, 252, /*8*/252, 252, 252, 252, 252, 252, 252, 252},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		},
	},
};

struct combo_dev_attr_s og01a1b_rx_attr = {
	.input_mode = INPUT_MODE_MIPI,
	.mac_clk = RX_MAC_CLK_400M,
	.mipi_attr = {
		.raw_data_type = RAW_DATA_8BIT,
		.lane_id = {0, 1, 2, -1, -1},
		.wdr_mode = CVI_MIPI_WDR_MODE_NONE,
		.dphy = {
			.enable = 1,
			.hs_settle = 8,
		},
	},
	.mclk = {
		.cam = 0,
		.freq = CAMPLL_FREQ_24M,
	},
	.devno = 0,
};

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */


#endif /* __OG01A1B_CMOS_PARAM_H_ */
