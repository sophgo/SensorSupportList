#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <unistd.h>
#include <cvi_type.h>
#include <cvi_comm_video.h>
#include "cvi_debug.h"
#include "cvi_comm_sns.h"
#include "cvi_sns_ctrl.h"
#include "cvi_ae_comm.h"
#include "cvi_awb_comm.h"
#include "cvi_ae.h"
#include "cvi_awb.h"
#include "cvi_isp.h"

#include "imx586_cmos_ex.h"
#include "imx586_cmos_param.h"

#define DIV_0_TO_1(a)   ((0 == (a)) ? 1 : (a))
#define DIV_0_TO_1_FLOAT(a) ((((a) < 1E-10) && ((a) > -1E-10)) ? 1 : (a))
#define IMX586_ID 586

#define SENSOR_IMX586_12M_WIDTH  4000
#define SENSOR_IMX586_12M_HEIGHT 3000
#define SENSOR_IMX586_8M_WIDTH  3840
#define SENSOR_IMX586_8M_HEIGHT 2160
#define SENSOR_IMX586_2M_WIDTH  1920
#define SENSOR_IMX586_2M_HEIGHT 1080
#define IMX586_I2C_ADDR_1 0x10
#define IMX586_I2C_ADDR_2 0x1A
#define IMX586_I2C_ADDR_IS_VALID(addr)      ((addr) == IMX586_I2C_ADDR_1 || (addr) == IMX586_I2C_ADDR_2)

/****************************************************************************
 * global variables                                                         *
 ****************************************************************************/

ISP_SNS_STATE_S *g_pastImx586[VI_MAX_PIPE_NUM] = {CVI_NULL};
SNS_COMBO_DEV_ATTR_S *g_pastImx586ComboDevArray[VI_MAX_PIPE_NUM] = {CVI_NULL};

#define IMX586_SENSOR_GET_CTX(dev, pstCtx)   (pstCtx = g_pastImx586[dev])
#define IMX586_SENSOR_SET_CTX(dev, pstCtx)   (g_pastImx586[dev] = pstCtx)
#define IMX586_SENSOR_RESET_CTX(dev)         (g_pastImx586[dev] = CVI_NULL)
#define IMX586_SENSOR_SET_COMBO(dev, pstCtx)   (g_pastImx586ComboDevArray[dev] = pstCtx)
#define IMX586_SENSOR_GET_COMBO(dev, pstCtx)   (pstCtx = g_pastImx586ComboDevArray[dev])

ISP_SNS_COMMBUS_U g_aunImx586_BusInfo[VI_MAX_PIPE_NUM] = {
	[0] = { .s8I2cDev = 2},
	[1 ... VI_MAX_PIPE_NUM - 1] = { .s8I2cDev = -1}
};

ISP_SNS_COMMADDR_U g_aunImx586_AddrInfo[VI_MAX_PIPE_NUM] = {
	[0] = { .s8I2cAddr = IMX586_I2C_ADDR_2},
	[1 ... VI_MAX_PIPE_NUM - 1] = { .s8I2cAddr = -1}
};

CVI_U16 g_au16Imx586_GainMode[VI_MAX_PIPE_NUM] = {0};

ISP_SNS_MIRRORFLIP_TYPE_E g_aeImx586_MirrorFip[VI_MAX_PIPE_NUM] = {0};

/****************************************************************************
 * local variables and functions                                            *
 ****************************************************************************/
static CVI_U32 g_au32InitExposure[VI_MAX_PIPE_NUM]  = {0};
static CVI_U32 g_au32LinesPer500ms[VI_MAX_PIPE_NUM] = {0};
static CVI_U16 g_au16InitWBGain[VI_MAX_PIPE_NUM][3] = {{0} };
static CVI_U16 g_au16SampleRgain[VI_MAX_PIPE_NUM] = {0};
static CVI_U16 g_au16SampleBgain[VI_MAX_PIPE_NUM] = {0};
static CVI_S32 cmos_get_wdr_size(VI_PIPE ViPipe, ISP_SNS_ISP_INFO_S *pstIspCfg);

/*****Imx586 Lines Range*****/
#define IMX586_FULL_LINES_MAX  (0xFFFF)

/*****Imx586 Register Address*****/
#define IMX586_HOLD_ADDR        0x0104

#define IMX586_CIT_H_ADDR       0x0202 //COARSE_INTEG_TIME[15:8]
#define IMX586_CIT_L_ADDR       0x0203 //COARSE_INTEG_TIME[7:0]

#define IMX586_GAIN_H_ADDR      0x0204 //ANA_GAIN_GLOBAL[9:8]
#define IMX586_GAIN_L_ADDR      0x0205 //ANA_GAIN_GLOBAL[7:0]
#define IMX586_DGAIN_H_ADDR     0x020E //DIG_GAIN_GLOBAL[15:8]
#define IMX586_DGAIN_L_ADDR     0x020F //DIG_GAIN_GLOBAL[7:0]

#define IMX586_VMAX_H_ADDR      0x0340 //FRM_LENGTH_LINES[15:8]
#define IMX586_VMAX_L_ADDR      0x0341 //FRM_LENGTH_LINES[7:0]

#define IMX586_FLIP_ADDR         0x0101 //IMG_ORIENTATION

#define IMX586_TABLE_END        0xffff

#define IMX586_RES_IS_12M(w, h)      ((w) == SENSOR_IMX586_12M_WIDTH && (h) == SENSOR_IMX586_12M_HEIGHT)
#define IMX586_RES_IS_8M(w, h)       ((w) == SENSOR_IMX586_8M_WIDTH && (h) == SENSOR_IMX586_8M_HEIGHT)
#define IMX586_RES_IS_2M(w, h)       ((w) == SENSOR_IMX586_2M_WIDTH && (h) == SENSOR_IMX586_2M_HEIGHT)
static CVI_S32 cmos_get_ae_default(VI_PIPE ViPipe, AE_SENSOR_DEFAULT_S *pstAeSnsDft)
{
	ISP_SNS_STATE_S *pstSnsState = CVI_NULL;

	CMOS_CHECK_POINTER(pstAeSnsDft);
	IMX586_SENSOR_GET_CTX(ViPipe, pstSnsState);
	CMOS_CHECK_POINTER(pstSnsState);

	pstAeSnsDft->u32FullLinesStd = pstSnsState->u32FLStd;
	pstAeSnsDft->u32FlickerFreq = 50 * 256;
	pstAeSnsDft->u32FullLinesMax = IMX586_FULL_LINES_MAX;
	pstAeSnsDft->u32HmaxTimes = (1000000) / (g_astImx586_mode[pstSnsState->u8ImgMode].u32VtsDef *
			g_astImx586_mode[pstSnsState->u8ImgMode].f32MaxFps);

	pstAeSnsDft->stIntTimeAccu.enAccuType = AE_ACCURACY_LINEAR;
	pstAeSnsDft->stIntTimeAccu.f32Accuracy = 1;
	pstAeSnsDft->stIntTimeAccu.f32Offset = 0;

	pstAeSnsDft->stAgainAccu.enAccuType = AE_ACCURACY_TABLE;
	pstAeSnsDft->stAgainAccu.f32Accuracy = 1;

	pstAeSnsDft->stDgainAccu.enAccuType = AE_ACCURACY_TABLE;
	pstAeSnsDft->stDgainAccu.f32Accuracy = 1;

	pstAeSnsDft->u32ISPDgainShift = 8;
	pstAeSnsDft->u32MinISPDgainTarget = 1 << pstAeSnsDft->u32ISPDgainShift;
	pstAeSnsDft->u32MaxISPDgainTarget = 2 << pstAeSnsDft->u32ISPDgainShift;

	if (g_au32LinesPer500ms[ViPipe] == 0)
		pstAeSnsDft->u32LinesPer500ms = g_astImx586_mode[pstSnsState->u8ImgMode].u32VtsDef *
			g_astImx586_mode[pstSnsState->u8ImgMode].f32MaxFps / 2;
	else
		pstAeSnsDft->u32LinesPer500ms = g_au32LinesPer500ms[ViPipe];
	pstAeSnsDft->u32SnsStableFrame = 8;

	switch (pstSnsState->enWDRMode) {
	case WDR_MODE_NONE:   /*linear mode*/
		pstAeSnsDft->f32Fps = g_astImx586_mode[pstSnsState->u8ImgMode].f32MaxFps;
		pstAeSnsDft->f32MinFps = g_astImx586_mode[pstSnsState->u8ImgMode].f32MinFps;
		pstAeSnsDft->au8HistThresh[0] = 0xd;
		pstAeSnsDft->au8HistThresh[1] = 0x28;
		pstAeSnsDft->au8HistThresh[2] = 0x60;
		pstAeSnsDft->au8HistThresh[3] = 0x80;

		pstAeSnsDft->u32MaxAgain = 65536;
		pstAeSnsDft->u32MinAgain = 1024;
		pstAeSnsDft->u32MaxAgainTarget = pstAeSnsDft->u32MaxAgain;
		pstAeSnsDft->u32MinAgainTarget = pstAeSnsDft->u32MinAgain;

		pstAeSnsDft->u32MaxDgain = 16380;
		pstAeSnsDft->u32MinDgain = 1024;
		pstAeSnsDft->u32MaxDgainTarget = pstAeSnsDft->u32MaxDgain;
		pstAeSnsDft->u32MinDgainTarget = pstAeSnsDft->u32MinDgain;

		pstAeSnsDft->u8AeCompensation = 40;
		pstAeSnsDft->u32InitAESpeed = 64;
		pstAeSnsDft->u32InitAETolerance = 5;
		pstAeSnsDft->u32AEResponseFrame = 4;
		pstAeSnsDft->enAeExpMode = AE_EXP_HIGHLIGHT_PRIOR;
		pstAeSnsDft->u32InitExposure = g_au32InitExposure[ViPipe] ? g_au32InitExposure[ViPipe] : 76151;

		pstAeSnsDft->u32MaxIntTime = pstSnsState->u32FLStd - 2;
		pstAeSnsDft->u32MinIntTime = 5;
		pstAeSnsDft->u32MaxIntTimeTarget = 65535;
		pstAeSnsDft->u32MinIntTimeTarget = 1;
		break;
	default:
		CVI_TRACE_SNS(CVI_DBG_ERR, "Not support WDR: %d\n", pstSnsState->enWDRMode);
		return CVI_FAILURE;
	}

	return CVI_SUCCESS;
}

static CVI_S32 cmos_fps_set(VI_PIPE ViPipe, CVI_FLOAT f32Fps, AE_SENSOR_DEFAULT_S *pstAeSnsDft)
{
	ISP_SNS_STATE_S *pstSnsState = CVI_NULL;
	CVI_U32 u32VMAX;
	CVI_FLOAT f32MaxFps = 0;
	CVI_FLOAT f32MinFps = 0;
	CVI_U32 u32Vts = 0;
	ISP_SNS_REGS_INFO_S *pstSnsRegsInfo = CVI_NULL;

	CMOS_CHECK_POINTER(pstAeSnsDft);
	IMX586_SENSOR_GET_CTX(ViPipe, pstSnsState);
	CMOS_CHECK_POINTER(pstSnsState);

	u32Vts = g_astImx586_mode[pstSnsState->u8ImgMode].u32VtsDef;
	pstSnsRegsInfo = &pstSnsState->astSyncInfo[0].snsCfg;
	f32MaxFps = g_astImx586_mode[pstSnsState->u8ImgMode].f32MaxFps;
	f32MinFps = g_astImx586_mode[pstSnsState->u8ImgMode].f32MinFps;

	if (pstSnsState->enWDRMode == WDR_MODE_NONE) {
		if ((f32Fps <= f32MaxFps) && (f32Fps >= f32MinFps)) {
			u32VMAX = u32Vts * f32MaxFps / DIV_0_TO_1_FLOAT(f32Fps);
		} else {
			CVI_TRACE_SNS(CVI_DBG_ERR, "Unsupport Fps: %f\n", f32Fps);
			return CVI_FAILURE;
		}
		u32VMAX = (u32VMAX > IMX586_FULL_LINES_MAX) ? IMX586_FULL_LINES_MAX : u32VMAX;

		pstSnsRegsInfo->astI2cData[LINEAR_VMAX_H].u32Data = ((u32VMAX & 0xFF00) >> 8);
		pstSnsRegsInfo->astI2cData[LINEAR_VMAX_L].u32Data = (u32VMAX & 0xFF);
	} else {
		CVI_TRACE_SNS(CVI_DBG_ERR, "Not support WDR: %d\n", pstSnsState->enWDRMode);
		return CVI_FAILURE;
	}

	pstSnsState->u32FLStd = u32VMAX;

	pstAeSnsDft->f32Fps = f32Fps;
	pstAeSnsDft->u32LinesPer500ms = pstSnsState->u32FLStd * f32Fps / 2;
	pstAeSnsDft->u32FullLinesStd = pstSnsState->u32FLStd;
	pstAeSnsDft->u32MaxIntTime = pstSnsState->u32FLStd - 2;
	pstSnsState->au32FL[0] = pstSnsState->u32FLStd;
	pstAeSnsDft->u32FullLines = pstSnsState->au32FL[0];
	pstAeSnsDft->u32HmaxTimes = (1000000) / (pstSnsState->u32FLStd * DIV_0_TO_1_FLOAT(f32Fps));

	return CVI_SUCCESS;
}

static CVI_S32 cmos_inttime_update(VI_PIPE ViPipe, CVI_U32 *u32IntTime)
{
	ISP_SNS_STATE_S *pstSnsState = CVI_NULL;
	ISP_SNS_REGS_INFO_S *pstSnsRegsInfo = CVI_NULL;
	CVI_U32 u32TmpIntTime, u32MinTime, u32MaxTime;

	IMX586_SENSOR_GET_CTX(ViPipe, pstSnsState);
	CMOS_CHECK_POINTER(pstSnsState);
	CMOS_CHECK_POINTER(u32IntTime);
	pstSnsRegsInfo = &pstSnsState->astSyncInfo[0].snsCfg;

	if (pstSnsState->enWDRMode == WDR_MODE_NONE) {
		u32MinTime = 5;
		u32MaxTime = pstSnsState->au32FL[0] - 2;
		u32TmpIntTime = (u32IntTime[0] > u32MaxTime) ? u32MaxTime : u32IntTime[0];
		u32TmpIntTime = (u32TmpIntTime < u32MinTime) ? u32MinTime : u32TmpIntTime;

		u32IntTime[0] = u32TmpIntTime;

		pstSnsRegsInfo->astI2cData[LINEAR_CIT_H].u32Data = ((u32TmpIntTime >> 8) & 0xFF);
		pstSnsRegsInfo->astI2cData[LINEAR_CIT_L].u32Data = (u32TmpIntTime & 0xFF);
	} else {
		CVI_TRACE_SNS(CVI_DBG_ERR, "Not support WDR: %d\n", pstSnsState->enWDRMode);
		return CVI_FAILURE;
	}

	return CVI_SUCCESS;
}

#define AGAIN_TABLE_SIZE	252

/* IMX586 analog gain table: linear gain * 1024, 252 entries uniform in dB
 * Gain = 1024 / (1024 - X), X = 0..1008, range 0dB ~ 36.12dB
 */
static CVI_U32 again_table[AGAIN_TABLE_SIZE] = {
	1024, 1036, 1049, 1061, 1074, 1088, 1101, 1114, 1128, 1141,
	1155, 1169, 1183, 1198, 1214, 1228, 1242, 1257, 1273, 1288,
	1304, 1321, 1337, 1353, 1369, 1385, 1402, 1419, 1436, 1454,
	1473, 1492, 1511, 1529, 1547, 1565, 1584, 1603, 1623, 1644,
	1664, 1686, 1708, 1730, 1751, 1771, 1792, 1814, 1836, 1859,
	1883, 1907, 1931, 1956, 1982, 2009, 2032, 2056, 2081, 2106,
	2131, 2158, 2185, 2212, 2241, 2270, 2300, 2330, 2362, 2394,
	2427, 2456, 2485, 2515, 2545, 2576, 2608, 2641, 2675, 2709,
	2745, 2781, 2819, 2857, 2897, 2937, 2979, 3022, 3057, 3093,
	3130, 3168, 3207, 3246, 3287, 3329, 3372, 3416, 3461, 3507,
	3554, 3603, 3654, 3705, 3758, 3813, 3869, 3927, 3987, 4049,
	4096, 4145, 4194, 4245, 4297, 4351, 4406, 4462, 4520, 4579,
	4640, 4702, 4766, 4832, 4900, 4970, 5041, 5115, 5191, 5269,
	5350, 5433, 5519, 5607, 5699, 5793, 5891, 5992, 6096, 6168,
	6242, 6317, 6394, 6473, 6554, 6637, 6722, 6809, 6899, 6991,
	7085, 7182, 7282, 7384, 7490, 7598, 7710, 7825, 7944, 8066,
	8192, 8322, 8456, 8595, 8738, 8886, 9039, 9198, 9362, 9533,
	9709, 9892, 10082, 10280, 10486, 10700, 10923, 11155, 11398, 11651,
	11916, 12193, 12336, 12483, 12633, 12788, 12945, 13107, 13273, 13443,
	13618, 13797, 13981, 14170, 14364, 14564, 14769, 14980, 15197, 15420,
	15650, 15888, 16132, 16384, 16644, 16913, 17190, 17476, 17772, 18079,
	18396, 18725, 19065, 19418, 19784, 20165, 20560, 20972, 21400, 21845,
	22310, 22795, 23302, 23831, 24385, 24966, 25575, 26214, 26887, 27594,
	28340, 29127, 29959, 30840, 31775, 32768, 33825, 34953, 36158, 37449,
	38836, 40330, 41943, 43691, 45590, 47663, 49932, 52429, 55188, 58254,
	61681, 65536
};

/* ANA_GAIN_GLOBAL register X values corresponding to each again_table entry */
static CVI_U16 again_reg[AGAIN_TABLE_SIZE] = {
	0, 12, 24, 36, 48, 60, 72, 83, 94, 105, 116, 127, 138, 149, 160, 170,
	180, 190, 200, 210, 220, 230, 240, 249, 258, 267, 276, 285, 294, 303, 312, 321,
	330, 338, 346, 354, 362, 370, 378, 386, 394, 402, 410, 418, 425, 432, 439, 446,
	453, 460, 467, 474, 481, 488, 495, 502, 508, 514, 520, 526, 532, 538, 544, 550,
	556, 562, 568, 574, 580, 586, 592, 597, 602, 607, 612, 617, 622, 627, 632, 637,
	642, 647, 652, 657, 662, 667, 672, 677, 681, 685, 689, 693, 697, 701, 705, 709,
	713, 717, 721, 725, 729, 733, 737, 741, 745, 749, 753, 757, 761, 765, 768, 771,
	774, 777, 780, 783, 786, 789, 792, 795, 798, 801, 804, 807, 810, 813, 816, 819,
	822, 825, 828, 831, 834, 837, 840, 843, 846, 849, 852, 854, 856, 858, 860, 862,
	864, 866, 868, 870, 872, 874, 876, 878, 880, 882, 884, 886, 888, 890, 892, 894,
	896, 898, 900, 902, 904, 906, 908, 910, 912, 914, 916, 918, 920, 922, 924, 926,
	928, 930, 932, 934, 936, 938, 939, 940, 941, 942, 943, 944, 945, 946, 947, 948,
	949, 950, 951, 952, 953, 954, 955, 956, 957, 958, 959, 960, 961, 962, 963, 964,
	965, 966, 967, 968, 969, 970, 971, 972, 973, 974, 975, 976, 977, 978, 979, 980,
	981, 982, 983, 984, 985, 986, 987, 988, 989, 990, 991, 992, 993, 994, 995, 996,
	997, 998, 999, 1000, 1001, 1002, 1003, 1004, 1005, 1006, 1007, 1008
};

#define DGAIN_TABLE_SIZE	128

/* IMX586 digital gain table: linear gain * 1024, 128 entries uniform in dB
 * Gain = reg_value / 256, reg range 0x0100 ~ 0x0FFF, range 0dB ~ 24.08dB
 */
static CVI_U32 dgain_table[DGAIN_TABLE_SIZE] = {
	1024, 1048, 1072, 1096, 1120, 1144, 1172, 1200, 1228, 1256,
	1284, 1312, 1344, 1376, 1408, 1440, 1472, 1504, 1540, 1576,
	1612, 1648, 1684, 1724, 1764, 1804, 1844, 1884, 1928, 1972,
	2016, 2060, 2108, 2156, 2204, 2252, 2304, 2356, 2408, 2460,
	2516, 2572, 2628, 2688, 2748, 2808, 2872, 2936, 3000, 3068,
	3136, 3204, 3276, 3348, 3420, 3496, 3572, 3652, 3732, 3812,
	3896, 3980, 4068, 4156, 4248, 4340, 4436, 4532, 4632, 4732,
	4836, 4940, 5048, 5156, 5268, 5384, 5500, 5620, 5744, 5868,
	5996, 6128, 6260, 6396, 6536, 6676, 6820, 6968, 7120, 7276,
	7432, 7592, 7756, 7924, 8096, 8272, 8452, 8636, 8824, 9016,
	9212, 9412, 9616, 9824, 10036, 10252, 10472, 10696, 10928, 11164,
	11404, 11648, 11900, 12156, 12420, 12688, 12960, 13240, 13524, 13816,
	14112, 14416, 14728, 15044, 15368, 15700, 16036, 16380
};

/* DIG_GAIN_GLOBAL register values corresponding to each dgain_table entry */
static CVI_U16 dgain_reg[DGAIN_TABLE_SIZE] = {
	0x0100, 0x0106, 0x010C, 0x0112, 0x0118, 0x011E, 0x0125, 0x012C, 0x0133, 0x013A,
	0x0141, 0x0148, 0x0150, 0x0158, 0x0160, 0x0168, 0x0170, 0x0178, 0x0181, 0x018A,
	0x0193, 0x019C, 0x01A5, 0x01AF, 0x01B9, 0x01C3, 0x01CD, 0x01D7, 0x01E2, 0x01ED,
	0x01F8, 0x0203, 0x020F, 0x021B, 0x0227, 0x0233, 0x0240, 0x024D, 0x025A, 0x0267,
	0x0275, 0x0283, 0x0291, 0x02A0, 0x02AF, 0x02BE, 0x02CE, 0x02DE, 0x02EE, 0x02FF,
	0x0310, 0x0321, 0x0333, 0x0345, 0x0357, 0x036A, 0x037D, 0x0391, 0x03A5, 0x03B9,
	0x03CE, 0x03E3, 0x03F9, 0x040F, 0x0426, 0x043D, 0x0455, 0x046D, 0x0486, 0x049F,
	0x04B9, 0x04D3, 0x04EE, 0x0509, 0x0525, 0x0542, 0x055F, 0x057D, 0x059C, 0x05BB,
	0x05DB, 0x05FC, 0x061D, 0x063F, 0x0662, 0x0685, 0x06A9, 0x06CE, 0x06F4, 0x071B,
	0x0742, 0x076A, 0x0793, 0x07BD, 0x07E8, 0x0814, 0x0841, 0x086F, 0x089E, 0x08CE,
	0x08FF, 0x0931, 0x0964, 0x0998, 0x09CD, 0x0A03, 0x0A3A, 0x0A72, 0x0AAC, 0x0AE7,
	0x0B23, 0x0B60, 0x0B9F, 0x0BDF, 0x0C21, 0x0C64, 0x0CA8, 0x0CEE, 0x0D35, 0x0D7E,
	0x0DC8, 0x0E14, 0x0E62, 0x0EB1, 0x0F02, 0x0F55, 0x0FA9, 0x0FFF
};

static CVI_S32 cmos_again_calc_table(VI_PIPE ViPipe, CVI_U32 *pu32AgainLin, CVI_U32 *pu32AgainDb)
{
	int i;

	(void) ViPipe;

	CMOS_CHECK_POINTER(pu32AgainLin);
	CMOS_CHECK_POINTER(pu32AgainDb);

	if (*pu32AgainLin >= again_table[AGAIN_TABLE_SIZE - 1]) {
		*pu32AgainLin = again_table[AGAIN_TABLE_SIZE - 1];
		*pu32AgainDb = AGAIN_TABLE_SIZE - 1;
		return CVI_SUCCESS;
	}

	for (i = 1; i < AGAIN_TABLE_SIZE; i++) {
		if (*pu32AgainLin < again_table[i]) {
			*pu32AgainLin = again_table[i - 1];
			*pu32AgainDb = i - 1;
			break;
		}
	}
	return CVI_SUCCESS;
}

static CVI_S32 cmos_dgain_calc_table(VI_PIPE ViPipe, CVI_U32 *pu32DgainLin, CVI_U32 *pu32DgainDb)
{
	int i;

	(void) ViPipe;

	CMOS_CHECK_POINTER(pu32DgainLin);
	CMOS_CHECK_POINTER(pu32DgainDb);

	if (*pu32DgainLin >= dgain_table[DGAIN_TABLE_SIZE - 1]) {
		*pu32DgainLin = dgain_table[DGAIN_TABLE_SIZE - 1];
		*pu32DgainDb = DGAIN_TABLE_SIZE - 1;
		return CVI_SUCCESS;
	}

	for (i = 1; i < DGAIN_TABLE_SIZE; i++) {
		if (*pu32DgainLin < dgain_table[i]) {
			*pu32DgainLin = dgain_table[i - 1];
			*pu32DgainDb = i - 1;
			break;
		}
	}
	return CVI_SUCCESS;
}

static CVI_S32 cmos_gains_update(VI_PIPE ViPipe, CVI_U32 *pu32Again, CVI_U32 *pu32Dgain)
{
	ISP_SNS_STATE_S *pstSnsState = CVI_NULL;
	ISP_SNS_REGS_INFO_S *pstSnsRegsInfo = CVI_NULL;
	CVI_U32 u32AgainIdx;
	CVI_U32 u32DgainIdx;
	CVI_U32 u32RegX;
	CVI_U32 u32DgainReg;

	IMX586_SENSOR_GET_CTX(ViPipe, pstSnsState);
	CMOS_CHECK_POINTER(pstSnsState);
	CMOS_CHECK_POINTER(pu32Again);
	CMOS_CHECK_POINTER(pu32Dgain);
	pstSnsRegsInfo = &pstSnsState->astSyncInfo[0].snsCfg;

	/* Analog gain: table index -> register X value */
	u32AgainIdx = pu32Again[0];
	if (u32AgainIdx >= AGAIN_TABLE_SIZE)
		u32AgainIdx = AGAIN_TABLE_SIZE - 1;

	u32RegX = again_reg[u32AgainIdx];

	pstSnsRegsInfo->astI2cData[LINEAR_GAIN_L].u32Data = (u32RegX & 0xFF);
	pstSnsRegsInfo->astI2cData[LINEAR_GAIN_H].u32Data = ((u32RegX >> 8) & 0x03);

	/* Digital gain: table index -> register value (upper + lower byte) */
	u32DgainIdx = pu32Dgain[0];
	if (u32DgainIdx >= DGAIN_TABLE_SIZE)
		u32DgainIdx = DGAIN_TABLE_SIZE - 1;

	u32DgainReg = dgain_reg[u32DgainIdx];

	pstSnsRegsInfo->astI2cData[LINEAR_DGAIN_H].u32Data = ((u32DgainReg >> 8) & 0xFF);
	pstSnsRegsInfo->astI2cData[LINEAR_DGAIN_L].u32Data = (u32DgainReg & 0xFF);

	return CVI_SUCCESS;
}

static CVI_S32 cmos_init_ae_exp_function(AE_SENSOR_EXP_FUNC_S *pstExpFuncs)
{
	CMOS_CHECK_POINTER(pstExpFuncs);

	memset(pstExpFuncs, 0, sizeof(AE_SENSOR_EXP_FUNC_S));

	pstExpFuncs->pfn_cmos_get_ae_default    = cmos_get_ae_default;
	pstExpFuncs->pfn_cmos_fps_set           = cmos_fps_set;
	pstExpFuncs->pfn_cmos_inttime_update    = cmos_inttime_update;
	pstExpFuncs->pfn_cmos_gains_update      = cmos_gains_update;
	pstExpFuncs->pfn_cmos_again_calc_table  = cmos_again_calc_table;
	pstExpFuncs->pfn_cmos_dgain_calc_table  = cmos_dgain_calc_table;

	return CVI_SUCCESS;
}

static CVI_S32 cmos_get_awb_default(VI_PIPE ViPipe, AWB_SENSOR_DEFAULT_S *pstAwbSnsDft)
{
	(void) ViPipe;

	CMOS_CHECK_POINTER(pstAwbSnsDft);

	memset(pstAwbSnsDft, 0, sizeof(AWB_SENSOR_DEFAULT_S));

	pstAwbSnsDft->u16InitGgain = 1024;
	pstAwbSnsDft->u8AWBRunInterval = 1;

	return CVI_SUCCESS;
}

static CVI_S32 cmos_init_awb_exp_function(AWB_SENSOR_EXP_FUNC_S *pstExpFuncs)
{
	CMOS_CHECK_POINTER(pstExpFuncs);

	memset(pstExpFuncs, 0, sizeof(AWB_SENSOR_EXP_FUNC_S));

	pstExpFuncs->pfn_cmos_get_awb_default = cmos_get_awb_default;

	return CVI_SUCCESS;
}

static CVI_S32 cmos_get_isp_default(VI_PIPE ViPipe, ISP_CMOS_DEFAULT_S *pstDef)
{
	(void) ViPipe;

	memset(pstDef, 0, sizeof(ISP_CMOS_DEFAULT_S));

	memcpy(pstDef->stNoiseCalibration.CalibrationCoef,
		&g_stIspNoiseCalibration, sizeof(ISP_CMOS_NOISE_CALIBRATION_S));
	return CVI_SUCCESS;
}

static CVI_S32 cmos_get_blc_default(VI_PIPE ViPipe, ISP_CMOS_BLACK_LEVEL_S *pstBlc)
{
	(void) ViPipe;

	CMOS_CHECK_POINTER(pstBlc);

	memset(pstBlc, 0, sizeof(ISP_CMOS_BLACK_LEVEL_S));

	memcpy(pstBlc,
		&g_stIspBlcCalibration, sizeof(ISP_CMOS_BLACK_LEVEL_S));
	return CVI_SUCCESS;
}

static CVI_S32 cmos_get_wdr_size(VI_PIPE ViPipe, ISP_SNS_ISP_INFO_S *pstIspCfg)
{
	const IMX586_MODE_S *pstMode = CVI_NULL;
	ISP_SNS_STATE_S *pstSnsState = CVI_NULL;

	IMX586_SENSOR_GET_CTX(ViPipe, pstSnsState);
	CMOS_CHECK_POINTER(pstSnsState);
	pstMode = &g_astImx586_mode[pstSnsState->u8ImgMode];

	pstIspCfg->frm_num = 1;
	memcpy(&pstIspCfg->img_size[0], &pstMode->astImg[0], sizeof(ISP_WDR_SIZE_S));

	return CVI_SUCCESS;
}

static CVI_S32 cmos_set_wdr_mode(VI_PIPE ViPipe, CVI_U8 u8Mode)
{
	ISP_SNS_STATE_S *pstSnsState = CVI_NULL;

	IMX586_SENSOR_GET_CTX(ViPipe, pstSnsState);
	CMOS_CHECK_POINTER(pstSnsState);
	if (u8Mode != WDR_MODE_NONE) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "Not support WDR: %d\n", u8Mode);
		return CVI_FAILURE;
	}
	pstSnsState->enWDRMode = WDR_MODE_NONE;
	pstSnsState->bSyncInit = CVI_FALSE;
	return CVI_SUCCESS;
}

static CVI_U32 sensor_cmp_wdr_size(ISP_SNS_ISP_INFO_S *pstWdr1, ISP_SNS_ISP_INFO_S *pstWdr2)
{
	CVI_U32 i;

	if (pstWdr1->frm_num != pstWdr2->frm_num)
		goto _mismatch;
	for (i = 0; i < 2; i++) {
		if (pstWdr1->img_size[i].stSnsSize.u32Width != pstWdr2->img_size[i].stSnsSize.u32Width)
			goto _mismatch;
		if (pstWdr1->img_size[i].stSnsSize.u32Height != pstWdr2->img_size[i].stSnsSize.u32Height)
			goto _mismatch;
		if (pstWdr1->img_size[i].stWndRect.s32X != pstWdr2->img_size[i].stWndRect.s32X)
			goto _mismatch;
		if (pstWdr1->img_size[i].stWndRect.s32Y != pstWdr2->img_size[i].stWndRect.s32Y)
			goto _mismatch;
		if (pstWdr1->img_size[i].stWndRect.u32Width != pstWdr2->img_size[i].stWndRect.u32Width)
			goto _mismatch;
		if (pstWdr1->img_size[i].stWndRect.u32Height != pstWdr2->img_size[i].stWndRect.u32Height)
			goto _mismatch;
	}

	return 0;
_mismatch:
	return 1;
}

static CVI_S32 cmos_get_sns_regs_info(VI_PIPE ViPipe, ISP_SNS_SYNC_INFO_S *pstSnsSyncInfo)
{
	CVI_U32 i;
	ISP_SNS_STATE_S *pstSnsState = CVI_NULL;
	ISP_SNS_REGS_INFO_S *pstSnsRegsInfo = CVI_NULL;
	ISP_SNS_SYNC_INFO_S *pstCfg0 = CVI_NULL;
	ISP_SNS_SYNC_INFO_S *pstCfg1 = CVI_NULL;
	ISP_I2C_DATA_S *pstI2c_data = CVI_NULL;

	CMOS_CHECK_POINTER(pstSnsSyncInfo);
	IMX586_SENSOR_GET_CTX(ViPipe, pstSnsState);
	CMOS_CHECK_POINTER(pstSnsState);
	pstSnsRegsInfo = &pstSnsSyncInfo->snsCfg;
	pstCfg0 = &pstSnsState->astSyncInfo[0];
	pstCfg1 = &pstSnsState->astSyncInfo[1];
	pstI2c_data = pstCfg0->snsCfg.astI2cData;

	if ((pstSnsState->bSyncInit == CVI_FALSE) || (pstSnsRegsInfo->bConfig == CVI_FALSE)) {
		pstCfg0->snsCfg.enSnsType = SNS_I2C_TYPE;
		pstCfg0->snsCfg.unComBus.s8I2cDev = g_aunImx586_BusInfo[ViPipe].s8I2cDev;
		pstCfg0->snsCfg.u8Cfg2ValidDelayMax = 2;
		pstCfg0->snsCfg.use_snsr_sram = CVI_TRUE;
		pstCfg0->snsCfg.u32RegNum = LINEAR_REGS_NUM;

		for (i = 0; i < pstCfg0->snsCfg.u32RegNum; i++) {
			pstI2c_data[i].bUpdate = CVI_TRUE;
			pstI2c_data[i].u8DevAddr = g_aunImx586_AddrInfo[ViPipe].s8I2cAddr;
			pstI2c_data[i].u32AddrByteNum = imx586_addr_byte;
			pstI2c_data[i].u32DataByteNum = imx586_data_byte;
		}

		pstI2c_data[LINEAR_HOLD].u32RegAddr = IMX586_HOLD_ADDR;
		pstI2c_data[LINEAR_HOLD].u32Data = 1;
		pstI2c_data[LINEAR_CIT_H].u32RegAddr = IMX586_CIT_H_ADDR;
		pstI2c_data[LINEAR_CIT_L].u32RegAddr = IMX586_CIT_L_ADDR;

		pstI2c_data[LINEAR_GAIN_L].u32RegAddr = IMX586_GAIN_L_ADDR;
		pstI2c_data[LINEAR_GAIN_H].u32RegAddr = IMX586_GAIN_H_ADDR;
		pstI2c_data[LINEAR_DGAIN_H].u32RegAddr = IMX586_DGAIN_H_ADDR;
		pstI2c_data[LINEAR_DGAIN_L].u32RegAddr = IMX586_DGAIN_L_ADDR;
		pstI2c_data[LINEAR_VMAX_H].u32RegAddr = IMX586_VMAX_H_ADDR;
		pstI2c_data[LINEAR_VMAX_L].u32RegAddr = IMX586_VMAX_L_ADDR;
		pstI2c_data[LINEAR_REL].u32RegAddr = IMX586_HOLD_ADDR;
		pstI2c_data[LINEAR_REL].u32Data = 0;
		pstI2c_data[LINEAR_FLIP_MIRROR].u32RegAddr = IMX586_FLIP_ADDR;
		pstI2c_data[LINEAR_FLIP_MIRROR].u32Data = 0;
		pstCfg0->ispCfg.u8DelayFrmNum = 0;
		pstSnsState->bSyncInit = CVI_TRUE;
		pstCfg0->snsCfg.need_update = CVI_TRUE;
		/* recalcualte WDR size */
		cmos_get_wdr_size(ViPipe, &pstCfg0->ispCfg);
		pstCfg0->ispCfg.need_update = CVI_TRUE;
	} else {
		pstCfg0->snsCfg.need_update = CVI_FALSE;
		for (i = 0; i < pstCfg0->snsCfg.u32RegNum; i++) {
			if (pstCfg0->snsCfg.astI2cData[i].u32Data == pstCfg1->snsCfg.astI2cData[i].u32Data) {
				pstCfg0->snsCfg.astI2cData[i].bUpdate = CVI_FALSE;
			} else {
				pstCfg0->snsCfg.astI2cData[i].bUpdate = CVI_TRUE;
				pstCfg0->snsCfg.need_update = CVI_TRUE;
			}
		}
		if (pstCfg0->snsCfg.need_update == CVI_TRUE) {
			pstI2c_data[LINEAR_HOLD].u32Data = 1;
			pstI2c_data[LINEAR_HOLD].bUpdate = CVI_TRUE;
			pstI2c_data[LINEAR_REL].u32Data = 0;
			pstI2c_data[LINEAR_REL].bUpdate = CVI_TRUE;
		}
		/* check update isp crop or not */
		pstCfg0->ispCfg.need_update = (sensor_cmp_wdr_size(&pstCfg0->ispCfg, &pstCfg1->ispCfg) ?
				CVI_TRUE : CVI_FALSE);
	}

	pstSnsRegsInfo->bConfig = CVI_FALSE;
	memcpy(pstSnsSyncInfo, &pstSnsState->astSyncInfo[0], sizeof(ISP_SNS_SYNC_INFO_S));
	memcpy(&pstSnsState->astSyncInfo[1], &pstSnsState->astSyncInfo[0], sizeof(ISP_SNS_SYNC_INFO_S));
	pstSnsState->au32FL[1] = pstSnsState->au32FL[0];

	return CVI_SUCCESS;
}

static CVI_S32 cmos_set_image_mode(VI_PIPE ViPipe, ISP_CMOS_SENSOR_IMAGE_MODE_S *pstSensorImageMode)
{
	CVI_U8 u8SensorImageMode = 0;
	ISP_SNS_STATE_S *pstSnsState = CVI_NULL;

	CMOS_CHECK_POINTER(pstSensorImageMode);
	IMX586_SENSOR_GET_CTX(ViPipe, pstSnsState);
	CMOS_CHECK_POINTER(pstSnsState);

	u8SensorImageMode = pstSnsState->u8ImgMode;
	pstSnsState->bSyncInit = CVI_FALSE;

	if (!(pstSensorImageMode->f32Fps > 0 && pstSensorImageMode->f32Fps <= 120)) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "Not support Fps: %f\n", pstSensorImageMode->f32Fps);
		return CVI_FAILURE;
	}

	if (pstSensorImageMode->f32Fps <= 30) {
		if (pstSnsState->enWDRMode == WDR_MODE_NONE) {
			if (IMX586_RES_IS_12M(pstSensorImageMode->u16Width, pstSensorImageMode->u16Height))
				u8SensorImageMode = IMX586_MODE_12M30;
			else {
				CVI_TRACE_SNS(CVI_DBG_ERR, "Not support! Width:%d, Height:%d, Fps:%f, WDRMode:%d\n",
				       pstSensorImageMode->u16Width,
				       pstSensorImageMode->u16Height,
				       pstSensorImageMode->f32Fps,
				       pstSnsState->enWDRMode);
				return CVI_FAILURE;
			}
		} else {
			CVI_TRACE_SNS(CVI_DBG_ERR, "Not support! Width:%d, Height:%d, Fps:%f, WDRMode:%d\n",
			       pstSensorImageMode->u16Width,
			       pstSensorImageMode->u16Height,
			       pstSensorImageMode->f32Fps,
			       pstSnsState->enWDRMode);
			return CVI_FAILURE;
		}
	} else if (pstSensorImageMode->f32Fps <= 60) {
		if (pstSnsState->enWDRMode == WDR_MODE_NONE) {
			if (IMX586_RES_IS_8M(pstSensorImageMode->u16Width, pstSensorImageMode->u16Height))
				u8SensorImageMode = IMX586_MODE_8M60;
			else {
				CVI_TRACE_SNS(CVI_DBG_ERR, "Not support! Width:%d, Height:%d, Fps:%f, WDRMode:%d\n",
				       pstSensorImageMode->u16Width,
				       pstSensorImageMode->u16Height,
				       pstSensorImageMode->f32Fps,
				       pstSnsState->enWDRMode);
				return CVI_FAILURE;
			}
		} else {
			CVI_TRACE_SNS(CVI_DBG_ERR, "Not support! Width:%d, Height:%d, Fps:%f, WDRMode:%d\n",
			       pstSensorImageMode->u16Width,
			       pstSensorImageMode->u16Height,
			       pstSensorImageMode->f32Fps,
			       pstSnsState->enWDRMode);
			return CVI_FAILURE;
		}
	} else if (pstSensorImageMode->f32Fps <= 120) {
		if (pstSnsState->enWDRMode == WDR_MODE_NONE) {
			if (IMX586_RES_IS_2M(pstSensorImageMode->u16Width, pstSensorImageMode->u16Height))
				u8SensorImageMode = IMX586_MODE_2M120;
			else {
				CVI_TRACE_SNS(CVI_DBG_ERR, "Not support! Width:%d, Height:%d, Fps:%f, WDRMode:%d\n",
				       pstSensorImageMode->u16Width,
				       pstSensorImageMode->u16Height,
				       pstSensorImageMode->f32Fps,
				       pstSnsState->enWDRMode);
				return CVI_FAILURE;
			}
		} else {
			CVI_TRACE_SNS(CVI_DBG_ERR, "Not support! Width:%d, Height:%d, Fps:%f, WDRMode:%d\n",
			       pstSensorImageMode->u16Width,
			       pstSensorImageMode->u16Height,
			       pstSensorImageMode->f32Fps,
			       pstSnsState->enWDRMode);
			return CVI_FAILURE;
		}
	}

	if ((pstSnsState->bInit == CVI_TRUE) && (u8SensorImageMode == pstSnsState->u8ImgMode)) {
		return CVI_FAILURE;
	}

	pstSnsState->u8ImgMode = u8SensorImageMode;
	pstSnsState->u32FLStd = g_astImx586_mode[u8SensorImageMode].u32VtsDef;
	pstSnsState->au32FL[0] = pstSnsState->u32FLStd;
	pstSnsState->au32FL[1] = pstSnsState->u32FLStd;

	return CVI_SUCCESS;
}

static CVI_VOID sensor_mirror_flip(VI_PIPE ViPipe, ISP_SNS_MIRRORFLIP_TYPE_E eSnsMirrorFlip)
{
	ISP_SNS_STATE_S *pstSnsState = CVI_NULL;

	IMX586_SENSOR_GET_CTX(ViPipe, pstSnsState);
	CMOS_CHECK_POINTER_VOID(pstSnsState);
	if (pstSnsState->bInit == CVI_TRUE && g_aeImx586_MirrorFip[ViPipe] != eSnsMirrorFlip) {
		imx586_mirror_flip(ViPipe, eSnsMirrorFlip);
		g_aeImx586_MirrorFip[ViPipe] = eSnsMirrorFlip;
	}
}

static CVI_VOID sensor_global_init(VI_PIPE ViPipe)
{
	ISP_SNS_STATE_S *pstSnsState = CVI_NULL;

	IMX586_SENSOR_GET_CTX(ViPipe, pstSnsState);
	CMOS_CHECK_POINTER_VOID(pstSnsState);

	pstSnsState->bInit = CVI_FALSE;
	pstSnsState->bSyncInit = CVI_FALSE;
	pstSnsState->u8ImgMode = IMX586_MODE_12M30;
	pstSnsState->enWDRMode = WDR_MODE_NONE;
	pstSnsState->u32FLStd  = g_astImx586_mode[pstSnsState->u8ImgMode].u32VtsDef;
	pstSnsState->au32FL[0] = g_astImx586_mode[pstSnsState->u8ImgMode].u32VtsDef;
	pstSnsState->au32FL[1] = g_astImx586_mode[pstSnsState->u8ImgMode].u32VtsDef;

	memset(&pstSnsState->astSyncInfo[0], 0, sizeof(ISP_SNS_SYNC_INFO_S));
	memset(&pstSnsState->astSyncInfo[1], 0, sizeof(ISP_SNS_SYNC_INFO_S));
}

static CVI_S32 sensor_rx_attr(VI_PIPE ViPipe, SNS_COMBO_DEV_ATTR_S *pstRxAttr)
{
	ISP_SNS_STATE_S *pstSnsState = CVI_NULL;
	SNS_COMBO_DEV_ATTR_S *pstRxAttrSrc = CVI_NULL;

	IMX586_SENSOR_GET_CTX(ViPipe, pstSnsState);
	IMX586_SENSOR_GET_COMBO(ViPipe, pstRxAttrSrc);

	CMOS_CHECK_POINTER(pstSnsState);
	CMOS_CHECK_POINTER(pstRxAttr);
	CMOS_CHECK_POINTER(pstRxAttrSrc);

	memcpy(pstRxAttr, pstRxAttrSrc, sizeof(*pstRxAttr));

	pstRxAttr->img_size.start_x = g_astImx586_mode[pstSnsState->u8ImgMode].astImg[0].stWndRect.s32X;
	pstRxAttr->img_size.start_y = g_astImx586_mode[pstSnsState->u8ImgMode].astImg[0].stWndRect.s32Y;
	pstRxAttr->img_size.active_w = g_astImx586_mode[pstSnsState->u8ImgMode].astImg[0].stWndRect.u32Width;
	pstRxAttr->img_size.active_h = g_astImx586_mode[pstSnsState->u8ImgMode].astImg[0].stWndRect.u32Height;
	pstRxAttr->img_size.width = g_astImx586_mode[pstSnsState->u8ImgMode].astImg[0].stSnsSize.u32Width;
	pstRxAttr->img_size.height = g_astImx586_mode[pstSnsState->u8ImgMode].astImg[0].stSnsSize.u32Height;
	pstRxAttr->img_size.max_width = g_astImx586_mode[pstSnsState->u8ImgMode].astImg[0].stMaxSize.u32Width;
	pstRxAttr->img_size.max_height = g_astImx586_mode[pstSnsState->u8ImgMode].astImg[0].stMaxSize.u32Height;

	pstRxAttr->mipi_attr.wdr_mode = CVI_MIPI_WDR_MODE_NONE;
	pstRxAttrSrc = CVI_NULL;
	return CVI_SUCCESS;

}

static CVI_S32 sensor_patch_rx_attr(VI_PIPE ViPipe, RX_INIT_ATTR_S *pstRxInitAttr)
{
	int i;
	SNS_COMBO_DEV_ATTR_S *pstRxAttr = CVI_NULL;

	CMOS_CHECK_POINTER(pstRxInitAttr);
	if (pstRxInitAttr->MipiDev >= VI_MAX_DEV_NUM)
		return CVI_FAILURE;

	if (!g_pastImx586ComboDevArray[ViPipe]) {
		pstRxAttr = malloc(sizeof(SNS_COMBO_DEV_ATTR_S));
	} else {
		IMX586_SENSOR_GET_COMBO(ViPipe, pstRxAttr);
	}
	CMOS_CHECK_POINTER(pstRxAttr);
	memcpy(pstRxAttr, &imx586_rx_attr, sizeof(SNS_COMBO_DEV_ATTR_S));
	IMX586_SENSOR_SET_COMBO(ViPipe, pstRxAttr);

	if (pstRxInitAttr->stMclkAttr.bMclkEn)
		pstRxAttr->mclk.cam = pstRxInitAttr->stMclkAttr.u8Mclk;

	pstRxAttr->devno = pstRxInitAttr->MipiDev;
	pstRxAttr->cif_mode = pstRxInitAttr->MipiMode;

	if (pstRxAttr->input_mode == INPUT_MODE_MIPI) {
		struct mipi_dev_attr_s *attr = &pstRxAttr->mipi_attr;

		for (i = 0; i < MIPI_LANE_NUM + 1; i++) {
			attr->lane_id[i] = pstRxInitAttr->as16LaneId[i];
			attr->pn_swap[i] = pstRxInitAttr->as8PNSwap[i];
		}
	} else {
		struct lvds_dev_attr_s *attr = &pstRxAttr->lvds_attr;

		for (i = 0; i < MIPI_LANE_NUM + 1; i++) {
			attr->lane_id[i] = pstRxInitAttr->as16LaneId[i];
			attr->pn_swap[i] = pstRxInitAttr->as8PNSwap[i];
		}
	}
	pstRxAttr = CVI_NULL;
	return CVI_SUCCESS;
}

void imx586_exit(VI_PIPE ViPipe)
{
	if (g_pastImx586ComboDevArray[ViPipe]) {
		free(g_pastImx586ComboDevArray[ViPipe]);
		g_pastImx586ComboDevArray[ViPipe] = CVI_NULL;
	}
	imx586_i2c_exit(ViPipe);
}

static CVI_S32 cmos_init_sensor_exp_function(ISP_SENSOR_EXP_FUNC_S *pstSensorExpFunc)
{
	CMOS_CHECK_POINTER(pstSensorExpFunc);

	memset(pstSensorExpFunc, 0, sizeof(ISP_SENSOR_EXP_FUNC_S));

	pstSensorExpFunc->pfn_cmos_sensor_init = imx586_init;
	pstSensorExpFunc->pfn_cmos_sensor_exit = imx586_exit;
	pstSensorExpFunc->pfn_cmos_sensor_global_init = sensor_global_init;
	pstSensorExpFunc->pfn_cmos_set_image_mode = cmos_set_image_mode;
	pstSensorExpFunc->pfn_cmos_set_wdr_mode = cmos_set_wdr_mode;

	pstSensorExpFunc->pfn_cmos_get_isp_default = cmos_get_isp_default;
	pstSensorExpFunc->pfn_cmos_get_isp_black_level = cmos_get_blc_default;
	pstSensorExpFunc->pfn_cmos_get_sns_reg_info = cmos_get_sns_regs_info;
	return CVI_SUCCESS;
}

/****************************************************************************
 * callback structure                                                       *
 ****************************************************************************/
static CVI_VOID sensor_patch_i2c_addr(VI_PIPE ViPipe, CVI_S32 s32I2cAddr)
{
	if (IMX586_I2C_ADDR_IS_VALID(s32I2cAddr))
		g_aunImx586_AddrInfo[ViPipe].s8I2cAddr = s32I2cAddr;
	else {
		CVI_TRACE_SNS(CVI_DBG_ERR, "I2C addr input error ,please check [0x%x]\n", s32I2cAddr);
		g_aunImx586_AddrInfo[ViPipe].s8I2cAddr = IMX586_I2C_ADDR_2;
	}
}

static CVI_S32 imx586_set_bus_info(VI_PIPE ViPipe, ISP_SNS_COMMBUS_U unSNSBusInfo)
{
	g_aunImx586_BusInfo[ViPipe].s8I2cDev = unSNSBusInfo.s8I2cDev;

	return CVI_SUCCESS;
}

static CVI_S32 sensor_ctx_init(VI_PIPE ViPipe)
{
	ISP_SNS_STATE_S *pastSnsStateCtx = CVI_NULL;

	IMX586_SENSOR_GET_CTX(ViPipe, pastSnsStateCtx);

	if (pastSnsStateCtx == CVI_NULL) {
		pastSnsStateCtx = (ISP_SNS_STATE_S *)malloc(sizeof(ISP_SNS_STATE_S));
		if (pastSnsStateCtx == CVI_NULL) {
			CVI_TRACE_SNS(CVI_DBG_ERR, "Isp[%d] SnsCtx malloc memory failed!\n", ViPipe);
			return -ENOMEM;
		}
	}

	memset(pastSnsStateCtx, 0, sizeof(ISP_SNS_STATE_S));

	IMX586_SENSOR_SET_CTX(ViPipe, pastSnsStateCtx);

	return CVI_SUCCESS;
}

static CVI_VOID sensor_ctx_exit(VI_PIPE ViPipe)
{
	ISP_SNS_STATE_S *pastSnsStateCtx = CVI_NULL;

	IMX586_SENSOR_GET_CTX(ViPipe, pastSnsStateCtx);
	SENSOR_FREE(pastSnsStateCtx);
	IMX586_SENSOR_RESET_CTX(ViPipe);
	g_aeImx586_MirrorFip[ViPipe] = ISP_SNS_NORMAL;
}

static CVI_S32 sensor_register_callback(VI_PIPE ViPipe, ALG_LIB_S *pstAeLib, ALG_LIB_S *pstAwbLib)
{
	CVI_S32 s32Ret;
	ISP_SENSOR_REGISTER_S stIspRegister;
	AE_SENSOR_REGISTER_S  stAeRegister;
	AWB_SENSOR_REGISTER_S stAwbRegister;
	ISP_SNS_ATTR_INFO_S   stSnsAttrInfo;

	CMOS_CHECK_POINTER(pstAeLib);
	CMOS_CHECK_POINTER(pstAwbLib);

	s32Ret = sensor_ctx_init(ViPipe);

	if (s32Ret != CVI_SUCCESS)
		return CVI_FAILURE;

	stSnsAttrInfo.eSensorId = IMX586_ID;

	s32Ret  = cmos_init_sensor_exp_function(&stIspRegister.stSnsExp);
	s32Ret |= CVI_ISP_SensorRegCallBack(ViPipe, &stSnsAttrInfo, &stIspRegister);

	if (s32Ret != CVI_SUCCESS) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "sensor register callback function failed!\n");
		return s32Ret;
	}

	s32Ret  = cmos_init_ae_exp_function(&stAeRegister.stAeExp);
	s32Ret |= CVI_AE_SensorRegCallBack(ViPipe, pstAeLib, &stSnsAttrInfo, &stAeRegister);

	if (s32Ret != CVI_SUCCESS) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "sensor register callback function to ae lib failed!\n");
		return s32Ret;
	}

	s32Ret  = cmos_init_awb_exp_function(&stAwbRegister.stAwbExp);
	s32Ret |= CVI_AWB_SensorRegCallBack(ViPipe, pstAwbLib, &stSnsAttrInfo, &stAwbRegister);

	if (s32Ret != CVI_SUCCESS) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "sensor register callback function to awb lib failed!\n");
		return s32Ret;
	}

	return CVI_SUCCESS;
}

static CVI_S32 sensor_unregister_callback(VI_PIPE ViPipe, ALG_LIB_S *pstAeLib, ALG_LIB_S *pstAwbLib)
{
	CVI_S32 s32Ret;

	CMOS_CHECK_POINTER(pstAeLib);
	CMOS_CHECK_POINTER(pstAwbLib);

	s32Ret = CVI_ISP_SensorUnRegCallBack(ViPipe, IMX586_ID);
	if (s32Ret != CVI_SUCCESS) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "sensor unregister callback function failed!\n");
		return s32Ret;
	}

	s32Ret = CVI_AE_SensorUnRegCallBack(ViPipe, pstAeLib, IMX586_ID);
	if (s32Ret != CVI_SUCCESS) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "sensor unregister callback function to ae lib failed!\n");
		return s32Ret;
	}

	s32Ret = CVI_AWB_SensorUnRegCallBack(ViPipe, pstAwbLib, IMX586_ID);
	if (s32Ret != CVI_SUCCESS) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "sensor unregister callback function to awb lib failed!\n");
		return s32Ret;
	}

	sensor_ctx_exit(ViPipe);

	return CVI_SUCCESS;
}

static CVI_S32 sensor_set_init(VI_PIPE ViPipe, ISP_INIT_ATTR_S *pstInitAttr)
{
	CMOS_CHECK_POINTER(pstInitAttr);

	g_au32InitExposure[ViPipe] = pstInitAttr->u32Exposure;
	g_au32LinesPer500ms[ViPipe] = pstInitAttr->u32LinesPer500ms;
	g_au16InitWBGain[ViPipe][0] = pstInitAttr->u16WBRgain;
	g_au16InitWBGain[ViPipe][1] = pstInitAttr->u16WBGgain;
	g_au16InitWBGain[ViPipe][2] = pstInitAttr->u16WBBgain;
	g_au16SampleRgain[ViPipe] = pstInitAttr->u16SampleRgain;
	g_au16SampleBgain[ViPipe] = pstInitAttr->u16SampleBgain;
	g_au16Imx586_GainMode[ViPipe] = pstInitAttr->enGainMode;

	return CVI_SUCCESS;
}

ISP_SNS_OBJ_S stSnsImx586_Obj = {
	.pfnRegisterCallback    = sensor_register_callback,
	.pfnUnRegisterCallback  = sensor_unregister_callback,
	.pfnStandby             = imx586_standby,
	.pfnRestart             = imx586_restart,
	.pfnMirrorFlip          = sensor_mirror_flip,
	.pfnWriteReg            = imx586_write_register,
	.pfnReadReg             = imx586_read_register,
	.pfnSetBusInfo          = imx586_set_bus_info,
	.pfnSetInit             = sensor_set_init,
	.pfnPatchRxAttr		= sensor_patch_rx_attr,
	.pfnPatchI2cAddr	= sensor_patch_i2c_addr,
	.pfnGetRxAttr		= sensor_rx_attr,
	.pfnExpSensorCb		= cmos_init_sensor_exp_function,
	.pfnExpAeCb		= cmos_init_ae_exp_function,
	.pfnSnsProbe		= imx586_probe,
};
