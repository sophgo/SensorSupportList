#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
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

#include "gc4683_cmos_ex.h"
#include "gc4683_cmos_param.h"

#define DIV_0_TO_1(a)   ((0 == (a)) ? 1 : (a))
#define DIV_0_TO_1_FLOAT(a) ((((a) < 1E-10) && ((a) > -1E-10)) ? 1 : (a))
#define GC4683_ID 4683
#define GC4683_I2C_ADDR_1 0x31
#define GC4683_I2C_ADDR_2 0x30
#define GC4683_I2C_ADDR_IS_VALID(addr)      ((addr) == GC4683_I2C_ADDR_1 || (addr) == GC4683_I2C_ADDR_2)

/****************************************************************************
 * global variables                                                         *
 ***************************************************************************/

ISP_SNS_STATE_S *g_pastGc4683[VI_MAX_PIPE_NUM] = {CVI_NULL};
SNS_COMBO_DEV_ATTR_S *g_pastGc4683ComboDevArray[VI_MAX_PIPE_NUM] = {CVI_NULL};

#define GC4683_SENSOR_GET_CTX(dev, pstCtx)   (pstCtx = g_pastGc4683[dev])
#define GC4683_SENSOR_SET_CTX(dev, pstCtx)   (g_pastGc4683[dev] = pstCtx)
#define GC4683_SENSOR_RESET_CTX(dev)         (g_pastGc4683[dev] = CVI_NULL)
#define GC4683_SENSOR_SET_COMBO(dev, pstCtx)   (g_pastGc4683ComboDevArray[dev] = pstCtx)
#define GC4683_SENSOR_GET_COMBO(dev, pstCtx)   (pstCtx = g_pastGc4683ComboDevArray[dev])

ISP_SNS_COMMBUS_U g_aunGc4683_BusInfo[VI_MAX_PIPE_NUM] = {
	[0] = { .s8I2cDev = 3},
	[1 ... VI_MAX_PIPE_NUM - 1] = { .s8I2cDev = -1}
};

ISP_SNS_COMMADDR_U g_aunGc4683_AddrInfo[VI_MAX_PIPE_NUM] = {
	[0] = { .s8I2cAddr = 0},
	[1 ... VI_MAX_PIPE_NUM - 1] = { .s8I2cAddr = -1}
};

ISP_SNS_MIRRORFLIP_TYPE_E g_aeGc4683_MirrorFip[VI_MAX_PIPE_NUM] = {0};

CVI_U16 g_au16Gc4683_GainMode[VI_MAX_PIPE_NUM] = {0};
static GC4683_STATE_S g_astGc4683_State[VI_MAX_PIPE_NUM] = {{0} };

#define GC4683_STATE_GET_CTX(dev) (&g_astGc4683_State[dev])

/****************************************************************************
 * local variables and functions                                            *
 ***************************************************************************/
static ISP_FSWDR_MODE_E genFSWDRMode[VI_MAX_PIPE_NUM] = {
	[0 ... VI_MAX_PIPE_NUM - 1] = ISP_FSWDR_NORMAL_MODE
};

static CVI_U32 gu32MaxTimeGetCnt[VI_MAX_PIPE_NUM] = {0};
static CVI_U32 g_au32InitExposure[VI_MAX_PIPE_NUM]  = {0};
static CVI_U32 g_au32LinesPer500ms[VI_MAX_PIPE_NUM] = {0};
static CVI_U16 g_au16InitWBGain[VI_MAX_PIPE_NUM][3] = {{0} };
static CVI_U16 g_au16SampleRgain[VI_MAX_PIPE_NUM] = {0};
static CVI_U16 g_au16SampleBgain[VI_MAX_PIPE_NUM] = {0};
static CVI_S32 cmos_get_wdr_size(VI_PIPE ViPipe, ISP_SNS_ISP_INFO_S *pstIspCfg);
/*****Gc4683 Lines Range*****/
#define GC4683_FULL_LINES_MAX  (0x3fff)

/*****Gc4683 Register Address*****/
#define GC4683_EXP_H_ADDR			0x0202 //bit[13:8]
#define GC4683_EXP_L_ADDR			0x0203

#define GC4683_AGAIN_PGA_ADDR		0x02a9 //bit[13]
#define GC4683_AGAIN_H_ADDR			0x02aa //bit[12:8]
#define GC4683_AGAIN_L_ADDR			0x02ab

#define GC4683_COL_AGAIN_H_ADDR			0x00a8 //bit[11:6]
#define GC4683_COL_AGAIN_L_ADDR			0x00a9
#define GC4683_DGAIN_H_ADDR			0x0064 //bit[9:6]
#define GC4683_DGAIN_L_ADDR			0x0065 //bit[5:0]
#define GC4683_AGAIN_MAG1_ADDR			0x14ac
#define GC4683_AGAIN_MAG2_ADDR			0x1476
#define GC4683_AGAIN_MAG3_ADDR			0x1467
#define GC4683_AGAIN_MAG4_ADDR			0x1468


#define GC4683_VTS_H_ADDR			0x0340 //bit[13:8]
#define GC4683_VTS_L_ADDR			0x0341

#define GC4683_FLIP_MIRROR_ADDR			0x022c
// #define GC4683_FRAME_BUF_ADDR			0x031D

/*****Gc4683 WDR Register Address*****/
#define GC4683_WDR_SEXP_H_ADDR			0x0200
#define GC4683_WDR_SEXP_L_ADDR			0x0201
#define GC4683_WDR_LEXP_H_ADDR			0x0202
#define GC4683_WDR_LEXP_L_ADDR			0x0203

#define GC4683_WDR_LAGAIN_H_ADDR		0x02aa
#define GC4683_WDR_LAGAIN_L_ADDR		0x02ab
#define GC4683_WDR_LAGAIN_PGA_ADDR		0x02a9
#define GC4683_WDR_LAGAIN_MAG1_ADDR		0x14ac
#define GC4683_WDR_LAGAIN_MAG2_ADDR		0x1467
#define GC4683_WDR_LAGAIN_MAG3_ADDR		0x1468
#define GC4683_WDR_LCOL_AGAIN_H_ADDR		0x00a8
#define GC4683_WDR_LCOL_AGAIN_L_ADDR		0x00a9
#define GC4683_WDR_LDGAIN_H_ADDR		0x0064
#define GC4683_WDR_LDGAIN_L_ADDR		0x0065

#define GC4683_WDR_SAGAIN_H_ADDR		0x0293
#define GC4683_WDR_SAGAIN_L_ADDR		0x0294
#define GC4683_WDR_SAGAIN_MAG1_ADDR		0x14ab
#define GC4683_WDR_SAGAIN_MAG2_ADDR		0x1457
#define GC4683_WDR_SAGAIN_MAG3_ADDR		0x1458
#define GC4683_WDR_SCOL_AGAIN_H_ADDR		0x00aa
#define GC4683_WDR_SCOL_AGAIN_L_ADDR		0x00ab
#define GC4683_WDR_SDGAIN_H_ADDR		0x007b
#define GC4683_WDR_SDGAIN_L_ADDR		0x007c

#define GC4683_WDR_VTS_H_ADDR			0x0340
#define GC4683_WDR_VTS_L_ADDR			0x0341

#define GC4683_RES_IS_1440P(w, h)      ((w) <= 2560 && (h) <= 1440)
#define GC4683_RES_IS_720P(w, h)      ((w) == 1280 && (h) == 720)
#define GC4683_RES_IS_1080P(w, h)	 ((w) == 1920 && (h) == 1080)

static CVI_S32 cmos_get_ae_default(VI_PIPE ViPipe, AE_SENSOR_DEFAULT_S *pstAeSnsDft)
{
	ISP_SNS_STATE_S *pstSnsState = CVI_NULL;

	CMOS_CHECK_POINTER(pstAeSnsDft);
	GC4683_SENSOR_GET_CTX(ViPipe, pstSnsState);
	CMOS_CHECK_POINTER(pstSnsState);

	pstAeSnsDft->u32FullLinesStd = pstSnsState->u32FLStd;
	pstAeSnsDft->u32FlickerFreq = 50 * 256;
	pstAeSnsDft->u32FullLinesMax = GC4683_FULL_LINES_MAX;
	pstAeSnsDft->u32HmaxTimes = (1000000) / (pstSnsState->u32FLStd * g_astGc4683_mode[pstSnsState->u8ImgMode].f32MaxFps);

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
		pstAeSnsDft->u32LinesPer500ms = pstSnsState->u32FLStd * g_astGc4683_mode[pstSnsState->u8ImgMode].f32MaxFps / 2;
	else
		pstAeSnsDft->u32LinesPer500ms = g_au32LinesPer500ms[ViPipe];

	switch (pstSnsState->enWDRMode) {
	default:
	case WDR_MODE_NONE:   /*linear mode*/
		pstAeSnsDft->f32Fps = g_astGc4683_mode[pstSnsState->u8ImgMode].f32MaxFps;
		pstAeSnsDft->f32MinFps = g_astGc4683_mode[pstSnsState->u8ImgMode].f32MinFps;
		pstAeSnsDft->au8HistThresh[0] = 0xd;
		pstAeSnsDft->au8HistThresh[1] = 0x28;
		pstAeSnsDft->au8HistThresh[2] = 0x60;
		pstAeSnsDft->au8HistThresh[3] = 0x80;

		pstAeSnsDft->u32MaxAgain = 151168;
		pstAeSnsDft->u32MinAgain = 1024;
		pstAeSnsDft->u32MaxAgainTarget = pstAeSnsDft->u32MaxAgain;
		pstAeSnsDft->u32MinAgainTarget = pstAeSnsDft->u32MinAgain;

		pstAeSnsDft->u32MaxDgain = 10240;
		pstAeSnsDft->u32MinDgain = 1024;
		pstAeSnsDft->u32MaxDgainTarget = pstAeSnsDft->u32MaxDgain;
		pstAeSnsDft->u32MinDgainTarget = pstAeSnsDft->u32MinDgain;

		pstAeSnsDft->u8AeCompensation = 40;
		pstAeSnsDft->u32InitAESpeed = 64;
		pstAeSnsDft->u32InitAETolerance = 2;
		pstAeSnsDft->u32AEResponseFrame = 4;
		pstAeSnsDft->enAeExpMode = AE_EXP_HIGHLIGHT_PRIOR;
		pstAeSnsDft->u32InitExposure = g_au32InitExposure[ViPipe] ?
			g_au32InitExposure[ViPipe] : g_astGc4683_mode[pstSnsState->u8ImgMode].stExp[0].u16Def;

		pstAeSnsDft->u32MaxIntTime = pstSnsState->u32FLStd - 8;
		pstAeSnsDft->u32MinIntTime = g_astGc4683_mode[pstSnsState->u8ImgMode].stExp[0].u16Min;
		pstAeSnsDft->u32MaxIntTimeTarget = 65535;
		pstAeSnsDft->u32MinIntTimeTarget = 1;
		break;

	case WDR_MODE_2To1_LINE:
		pstAeSnsDft->f32Fps = g_astGc4683_mode[pstSnsState->u8ImgMode].f32MaxFps;
		pstAeSnsDft->f32MinFps = g_astGc4683_mode[pstSnsState->u8ImgMode].f32MinFps;
		pstAeSnsDft->au8HistThresh[0] = 0xC;
		pstAeSnsDft->au8HistThresh[1] = 0x18;
		pstAeSnsDft->au8HistThresh[2] = 0x60;
		pstAeSnsDft->au8HistThresh[3] = 0x80;

		pstAeSnsDft->u32MaxIntTime = pstSnsState->u32FLStd - 1;
		pstAeSnsDft->u32MinIntTime = 1;

		pstAeSnsDft->u32MaxIntTimeTarget = 65535;
		pstAeSnsDft->u32MinIntTimeTarget = pstAeSnsDft->u32MinIntTime;

		pstAeSnsDft->u32MaxAgain = g_astGc4683_mode[pstSnsState->u8ImgMode].stAgain[0].u32Max;
		pstAeSnsDft->u32MinAgain = g_astGc4683_mode[pstSnsState->u8ImgMode].stAgain[0].u32Min;
		pstAeSnsDft->u32MaxAgainTarget = pstAeSnsDft->u32MaxAgain;
		pstAeSnsDft->u32MinAgainTarget = pstAeSnsDft->u32MinAgain;

		pstAeSnsDft->u32MaxDgain = g_astGc4683_mode[pstSnsState->u8ImgMode].stDgain[0].u32Max;
		pstAeSnsDft->u32MinDgain = g_astGc4683_mode[pstSnsState->u8ImgMode].stDgain[0].u32Min;
		pstAeSnsDft->u32MaxDgainTarget = pstAeSnsDft->u32MaxDgain;
		pstAeSnsDft->u32MinDgainTarget = pstAeSnsDft->u32MinDgain;
		pstAeSnsDft->u32MaxISPDgainTarget = 16 << pstAeSnsDft->u32ISPDgainShift;

		pstAeSnsDft->u32InitExposure = g_au32InitExposure[ViPipe] ?
			g_au32InitExposure[ViPipe] : g_astGc4683_mode[pstSnsState->u8ImgMode].stExp[1].u16Def;
		pstAeSnsDft->u32InitAESpeed = 64;
		pstAeSnsDft->u32InitAETolerance = 5;
		pstAeSnsDft->u32AEResponseFrame = 4;
		if (genFSWDRMode[ViPipe] == ISP_FSWDR_LONG_FRAME_MODE) {
			pstAeSnsDft->u8AeCompensation = 64;
			pstAeSnsDft->enAeExpMode = AE_EXP_HIGHLIGHT_PRIOR;
		} else {
			pstAeSnsDft->u8AeCompensation = 40;
			pstAeSnsDft->enAeExpMode = AE_EXP_LOWLIGHT_PRIOR;
		}
		break;
	}

	return CVI_SUCCESS;
}

/* the function of sensor set fps */
static CVI_S32 cmos_fps_set(VI_PIPE ViPipe, CVI_FLOAT f32Fps, AE_SENSOR_DEFAULT_S *pstAeSnsDft)
{
	ISP_SNS_STATE_S *pstSnsState = CVI_NULL;
	CVI_U32 u32VMAX;
	CVI_FLOAT f32MaxFps = 0;
	CVI_FLOAT f32MinFps = 0;
	CVI_U32 u32Vts = 0;
	ISP_SNS_REGS_INFO_S *pstSnsRegsInfo = CVI_NULL;

	CMOS_CHECK_POINTER(pstAeSnsDft);
	GC4683_SENSOR_GET_CTX(ViPipe, pstSnsState);
	CMOS_CHECK_POINTER(pstSnsState);

	u32Vts = g_astGc4683_mode[pstSnsState->u8ImgMode].u32VtsDef;
	pstSnsRegsInfo = &pstSnsState->astSyncInfo[0].snsCfg;
	f32MaxFps = g_astGc4683_mode[pstSnsState->u8ImgMode].f32MaxFps;
	f32MinFps = g_astGc4683_mode[pstSnsState->u8ImgMode].f32MinFps;

	if ((f32Fps <= f32MaxFps) && (f32Fps >= f32MinFps)) {
		u32VMAX = u32Vts * f32MaxFps / DIV_0_TO_1_FLOAT(f32Fps);
	} else {
		CVI_TRACE_SNS(CVI_DBG_ERR, "Unsupport Fps: %f\n", f32Fps);
		return CVI_FAILURE;
	}

	u32VMAX = (u32VMAX > GC4683_FULL_LINES_MAX) ? GC4683_FULL_LINES_MAX : u32VMAX;

	if (pstSnsState->enWDRMode == WDR_MODE_NONE) {
		pstSnsRegsInfo->astI2cData[LINEAR_VTS_H].u32Data = ((u32VMAX & 0xFF00) >> 8);
		pstSnsRegsInfo->astI2cData[LINEAR_VTS_L].u32Data = (u32VMAX & 0xFF);
	} else if (pstSnsState->enWDRMode == WDR_MODE_2To1_LINE) {
		pstSnsRegsInfo->astI2cData[WDR2_VTS_H].u32Data = ((u32VMAX & 0xFF00) >> 8);
		pstSnsRegsInfo->astI2cData[WDR2_VTS_L].u32Data = (u32VMAX & 0xFF);
	} else {
		CVI_TRACE_SNS(CVI_DBG_ERR, "Unsupport WDRMode: %d\n", pstSnsState->enWDRMode);
		return CVI_FAILURE;
	}

	pstSnsState->u32FLStd = u32VMAX;

	if (pstSnsState->enWDRMode == WDR_MODE_2To1_LINE) {
		GC4683_STATE_S *pstState = GC4683_STATE_GET_CTX(ViPipe);
		pstState->u32Sexp_MAX = u32VMAX - 1440 - 8;
	}

	pstAeSnsDft->f32Fps = f32Fps;
	pstAeSnsDft->u32LinesPer500ms = pstSnsState->u32FLStd * f32Fps / 2;
	pstAeSnsDft->u32FullLinesStd = pstSnsState->u32FLStd;
	pstAeSnsDft->u32MaxIntTime = pstSnsState->u32FLStd - 8;
	pstSnsState->au32FL[0] = pstSnsState->u32FLStd;
	pstSnsState->au32FL[1] = pstSnsState->u32FLStd;
	pstAeSnsDft->u32FullLines = pstSnsState->au32FL[0];
	pstAeSnsDft->u32HmaxTimes = (1000000) / (pstSnsState->u32FLStd * DIV_0_TO_1_FLOAT(f32Fps));

	return CVI_SUCCESS;
}

/* while isp notify ae to update sensor regs, ae call these funcs. */
static CVI_S32 cmos_inttime_update(VI_PIPE ViPipe, CVI_U32 *u32IntTime)
{
	ISP_SNS_STATE_S *pstSnsState = CVI_NULL;
	ISP_SNS_REGS_INFO_S *pstSnsRegsInfo = CVI_NULL;

	GC4683_SENSOR_GET_CTX(ViPipe, pstSnsState);
	CMOS_CHECK_POINTER(pstSnsState);
	CMOS_CHECK_POINTER(u32IntTime);
	pstSnsRegsInfo = &pstSnsState->astSyncInfo[0].snsCfg;

	if (pstSnsState->enWDRMode == WDR_MODE_2To1_LINE) {
		GC4683_STATE_S *pstState = GC4683_STATE_GET_CTX(ViPipe);
		CVI_U32 u32ShortIntTime = u32IntTime[0];
		CVI_U32 u32LongIntTime = u32IntTime[1];
		CVI_U32 u32MaxSexp = pstState->u32Sexp_MAX;
		CVI_U32 u32MaxLexp;

		pstSnsState->au32WDRIntTime[0] = (u32ShortIntTime <= u32MaxSexp) ?
			u32ShortIntTime : u32MaxSexp;
		if (!pstSnsState->au32WDRIntTime[0])
			pstSnsState->au32WDRIntTime[0] = 1;

		u32MaxLexp = pstSnsState->au32FL[0] - pstSnsState->au32WDRIntTime[0] - 8;
		pstSnsState->au32WDRIntTime[1] = (u32LongIntTime <= u32MaxLexp) ?
			u32LongIntTime : u32MaxLexp;
		if (!pstSnsState->au32WDRIntTime[1])
			pstSnsState->au32WDRIntTime[1] = 1;

		pstSnsRegsInfo->astI2cData[WDR2_SEXP_H].u32Data = ((pstSnsState->au32WDRIntTime[0] >> 8) & 0xFF);
		pstSnsRegsInfo->astI2cData[WDR2_SEXP_L].u32Data = (pstSnsState->au32WDRIntTime[0] & 0xFF);
		pstSnsRegsInfo->astI2cData[WDR2_LEXP_H].u32Data = ((pstSnsState->au32WDRIntTime[1] >> 8) & 0xFF);
		pstSnsRegsInfo->astI2cData[WDR2_LEXP_L].u32Data = (pstSnsState->au32WDRIntTime[1] & 0xFF);
	} else {
		pstSnsRegsInfo->astI2cData[LINEAR_EXP_H].u32Data = ((u32IntTime[0] >> 8));
		pstSnsRegsInfo->astI2cData[LINEAR_EXP_L].u32Data = (u32IntTime[0] & 0xFF);
	}

	return CVI_SUCCESS;
}

static CVI_U32 regValTable_linear[16][9] = {
	// 02aa, 02ab, 02a9, 14ac  1476, 1467, 1468, 00a8, 00a9,    |  实际倍数|Again dB|
	{  0x00, 0x02, 0x00, 0x03, 0xe0, 0x23, 0x23, 0x01, 0x00,},//|    1.000 |  0.000 |
	{  0x00, 0x03, 0x00, 0x03, 0xe0, 0x24, 0x24, 0x01, 0x18,},//|    1.376 |  2.850 |
	{  0x00, 0x00, 0x01, 0x03, 0xe8, 0x29, 0x29, 0x02, 0x04,},//|    2.070 |  5.903 |
	{  0x00, 0x00, 0x00, 0x00, 0xe0, 0x23, 0x23, 0x02, 0x27,},//|    2.622 |  8.749 |
	{  0x00, 0x01, 0x00, 0x00, 0xe0, 0x23, 0x23, 0x03, 0x28,},//|    3.633 | 12.026 |
	{  0x00, 0x02, 0x00, 0x00, 0xe0, 0x23, 0x23, 0x05, 0x08,},//|    5.133 | 13.680 |
	{  0x00, 0x03, 0x00, 0x00, 0xe0, 0x24, 0x24, 0x07, 0x04,},//|    7.072 | 16.523 |
	{  0x00, 0x00, 0x01, 0x00, 0xe8, 0x29, 0x29, 0x0A, 0x27,},//|   10.611 | 19.527 |
	{  0x00, 0x01, 0x01, 0x00, 0xe8, 0x2a, 0x2a, 0x0E, 0x2D,},//|   14.708 | 22.259 |
	{  0x00, 0x02, 0x01, 0x00, 0xe8, 0x2b, 0x2b, 0x14, 0x2F,},//|   20.746 | 25.558 |
	{  0x00, 0x03, 0x01, 0x00, 0xe8, 0x2c, 0x2c, 0x1C, 0x24,},//|   28.566 | 28.403 |
	{  0x00, 0x04, 0x01, 0x00, 0xe8, 0x2e, 0x2e, 0x27, 0x3D,},//|   39.961 | 31.440 |
	{  0x00, 0x05, 0x01, 0x00, 0xe8, 0x30, 0x30, 0x36, 0x12,},//|   54.289 | 34.312 |
	{  0x00, 0x06, 0x01, 0x00, 0xe8, 0x32, 0x32, 0x4A, 0x16,},//|   74.354 | 37.340 |
	{  0x04, 0xb6, 0x01, 0x00, 0xe8, 0x34, 0x34, 0x68, 0x22,},//|  104.542 | 39.980 |
	{  0x08, 0x06, 0x01, 0x00, 0xe8, 0x37, 0x37, 0x93, 0x28,},//|  147.627 | 42.710 |
};

static CVI_U32 gain_table[16] = {
	1024,
	1408,
	2112,
	2688,
	3728,
	5264,
	7248,
	10864,
	15056,
	21248,
	29248,
	40928,
	55584,
	76144,
	107056,

	151168  // 9334 * 16 = 151168
};

static CVI_U32 regValTable_WDR_LEF[14][8] = {
	// 0x02aa, 0x02ab, 0x02a9, 0x14ac, 0x1467, 0x1468, 0x00a8, 0x00a9
	{0x00, 0x02, 0x00, 0x03, 0x23, 0x23, 0x01, 0x00},// 1.000x
	{0x00, 0x03, 0x00, 0x03, 0x24, 0x24, 0x01, 0x17},// 1.366x
	{0x00, 0x00, 0x01, 0x03, 0x29, 0x29, 0x01, 0x3b},// 1.923x
	{0x00, 0x00, 0x00, 0x00, 0x23, 0x23, 0x02, 0x29},// 2.650x
	{0x00, 0x01, 0x00, 0x00, 0x23, 0x23, 0x03, 0x27},// 3.619x
	{0x00, 0x02, 0x00, 0x00, 0x23, 0x23, 0x05, 0x06},// 5.095x
	{0x00, 0x03, 0x00, 0x00, 0x24, 0x24, 0x07, 0x01},// 7.016x
	{0x00, 0x00, 0x01, 0x00, 0x29, 0x29, 0x0a, 0x2c},// 10.694x
	{0x00, 0x01, 0x01, 0x00, 0x2a, 0x2a, 0x0e, 0x25},// 14.591x
	{0x00, 0x02, 0x01, 0x00, 0x2b, 0x2b, 0x14, 0x1c},// 20.445x
	{0x00, 0x03, 0x01, 0x00, 0x2c, 0x2c, 0x1c, 0x0c},// 28.192x
	{0x00, 0x04, 0x01, 0x00, 0x2e, 0x2e, 0x27, 0x2b},// 39.684x
	{0x00, 0x05, 0x01, 0x00, 0x30, 0x30, 0x36, 0x0d},// 54.218x
	{0x00, 0x06, 0x01, 0x00, 0x32, 0x32, 0x4a, 0x13},// 74.299x
};

static CVI_U32 regValTable_WDR_SEF[14][7] = {
	// 0x0293, 0x0294, 0x14ab, 0x1457, 0x1458, 0x00aa, 0x00ab
	{0x00, 0x02, 0x33, 0x23, 0x23, 0x01, 0x00},// 1.000x
	{0x00, 0x03, 0x33, 0x24, 0x24, 0x01, 0x17},// 1.366x
	{0x10, 0x00, 0x33, 0x29, 0x29, 0x01, 0x3b},// 1.923x
	{0x00, 0x00, 0x30, 0x23, 0x23, 0x02, 0x29},// 2.650x
	{0x00, 0x01, 0x30, 0x23, 0x23, 0x03, 0x27},// 3.619x
	{0x00, 0x02, 0x30, 0x23, 0x23, 0x05, 0x06},// 5.095x
	{0x00, 0x03, 0x30, 0x24, 0x24, 0x07, 0x01},// 7.016x
	{0x10, 0x00, 0x30, 0x29, 0x29, 0x0a, 0x2c},// 10.694x
	{0x10, 0x01, 0x30, 0x2a, 0x2a, 0x0e, 0x25},// 14.591x
	{0x10, 0x02, 0x30, 0x2b, 0x2b, 0x14, 0x1c},// 20.445x
	{0x10, 0x03, 0x30, 0x2c, 0x2c, 0x1c, 0x0c},// 28.192x
	{0x10, 0x04, 0x30, 0x2e, 0x2e, 0x27, 0x2b},// 39.684x
	{0x10, 0x05, 0x30, 0x30, 0x30, 0x36, 0x0d},// 54.218x
	{0x10, 0x06, 0x30, 0x32, 0x32, 0x4a, 0x13},// 74.299x
};

static CVI_U32 gain_table_wdr[14] = {
	1024, 1392, 2086, 2704, 3696, 5216, 7184,
	10944, 14928, 20928, 28864, 40624, 55504, 76080
};


static CVI_S32 cmos_again_calc_table(VI_PIPE ViPipe, CVI_U32 *pu32AgainLin, CVI_U32 *pu32AgainDb)
{
	ISP_SNS_STATE_S *pstSnsState = CVI_NULL;
	CVI_U32 *table;
	int i, total;
	CVI_U32 pregain;

	CMOS_CHECK_POINTER(pu32AgainLin);
	CMOS_CHECK_POINTER(pu32AgainDb);

	GC4683_SENSOR_GET_CTX(ViPipe, pstSnsState);
	CMOS_CHECK_POINTER(pstSnsState);

	if (pstSnsState->enWDRMode == WDR_MODE_2To1_LINE) {
		table = gain_table_wdr;
		total = sizeof(gain_table_wdr) / sizeof(CVI_U32);
	} else {
		table = gain_table;
		total = sizeof(gain_table) / sizeof(CVI_U32);
	}

	if (*pu32AgainLin >= table[total - 1]) {
		*pu32AgainLin = *pu32AgainDb = table[total - 1];
		return CVI_SUCCESS;
	}

	for (i = 1; i < total; i++) {
		if (*pu32AgainLin < table[i])
			break;
	}
	i--;
	pregain = (*pu32AgainLin * 64) / table[i];
	*pu32AgainDb = *pu32AgainLin;
	*pu32AgainLin = pregain * table[i] / 64;

	return CVI_SUCCESS;
}

static CVI_S32 cmos_dgain_calc_table(VI_PIPE ViPipe, CVI_U32 *pu32DgainLin, CVI_U32 *pu32DgainDb)
{
	CVI_U32 pregain;

	CMOS_CHECK_POINTER(pu32DgainLin);
	CMOS_CHECK_POINTER(pu32DgainDb);
	UNUSED(ViPipe);
	//find the pregain
	pregain = (*pu32DgainLin * 64) / 1024;
	//set the Db as the AE algo gain, we need this to do gain update
	*pu32DgainDb = *pu32DgainLin;
	//set the Lin as the closest sensor gain for AE algo reference
	*pu32DgainLin = pregain * 16;

	return CVI_SUCCESS;
}

static CVI_S32 cmos_gains_update(VI_PIPE ViPipe, CVI_U32 *pu32Again, CVI_U32 *pu32Dgain)
{
	ISP_SNS_STATE_S *pstSnsState = CVI_NULL;
	ISP_SNS_REGS_INFO_S *pstSnsRegsInfo = CVI_NULL;
	CVI_U32 u16Mode = g_au16Gc4683_GainMode[ViPipe];
	CVI_U32 u32Again;
	CVI_U32 u32Dgain;
	int i, total;

	GC4683_SENSOR_GET_CTX(ViPipe, pstSnsState);
	CMOS_CHECK_POINTER(pstSnsState);
	CMOS_CHECK_POINTER(pu32Again);
	CMOS_CHECK_POINTER(pu32Dgain);
	pstSnsRegsInfo = &pstSnsState->astSyncInfo[0].snsCfg;

	if (pstSnsState->enWDRMode == WDR_MODE_2To1_LINE) {
		int total_wdr = sizeof(gain_table_wdr) / sizeof(CVI_U32);
		CVI_U32 u32Again_SE, u32Again_LE;
		CVI_U32 u32Dgain_SE, u32Dgain_LE;

		/* Short exposure gain - index 0 */
		u32Again_SE = pu32Again[0];
		if (u32Again_SE < gain_table_wdr[total_wdr - 1]) {
			for (i = 1; i < total_wdr; i++) {
				if (u32Again_SE < gain_table_wdr[i])
					break;
			}
			i--;
			u32Dgain_SE = (u32Again_SE * 1024) / gain_table_wdr[i];
			u32Again_SE = i;
		} else {
			u32Again_SE = total_wdr - 1;
			u32Dgain_SE = pu32Dgain[0];
		}

		if (u16Mode == SNS_GAIN_MODE_WDR_2F) {
			/* Long exposure gain - index 1 */
			u32Again_LE = pu32Again[1];
			if (u32Again_LE < gain_table_wdr[total_wdr - 1]) {
				for (i = 1; i < total_wdr; i++) {
					if (u32Again_LE < gain_table_wdr[i])
						break;
				}
				i--;
				u32Dgain_LE = (u32Again_LE * 1024) / gain_table_wdr[i];
				u32Again_LE = i;
			} else {
				u32Again_LE = total_wdr - 1;
				u32Dgain_LE = pu32Dgain[1];
			}
		} else {
			/* SNS_GAIN_MODE_SHARE: LE/SE use the same gain */
			u32Again_LE = u32Again_SE;
			u32Dgain_LE = u32Dgain_SE;
		}

		/* LE analog gain */
		pstSnsRegsInfo->astI2cData[WDR2_LAGAIN_H].u32Data = regValTable_WDR_LEF[u32Again_LE][0];
		pstSnsRegsInfo->astI2cData[WDR2_LAGAIN_L].u32Data = regValTable_WDR_LEF[u32Again_LE][1];
		pstSnsRegsInfo->astI2cData[WDR2_LAGAIN_PGA].u32Data = regValTable_WDR_LEF[u32Again_LE][2];
		pstSnsRegsInfo->astI2cData[WDR2_LAGAIN_MAG1].u32Data = regValTable_WDR_LEF[u32Again_LE][3];
		pstSnsRegsInfo->astI2cData[WDR2_LAGAIN_MAG2].u32Data = regValTable_WDR_LEF[u32Again_LE][4];
		pstSnsRegsInfo->astI2cData[WDR2_LAGAIN_MAG3].u32Data = regValTable_WDR_LEF[u32Again_LE][5];
		pstSnsRegsInfo->astI2cData[WDR2_LCOL_AGAIN_H].u32Data = regValTable_WDR_LEF[u32Again_LE][6];
		pstSnsRegsInfo->astI2cData[WDR2_LCOL_AGAIN_L].u32Data = regValTable_WDR_LEF[u32Again_LE][7];
		/* LE digital gain */
		pstSnsRegsInfo->astI2cData[WDR2_LDGAIN_H].u32Data = (u32Dgain_LE >> 8);
		pstSnsRegsInfo->astI2cData[WDR2_LDGAIN_L].u32Data = (u32Dgain_LE & 0xFF);

		/* SE analog gain */
		pstSnsRegsInfo->astI2cData[WDR2_SAGAIN_H].u32Data = regValTable_WDR_SEF[u32Again_SE][0];
		pstSnsRegsInfo->astI2cData[WDR2_SAGAIN_L].u32Data = regValTable_WDR_SEF[u32Again_SE][1];
		pstSnsRegsInfo->astI2cData[WDR2_SAGAIN_MAG1].u32Data = regValTable_WDR_SEF[u32Again_SE][2];
		pstSnsRegsInfo->astI2cData[WDR2_SAGAIN_MAG2].u32Data = regValTable_WDR_SEF[u32Again_SE][3];
		pstSnsRegsInfo->astI2cData[WDR2_SAGAIN_MAG3].u32Data = regValTable_WDR_SEF[u32Again_SE][4];
		pstSnsRegsInfo->astI2cData[WDR2_SCOL_AGAIN_H].u32Data = regValTable_WDR_SEF[u32Again_SE][5];
		pstSnsRegsInfo->astI2cData[WDR2_SCOL_AGAIN_L].u32Data = regValTable_WDR_SEF[u32Again_SE][6];
		/* SE digital gain */
		pstSnsRegsInfo->astI2cData[WDR2_SDGAIN_H].u32Data = (u32Dgain_SE >> 8);
		pstSnsRegsInfo->astI2cData[WDR2_SDGAIN_L].u32Data = (u32Dgain_SE & 0xFF);
	} else if (pstSnsState->enWDRMode == WDR_MODE_NONE) {
		total = sizeof(gain_table) / sizeof(CVI_U32);

		u32Again = pu32Again[0];
		if (u32Again < (151168)) {
			for (i = 1; i < total; i++) {
				if (*pu32Again < gain_table[i])
					break;
			}
			i--;
			u32Dgain = (u32Again * 1024) / gain_table[i];
			u32Again = i;
		} else {
			u32Again = total - 1;
			u32Dgain = pu32Dgain[0];
		}

		pstSnsRegsInfo->astI2cData[LINEAR_AGAIN_H].u32Data = regValTable_linear[u32Again][0];
		pstSnsRegsInfo->astI2cData[LINEAR_AGAIN_L].u32Data = regValTable_linear[u32Again][1];
		pstSnsRegsInfo->astI2cData[LINEAR_AGAIN_PGA].u32Data = regValTable_linear[u32Again][2];
		pstSnsRegsInfo->astI2cData[LINEAR_AGAIN_MAG1].u32Data = regValTable_linear[u32Again][3];
		pstSnsRegsInfo->astI2cData[LINEAR_AGAIN_MAG2].u32Data = regValTable_linear[u32Again][4];
		pstSnsRegsInfo->astI2cData[LINEAR_AGAIN_MAG3].u32Data = regValTable_linear[u32Again][5];
		pstSnsRegsInfo->astI2cData[LINEAR_AGAIN_MAG4].u32Data = regValTable_linear[u32Again][6];
		pstSnsRegsInfo->astI2cData[LINEAR_COL_AGAIN_H].u32Data = regValTable_linear[u32Again][7];
		pstSnsRegsInfo->astI2cData[LINEAR_COL_AGAIN_L].u32Data = regValTable_linear[u32Again][8];

		pstSnsRegsInfo->astI2cData[LINEAR_DGAIN_H].u32Data = (u32Dgain >> 8);
		pstSnsRegsInfo->astI2cData[LINEAR_DGAIN_L].u32Data = (u32Dgain & 0xFF);
	} else {
		CVI_TRACE_SNS(CVI_DBG_ERR, "Unsupport WDRMode: %d\n", pstSnsState->enWDRMode);
		return CVI_FAILURE;
	}
	return CVI_SUCCESS;
}

static CVI_S32 cmos_get_inttime_max(VI_PIPE ViPipe, CVI_U16 u16ManRatioEnable, CVI_U32 *au32Ratio,
				    CVI_U32 *au32IntTimeMax, CVI_U32 *au32IntTimeMin, CVI_U32 *pu32LFMaxIntTime)
{
	CVI_U32 u32IntTimeMaxTmp = 0, u32IntTimeMaxTmp0 = 0, u32IntTimeMaxTmp1 = 0;
	CVI_U32 u32ShortTimeMinLimit = 1;
	CVI_U32 u32SexpMax;
	CVI_U32 u32FL;
	ISP_SNS_STATE_S *pstSnsState = CVI_NULL;
	GC4683_STATE_S *pstState = CVI_NULL;

	(void)u16ManRatioEnable;

	CMOS_CHECK_POINTER(au32Ratio);
	CMOS_CHECK_POINTER(au32IntTimeMax);
	CMOS_CHECK_POINTER(au32IntTimeMin);
	CMOS_CHECK_POINTER(pu32LFMaxIntTime);
	GC4683_SENSOR_GET_CTX(ViPipe, pstSnsState);
	CMOS_CHECK_POINTER(pstSnsState);

	if (pstSnsState->enWDRMode != WDR_MODE_2To1_LINE)
		return CVI_SUCCESS;

	pstState = GC4683_STATE_GET_CTX(ViPipe);
	u32FL = pstSnsState->au32FL[0];
	u32SexpMax = pstState->u32Sexp_MAX;

	/* short frame: T1 < VTS - 1440 - 8 */
	/* T1 + T2 < VTS - 8, T2 = T1 * ratio / 64 */
	/* => T1 < (FL[0] - 8) * 64 / (64 + ratio) */
	u32IntTimeMaxTmp0 = ((u32FL - 8) * 0x40) / DIV_0_TO_1(au32Ratio[0] + 0x40);
	u32IntTimeMaxTmp = (u32SexpMax < u32IntTimeMaxTmp0) ? u32SexpMax : u32IntTimeMaxTmp0;

	/* long frame: T1 < (FL[1] - 8) * 64 / (64 + ratio) */
	u32IntTimeMaxTmp1 = ((pstSnsState->au32FL[1] - 8) * 0x40) / DIV_0_TO_1(au32Ratio[0] + 0x40);
	if (u32IntTimeMaxTmp1 < u32IntTimeMaxTmp)
		u32IntTimeMaxTmp = u32IntTimeMaxTmp1;

	u32IntTimeMaxTmp = (!u32IntTimeMaxTmp) ? u32ShortTimeMinLimit : u32IntTimeMaxTmp;
	if (u32IntTimeMaxTmp < u32ShortTimeMinLimit)
		u32IntTimeMaxTmp = u32ShortTimeMinLimit;

	au32IntTimeMax[0] = u32IntTimeMaxTmp;
	au32IntTimeMax[1] = au32IntTimeMax[0] * au32Ratio[0] >> 6;
	au32IntTimeMin[0] = u32ShortTimeMinLimit;
	au32IntTimeMin[1] = au32IntTimeMin[0] * au32Ratio[0] >> 6;
	*pu32LFMaxIntTime = au32IntTimeMax[1];

	return CVI_SUCCESS;
}

/* Only used in LINE_WDR mode */
static CVI_S32 cmos_ae_fswdr_attr_set(VI_PIPE ViPipe, AE_FSWDR_ATTR_S *pstAeFSWDRAttr)
{
	CMOS_CHECK_POINTER(pstAeFSWDRAttr);

	genFSWDRMode[ViPipe] = pstAeFSWDRAttr->enFSWDRMode;
	gu32MaxTimeGetCnt[ViPipe] = 0;

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
	pstExpFuncs->pfn_cmos_get_inttime_max   = cmos_get_inttime_max;
	pstExpFuncs->pfn_cmos_ae_fswdr_attr_set = cmos_ae_fswdr_attr_set;

	return CVI_SUCCESS;
}

static CVI_S32 cmos_get_awb_default(VI_PIPE ViPipe, AWB_SENSOR_DEFAULT_S *pstAwbSnsDft)
{
	CMOS_CHECK_POINTER(pstAwbSnsDft);
	UNUSED(ViPipe);
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
	UNUSED(ViPipe);
	memset(pstDef, 0, sizeof(ISP_CMOS_DEFAULT_S));

	memcpy(pstDef->stNoiseCalibration.CalibrationCoef,
		&g_stIspNoiseCalibratio, sizeof(ISP_CMOS_NOISE_CALIBRATION_S));

	return CVI_SUCCESS;
}

static CVI_S32 cmos_get_blc_default(VI_PIPE ViPipe, ISP_CMOS_BLACK_LEVEL_S *pstBlc)
{
	CMOS_CHECK_POINTER(pstBlc);
	UNUSED(ViPipe);
	memset(pstBlc, 0, sizeof(ISP_CMOS_BLACK_LEVEL_S));

	memcpy(pstBlc,
		&g_stIspBlcCalibratio, sizeof(ISP_CMOS_BLACK_LEVEL_S));
	return CVI_SUCCESS;
}

static CVI_S32 cmos_get_wdr_size(VI_PIPE ViPipe, ISP_SNS_ISP_INFO_S *pstIspCfg)
{
	const GC4683_MODE_S *pstMode = CVI_NULL;
	ISP_SNS_STATE_S *pstSnsState = CVI_NULL;

	GC4683_SENSOR_GET_CTX(ViPipe, pstSnsState);
	CMOS_CHECK_POINTER(pstSnsState);
	pstMode = &g_astGc4683_mode[pstSnsState->u8ImgMode];

	if (pstSnsState->enWDRMode == WDR_MODE_2To1_LINE) {
		pstIspCfg->frm_num = 2;
		memcpy(&pstIspCfg->img_size[0], &pstMode->astImg[0], sizeof(ISP_WDR_SIZE_S));
		memcpy(&pstIspCfg->img_size[1], &pstMode->astImg[1], sizeof(ISP_WDR_SIZE_S));
	} else {
		pstIspCfg->frm_num = 1;
		memcpy(&pstIspCfg->img_size[0], &pstMode->astImg[0], sizeof(ISP_WDR_SIZE_S));
	}

	return CVI_SUCCESS;
}

static CVI_S32 cmos_set_wdr_mode(VI_PIPE ViPipe, CVI_U8 u8Mode)
{
	ISP_SNS_STATE_S *pstSnsState = CVI_NULL;

	GC4683_SENSOR_GET_CTX(ViPipe, pstSnsState);
	CMOS_CHECK_POINTER(pstSnsState);

	pstSnsState->bSyncInit = CVI_FALSE;

	switch (u8Mode) {
	case WDR_MODE_NONE:
		if (pstSnsState->u8ImgMode == GC4683_MODE_2560X1440P30_WDR)
			pstSnsState->u8ImgMode = GC4683_MODE_2560X1440P60;
		pstSnsState->enWDRMode = WDR_MODE_NONE;
		pstSnsState->u32FLStd = g_astGc4683_mode[pstSnsState->u8ImgMode].u32VtsDef;
		CVI_TRACE_SNS(CVI_DBG_INFO, "WDR_MODE_NONE\n");
		break;

	case WDR_MODE_2To1_LINE:
		if (pstSnsState->u8ImgMode == GC4683_MODE_2560X1440P60)
			pstSnsState->u8ImgMode = GC4683_MODE_2560X1440P30_WDR;
		pstSnsState->enWDRMode = WDR_MODE_2To1_LINE;
		pstSnsState->u32FLStd = g_astGc4683_mode[pstSnsState->u8ImgMode].u32VtsDef;
		break;

	default:
		CVI_TRACE_SNS(CVI_DBG_ERR, "Unsupport sensor mode!\n");
		return CVI_FAILURE;
	}

	pstSnsState->au32FL[0] = pstSnsState->u32FLStd;
	pstSnsState->au32FL[1] = pstSnsState->au32FL[0];
	memset(pstSnsState->au32WDRIntTime, 0, sizeof(pstSnsState->au32WDRIntTime));

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
	GC4683_SENSOR_GET_CTX(ViPipe, pstSnsState);
	CMOS_CHECK_POINTER(pstSnsState);
	pstSnsRegsInfo = &pstSnsSyncInfo->snsCfg;
	pstCfg0 = &pstSnsState->astSyncInfo[0];
	pstCfg1 = &pstSnsState->astSyncInfo[1];
	pstI2c_data = pstCfg0->snsCfg.astI2cData;

	if ((pstSnsState->bSyncInit == CVI_FALSE) || (pstSnsRegsInfo->bConfig == CVI_FALSE)) {
		pstCfg0->snsCfg.enSnsType = SNS_I2C_TYPE;
		pstCfg0->snsCfg.unComBus.s8I2cDev = g_aunGc4683_BusInfo[ViPipe].s8I2cDev;
		pstCfg0->snsCfg.u8Cfg2ValidDelayMax = 0;
		pstCfg0->snsCfg.use_snsr_sram = CVI_TRUE;

		if (pstSnsState->enWDRMode == WDR_MODE_2To1_LINE) {
			pstCfg0->snsCfg.u32RegNum = WDR2_REGS_NUM;
		} else {
			pstCfg0->snsCfg.u32RegNum = LINEAR_REGS_NUM;
		}

		for (i = 0; i < pstCfg0->snsCfg.u32RegNum; i++) {
			pstI2c_data[i].bUpdate = CVI_TRUE;
			pstI2c_data[i].u8DevAddr = g_aunGc4683_AddrInfo[ViPipe].s8I2cAddr;
			pstI2c_data[i].u32AddrByteNum = gc4683_addr_byte;
			pstI2c_data[i].u32DataByteNum = gc4683_data_byte;
		}

		if (pstSnsState->enWDRMode == WDR_MODE_2To1_LINE) {
			pstI2c_data[WDR2_SEXP_H].u32RegAddr = GC4683_WDR_SEXP_H_ADDR;
			pstI2c_data[WDR2_SEXP_L].u32RegAddr = GC4683_WDR_SEXP_L_ADDR;
			pstI2c_data[WDR2_LEXP_H].u32RegAddr = GC4683_WDR_LEXP_H_ADDR;
			pstI2c_data[WDR2_LEXP_L].u32RegAddr = GC4683_WDR_LEXP_L_ADDR;
			pstI2c_data[WDR2_LAGAIN_H].u32RegAddr = GC4683_WDR_LAGAIN_H_ADDR;
			pstI2c_data[WDR2_LAGAIN_L].u32RegAddr = GC4683_WDR_LAGAIN_L_ADDR;
			pstI2c_data[WDR2_LAGAIN_PGA].u32RegAddr = GC4683_WDR_LAGAIN_PGA_ADDR;
			pstI2c_data[WDR2_LAGAIN_MAG1].u32RegAddr = GC4683_WDR_LAGAIN_MAG1_ADDR;
			pstI2c_data[WDR2_LAGAIN_MAG2].u32RegAddr = GC4683_WDR_LAGAIN_MAG2_ADDR;
			pstI2c_data[WDR2_LAGAIN_MAG3].u32RegAddr = GC4683_WDR_LAGAIN_MAG3_ADDR;
			pstI2c_data[WDR2_LCOL_AGAIN_H].u32RegAddr = GC4683_WDR_LCOL_AGAIN_H_ADDR;
			pstI2c_data[WDR2_LCOL_AGAIN_L].u32RegAddr = GC4683_WDR_LCOL_AGAIN_L_ADDR;
			pstI2c_data[WDR2_LDGAIN_H].u32RegAddr = GC4683_WDR_LDGAIN_H_ADDR;
			pstI2c_data[WDR2_LDGAIN_L].u32RegAddr = GC4683_WDR_LDGAIN_L_ADDR;
			pstI2c_data[WDR2_SAGAIN_H].u32RegAddr = GC4683_WDR_SAGAIN_H_ADDR;
			pstI2c_data[WDR2_SAGAIN_L].u32RegAddr = GC4683_WDR_SAGAIN_L_ADDR;
			pstI2c_data[WDR2_SAGAIN_MAG1].u32RegAddr = GC4683_WDR_SAGAIN_MAG1_ADDR;
			pstI2c_data[WDR2_SAGAIN_MAG2].u32RegAddr = GC4683_WDR_SAGAIN_MAG2_ADDR;
			pstI2c_data[WDR2_SAGAIN_MAG3].u32RegAddr = GC4683_WDR_SAGAIN_MAG3_ADDR;
			pstI2c_data[WDR2_SCOL_AGAIN_H].u32RegAddr = GC4683_WDR_SCOL_AGAIN_H_ADDR;
			pstI2c_data[WDR2_SCOL_AGAIN_L].u32RegAddr = GC4683_WDR_SCOL_AGAIN_L_ADDR;
			pstI2c_data[WDR2_SDGAIN_H].u32RegAddr = GC4683_WDR_SDGAIN_H_ADDR;
			pstI2c_data[WDR2_SDGAIN_L].u32RegAddr = GC4683_WDR_SDGAIN_L_ADDR;
			pstI2c_data[WDR2_VTS_H].u32RegAddr = GC4683_WDR_VTS_H_ADDR;
			pstI2c_data[WDR2_VTS_L].u32RegAddr = GC4683_WDR_VTS_L_ADDR;
			pstI2c_data[WDR2_FLIP_MIRROR].u32RegAddr = GC4683_FLIP_MIRROR_ADDR;
		} else {
			pstI2c_data[LINEAR_EXP_H].u32RegAddr = GC4683_EXP_H_ADDR;
			pstI2c_data[LINEAR_EXP_L].u32RegAddr = GC4683_EXP_L_ADDR;
			pstI2c_data[LINEAR_AGAIN_H].u32RegAddr = GC4683_AGAIN_H_ADDR;
			pstI2c_data[LINEAR_AGAIN_L].u32RegAddr = GC4683_AGAIN_L_ADDR;
			pstI2c_data[LINEAR_AGAIN_PGA].u32RegAddr = GC4683_AGAIN_PGA_ADDR;
			pstI2c_data[LINEAR_AGAIN_MAG1].u32RegAddr = GC4683_AGAIN_MAG1_ADDR;
			pstI2c_data[LINEAR_AGAIN_MAG2].u32RegAddr = GC4683_AGAIN_MAG2_ADDR;
			pstI2c_data[LINEAR_AGAIN_MAG3].u32RegAddr = GC4683_AGAIN_MAG3_ADDR;
			pstI2c_data[LINEAR_AGAIN_MAG4].u32RegAddr = GC4683_AGAIN_MAG4_ADDR;
			pstI2c_data[LINEAR_COL_AGAIN_H].u32RegAddr = GC4683_COL_AGAIN_H_ADDR;
			pstI2c_data[LINEAR_COL_AGAIN_L].u32RegAddr = GC4683_COL_AGAIN_L_ADDR;
			pstI2c_data[LINEAR_DGAIN_H].u32RegAddr = GC4683_DGAIN_H_ADDR;
			pstI2c_data[LINEAR_DGAIN_L].u32RegAddr = GC4683_DGAIN_L_ADDR;
			pstI2c_data[LINEAR_VTS_H].u32RegAddr = GC4683_VTS_H_ADDR;
			pstI2c_data[LINEAR_VTS_L].u32RegAddr = GC4683_VTS_L_ADDR;
			pstI2c_data[LINEAR_FLIP_MIRROR].u32RegAddr = GC4683_FLIP_MIRROR_ADDR;
		}

		pstSnsState->bSyncInit = CVI_TRUE;
		pstCfg0->snsCfg.need_update = CVI_TRUE;
		/* recalcualte WDR size */
		cmos_get_wdr_size(ViPipe, &pstCfg0->ispCfg);
		pstCfg0->ispCfg.need_update = CVI_TRUE;
	} else {

		CVI_U32 gainsUpdate = 0, shutterUpdate = 0, vtsUpdate = 0;

		pstCfg0->snsCfg.need_update = CVI_FALSE;
		for (i = 0; i < pstCfg0->snsCfg.u32RegNum; i++) {
			if (pstCfg0->snsCfg.astI2cData[i].u32Data == pstCfg1->snsCfg.astI2cData[i].u32Data) {
				pstCfg0->snsCfg.astI2cData[i].bUpdate = CVI_FALSE;
			} else {
				pstCfg0->snsCfg.astI2cData[i].bUpdate = CVI_TRUE;
				pstCfg0->snsCfg.need_update = CVI_TRUE;

				if (pstSnsState->enWDRMode == WDR_MODE_2To1_LINE) {
					if ((i >= WDR2_LAGAIN_H) && (i <= WDR2_SDGAIN_L))
						gainsUpdate = 1;
					if ((i >= WDR2_SEXP_H) && (i <= WDR2_LEXP_L))
						shutterUpdate = 1;
					if ((i >= WDR2_VTS_H) && (i <= WDR2_VTS_L))
						vtsUpdate = 1;
				} else {
					if ((i >= LINEAR_AGAIN_H) && (i <= LINEAR_DGAIN_L))
						gainsUpdate = 1;
					if (i <= LINEAR_EXP_L)
						shutterUpdate = 1;
					if ((i >= LINEAR_VTS_H) && (i <= LINEAR_VTS_L))
						vtsUpdate = 1;
				}
			}
		}

		if (pstSnsState->enWDRMode == WDR_MODE_2To1_LINE) {
			if (gainsUpdate) {
				for (i = WDR2_LAGAIN_H; i <= WDR2_SDGAIN_L; i++)
					pstCfg0->snsCfg.astI2cData[i].bUpdate = CVI_TRUE;
			}
			if (shutterUpdate) {
				for (i = WDR2_SEXP_H; i <= WDR2_LEXP_L; i++)
					pstCfg0->snsCfg.astI2cData[i].bUpdate = CVI_TRUE;
			}
			if (vtsUpdate) {
				pstCfg0->snsCfg.astI2cData[WDR2_VTS_H].bUpdate = CVI_TRUE;
				pstCfg0->snsCfg.astI2cData[WDR2_VTS_L].bUpdate = CVI_TRUE;
			}
		} else {
			if (gainsUpdate) {
				pstCfg0->snsCfg.astI2cData[LINEAR_AGAIN_H].bUpdate = CVI_TRUE;
				pstCfg0->snsCfg.astI2cData[LINEAR_AGAIN_L].bUpdate = CVI_TRUE;
				pstCfg0->snsCfg.astI2cData[LINEAR_AGAIN_PGA].bUpdate = CVI_TRUE;
				pstCfg0->snsCfg.astI2cData[LINEAR_AGAIN_MAG1].bUpdate = CVI_TRUE;
				pstCfg0->snsCfg.astI2cData[LINEAR_AGAIN_MAG2].bUpdate = CVI_TRUE;
				pstCfg0->snsCfg.astI2cData[LINEAR_AGAIN_MAG3].bUpdate = CVI_TRUE;
				pstCfg0->snsCfg.astI2cData[LINEAR_AGAIN_MAG4].bUpdate = CVI_TRUE;
				pstCfg0->snsCfg.astI2cData[LINEAR_COL_AGAIN_H].bUpdate = CVI_TRUE;
				pstCfg0->snsCfg.astI2cData[LINEAR_COL_AGAIN_L].bUpdate = CVI_TRUE;
				pstCfg0->snsCfg.astI2cData[LINEAR_DGAIN_H].bUpdate = CVI_TRUE;
				pstCfg0->snsCfg.astI2cData[LINEAR_DGAIN_L].bUpdate = CVI_TRUE;
			}
			if (shutterUpdate) {
				pstCfg0->snsCfg.astI2cData[LINEAR_EXP_H].bUpdate = CVI_TRUE;
				pstCfg0->snsCfg.astI2cData[LINEAR_EXP_L].bUpdate = CVI_TRUE;
			}
			if (vtsUpdate) {
				pstCfg0->snsCfg.astI2cData[LINEAR_VTS_H].bUpdate = CVI_TRUE;
				pstCfg0->snsCfg.astI2cData[LINEAR_VTS_L].bUpdate = CVI_TRUE;
			}
		}

		/* check update isp crop or not */
		pstCfg0->ispCfg.need_update = (sensor_cmp_wdr_size(&pstCfg0->ispCfg, &pstCfg1->ispCfg) ?
				CVI_TRUE : CVI_FALSE);
	}

	pstSnsRegsInfo->bConfig = CVI_FALSE;
	memcpy(pstSnsSyncInfo, &pstSnsState->astSyncInfo[0], sizeof(ISP_SNS_SYNC_INFO_S));
	memcpy(&pstSnsState->astSyncInfo[1], &pstSnsState->astSyncInfo[0], sizeof(ISP_SNS_SYNC_INFO_S));
	pstSnsState->au32FL[1] = pstSnsState->au32FL[0];

	if (pstSnsState->enWDRMode == WDR_MODE_2To1_LINE)
		pstCfg0->snsCfg.astI2cData[WDR2_FLIP_MIRROR].bDropFrm = CVI_FALSE;
	else
		pstCfg0->snsCfg.astI2cData[LINEAR_FLIP_MIRROR].bDropFrm = CVI_FALSE;

	return CVI_SUCCESS;
}

static CVI_S32 cmos_set_image_mode(VI_PIPE ViPipe, ISP_CMOS_SENSOR_IMAGE_MODE_S *pstSensorImageMode)
{
	CVI_U8 u8SensorImageMode = 0;
	ISP_SNS_STATE_S *pstSnsState = CVI_NULL;

	CMOS_CHECK_POINTER(pstSensorImageMode);
	GC4683_SENSOR_GET_CTX(ViPipe, pstSnsState);
	CMOS_CHECK_POINTER(pstSnsState);

	u8SensorImageMode = pstSnsState->u8ImgMode;
	pstSnsState->bSyncInit = CVI_FALSE;

	if (pstSensorImageMode->f32Fps <= 60) {
		if (pstSnsState->enWDRMode == WDR_MODE_NONE) {
			if (GC4683_RES_IS_1080P(pstSensorImageMode->u16Width, pstSensorImageMode->u16Height)) {
				u8SensorImageMode = GC4683_MODE_1080P60;
			} else if (GC4683_RES_IS_1440P(pstSensorImageMode->u16Width, pstSensorImageMode->u16Height)) {
				u8SensorImageMode = GC4683_MODE_2560X1440P60;
			} else {
				CVI_TRACE_SNS(CVI_DBG_ERR, "Not support! Width:%d, Height:%d, Fps:%f, WDRMode:%d\n",
					      pstSensorImageMode->u16Width,
					      pstSensorImageMode->u16Height,
					      pstSensorImageMode->f32Fps,
					      pstSnsState->enWDRMode);
				return CVI_FAILURE;
			}
		} else if (pstSnsState->enWDRMode == WDR_MODE_2To1_LINE) {
			if (GC4683_RES_IS_1440P(pstSensorImageMode->u16Width, pstSensorImageMode->u16Height)) {
				u8SensorImageMode = GC4683_MODE_2560X1440P30_WDR;
			} else {
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
	} else {
		if (pstSnsState->enWDRMode == WDR_MODE_NONE) {
			if (GC4683_RES_IS_720P(pstSensorImageMode->u16Width, pstSensorImageMode->u16Height)) {
				u8SensorImageMode = GC4683_MODE_1280X720P120;
			} else {
				CVI_TRACE_SNS(CVI_DBG_ERR, "Not support this Fps:%f\n", pstSensorImageMode->f32Fps);
				return CVI_FAILURE;
			}
		} else {
			CVI_TRACE_SNS(CVI_DBG_ERR, "Not support this Fps:%f\n", pstSensorImageMode->f32Fps);
			return CVI_FAILURE;
		}

	}

	if ((pstSnsState->bInit == CVI_TRUE) && (u8SensorImageMode == pstSnsState->u8ImgMode)) {
		/* Don't need to switch SensorImageMode */
		return CVI_FAILURE;
	}

	pstSnsState->u8ImgMode = u8SensorImageMode;

	return CVI_SUCCESS;
}

static CVI_VOID sensor_mirror_flip(VI_PIPE ViPipe, ISP_SNS_MIRRORFLIP_TYPE_E eSnsMirrorFlip)
{
	ISP_SNS_STATE_S *pstSnsState = CVI_NULL;
	ISP_SNS_REGS_INFO_S *pstSnsRegsInfo = CVI_NULL;
	CVI_U8 value;

	GC4683_SENSOR_GET_CTX(ViPipe, pstSnsState);
	CMOS_CHECK_POINTER_VOID(pstSnsState);

	pstSnsRegsInfo = &pstSnsState->astSyncInfo[0].snsCfg;

	/* Apply the setting on the fly  */
	if (pstSnsState->bInit == CVI_TRUE && g_aeGc4683_MirrorFip[ViPipe] != eSnsMirrorFlip) {
		switch (eSnsMirrorFlip) {
		case ISP_SNS_NORMAL:
			value = 0;
			break;
		case ISP_SNS_MIRROR:
			value = 1;
			break;
		case ISP_SNS_FLIP:
			value = 2;
			break;
		case ISP_SNS_MIRROR_FLIP:
			value = 3;
			break;
		default:
			return;
		}

		if (pstSnsState->enWDRMode == WDR_MODE_2To1_LINE) {
			pstSnsRegsInfo->astI2cData[WDR2_FLIP_MIRROR].u32Data = value;
			pstSnsRegsInfo->astI2cData[WDR2_FLIP_MIRROR].bDropFrm = 1;
			pstSnsRegsInfo->astI2cData[WDR2_FLIP_MIRROR].u8DropFrmNum = 2;
		} else {
			pstSnsRegsInfo->astI2cData[LINEAR_FLIP_MIRROR].u32Data = value;
			pstSnsRegsInfo->astI2cData[LINEAR_FLIP_MIRROR].bDropFrm = 1;
			pstSnsRegsInfo->astI2cData[LINEAR_FLIP_MIRROR].u8DropFrmNum = 2;
		}
		g_aeGc4683_MirrorFip[ViPipe] = eSnsMirrorFlip;
	}
}

static CVI_VOID sensor_global_init(VI_PIPE ViPipe)
{
	ISP_SNS_STATE_S *pstSnsState = CVI_NULL;

	GC4683_SENSOR_GET_CTX(ViPipe, pstSnsState);
	CMOS_CHECK_POINTER_VOID(pstSnsState);

	pstSnsState->bInit = CVI_FALSE;
	pstSnsState->bSyncInit = CVI_FALSE;
	pstSnsState->u8ImgMode = GC4683_MODE_2560X1440P60;
	pstSnsState->enWDRMode = WDR_MODE_NONE;
	pstSnsState->u32FLStd  = g_astGc4683_mode[pstSnsState->u8ImgMode].u32VtsDef;
	pstSnsState->au32FL[0] = g_astGc4683_mode[pstSnsState->u8ImgMode].u32VtsDef;
	pstSnsState->au32FL[1] = g_astGc4683_mode[pstSnsState->u8ImgMode].u32VtsDef;

	memset(&pstSnsState->astSyncInfo[0], 0, sizeof(ISP_SNS_SYNC_INFO_S));
	memset(&pstSnsState->astSyncInfo[1], 0, sizeof(ISP_SNS_SYNC_INFO_S));
}

static CVI_S32 sensor_rx_attr(VI_PIPE ViPipe, SNS_COMBO_DEV_ATTR_S *pstRxAttr)
{
	ISP_SNS_STATE_S *pstSnsState = CVI_NULL;
	SNS_COMBO_DEV_ATTR_S *pstRxAttrSrc = CVI_NULL;

	GC4683_SENSOR_GET_CTX(ViPipe, pstSnsState);
	GC4683_SENSOR_GET_COMBO(ViPipe, pstRxAttrSrc);

	CMOS_CHECK_POINTER(pstSnsState);
	CMOS_CHECK_POINTER(pstRxAttr);
	CMOS_CHECK_POINTER(pstRxAttrSrc);

	memcpy(pstRxAttr, pstRxAttrSrc, sizeof(*pstRxAttr));

	pstRxAttr->img_size.start_x = g_astGc4683_mode[pstSnsState->u8ImgMode].astImg[0].stWndRect.s32X;
	pstRxAttr->img_size.start_y = g_astGc4683_mode[pstSnsState->u8ImgMode].astImg[0].stWndRect.s32Y;
	pstRxAttr->img_size.active_w = g_astGc4683_mode[pstSnsState->u8ImgMode].astImg[0].stWndRect.u32Width;
	pstRxAttr->img_size.active_h = g_astGc4683_mode[pstSnsState->u8ImgMode].astImg[0].stWndRect.u32Height;
	pstRxAttr->img_size.width = g_astGc4683_mode[pstSnsState->u8ImgMode].astImg[0].stSnsSize.u32Width;
	pstRxAttr->img_size.height = g_astGc4683_mode[pstSnsState->u8ImgMode].astImg[0].stSnsSize.u32Height;
	pstRxAttr->img_size.max_width = g_astGc4683_mode[pstSnsState->u8ImgMode].astImg[0].stMaxSize.u32Width;
	pstRxAttr->img_size.max_height = g_astGc4683_mode[pstSnsState->u8ImgMode].astImg[0].stMaxSize.u32Height;

	if (pstSnsState->enWDRMode == WDR_MODE_NONE)
		pstRxAttr->mipi_attr.wdr_mode = CVI_MIPI_WDR_MODE_NONE;
	else if (pstSnsState->enWDRMode == WDR_MODE_2To1_LINE)
		pstRxAttr->mipi_attr.wdr_mode = CVI_MIPI_WDR_MODE_VC;

	pstRxAttrSrc = CVI_NULL;
	return CVI_SUCCESS;

}

static CVI_S32 sensor_patch_rx_attr(VI_PIPE ViPipe, RX_INIT_ATTR_S *pstRxInitAttr)
{
	int i;
	SNS_COMBO_DEV_ATTR_S *pstRxAttr = CVI_NULL;

	if (!g_pastGc4683ComboDevArray[ViPipe]) {
		pstRxAttr = malloc(sizeof(SNS_COMBO_DEV_ATTR_S));
	} else {
		GC4683_SENSOR_GET_COMBO(ViPipe, pstRxAttr);
	}
	memcpy(pstRxAttr, &gc4683_rx_attr, sizeof(SNS_COMBO_DEV_ATTR_S));
	GC4683_SENSOR_SET_COMBO(ViPipe, pstRxAttr);

	CMOS_CHECK_POINTER(pstRxInitAttr);

	if (pstRxInitAttr->stMclkAttr.bMclkEn)
		pstRxAttr->mclk.cam = pstRxInitAttr->stMclkAttr.u8Mclk;

	if (pstRxInitAttr->MipiDev >= VI_MAX_DEV_NUM)
		return CVI_SUCCESS;

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

void gc4683_exit(VI_PIPE ViPipe)
{
	if (g_pastGc4683ComboDevArray[ViPipe]) {
		free(g_pastGc4683ComboDevArray[ViPipe]);
		g_pastGc4683ComboDevArray[ViPipe] = CVI_NULL;
	}
	gc4683_i2c_exit(ViPipe);
}

static CVI_S32 cmos_init_sensor_exp_function(ISP_SENSOR_EXP_FUNC_S *pstSensorExpFunc)
{
	CMOS_CHECK_POINTER(pstSensorExpFunc);

	memset(pstSensorExpFunc, 0, sizeof(ISP_SENSOR_EXP_FUNC_S));

	pstSensorExpFunc->pfn_cmos_sensor_init = gc4683_init;
	pstSensorExpFunc->pfn_cmos_sensor_exit = gc4683_exit;
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
	if (GC4683_I2C_ADDR_IS_VALID(s32I2cAddr))
		g_aunGc4683_AddrInfo[ViPipe].s8I2cAddr = s32I2cAddr;
	else {
		CVI_TRACE_SNS(CVI_DBG_ERR, "I2C addr input error ,please check [0x%x]\n", s32I2cAddr);
		g_aunGc4683_AddrInfo[ViPipe].s8I2cAddr = GC4683_I2C_ADDR_2;
	}
}

static CVI_S32 gc4683_set_bus_info(VI_PIPE ViPipe, ISP_SNS_COMMBUS_U unSNSBusInfo)
{
	g_aunGc4683_BusInfo[ViPipe].s8I2cDev = unSNSBusInfo.s8I2cDev;

	return CVI_SUCCESS;
}

static CVI_S32 sensor_ctx_init(VI_PIPE ViPipe)
{
	ISP_SNS_STATE_S *pastSnsStateCtx = CVI_NULL;

	GC4683_SENSOR_GET_CTX(ViPipe, pastSnsStateCtx);

	if (pastSnsStateCtx == CVI_NULL) {
		pastSnsStateCtx = (ISP_SNS_STATE_S *)malloc(sizeof(ISP_SNS_STATE_S));
		if (pastSnsStateCtx == CVI_NULL) {
			CVI_TRACE_SNS(CVI_DBG_ERR, "Isp[%d] SnsCtx malloc memory failed!\n", ViPipe);
			return -ENOMEM;
		}
	}

	memset(pastSnsStateCtx, 0, sizeof(ISP_SNS_STATE_S));

	GC4683_SENSOR_SET_CTX(ViPipe, pastSnsStateCtx);

	return CVI_SUCCESS;
}

static CVI_VOID sensor_ctx_exit(VI_PIPE ViPipe)
{
	ISP_SNS_STATE_S *pastSnsStateCtx = CVI_NULL;

	GC4683_SENSOR_GET_CTX(ViPipe, pastSnsStateCtx);
	SENSOR_FREE(pastSnsStateCtx);
	GC4683_SENSOR_RESET_CTX(ViPipe);
	g_aeGc4683_MirrorFip[ViPipe] = ISP_SNS_NORMAL;
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

	stSnsAttrInfo.eSensorId = GC4683_ID;

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

	s32Ret = CVI_ISP_SensorUnRegCallBack(ViPipe, GC4683_ID);
	if (s32Ret != CVI_SUCCESS) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "sensor unregister callback function failed!\n");
		return s32Ret;
	}

	s32Ret = CVI_AE_SensorUnRegCallBack(ViPipe, pstAeLib, GC4683_ID);
	if (s32Ret != CVI_SUCCESS) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "sensor unregister callback function to ae lib failed!\n");
		return s32Ret;
	}

	s32Ret = CVI_AWB_SensorUnRegCallBack(ViPipe, pstAwbLib, GC4683_ID);
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
	g_au16Gc4683_GainMode[ViPipe] = pstInitAttr->enGainMode;

	return CVI_SUCCESS;
}

static CVI_S32 sensor_probe(VI_PIPE ViPipe)
{
	return gc4683_probe(ViPipe);
}

ISP_SNS_OBJ_S stSnsGc4683_Obj = {
	.pfnRegisterCallback    = sensor_register_callback,
	.pfnUnRegisterCallback  = sensor_unregister_callback,
	.pfnStandby             = gc4683_standby,
	.pfnRestart             = gc4683_restart,
	.pfnWriteReg            = gc4683_write_register,
	.pfnReadReg             = gc4683_read_register,
	.pfnSetBusInfo          = gc4683_set_bus_info,
	.pfnSetInit             = sensor_set_init,
	.pfnMirrorFlip          = sensor_mirror_flip,
	.pfnPatchRxAttr         = sensor_patch_rx_attr,
	.pfnPatchI2cAddr        = sensor_patch_i2c_addr,
	.pfnGetRxAttr           = sensor_rx_attr,
	.pfnExpSensorCb         = cmos_init_sensor_exp_function,
	.pfnExpAeCb             = cmos_init_ae_exp_function,
	.pfnSnsProbe            = sensor_probe,
};

