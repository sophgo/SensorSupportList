#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <syslog.h>
#include <errno.h>

#include <linux/cvi_type.h>
#include <linux/cvi_comm_video.h>
#include "cvi_debug.h"
#include "cvi_comm_sns.h"
#include "cvi_sns_ctrl.h"
#include "cvi_ae_comm.h"
#include "cvi_awb_comm.h"
#include "cvi_ae.h"
#include "cvi_awb.h"
#include "cvi_isp.h"

#include "os08c10_cmos_ex.h"
#include "os08c10_cmos_param.h"

#define DIV_0_TO_1(a)   ((0 == (a)) ? 1 : (a))
#define DIV_0_TO_1_FLOAT(a) ((((a) < 1E-10) && ((a) > -1E-10)) ? 1 : (a))
#define OS08C10_ID 810
/****************************************************************************
 * global variables                                                            *
 ****************************************************************************/

ISP_SNS_STATE_S *g_pastOs08c10[VI_MAX_PIPE_NUM] = {CVI_NULL};

#define OS08C10_SENSOR_GET_CTX(dev, pstCtx)   (pstCtx = g_pastOs08c10[dev])
#define OS08C10_SENSOR_SET_CTX(dev, pstCtx)   (g_pastOs08c10[dev] = pstCtx)
#define OS08C10_SENSOR_RESET_CTX(dev)         (g_pastOs08c10[dev] = CVI_NULL)

ISP_SNS_COMMBUS_U g_aunOs08c10_BusInfo[VI_MAX_PIPE_NUM] = {
	[0] = { .s8I2cDev = 0},
	[1 ... VI_MAX_PIPE_NUM - 1] = { .s8I2cDev = -1}
};

CVI_U16 g_au16Os08c10_GainMode[VI_MAX_PIPE_NUM] = {0};
CVI_U16 g_au16Os08c10_L2SMode[VI_MAX_PIPE_NUM] = {0};
OS08C10_STATE_S g_astOs08c10_State[VI_MAX_PIPE_NUM] = {{0}};

/****************************************************************************
 * local variables and functions                                                           *
 ****************************************************************************/
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
/*****Os08c10 Lines Range*****/
#define OS08C10_FULL_LINES_MAX_2TO1_WDR  (0xFFFF)

/*****Os08c10 Register Address*****/
#define OS08C10_GROUP_ACCESS		0x3208
#define OS08C10_EXPL_ADDR		0x3501
#define OS08C10_EXPS_ADDR		0x3541
#define OS08C10_AGAINS_ADDR		0x3548
#define OS08C10_DGAINS_ADDR		0x354A
#define OS08C10_AGAINL_ADDR		0x3508
#define OS08C10_DGAINL_ADDR		0x350A
#define OS08C10_VTS_ADDR		0x380E
#define OS08C10_TABLE_END		0xffff

#define OS08C10_RES_IS_1080P(w, h)      ((w) <= 1920 && (h) <= 1080)
#define OS08C10_RES_IS_720P(w, h)       ((w) <= 1280 && (h) <= 720)

static CVI_S32 cmos_get_ae_default(VI_PIPE ViPipe, AE_SENSOR_DEFAULT_S *pstAeSnsDft)
{
	const OS08C10_MODE_S *pstMode;
	ISP_SNS_STATE_S *pstSnsState = CVI_NULL;

	CMOS_CHECK_POINTER(pstAeSnsDft);
	OS08C10_SENSOR_GET_CTX(ViPipe, pstSnsState);
	CMOS_CHECK_POINTER(pstSnsState);

	pstMode = &g_astOs08c10_mode[pstSnsState->u8ImgMode];
#if 0
	memset(&pstAeSnsDft->stAERouteAttr, 0, sizeof(ISP_AE_ROUTE_S));
#endif
	pstAeSnsDft->u32FullLinesStd = pstSnsState->u32FLStd;
	pstAeSnsDft->u32FlickerFreq = 50 * 256;
	pstAeSnsDft->u32FullLinesMax = OS08C10_FULL_LINES_MAX_2TO1_WDR;
	pstAeSnsDft->u32HmaxTimes =
			(1000000) / (pstSnsState->u32FLStd * g_astOs08c10_mode[pstSnsState->u8ImgMode].f32MaxFps);

	pstAeSnsDft->stIntTimeAccu.enAccuType = AE_ACCURACY_LINEAR;
	pstAeSnsDft->stIntTimeAccu.f32Accuracy = 1;
	pstAeSnsDft->stIntTimeAccu.f32Offset = 0;

	pstAeSnsDft->stAgainAccu.enAccuType = AE_ACCURACY_TABLE;
	pstAeSnsDft->stAgainAccu.f32Accuracy = 1;

	pstAeSnsDft->stDgainAccu.enAccuType = AE_ACCURACY_TABLE;
	pstAeSnsDft->stDgainAccu.f32Accuracy = 1;

	pstAeSnsDft->u32ISPDgainShift = 8;
	pstAeSnsDft->u32MinISPDgainTarget = 1 << pstAeSnsDft->u32ISPDgainShift;
	pstAeSnsDft->u32MaxISPDgainTarget = 16 << pstAeSnsDft->u32ISPDgainShift;

	if (g_au32LinesPer500ms[ViPipe] == 0)
		pstAeSnsDft->u32LinesPer500ms =
				pstSnsState->u32FLStd * g_astOs08c10_mode[pstSnsState->u8ImgMode].f32MaxFps / 2;
	else
		pstAeSnsDft->u32LinesPer500ms = g_au32LinesPer500ms[ViPipe];
	pstAeSnsDft->u32SnsStableFrame = 0;

	/* WDR only mode */
	pstAeSnsDft->f32Fps = pstMode->f32MaxFps;
	pstAeSnsDft->f32MinFps = pstMode->f32MinFps;
	pstAeSnsDft->au8HistThresh[0] = 0xC;
	pstAeSnsDft->au8HistThresh[1] = 0x18;
	pstAeSnsDft->au8HistThresh[2] = 0x60;
	pstAeSnsDft->au8HistThresh[3] = 0x80;

	pstAeSnsDft->u32MaxIntTime = pstMode->stExp[0].u16Max;
	pstAeSnsDft->u32MinIntTime = pstMode->stExp[0].u16Min;
	pstAeSnsDft->u32MaxIntTimeTarget = 65535;
	pstAeSnsDft->u32MinIntTimeTarget = 1;

	pstAeSnsDft->u32MaxAgain = pstMode->stAgain[0].u16Max;
	pstAeSnsDft->u32MinAgain = pstMode->stAgain[0].u16Min;
	pstAeSnsDft->u32MaxAgainTarget = pstAeSnsDft->u32MaxAgain;
	pstAeSnsDft->u32MinAgainTarget = pstAeSnsDft->u32MinAgain;

	pstAeSnsDft->u32MaxDgain = pstMode->stDgain[0].u16Max;
	pstAeSnsDft->u32MinDgain = pstMode->stDgain[0].u16Min;
	pstAeSnsDft->u32MaxDgainTarget = pstAeSnsDft->u32MaxDgain;
	pstAeSnsDft->u32MinDgainTarget = pstAeSnsDft->u32MinDgain;

	pstAeSnsDft->u32InitExposure = g_au32InitExposure[ViPipe] ? g_au32InitExposure[ViPipe] : 52000;
	pstAeSnsDft->u32InitAESpeed = 64;
	pstAeSnsDft->u32InitAETolerance = 5;
	pstAeSnsDft->u32AEResponseFrame = 5;
	if (genFSWDRMode[ViPipe] == ISP_FSWDR_LONG_FRAME_MODE) {
		pstAeSnsDft->u8AeCompensation = 64;
		pstAeSnsDft->enAeExpMode = AE_EXP_HIGHLIGHT_PRIOR;
	} else {
		pstAeSnsDft->u8AeCompensation = 40;
		pstAeSnsDft->enAeExpMode = AE_EXP_LOWLIGHT_PRIOR;
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
	OS08C10_SENSOR_GET_CTX(ViPipe, pstSnsState);
	CMOS_CHECK_POINTER(pstSnsState);

	u32Vts = g_astOs08c10_mode[pstSnsState->u8ImgMode].u32VtsDef;
	pstSnsRegsInfo = &pstSnsState->astSyncInfo[0].snsCfg;
	f32MaxFps = g_astOs08c10_mode[pstSnsState->u8ImgMode].f32MaxFps;
	f32MinFps = g_astOs08c10_mode[pstSnsState->u8ImgMode].f32MinFps;

	if (pstSnsState->enWDRMode == WDR_MODE_NONE) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "Linear mode fps set not support!\n");
		return CVI_FAILURE;
	} else {
		if ((f32Fps <= f32MaxFps) && (f32Fps >= f32MinFps)) {
			u32VMAX = u32Vts * f32MaxFps / DIV_0_TO_1_FLOAT(f32Fps);
		} else {
			CVI_TRACE_SNS(CVI_DBG_ERR, "Unsupport Fps: %f\n", f32Fps);
			return CVI_FAILURE;
		}
		u32VMAX = (u32VMAX > OS08C10_FULL_LINES_MAX_2TO1_WDR) ? OS08C10_FULL_LINES_MAX_2TO1_WDR : u32VMAX;
		g_astOs08c10_State[ViPipe].u32Sexp_MAX = g_astOs08c10_mode[pstSnsState->u8ImgMode].u32Sexp_MAX;
		pstSnsRegsInfo->astI2cData[WDR_VTS_0].u32Data = (u32VMAX >> 8) & 0xFF;
		pstSnsRegsInfo->astI2cData[WDR_VTS_1].u32Data = u32VMAX & 0xFF;
	}

	pstSnsState->u32FLStd = u32VMAX;

	pstAeSnsDft->f32Fps = f32Fps;
	pstAeSnsDft->u32LinesPer500ms = pstSnsState->u32FLStd * f32Fps / 2;
	pstAeSnsDft->u32FullLinesStd = pstSnsState->u32FLStd;
	pstAeSnsDft->u32MaxIntTime = pstSnsState->u32FLStd - 8;
	pstSnsState->au32FL[0] = pstSnsState->u32FLStd;
	pstAeSnsDft->u32FullLines = pstSnsState->au32FL[0];
	pstAeSnsDft->u32HmaxTimes = (1000000) / (pstSnsState->u32FLStd * DIV_0_TO_1_FLOAT(f32Fps));

	return CVI_SUCCESS;
}

/* while isp notify ae to update sensor regs, ae call these funcs. */
static CVI_S32 cmos_inttime_update(VI_PIPE ViPipe, CVI_U32 *u32IntTime)
{
	ISP_SNS_STATE_S *pstSnsState = CVI_NULL;
	ISP_SNS_REGS_INFO_S *pstSnsRegsInfo = CVI_NULL;

	OS08C10_SENSOR_GET_CTX(ViPipe, pstSnsState);
	CMOS_CHECK_POINTER(pstSnsState);
	CMOS_CHECK_POINTER(u32IntTime);
	pstSnsRegsInfo = &pstSnsState->astSyncInfo[0].snsCfg;

	if (WDR_MODE_2To1_LINE == pstSnsState->enWDRMode) {
		/* short exposure */
		pstSnsState->au32WDRIntTime[0] = u32IntTime[0];
		/* long exposure */
		pstSnsState->au32WDRIntTime[1] = u32IntTime[1];
		/* Make sure short exposure <= long exposure */
		if (pstSnsState->au32WDRIntTime[0] > pstSnsState->au32WDRIntTime[1]) {
			pstSnsState->au32WDRIntTime[0] = pstSnsState->au32WDRIntTime[1];
			u32IntTime[0] = u32IntTime[1];
		}

		pstSnsRegsInfo->astI2cData[WDR_EXPS_0].u32Data = ((pstSnsState->au32WDRIntTime[0] & 0xFF00) >> 8);
		pstSnsRegsInfo->astI2cData[WDR_EXPS_1].u32Data = (pstSnsState->au32WDRIntTime[0] & 0xFF);
		pstSnsRegsInfo->astI2cData[WDR_EXPL_0].u32Data = ((pstSnsState->au32WDRIntTime[1] & 0xFF00) >> 8);
		pstSnsRegsInfo->astI2cData[WDR_EXPL_1].u32Data = (pstSnsState->au32WDRIntTime[1] & 0xFF);
		/* update isp */
		cmos_get_wdr_size(ViPipe, &pstSnsState->astSyncInfo[0].ispCfg);
	} else {
		CVI_TRACE_SNS(CVI_DBG_ERR, "Linear mode fps set not support!\n");
		return CVI_FAILURE;
	}

	return CVI_SUCCESS;

}


static CVI_U32 gain_table[64] = {
	1024, 1088, 1152, 1216, 1280, 1344, 1408, 1472, 1536, 1600, 1664, 1728, 1792,
	1856, 1920, 1984, 2048, 2176, 2304, 2432, 2560, 2688, 2816, 2944, 3072, 3200,
	3328, 3456, 3584, 3712, 3840, 3968, 4096, 4352, 4608, 4864, 5120, 5376, 5632,
	5888, 6144, 6400, 6656, 6912, 7168, 7424, 7680, 7936, 8192, 8704, 9216, 9728,
	10240, 10752, 11264, 11776, 12288, 12800, 13312, 13824, 14336, 14848, 15360,
	15872
};

static CVI_S32 cmos_again_calc_table(VI_PIPE ViPipe, CVI_U32 *pu32AgainLin, CVI_U32 *pu32AgainDb)
{
	int i;

	UNUSED(ViPipe);
	CMOS_CHECK_POINTER(pu32AgainLin);
	CMOS_CHECK_POINTER(pu32AgainDb);
	int total = ARRAY_SIZE(gain_table);

	if (*pu32AgainLin >= gain_table[total - 1]) {
		*pu32AgainDb = total - 1;
		*pu32AgainLin = gain_table[total - 1];
		return CVI_SUCCESS;
	}

	for (i = 0; i < total; i++) {
		if (*pu32AgainLin < gain_table[i]) {
			*pu32AgainDb = i - 1;
			break;
		}
	}
	*pu32AgainLin = gain_table[i - 1];
	return CVI_SUCCESS;
}

static CVI_S32 cmos_dgain_calc_table(VI_PIPE ViPipe, CVI_U32 *pu32DgainLin, CVI_U32 *pu32DgainDb)
{
	int i;

	UNUSED(ViPipe);
	CMOS_CHECK_POINTER(pu32DgainLin);
	CMOS_CHECK_POINTER(pu32DgainDb);
	int total = ARRAY_SIZE(gain_table);

	if (*pu32DgainLin >= gain_table[total - 1]) {
		*pu32DgainDb = total - 1;
		*pu32DgainLin = gain_table[total - 1];
		return CVI_SUCCESS;
	}

	for (i = 0; i < total; i++) {
		if (*pu32DgainLin < gain_table[i]) {
			*pu32DgainDb = i - 1;
			break;
		}
	}
	*pu32DgainLin = gain_table[i - 1];
	return CVI_SUCCESS;
}

static CVI_S32 cmos_gains_update(VI_PIPE ViPipe, CVI_U32 *pu32Again, CVI_U32 *pu32Dgain)
{
	ISP_SNS_STATE_S *pstSnsState = CVI_NULL;
	ISP_SNS_REGS_INFO_S *pstSnsRegsInfo = CVI_NULL;
	CVI_U32 u32AgainIdx;
	CVI_U32 u32DgainIdx;
	CVI_U32 u32AgainRegH;
	CVI_U32 u32AgainRegL;
	CVI_U32 u32DgainRegH;
	CVI_U32 u32DgainRegL;

	OS08C10_SENSOR_GET_CTX(ViPipe, pstSnsState);
	CMOS_CHECK_POINTER(pstSnsState);
	CMOS_CHECK_POINTER(pu32Again);
	CMOS_CHECK_POINTER(pu32Dgain);
	pstSnsRegsInfo = &pstSnsState->astSyncInfo[0].snsCfg;

	if (pstSnsState->enWDRMode == WDR_MODE_NONE) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "Linear mode fps set not support!\n");
		return CVI_FAILURE;
	} else {
		/*
		 * Again: 0x3508[4:0] = integer part, 0x3509[7:0] = fractional part (1/256)
		 * gain_table stores Again * 1024 (base1024 format)
		 * total_256 = gain_table[idx] * 256 / 1024 = gain_table[idx] / 4
		 * 0x3508[4:0] = (total_256 >> 8) & 0x1F
		 * 0x3509[7:0] = total_256 & 0xFF
		 */
		/* short frame again: 0x3548, 0x3549 */
		u32AgainIdx = pu32Again[0];
		u32AgainRegH = ((gain_table[u32AgainIdx] / 4) >> 8) & 0x1F;
		u32AgainRegL = (gain_table[u32AgainIdx] / 4) & 0xFF;
		pstSnsRegsInfo->astI2cData[WDR_AGAINS_0].u32Data = u32AgainRegH;
		pstSnsRegsInfo->astI2cData[WDR_AGAINS_1].u32Data = u32AgainRegL;

		/*
		 * Dgain: 0x350a[5:0] = integer part, 0x350b[7:0] = fractional part (1/256)
		 * gain_table stores Dgain * 1024 (base1024 format)
		 * total_256 = gain_table[idx] * 256 / 1024 = gain_table[idx] / 4
		 * 0x350a[5:0] = (total_256 >> 8) & 0x3F
		 * 0x350b[7:0] = total_256 & 0xFF
		 */
		/* short frame dgain: 0x354a, 0x354b */
		u32DgainIdx = pu32Dgain[0];
		u32DgainRegH = ((gain_table[u32DgainIdx] / 4) >> 8) & 0x3F;
		u32DgainRegL = (gain_table[u32DgainIdx] / 4) & 0xFF;
		pstSnsRegsInfo->astI2cData[WDR_DGAINS_0].u32Data = u32DgainRegH;
		pstSnsRegsInfo->astI2cData[WDR_DGAINS_1].u32Data = u32DgainRegL;

		/* long frame again: 0x3508, 0x3509 */
		u32AgainIdx = pu32Again[1];
		u32AgainRegH = ((gain_table[u32AgainIdx] / 4) >> 8) & 0x1F;
		u32AgainRegL = (gain_table[u32AgainIdx] / 4) & 0xFF;
		pstSnsRegsInfo->astI2cData[WDR_AGAINL_0].u32Data = u32AgainRegH;
		pstSnsRegsInfo->astI2cData[WDR_AGAINL_1].u32Data = u32AgainRegL;

		/* long frame dgain: 0x350a, 0x350b */
		u32DgainIdx = pu32Dgain[1];
		u32DgainRegH = ((gain_table[u32DgainIdx] / 4) >> 8) & 0x3F;
		u32DgainRegL = (gain_table[u32DgainIdx] / 4) & 0xFF;
		pstSnsRegsInfo->astI2cData[WDR_DGAINL_0].u32Data = u32DgainRegH;
		pstSnsRegsInfo->astI2cData[WDR_DGAINL_1].u32Data = u32DgainRegL;

	}

	return CVI_SUCCESS;
}

static CVI_S32 cmos_get_inttime_max(VI_PIPE ViPipe, CVI_U16 u16ManRatioEnable, CVI_U32 *au32Ratio,
		CVI_U32 *au32IntTimeMax, CVI_U32 *au32IntTimeMin, CVI_U32 *pu32LFMaxIntTime)
{
	CVI_U32 u32IntTimeMaxTmp  = 0, u32IntTimeMaxTmp0 = 0;
	CVI_U32 u32RatioTmp = 0x40;
	CVI_U32 u32ShortTimeMinLimit = 0;
	ISP_SNS_STATE_S *pstSnsState = CVI_NULL;

	CMOS_CHECK_POINTER(au32Ratio);
	CMOS_CHECK_POINTER(au32IntTimeMax);
	CMOS_CHECK_POINTER(au32IntTimeMin);
	CMOS_CHECK_POINTER(pu32LFMaxIntTime);
	OS08C10_SENSOR_GET_CTX(ViPipe, pstSnsState);
	CMOS_CHECK_POINTER(pstSnsState);

	u32ShortTimeMinLimit = 8;
	/*
	 * 8 < Long exp + Short exp < VTS - 16
	 */

	u32IntTimeMaxTmp0 = ((pstSnsState->au32FL[1] - 16 - pstSnsState->au32WDRIntTime[0]) * 0x40) /
						DIV_0_TO_1(au32Ratio[0]);
	u32IntTimeMaxTmp  = ((pstSnsState->au32FL[0] - 16) * 0x40)  / DIV_0_TO_1(au32Ratio[0] + 0x40);
	u32IntTimeMaxTmp = (u32IntTimeMaxTmp > u32IntTimeMaxTmp0) ? u32IntTimeMaxTmp0 : u32IntTimeMaxTmp;

	u32IntTimeMaxTmp = (g_astOs08c10_State[ViPipe].u32Sexp_MAX < u32IntTimeMaxTmp) ?
					g_astOs08c10_State[ViPipe].u32Sexp_MAX : u32IntTimeMaxTmp;
	u32IntTimeMaxTmp  = (!u32IntTimeMaxTmp) ? 8 : u32IntTimeMaxTmp;

	if (u32IntTimeMaxTmp >= u32ShortTimeMinLimit) {
		if (pstSnsState->enWDRMode == WDR_MODE_2To1_LINE) {
			au32IntTimeMax[0] = u32IntTimeMaxTmp;
			au32IntTimeMax[1] = au32IntTimeMax[0] * au32Ratio[0] >> 6;
			au32IntTimeMax[2] = au32IntTimeMax[1] * au32Ratio[1] >> 6;
			au32IntTimeMax[3] = au32IntTimeMax[2] * au32Ratio[2] >> 6;
			au32IntTimeMin[0] = u32ShortTimeMinLimit;
			au32IntTimeMin[1] = au32IntTimeMin[0] * au32Ratio[0] >> 6;
			au32IntTimeMin[2] = au32IntTimeMin[1] * au32Ratio[1] >> 6;
			au32IntTimeMin[3] = au32IntTimeMin[2] * au32Ratio[2] >> 6;
		} else {
		}
	} else {
		if (u16ManRatioEnable) {
			CVI_TRACE_SNS(CVI_DBG_ERR, "Manaul ExpRatio is too large!\n");
			return CVI_FAILURE;
		}
		u32IntTimeMaxTmp = u32ShortTimeMinLimit;

		if (pstSnsState->enWDRMode == WDR_MODE_2To1_LINE) {
			u32RatioTmp = 0xFFF;
			au32IntTimeMax[0] = u32IntTimeMaxTmp;
			au32IntTimeMax[1] = au32IntTimeMax[0] * u32RatioTmp >> 6;
		} else {
		}
		au32IntTimeMin[0] = au32IntTimeMax[0];
		au32IntTimeMin[1] = au32IntTimeMax[1];
		au32IntTimeMin[2] = au32IntTimeMax[2];
		au32IntTimeMin[3] = au32IntTimeMax[3];
	}
	CVI_TRACE_SNS(CVI_DBG_DEBUG, "sexp[%d, %d], lexp[%d, %d], ratio:%d\n",
			au32IntTimeMin[0], au32IntTimeMax[0], au32IntTimeMin[1], au32IntTimeMax[1], au32Ratio[0]);

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
	//pstExpFuncs->pfn_cmos_slow_framerate_set = cmos_slow_framerate_set;
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
	UNUSED(ViPipe);

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
	UNUSED(ViPipe);

	memset(pstDef, 0, sizeof(ISP_CMOS_DEFAULT_S));

	memcpy(pstDef->stNoiseCalibration.CalibrationCoef,
		&g_stIspNoiseCalibratio, sizeof(ISP_CMOS_NOISE_CALIBRATION_S));

	return CVI_SUCCESS;
}

static CVI_S32 cmos_get_blc_default(VI_PIPE ViPipe, ISP_CMOS_BLACK_LEVEL_S *pstBlc)
{
	UNUSED(ViPipe);

	CMOS_CHECK_POINTER(pstBlc);

	memset(pstBlc, 0, sizeof(ISP_CMOS_BLACK_LEVEL_S));

	memcpy(pstBlc,
		&g_stIspBlcCalibratio, sizeof(ISP_CMOS_BLACK_LEVEL_S));
	return CVI_SUCCESS;
}

static CVI_S32 cmos_get_wdr_size(VI_PIPE ViPipe, ISP_SNS_ISP_INFO_S *pstIspCfg)
{
	const OS08C10_MODE_S *pstMode = CVI_NULL;
	ISP_SNS_STATE_S *pstSnsState = CVI_NULL;

	OS08C10_SENSOR_GET_CTX(ViPipe, pstSnsState);
	CMOS_CHECK_POINTER(pstSnsState);
	pstMode = &g_astOs08c10_mode[pstSnsState->u8ImgMode];

	pstIspCfg->frm_num = 2;
	memcpy(&pstIspCfg->img_size[0], &pstMode->astImg[0], sizeof(ISP_WDR_SIZE_S));
	memcpy(&pstIspCfg->img_size[1], &pstMode->astImg[1], sizeof(ISP_WDR_SIZE_S));

	return CVI_SUCCESS;
}

static CVI_S32 cmos_set_wdr_mode(VI_PIPE ViPipe, CVI_U8 u8Mode)
{
	ISP_SNS_STATE_S *pstSnsState = CVI_NULL;

	OS08C10_SENSOR_GET_CTX(ViPipe, pstSnsState);
	CMOS_CHECK_POINTER(pstSnsState);

	pstSnsState->bSyncInit = CVI_FALSE;

	/* WDR only: accept WDR_MODE_2To1_LINE only */
	if (u8Mode != WDR_MODE_2To1_LINE) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "Unsupport sensor mode! Only WDR_MODE_2To1_LINE supported\n");
		return CVI_FAILURE;
	}
	pstSnsState->enWDRMode = WDR_MODE_2To1_LINE;
	pstSnsState->u32FLStd = g_astOs08c10_mode[pstSnsState->u8ImgMode].u32VtsDef;
	syslog(LOG_INFO, "WDR_MODE_2To1_LINE\n");

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
	OS08C10_SENSOR_GET_CTX(ViPipe, pstSnsState);
	CMOS_CHECK_POINTER(pstSnsState);
	pstSnsRegsInfo = &pstSnsSyncInfo->snsCfg;
	pstCfg0 = &pstSnsState->astSyncInfo[0];
	pstCfg1 = &pstSnsState->astSyncInfo[1];
	pstI2c_data = pstCfg0->snsCfg.astI2cData;

	if ((pstSnsState->bSyncInit == CVI_FALSE) || (pstSnsRegsInfo->bConfig == CVI_FALSE)) {
		pstCfg0->snsCfg.enSnsType = SNS_I2C_TYPE;
		pstCfg0->snsCfg.unComBus.s8I2cDev = g_aunOs08c10_BusInfo[ViPipe].s8I2cDev;
		pstCfg0->snsCfg.u8Cfg2ValidDelayMax = 0;
		pstCfg0->snsCfg.use_snsr_sram = CVI_TRUE;
		/* WDR only: only support WDR mode */
		pstCfg0->snsCfg.u32RegNum = WDR_REGS_NUM;

		for (i = 0; i < pstCfg0->snsCfg.u32RegNum; i++) {
			pstI2c_data[i].bUpdate = CVI_TRUE;
			pstI2c_data[i].u8DevAddr = os08c10_i2c_addr;
			pstI2c_data[i].u32AddrByteNum = os08c10_addr_byte;
			pstI2c_data[i].u32DataByteNum = os08c10_data_byte;
			pstI2c_data[i].bvblankUpdate = CVI_FALSE;
		}

		/* WDR only: only support WDR mode */
		pstI2c_data[WDR_HOLD_START].u32RegAddr = OS08C10_GROUP_ACCESS;
		pstI2c_data[WDR_HOLD_START].u32Data = 0x00;
		pstI2c_data[WDR_EXPS_0].u32RegAddr = OS08C10_EXPS_ADDR;
		pstI2c_data[WDR_EXPS_1].u32RegAddr = OS08C10_EXPS_ADDR + 1;
		pstI2c_data[WDR_EXPL_0].u32RegAddr = OS08C10_EXPL_ADDR;
		pstI2c_data[WDR_EXPL_1].u32RegAddr = OS08C10_EXPL_ADDR + 1;
		pstI2c_data[WDR_AGAINS_0].u32RegAddr = OS08C10_AGAINS_ADDR;
		pstI2c_data[WDR_AGAINS_1].u32RegAddr = OS08C10_AGAINS_ADDR + 1;
		pstI2c_data[WDR_AGAINL_0].u32RegAddr = OS08C10_AGAINL_ADDR;
		pstI2c_data[WDR_AGAINL_1].u32RegAddr = OS08C10_AGAINL_ADDR + 1;
		pstI2c_data[WDR_DGAINS_0].u32RegAddr = OS08C10_DGAINS_ADDR;
		pstI2c_data[WDR_DGAINS_1].u32RegAddr = OS08C10_DGAINS_ADDR + 1;
		pstI2c_data[WDR_DGAINL_0].u32RegAddr = OS08C10_DGAINL_ADDR;
		pstI2c_data[WDR_DGAINL_1].u32RegAddr = OS08C10_DGAINL_ADDR + 1;

		pstI2c_data[WDR_VTS_0].u32RegAddr = OS08C10_VTS_ADDR;
		pstI2c_data[WDR_VTS_1].u32RegAddr = OS08C10_VTS_ADDR + 1;
		pstI2c_data[WDR_HOLD_END].u32RegAddr = OS08C10_GROUP_ACCESS;
		pstI2c_data[WDR_HOLD_END].u32Data = 0x10;
		pstI2c_data[WDR_LAUNCH_1].u32RegAddr = OS08C10_GROUP_ACCESS;
		pstI2c_data[WDR_LAUNCH_1].u32Data = 0xA0;

		pstSnsState->bSyncInit = CVI_TRUE;
		pstCfg0->snsCfg.need_update = CVI_TRUE;
		/* recalculate WDR size */
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
			pstI2c_data[WDR_HOLD_START].bUpdate = CVI_TRUE;
			pstI2c_data[WDR_HOLD_END].bUpdate = CVI_TRUE;
			pstI2c_data[WDR_LAUNCH_1].bUpdate = CVI_TRUE;
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
	OS08C10_SENSOR_GET_CTX(ViPipe, pstSnsState);
	CMOS_CHECK_POINTER(pstSnsState);

	u8SensorImageMode = pstSnsState->u8ImgMode;
	pstSnsState->bSyncInit = CVI_FALSE;

	/* WDR only: support 720P120 and 1080P60 WDR mode */
	if (OS08C10_RES_IS_720P(pstSensorImageMode->u16Width, pstSensorImageMode->u16Height))
		u8SensorImageMode = OS08C10_MODE_1280X720P120_WDR;
	else if (OS08C10_RES_IS_1080P(pstSensorImageMode->u16Width, pstSensorImageMode->u16Height))
		u8SensorImageMode = OS08C10_MODE_1920X1080P60_WDR;
	else {
		CVI_TRACE_SNS(CVI_DBG_ERR, "Not support! Width:%d, Height:%d, Fps:%f, WDRMode:%d\n",
		       pstSensorImageMode->u16Width,
		       pstSensorImageMode->u16Height,
		       pstSensorImageMode->f32Fps,
		       pstSnsState->enWDRMode);
		return CVI_FAILURE;
	}

	if ((pstSnsState->bInit == CVI_TRUE) && (u8SensorImageMode == pstSnsState->u8ImgMode)) {
		/* Don't need to switch SensorImageMode */
		return CVI_FAILURE;
	}

	pstSnsState->u8ImgMode = u8SensorImageMode;

	return CVI_SUCCESS;
}

static CVI_VOID sensor_global_init(VI_PIPE ViPipe)
{
	ISP_SNS_STATE_S *pstSnsState = CVI_NULL;

	OS08C10_SENSOR_GET_CTX(ViPipe, pstSnsState);
	CMOS_CHECK_POINTER_VOID(pstSnsState);

	pstSnsState->bInit = CVI_FALSE;
	pstSnsState->bSyncInit = CVI_FALSE;
	pstSnsState->u8ImgMode = OS08C10_MODE_1920X1080P60_WDR;
	pstSnsState->enWDRMode = WDR_MODE_2To1_LINE;
	pstSnsState->u32FLStd  = g_astOs08c10_mode[pstSnsState->u8ImgMode].u32VtsDef;
	pstSnsState->au32FL[0] = g_astOs08c10_mode[pstSnsState->u8ImgMode].u32VtsDef;
	pstSnsState->au32FL[1] = g_astOs08c10_mode[pstSnsState->u8ImgMode].u32VtsDef;

	memset(&pstSnsState->astSyncInfo[0], 0, sizeof(ISP_SNS_SYNC_INFO_S));
	memset(&pstSnsState->astSyncInfo[1], 0, sizeof(ISP_SNS_SYNC_INFO_S));
}

static CVI_S32 sensor_rx_attr(VI_PIPE ViPipe, SNS_COMBO_DEV_ATTR_S *pstRxAttr)
{
	ISP_SNS_STATE_S *pstSnsState = CVI_NULL;

	OS08C10_SENSOR_GET_CTX(ViPipe, pstSnsState);
	CMOS_CHECK_POINTER(pstSnsState);
	CMOS_CHECK_POINTER(pstRxAttr);

	memcpy(pstRxAttr, &os08c10_rx_attr, sizeof(*pstRxAttr));

	pstRxAttr->img_size.width = g_astOs08c10_mode[pstSnsState->u8ImgMode].astImg[0].stSnsSize.u32Width;
	pstRxAttr->img_size.height = g_astOs08c10_mode[pstSnsState->u8ImgMode].astImg[0].stSnsSize.u32Height;
	/* WDR only: always use VC WDR mode */
	pstRxAttr->mipi_attr.wdr_mode = CVI_MIPI_WDR_MODE_VC;

	return CVI_SUCCESS;

}

static CVI_S32 sensor_patch_rx_attr(RX_INIT_ATTR_S *pstRxInitAttr)
{
	SNS_COMBO_DEV_ATTR_S *pstRxAttr = &os08c10_rx_attr;
	int i;

	CMOS_CHECK_POINTER(pstRxInitAttr);

	if (pstRxInitAttr->stMclkAttr.bMclkEn)
		pstRxAttr->mclk.cam = pstRxInitAttr->stMclkAttr.u8Mclk;

	if (pstRxInitAttr->MipiDev >= VI_MAX_DEV_NUM)
		return CVI_SUCCESS;

	pstRxAttr->devno = pstRxInitAttr->MipiDev;

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

	return CVI_SUCCESS;
}

static CVI_S32 cmos_init_sensor_exp_function(ISP_SENSOR_EXP_FUNC_S *pstSensorExpFunc)
{
	CMOS_CHECK_POINTER(pstSensorExpFunc);

	memset(pstSensorExpFunc, 0, sizeof(ISP_SENSOR_EXP_FUNC_S));

	pstSensorExpFunc->pfn_cmos_sensor_init = os08c10_init;
	pstSensorExpFunc->pfn_cmos_sensor_exit = os08c10_exit;
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

static CVI_S32 os08c10_set_bus_info(VI_PIPE ViPipe, ISP_SNS_COMMBUS_U unSNSBusInfo)
{
	g_aunOs08c10_BusInfo[ViPipe].s8I2cDev = unSNSBusInfo.s8I2cDev;

	return CVI_SUCCESS;
}

static CVI_S32 sensor_ctx_init(VI_PIPE ViPipe)
{
	ISP_SNS_STATE_S *pastSnsStateCtx = CVI_NULL;

	OS08C10_SENSOR_GET_CTX(ViPipe, pastSnsStateCtx);

	if (pastSnsStateCtx == CVI_NULL) {
		pastSnsStateCtx = (ISP_SNS_STATE_S *)malloc(sizeof(ISP_SNS_STATE_S));
		if (pastSnsStateCtx == CVI_NULL) {
			CVI_TRACE_SNS(CVI_DBG_ERR, "Isp[%d] SnsCtx malloc memory failed!\n", ViPipe);
			return -ENOMEM;
		}
	}

	memset(pastSnsStateCtx, 0, sizeof(ISP_SNS_STATE_S));

	OS08C10_SENSOR_SET_CTX(ViPipe, pastSnsStateCtx);

	return CVI_SUCCESS;
}

static CVI_VOID sensor_ctx_exit(VI_PIPE ViPipe)
{
	ISP_SNS_STATE_S *pastSnsStateCtx = CVI_NULL;

	OS08C10_SENSOR_GET_CTX(ViPipe, pastSnsStateCtx);
	SENSOR_FREE(pastSnsStateCtx);
	OS08C10_SENSOR_RESET_CTX(ViPipe);
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

	stSnsAttrInfo.eSensorId = OS08C10_ID;

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

	s32Ret = CVI_ISP_SensorUnRegCallBack(ViPipe, OS08C10_ID);
	if (s32Ret != CVI_SUCCESS) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "sensor unregister callback function failed!\n");
		return s32Ret;
	}

	s32Ret = CVI_AE_SensorUnRegCallBack(ViPipe, pstAeLib, OS08C10_ID);
	if (s32Ret != CVI_SUCCESS) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "sensor unregister callback function to ae lib failed!\n");
		return s32Ret;
	}

	s32Ret = CVI_AWB_SensorUnRegCallBack(ViPipe, pstAwbLib, OS08C10_ID);
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
	g_au16Os08c10_GainMode[ViPipe] = pstInitAttr->enGainMode;
	g_au16Os08c10_L2SMode[ViPipe] = pstInitAttr->enL2SMode;

	return CVI_SUCCESS;
}

static CVI_VOID sensor_patch_i2c_addr(CVI_S32 s32I2cAddr)
{
	if (OS08C10_I2C_ADDR_IS_VALID(s32I2cAddr))
		os08c10_i2c_addr = s32I2cAddr;
}

ISP_SNS_OBJ_S stSnsOs08c10_Obj = {
	.pfnRegisterCallback    = sensor_register_callback,
	.pfnUnRegisterCallback  = sensor_unregister_callback,
	.pfnStandby             = os08c10_standby,
	.pfnRestart             = os08c10_restart,
	.pfnMirrorFlip          = CVI_NULL,
	.pfnWriteReg            = os08c10_write_register,
	.pfnReadReg             = os08c10_read_register,
	.pfnSetBusInfo          = os08c10_set_bus_info,
	.pfnSetInit             = sensor_set_init,
	.pfnPatchRxAttr		= sensor_patch_rx_attr,
	.pfnPatchI2cAddr	= sensor_patch_i2c_addr,
	.pfnGetRxAttr		= sensor_rx_attr,
	.pfnExpSensorCb		= cmos_init_sensor_exp_function,
	.pfnExpAeCb		= cmos_init_ae_exp_function,
	.pfnSnsProbe		= os08c10_probe,
};