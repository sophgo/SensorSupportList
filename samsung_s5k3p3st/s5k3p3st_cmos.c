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

#include "s5k3p3st_cmos_ex.h"
#include "s5k3p3st_cmos_param.h"

#define DIV_0_TO_1(a)   ((0 == (a)) ? 1 : (a))
#define DIV_0_TO_1_FLOAT(a) ((((a) < 1E-10) && ((a) > -1E-10)) ? 1 : (a))
#define S5K3P3ST_ID 533
#define S5K3P3ST_I2C_ADDR_1 0x10
#define S5K3P3ST_I2C_ADDR_2 0x12
#define S5K3P3ST_I2C_ADDR_IS_VALID(addr) ((addr) == S5K3P3ST_I2C_ADDR_1 || (addr) == S5K3P3ST_I2C_ADDR_2)

#define SENSOR_S5K3P3ST_WIDTH 2304
#define SENSOR_S5K3P3ST_HEIGHT 1296

#define S5K3P3ST_EXPACCURACY                    (1)

/****************************************************************************
 * global variables                                                            *
 ****************************************************************************/

ISP_SNS_STATE_S *g_pastS5K3P3ST[VI_MAX_PIPE_NUM] = {CVI_NULL};
SNS_COMBO_DEV_ATTR_S* g_pastS5K3P3STComboDevArray[VI_MAX_PIPE_NUM] = {CVI_NULL};

#define S5K3P3ST_SENSOR_GET_CTX(dev, pstCtx)   (pstCtx = g_pastS5K3P3ST[dev])
#define S5K3P3ST_SENSOR_SET_CTX(dev, pstCtx)   (g_pastS5K3P3ST[dev] = pstCtx)
#define S5K3P3ST_SENSOR_RESET_CTX(dev)         (g_pastS5K3P3ST[dev] = CVI_NULL)
#define S5K3P3ST_SENSOR_GET_COMBO(dev, pstCtx)   (pstCtx = g_pastS5K3P3STComboDevArray[dev])
#define S5K3P3ST_SENSOR_SET_COMBO(dev, pstCtx)   (g_pastS5K3P3STComboDevArray[dev] = pstCtx)

ISP_SNS_COMMBUS_U g_aunS5K3P3ST_BusInfo[VI_MAX_PIPE_NUM] = {
	[0] = { .s8I2cDev = 0},
	[1 ... VI_MAX_PIPE_NUM - 1] = { .s8I2cDev = -1}
};

ISP_SNS_COMMADDR_U g_aunS5K3P3ST_AddrInfo[VI_MAX_PIPE_NUM] = {
	[0] = { .s8I2cAddr = 0},
	[1 ... VI_MAX_PIPE_NUM - 1] = { .s8I2cAddr = -1}
};

S5K3P3ST_STATE_S g_astS5K3P3ST_State[VI_MAX_PIPE_NUM] = {{0} };

ISP_SNS_MIRRORFLIP_TYPE_E g_aeS5K3P3ST_MirrorFip[VI_MAX_PIPE_NUM] = {0};

CVI_U16 g_au16S5K3P3ST_GainMode[VI_MAX_PIPE_NUM] = {0};
CVI_U16 g_au16S5K3P3ST_L2SMode[VI_MAX_PIPE_NUM] = {0};

/****************************************************************************
 * local variables and functions                                             *
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
/*****S5K3P3ST Lines Range*****/
#define S5K3P3ST_FULL_LINES_MAX  (0x1FFF0)

/*****S5K3P3ST Register Address*****/
#define S5K3P3ST_SHS_COARSE_ADDR		0x0202
#define S5K3P3ST_SHS_FINE_ADDR		0x0200
#define S5K3P3ST_AGAIN_ADDR			0x0204
#define S5K3P3ST_DGAIN_GREENR_ADDR			0x020E
#define S5K3P3ST_DGAIN_RED_ADDR			0x0210
#define S5K3P3ST_DGAIN_BLUE_ADDR			0x0212
#define S5K3P3ST_DGAIN_GREENB_ADDR			0x0214
#define S5K3P3ST_VMAX_ADDR		0x0340

#define S5K3P3ST_RES_IS_1744P(w, h)      ((w) <= 2320 && (h) <= 1744)

static CVI_S32 cmos_get_ae_default(VI_PIPE ViPipe, AE_SENSOR_DEFAULT_S *pstAeSnsDft)
{
	ISP_SNS_STATE_S *pstSnsState = CVI_NULL;

	CMOS_CHECK_POINTER(pstAeSnsDft);
	S5K3P3ST_SENSOR_GET_CTX(ViPipe, pstSnsState);
	CMOS_CHECK_POINTER(pstSnsState);

	pstAeSnsDft->u32FullLinesStd = pstSnsState->u32FLStd;
	pstAeSnsDft->u32FlickerFreq = 50 * 256;
	pstAeSnsDft->u32FullLinesMax = S5K3P3ST_FULL_LINES_MAX;
	pstAeSnsDft->u32HmaxTimes =
		(1000000) / (pstSnsState->u32FLStd * g_astS5K3P3ST_mode[pstSnsState->u8ImgMode].f32MaxFps);

	pstAeSnsDft->stIntTimeAccu.enAccuType = AE_ACCURACY_LINEAR;
	pstAeSnsDft->stIntTimeAccu.f32Accuracy = S5K3P3ST_EXPACCURACY;
	pstAeSnsDft->stIntTimeAccu.f32Offset = 0;

	pstAeSnsDft->stAgainAccu.enAccuType = AE_ACCURACY_TABLE;
	pstAeSnsDft->stAgainAccu.f32Accuracy = 1;

	pstAeSnsDft->stDgainAccu.enAccuType = AE_ACCURACY_TABLE;
	pstAeSnsDft->stDgainAccu.f32Accuracy = 1;

	pstAeSnsDft->u32ISPDgainShift = 8;
	pstAeSnsDft->u32MinISPDgainTarget = 1 << pstAeSnsDft->u32ISPDgainShift;
	pstAeSnsDft->u32MaxISPDgainTarget = 2 << pstAeSnsDft->u32ISPDgainShift;

	if (g_au32LinesPer500ms[ViPipe] == 0)
		pstAeSnsDft->u32LinesPer500ms =
			pstSnsState->u32FLStd * g_astS5K3P3ST_mode[pstSnsState->u8ImgMode].f32MaxFps / 2;
	else
		pstAeSnsDft->u32LinesPer500ms = g_au32LinesPer500ms[ViPipe];
	pstAeSnsDft->u32SnsStableFrame = 0;

	switch (pstSnsState->enWDRMode) {
	case WDR_MODE_NONE:   /*linear mode*/
		pstAeSnsDft->f32Fps = g_astS5K3P3ST_mode[pstSnsState->u8ImgMode].f32MaxFps;
		pstAeSnsDft->f32MinFps = g_astS5K3P3ST_mode[pstSnsState->u8ImgMode].f32MinFps;
		pstAeSnsDft->au8HistThresh[0] = 0xd;
		pstAeSnsDft->au8HistThresh[1] = 0x28;
		pstAeSnsDft->au8HistThresh[2] = 0x60;
		pstAeSnsDft->au8HistThresh[3] = 0x80;

		pstAeSnsDft->u32MaxAgain = g_astS5K3P3ST_mode[pstSnsState->u8ImgMode].stAgain[0].u32Max;
		pstAeSnsDft->u32MinAgain = g_astS5K3P3ST_mode[pstSnsState->u8ImgMode].stAgain[0].u32Min;
		pstAeSnsDft->u32MaxAgainTarget = pstAeSnsDft->u32MaxAgain;
		pstAeSnsDft->u32MinAgainTarget = pstAeSnsDft->u32MinAgain;

		pstAeSnsDft->u32MaxDgain = g_astS5K3P3ST_mode[pstSnsState->u8ImgMode].stDgain[0].u32Max;
		pstAeSnsDft->u32MinDgain = g_astS5K3P3ST_mode[pstSnsState->u8ImgMode].stDgain[0].u32Min;
		pstAeSnsDft->u32MaxDgainTarget = pstAeSnsDft->u32MaxDgain;
		pstAeSnsDft->u32MinDgainTarget = pstAeSnsDft->u32MinDgain;

		pstAeSnsDft->u8AeCompensation = 40;
		pstAeSnsDft->u32InitAESpeed = 64;
		pstAeSnsDft->u32InitAETolerance = 5;
		pstAeSnsDft->u32AEResponseFrame = 4;
		pstAeSnsDft->enAeExpMode = AE_EXP_HIGHLIGHT_PRIOR;
		pstAeSnsDft->u32InitExposure = g_au32InitExposure[ViPipe] ? g_au32InitExposure[ViPipe] : 76151;

		pstAeSnsDft->u32MaxIntTime = g_astS5K3P3ST_mode[pstSnsState->u8ImgMode].stExp[0].u32Max;
		pstAeSnsDft->u32MinIntTime = g_astS5K3P3ST_mode[pstSnsState->u8ImgMode].stExp[0].u32Min;
		pstAeSnsDft->u32MaxIntTimeTarget = 65535;
		pstAeSnsDft->u32MinIntTimeTarget = 1;
		break;
	default:
		CVI_TRACE_SNS(CVI_DBG_ERR, "Not support sensor mode: %d\n", pstSnsState->u8ImgMode);
		return CVI_FAILURE;
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
	S5K3P3ST_SENSOR_GET_CTX(ViPipe, pstSnsState);
	CMOS_CHECK_POINTER(pstSnsState);

	u32Vts = g_astS5K3P3ST_mode[pstSnsState->u8ImgMode].u32VtsDef;
	pstSnsRegsInfo = &pstSnsState->astSyncInfo[0].snsCfg;
	f32MaxFps = g_astS5K3P3ST_mode[pstSnsState->u8ImgMode].f32MaxFps;
	f32MinFps = g_astS5K3P3ST_mode[pstSnsState->u8ImgMode].f32MinFps;

	switch (pstSnsState->u8ImgMode) {
	case S5K3P3ST_MODE_1744P30:
		if ((f32Fps <= f32MaxFps) && (f32Fps >= f32MinFps)) {
			u32VMAX = u32Vts * f32MaxFps / DIV_0_TO_1_FLOAT(f32Fps);
		} else {
			CVI_TRACE_SNS(CVI_DBG_ERR, "Not support Fps: %f\n", f32Fps);
			return CVI_FAILURE;
		}
		u32VMAX = (u32VMAX > S5K3P3ST_FULL_LINES_MAX) ? S5K3P3ST_FULL_LINES_MAX : u32VMAX;
		break;
	default:
		CVI_TRACE_SNS(CVI_DBG_ERR, "Not support sensor mode: %d\n", pstSnsState->u8ImgMode);
		return CVI_FAILURE;
	}


	pstSnsState->u32FLStd = u32VMAX;


	pstSnsRegsInfo->astI2cData[LINEAR_VMAX_ADDR].u32Data = (u32VMAX & 0xFFFF);

	pstAeSnsDft->f32Fps = f32Fps;
	pstAeSnsDft->u32LinesPer500ms = pstSnsState->u32FLStd * f32Fps / 2;
	pstAeSnsDft->u32FullLinesStd = pstSnsState->u32FLStd;
	pstAeSnsDft->u32MaxIntTime = pstSnsState->u32FLStd;
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

	S5K3P3ST_SENSOR_GET_CTX(ViPipe, pstSnsState);
	CMOS_CHECK_POINTER(pstSnsState);
	CMOS_CHECK_POINTER(u32IntTime);
	pstSnsRegsInfo = &pstSnsState->astSyncInfo[0].snsCfg;

	CVI_U32 u32TmpIntTime = u32IntTime[0];
	CVI_U32 maxExp = pstSnsState->au32FL[0];

	/* linear exposure reg range:
		* min : 2
		* max : vts - 8
		* step : 1
		*/
	u32TmpIntTime = (u32TmpIntTime > maxExp) ? maxExp : u32TmpIntTime;
	if (u32TmpIntTime < 1)
		u32TmpIntTime = 1;
	u32IntTime[0] = u32TmpIntTime;
	pstSnsRegsInfo->astI2cData[LINEAR_SHS_COARSE_ADDR].u32Data = (u32TmpIntTime & 0xFFFF);

	return CVI_SUCCESS;
}


static CVI_U32 Again_table[] = {
    1024,  1088,  1152,  1216,  1280,  1344,  1408,  1472,  1536,  1600,
    1664,  1728,  1792,  1856,  1920,  1984,  2048,  2112,  2176,  2240,
    2304,  2368,  2432,  2496,  2560,  2624,  2688,  2752,  2816,  2880,
    2944,  3008,  3072,  3136,  3200,  3264,  3328,  3392,  3456,  3520,
    3584,  3648,  3712,  3776,  3840,  3904,  3968,  4032,  4096,  4160,
    4224,  4288,  4352,  4416,  4480,  4544,  4608,  4672,  4736,  4800,
    4864,  4928,  4992,  5056,  5120,  5184,  5248,  5312,  5376,  5440,
    5504,  5568,  5632,  5696,  5760,  5824,  5888,  5952,  6016,  6080,
    6144,  6208,  6272,  6336,  6400,  6464,  6528,  6592,  6656,  6720,
    6784,  6848,  6912,  6976,  7040,  7104,  7168,  7232,  7296,  7360,
    7424,  7488,  7552,  7616,  7680,  7744,  7808,  7872,  7936,  8000,
    8064,  8128,  8192,  8256,  8320,  8384,  8448,  8512,  8576,  8640,
    8704,  8768,  8832,  8896,  8960,  9024,  9088,  9152,  9216,  9280,
    9344,  9408,  9472,  9536,  9600,  9664,  9728,  9792,  9856,  9920,
    9984, 10048, 10112, 10176, 10240, 10304, 10368, 10432, 10496, 10560,
    10624, 10688, 10752, 10816, 10880, 10944, 11008, 11072, 11136, 11200,
    11264, 11328, 11392, 11456, 11520, 11584, 11648, 11712, 11776, 11840,
    11904, 11968, 12032, 12096, 12160, 12224, 12288, 12352, 12416, 12480,
    12544, 12608, 12672, 12736, 12800, 12864, 12928, 12992, 13056, 13120,
    13184, 13248, 13312, 13376, 13440, 13504, 13568, 13632, 13696, 13760,
    13824, 13888, 13952, 14016, 14080, 14144, 14208, 14272, 14336, 14400,
    14464, 14528, 14592, 14656, 14720, 14784, 14848, 14912, 14976, 15040,
    15104, 15168, 15232, 15296, 15360, 15424, 15488, 15552, 15616, 15680,
    15744, 15808, 15872, 15936, 16000, 16064, 16128, 16192, 16256, 16320,
    16384
};


static CVI_U32 Dgain_table[] = {
    1024,  1088,  1152,  1216,  1280,  1344,  1408,  1472,  1536,  1600,
    1664,  1728,  1792,  1856,  1920,  1984,  2048,  2112,  2176,  2240,
    2304,  2368,  2432,  2496,  2560,  2624,  2688,  2752,  2816,  2880,
    2944,  3008,  3072,  3136,  3200,  3264,  3328,  3392,  3456,  3520,
    3584,  3648,  3712,  3776,  3840,  3904,  3968,  4032,  4096,  4160,
    4224,  4288,  4352,  4416,  4480,  4544,  4608,  4672,  4736,  4800,
    4864,  4928,  4992,  5056,  5120,  5184,  5248,  5312,  5376,  5440,
    5504,  5568,  5632,  5696,  5760,  5824,  5888,  5952,  6016,  6080,
    6144,  6208,  6272,  6336,  6400,  6464,  6528,  6592,  6656,  6720,
    6784,  6848,  6912,  6976,  7040,  7104,  7168,  7232,  7296,  7360,
    7424,  7488,  7552,  7616,  7680,  7744,  7808,  7872,  7936,  8000,
    8064,  8128,  8192,  8256,  8320,  8384,  8448,  8512,  8576,  8640,
    8704,  8768,  8832,  8896,  8960,  9024,  9088,  9152,  9216,  9280,
    9344,  9408,  9472,  9536,  9600,  9664,  9728,  9792,  9856,  9920,
    9984, 10048, 10112, 10176, 10240, 10304, 10368, 10432, 10496, 10560,
    10624, 10688, 10752, 10816, 10880, 10944, 11008, 11072, 11136, 11200,
    11264, 11328, 11392, 11456, 11520, 11584, 11648, 11712, 11776, 11840,
    11904, 11968, 12032, 12096, 12160, 12224, 12288, 12352, 12416, 12480,
    12544, 12608, 12672, 12736, 12800, 12864, 12928, 12992, 13056, 13120,
    13184, 13248, 13312, 13376, 13440, 13504, 13568, 13632, 13696, 13760,
    13824, 13888, 13952, 14016, 14080, 14144, 14208, 14272, 14336, 14400,
    14464, 14528, 14592, 14656, 14720, 14784, 14848, 14912, 14976, 15040,
    15104, 15168, 15232, 15296, 15360, 15424, 15488, 15552, 15616, 15680,
    15744, 15808, 15872, 15936, 16000, 16064, 16128, 16192, 16256, 16320,
    16384
};

static CVI_S32 cmos_again_calc_table(VI_PIPE ViPipe, CVI_U32 *pu32AgainLin, CVI_U32 *pu32AgainDb)
{
	int i;
	const int array_size = sizeof(Again_table)/sizeof(Again_table[0]);

	(void)ViPipe;

	CMOS_CHECK_POINTER(pu32AgainLin);
	CMOS_CHECK_POINTER(pu32AgainDb);

	if (*pu32AgainLin >= Again_table[array_size - 1]) {
		*pu32AgainLin = Again_table[array_size - 1];
		*pu32AgainDb = array_size - 1;
		return CVI_SUCCESS;
	}

	for (i = 1; i < array_size; i++) {
		if (*pu32AgainLin < Again_table[i]) {
			*pu32AgainLin = Again_table[i - 1];
			*pu32AgainDb = i - 1;
			break;
		}
	}
	return CVI_SUCCESS;
}

static CVI_S32 cmos_dgain_calc_table(VI_PIPE ViPipe, CVI_U32 *pu32DgainLin, CVI_U32 *pu32DgainDb)
{
	int i;
	const int array_size = sizeof(Dgain_table)/sizeof(Dgain_table[0]);

	(void)ViPipe;

	CMOS_CHECK_POINTER(pu32DgainLin);
	CMOS_CHECK_POINTER(pu32DgainDb);

	if (*pu32DgainLin >= Dgain_table[array_size - 1]) {
		*pu32DgainLin = Dgain_table[array_size - 1];
		*pu32DgainDb = array_size - 1;
		return CVI_SUCCESS;
	}

	for (i = 1; i < array_size; i++) {
		if (*pu32DgainLin < Dgain_table[i]) {
			*pu32DgainLin = Dgain_table[i - 1];
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
	CVI_U32 u32Again;
	CVI_U32 u32Dgain;
	CVI_U32 tmp;

	S5K3P3ST_SENSOR_GET_CTX(ViPipe, pstSnsState);
	CMOS_CHECK_POINTER(pstSnsState);
	CMOS_CHECK_POINTER(pu32Again);
	CMOS_CHECK_POINTER(pu32Dgain);
	pstSnsRegsInfo = &pstSnsState->astSyncInfo[0].snsCfg;

	u32Again = pu32Again[0];
	u32Dgain = pu32Dgain[0];

	if (pstSnsState->enWDRMode == WDR_MODE_NONE) {
		/* find Again register setting. */
		tmp = Again_table[u32Again] / 32;
		pstSnsRegsInfo->astI2cData[LINEAR_AGAIN_ADDR].u32Data = tmp & 0xFFFF;

		/* find Dgain register setting. */
		tmp = Dgain_table[u32Dgain] / 4;
		pstSnsRegsInfo->astI2cData[LINEAR_DGAIN_GREENR_ADDR].u32Data = tmp & 0xFFFF;
		pstSnsRegsInfo->astI2cData[LINEAR_DGAIN_RED_ADDR].u32Data = tmp & 0xFFFF;
		pstSnsRegsInfo->astI2cData[LINEAR_DGAIN_BLUE_ADDR].u32Data = tmp & 0xFFFF;
		pstSnsRegsInfo->astI2cData[LINEAR_DGAIN_GREENB_ADDR].u32Data = tmp & 0xFFFF;
	}

	return CVI_SUCCESS;
}

static CVI_S32 cmos_get_inttime_max(VI_PIPE ViPipe, CVI_U16 u16ManRatioEnable, CVI_U32 *au32Ratio,
		CVI_U32 *au32IntTimeMax, CVI_U32 *au32IntTimeMin, CVI_U32 *pu32LFMaxIntTime)
{
	CVI_U32 u32IntTimeMaxTmp  = 0, u32IntTimeMaxTmp0 = 0;
	CVI_U32 u32ShortTimeMinLimit = 4;
	ISP_SNS_STATE_S *pstSnsState = CVI_NULL;
	CVI_U32 u32MaxSexp = g_astS5K3P3ST_State[ViPipe].u32Sexp_MAX - 13;

	(void) u16ManRatioEnable;

	CMOS_CHECK_POINTER(au32Ratio);
	CMOS_CHECK_POINTER(au32IntTimeMax);
	CMOS_CHECK_POINTER(au32IntTimeMin);
	CMOS_CHECK_POINTER(pu32LFMaxIntTime);
	S5K3P3ST_SENSOR_GET_CTX(ViPipe, pstSnsState);
	CMOS_CHECK_POINTER(pstSnsState);

	u32ShortTimeMinLimit = 4;
	/*
	 * Long exp + Short exp < VTS - 27
	 */
	u32IntTimeMaxTmp0 = ((pstSnsState->au32FL[1] - 28 - pstSnsState->au32WDRIntTime[0]) * 0x40) /
					DIV_0_TO_1(au32Ratio[0]);
	u32IntTimeMaxTmp  = ((pstSnsState->au32FL[0] - 28) * 0x40)  / DIV_0_TO_1(au32Ratio[0] + 0x40);
	u32IntTimeMaxTmp = (u32IntTimeMaxTmp > u32IntTimeMaxTmp0) ? u32IntTimeMaxTmp0 : u32IntTimeMaxTmp;

	u32IntTimeMaxTmp  = (u32MaxSexp < u32IntTimeMaxTmp) ?
					u32MaxSexp : u32IntTimeMaxTmp;
	u32IntTimeMaxTmp  = (!u32IntTimeMaxTmp) ? u32ShortTimeMinLimit : u32IntTimeMaxTmp;
	CVI_TRACE_SNS(CVI_DBG_DEBUG, "Max inttime0 = %u\n", u32IntTimeMaxTmp);

	if (pstSnsState->enWDRMode == WDR_MODE_2To1_LINE) {
		au32IntTimeMax[0] = u32IntTimeMaxTmp;
		au32IntTimeMax[1] = au32IntTimeMax[0] * au32Ratio[0] >> 6;
		au32IntTimeMax[2] = au32IntTimeMax[1] * au32Ratio[1] >> 6;
		au32IntTimeMax[3] = au32IntTimeMax[2] * au32Ratio[2] >> 6;
		au32IntTimeMin[0] = u32ShortTimeMinLimit;
		au32IntTimeMin[1] = au32IntTimeMin[0] * au32Ratio[0] >> 6;
		au32IntTimeMin[2] = au32IntTimeMin[1] * au32Ratio[1] >> 6;
		au32IntTimeMin[3] = au32IntTimeMin[2] * au32Ratio[2] >> 6;
		CVI_TRACE_SNS(CVI_DBG_DEBUG, "ViPipe = %d ratio = %d, (%d, %d)\n", ViPipe, au32Ratio[0],
				u32IntTimeMaxTmp, u32ShortTimeMinLimit);
	}

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

	return CVI_SUCCESS;
}

static CVI_S32 cmos_get_blc_default(VI_PIPE ViPipe, ISP_CMOS_BLACK_LEVEL_S *pstBlc)
{
	(void) ViPipe;

	CMOS_CHECK_POINTER(pstBlc);

	memset(pstBlc, 0, sizeof(ISP_CMOS_BLACK_LEVEL_S));

	memcpy(pstBlc,
		&g_stIspBlcCalibratio, sizeof(ISP_CMOS_BLACK_LEVEL_S));
	return CVI_SUCCESS;
}

static CVI_S32 cmos_get_wdr_size(VI_PIPE ViPipe, ISP_SNS_ISP_INFO_S *pstIspCfg)
{
	const S5K3P3ST_MODE_S *pstMode = CVI_NULL;
	ISP_SNS_STATE_S *pstSnsState = CVI_NULL;

	S5K3P3ST_SENSOR_GET_CTX(ViPipe, pstSnsState);
	CMOS_CHECK_POINTER(pstSnsState);
	pstMode = &g_astS5K3P3ST_mode[pstSnsState->u8ImgMode];

	if (pstSnsState->enWDRMode != WDR_MODE_NONE) {
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

	S5K3P3ST_SENSOR_GET_CTX(ViPipe, pstSnsState);
	CMOS_CHECK_POINTER(pstSnsState);

	pstSnsState->bSyncInit = CVI_FALSE;

	switch (u8Mode) {
	case WDR_MODE_NONE:
		pstSnsState->u8ImgMode = pstSnsState->u8ImgMode < S5K3P3ST_LINEAR_NUM ?
			pstSnsState->u8ImgMode: S5K3P3ST_MODE_1744P30;
		pstSnsState->enWDRMode = WDR_MODE_NONE;
		pstSnsState->u32FLStd = g_astS5K3P3ST_mode[pstSnsState->u8ImgMode].u32VtsDef;
		CVI_TRACE_SNS(CVI_DBG_INFO, "linear mode\n");
		break;
	default:
		pstSnsState->u8ImgMode = S5K3P3ST_MODE_1744P30;
		pstSnsState->enWDRMode = WDR_MODE_NONE;
		pstSnsState->u32FLStd = g_astS5K3P3ST_mode[pstSnsState->u8ImgMode].u32VtsDef;
		CVI_TRACE_SNS(CVI_DBG_INFO, "wdr mode\n");
		break;
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

static CVI_U32 sensor_cmp_cif_wdr(ISP_SNS_CIF_INFO_S *pstWdr1, ISP_SNS_CIF_INFO_S *pstWdr2)
{
	if (pstWdr1->wdr_manual.l2s_distance != pstWdr2->wdr_manual.l2s_distance)
		goto _mismatch;
	if (pstWdr1->wdr_manual.lsef_length != pstWdr2->wdr_manual.lsef_length)
		goto _mismatch;

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
	S5K3P3ST_SENSOR_GET_CTX(ViPipe, pstSnsState);
	CMOS_CHECK_POINTER(pstSnsState);
	pstSnsRegsInfo = &pstSnsSyncInfo->snsCfg;
	pstCfg0 = &pstSnsState->astSyncInfo[0];
	pstCfg1 = &pstSnsState->astSyncInfo[1];
	pstI2c_data = pstCfg0->snsCfg.astI2cData;

	if ((pstSnsState->bSyncInit == CVI_FALSE) || (pstSnsRegsInfo->bConfig == CVI_FALSE)) {
		pstCfg0->snsCfg.enSnsType = SNS_I2C_TYPE;
		pstCfg0->snsCfg.unComBus.s8I2cDev = g_aunS5K3P3ST_BusInfo[ViPipe].s8I2cDev;
		pstCfg0->snsCfg.u8Cfg2ValidDelayMax = 0;
		pstCfg0->snsCfg.use_snsr_sram = CVI_TRUE;
		pstCfg0->snsCfg.u32RegNum = LINEAR_REGS_NUM;

		for (i = 0; i < pstCfg0->snsCfg.u32RegNum; i++) {
			pstI2c_data[i].bUpdate = CVI_TRUE;
			pstI2c_data[i].u8DevAddr = g_aunS5K3P3ST_AddrInfo[ViPipe].s8I2cAddr;
			pstI2c_data[i].u32AddrByteNum = s5k3p3st_addr_byte;
			pstI2c_data[i].u32DataByteNum = s5k3p3st_data_byte;
		}
		switch (pstSnsState->enWDRMode) {
		default:
			pstI2c_data[LINEAR_SHS_COARSE_ADDR].u32RegAddr     = S5K3P3ST_SHS_COARSE_ADDR;
			pstI2c_data[LINEAR_AGAIN_ADDR].u32RegAddr     = S5K3P3ST_AGAIN_ADDR;
			pstI2c_data[LINEAR_DGAIN_GREENR_ADDR].u32RegAddr     = S5K3P3ST_DGAIN_GREENR_ADDR;
			pstI2c_data[LINEAR_DGAIN_RED_ADDR].u32RegAddr     = S5K3P3ST_DGAIN_RED_ADDR;

			pstI2c_data[LINEAR_DGAIN_BLUE_ADDR].u32RegAddr      = S5K3P3ST_DGAIN_BLUE_ADDR;
			pstI2c_data[LINEAR_DGAIN_GREENB_ADDR].u32RegAddr = S5K3P3ST_DGAIN_GREENB_ADDR;
			pstI2c_data[LINEAR_VMAX_ADDR].u32RegAddr      = S5K3P3ST_VMAX_ADDR;
			break;
		}

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
				// printf("i2c_addr = 0x%x, i2c_data = 0x%x\n", pstCfg0->snsCfg.astI2cData[i].u32RegAddr, pstCfg0->snsCfg.astI2cData[i].u32Data);
				pstCfg0->snsCfg.astI2cData[i].bUpdate = CVI_TRUE;
				pstCfg0->snsCfg.need_update = CVI_TRUE;
			}
		}

		/* check update isp crop or not */
		pstCfg0->ispCfg.need_update = (sensor_cmp_wdr_size(&pstCfg0->ispCfg, &pstCfg1->ispCfg) ?
				CVI_TRUE : CVI_FALSE);

		/* check update cif wdr manual or not */
		pstCfg0->cifCfg.need_update = (sensor_cmp_cif_wdr(&pstCfg0->cifCfg, &pstCfg1->cifCfg) ?
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
	S5K3P3ST_SENSOR_GET_CTX(ViPipe, pstSnsState);
	CMOS_CHECK_POINTER(pstSnsState);

	u8SensorImageMode = pstSnsState->u8ImgMode;
	pstSnsState->bSyncInit = CVI_FALSE;

	if (pstSensorImageMode->f32Fps <= 30) {
		if (pstSnsState->enWDRMode == WDR_MODE_NONE) {
			if (S5K3P3ST_RES_IS_1744P(pstSensorImageMode->u16Width, pstSensorImageMode->u16Height)) {
					u8SensorImageMode = S5K3P3ST_MODE_1744P30;

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

static CVI_VOID sensor_mirror_flip(VI_PIPE ViPipe, ISP_SNS_MIRRORFLIP_TYPE_E eSnsMirrorFlip)
{
	ISP_SNS_STATE_S *pstSnsState = CVI_NULL;

	S5K3P3ST_SENSOR_GET_CTX(ViPipe, pstSnsState);
	CMOS_CHECK_POINTER_VOID(pstSnsState);
	/* Apply the setting on the fly  */
	if (pstSnsState->bInit == CVI_TRUE && g_aeS5K3P3ST_MirrorFip[ViPipe] != eSnsMirrorFlip) {
		s5k3p3st_mirror_flip(ViPipe, eSnsMirrorFlip);
		g_aeS5K3P3ST_MirrorFip[ViPipe] = eSnsMirrorFlip;
	}
}

static CVI_VOID sensor_global_init(VI_PIPE ViPipe)
{
	ISP_SNS_STATE_S *pstSnsState = CVI_NULL;
	const S5K3P3ST_MODE_S *pstMode = CVI_NULL;

	S5K3P3ST_SENSOR_GET_CTX(ViPipe, pstSnsState);
	CMOS_CHECK_POINTER_VOID(pstSnsState);

	pstSnsState->bInit = CVI_FALSE;
	pstSnsState->bSyncInit = CVI_FALSE;
	pstSnsState->u8ImgMode = S5K3P3ST_MODE_1744P30;
	pstSnsState->enWDRMode = WDR_MODE_NONE;
	pstMode = &g_astS5K3P3ST_mode[pstSnsState->u8ImgMode];
	pstSnsState->u32FLStd  = pstMode->u32VtsDef;
	pstSnsState->au32FL[0] = pstMode->u32VtsDef;
	pstSnsState->au32FL[1] = pstMode->u32VtsDef;

	memset(&pstSnsState->astSyncInfo[0], 0, sizeof(ISP_SNS_SYNC_INFO_S));
	memset(&pstSnsState->astSyncInfo[1], 0, sizeof(ISP_SNS_SYNC_INFO_S));
}

static CVI_S32 sensor_rx_attr(VI_PIPE ViPipe, SNS_COMBO_DEV_ATTR_S *pstRxAttr)
{
	ISP_SNS_STATE_S *pstSnsState = CVI_NULL;
	SNS_COMBO_DEV_ATTR_S *pstRxAttrSrc = CVI_NULL;

	S5K3P3ST_SENSOR_GET_CTX(ViPipe, pstSnsState);
	S5K3P3ST_SENSOR_GET_COMBO(ViPipe, pstRxAttrSrc);

	CMOS_CHECK_POINTER(pstSnsState);
	CMOS_CHECK_POINTER(pstRxAttr);
	CMOS_CHECK_POINTER(pstRxAttrSrc);

	memcpy(pstRxAttr, pstRxAttrSrc, sizeof(*pstRxAttr));

	pstRxAttr->img_size.start_x = g_astS5K3P3ST_mode[pstSnsState->u8ImgMode].astImg[0].stWndRect.s32X;
	pstRxAttr->img_size.start_y = g_astS5K3P3ST_mode[pstSnsState->u8ImgMode].astImg[0].stWndRect.s32Y;
	pstRxAttr->img_size.active_w = g_astS5K3P3ST_mode[pstSnsState->u8ImgMode].astImg[0].stWndRect.u32Width;
	pstRxAttr->img_size.active_h = g_astS5K3P3ST_mode[pstSnsState->u8ImgMode].astImg[0].stWndRect.u32Height;
	pstRxAttr->img_size.width = g_astS5K3P3ST_mode[pstSnsState->u8ImgMode].astImg[0].stSnsSize.u32Width;
	pstRxAttr->img_size.height = g_astS5K3P3ST_mode[pstSnsState->u8ImgMode].astImg[0].stSnsSize.u32Height;
	pstRxAttr->img_size.max_width = g_astS5K3P3ST_mode[pstSnsState->u8ImgMode].astImg[0].stMaxSize.u32Width;
	pstRxAttr->img_size.max_height = g_astS5K3P3ST_mode[pstSnsState->u8ImgMode].astImg[0].stMaxSize.u32Height;

	if (pstSnsState->enWDRMode == WDR_MODE_NONE) {
		pstRxAttr->mipi_attr.wdr_mode = CVI_MIPI_WDR_MODE_NONE;
	} else if (pstSnsState->enWDRMode == WDR_MODE_2To1_LINE) {
		pstRxAttr->mipi_attr.wdr_mode = CVI_MIPI_WDR_MODE_VC;
	}
	pstRxAttrSrc = CVI_NULL;
	return CVI_SUCCESS;

}

static CVI_S32 sensor_patch_rx_attr(VI_PIPE ViPipe, RX_INIT_ATTR_S *pstRxInitAttr)
{
	int i;
	SNS_COMBO_DEV_ATTR_S* pstRxAttr = malloc(sizeof(SNS_COMBO_DEV_ATTR_S));

	if(!g_pastS5K3P3STComboDevArray[ViPipe]) {
		pstRxAttr = malloc(sizeof(SNS_COMBO_DEV_ATTR_S));
	} else {
		S5K3P3ST_SENSOR_GET_COMBO(ViPipe, pstRxAttr);
	}
	memcpy(pstRxAttr, &s5k3p3st_rx_attr, sizeof(SNS_COMBO_DEV_ATTR_S));
	S5K3P3ST_SENSOR_SET_COMBO(ViPipe, pstRxAttr);

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

void s5k3p3st_exit(VI_PIPE ViPipe)
{
	if(g_pastS5K3P3STComboDevArray[ViPipe]) {
		free(g_pastS5K3P3STComboDevArray[ViPipe]);
		g_pastS5K3P3STComboDevArray[ViPipe] = CVI_NULL;
	}
	s5k3p3st_i2c_exit(ViPipe);
}

static CVI_S32 cmos_init_sensor_exp_function(ISP_SENSOR_EXP_FUNC_S *pstSensorExpFunc)
{
	CMOS_CHECK_POINTER(pstSensorExpFunc);

	memset(pstSensorExpFunc, 0, sizeof(ISP_SENSOR_EXP_FUNC_S));

	pstSensorExpFunc->pfn_cmos_sensor_init = s5k3p3st_init;
	pstSensorExpFunc->pfn_cmos_sensor_exit = s5k3p3st_exit;
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
	if (S5K3P3ST_I2C_ADDR_IS_VALID(s32I2cAddr))
		g_aunS5K3P3ST_AddrInfo[ViPipe].s8I2cAddr = s32I2cAddr;
	else {
		CVI_TRACE_SNS(CVI_DBG_ERR, "I2C addr input error ,please check [0x%x]\n", s32I2cAddr);
		g_aunS5K3P3ST_AddrInfo[ViPipe].s8I2cAddr = S5K3P3ST_I2C_ADDR_1;
	}
}

static CVI_S32 s5k3p3st_set_bus_info(VI_PIPE ViPipe, ISP_SNS_COMMBUS_U unSNSBusInfo)
{
	g_aunS5K3P3ST_BusInfo[ViPipe].s8I2cDev = unSNSBusInfo.s8I2cDev;

	return CVI_SUCCESS;
}

static CVI_S32 sensor_ctx_init(VI_PIPE ViPipe)
{
	ISP_SNS_STATE_S *pastSnsStateCtx = CVI_NULL;

	S5K3P3ST_SENSOR_GET_CTX(ViPipe, pastSnsStateCtx);

	if (pastSnsStateCtx == CVI_NULL) {
		pastSnsStateCtx = (ISP_SNS_STATE_S *)malloc(sizeof(ISP_SNS_STATE_S));
		if (pastSnsStateCtx == CVI_NULL) {
			CVI_TRACE_SNS(CVI_DBG_ERR, "Isp[%d] SnsCtx malloc memory failed!\n", ViPipe);
			return -ENOMEM;
		}
	}

	memset(pastSnsStateCtx, 0, sizeof(ISP_SNS_STATE_S));

	S5K3P3ST_SENSOR_SET_CTX(ViPipe, pastSnsStateCtx);

	return CVI_SUCCESS;
}

static CVI_VOID sensor_ctx_exit(VI_PIPE ViPipe)
{
	ISP_SNS_STATE_S *pastSnsStateCtx = CVI_NULL;

	S5K3P3ST_SENSOR_GET_CTX(ViPipe, pastSnsStateCtx);
	SENSOR_FREE(pastSnsStateCtx);
	S5K3P3ST_SENSOR_RESET_CTX(ViPipe);
	g_aeS5K3P3ST_MirrorFip[ViPipe] = ISP_SNS_NORMAL;
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

	stSnsAttrInfo.eSensorId = S5K3P3ST_ID;

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

	s32Ret = CVI_ISP_SensorUnRegCallBack(ViPipe, S5K3P3ST_ID);
	if (s32Ret != CVI_SUCCESS) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "sensor unregister callback function failed!\n");
		return s32Ret;
	}

	s32Ret = CVI_AE_SensorUnRegCallBack(ViPipe, pstAeLib, S5K3P3ST_ID);
	if (s32Ret != CVI_SUCCESS) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "sensor unregister callback function to ae lib failed!\n");
		return s32Ret;
	}

	s32Ret = CVI_AWB_SensorUnRegCallBack(ViPipe, pstAwbLib, S5K3P3ST_ID);
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
	g_au16S5K3P3ST_GainMode[ViPipe] = pstInitAttr->enGainMode;
	g_au16S5K3P3ST_L2SMode[ViPipe] = pstInitAttr->enL2SMode;

	return CVI_SUCCESS;
}

ISP_SNS_OBJ_S stSnsS5K3P3ST_Obj = {
	.pfnRegisterCallback    = sensor_register_callback,
	.pfnUnRegisterCallback  = sensor_unregister_callback,
	.pfnStandby             = s5k3p3st_standby,
	.pfnRestart             = s5k3p3st_restart,
	.pfnMirrorFlip          = sensor_mirror_flip,
	.pfnWriteReg            = s5k3p3st_write_register,
	.pfnReadReg             = s5k3p3st_read_register,
	.pfnSetBusInfo          = s5k3p3st_set_bus_info,
	.pfnSetInit             = sensor_set_init,
	.pfnPatchRxAttr		= sensor_patch_rx_attr,
	.pfnPatchI2cAddr	= sensor_patch_i2c_addr,
	.pfnGetRxAttr		= sensor_rx_attr,
	.pfnExpSensorCb		= cmos_init_sensor_exp_function,
	.pfnExpAeCb		= cmos_init_ae_exp_function,
	.pfnSnsProbe		= s5k3p3st_probe,
};

