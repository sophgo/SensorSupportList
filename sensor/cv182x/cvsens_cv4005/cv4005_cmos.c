#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <syslog.h>
#include <errno.h>

#ifdef ARCH_CV182X
#include "cvi_type.h"
#include "cvi_comm_video.h"
#include <linux/cvi_vip_snsr.h>
#else
#include <linux/cvi_type.h>
#include <linux/cvi_comm_video.h>
#include <linux/vi_snsr.h>
#endif
#include "cvi_debug.h"
#include "cvi_comm_sns.h"
#include "cvi_sns_ctrl.h"
#include "cvi_ae_comm.h"
#include "cvi_awb_comm.h"
#include "cvi_ae.h"
#include "cvi_awb.h"
#include "cvi_isp.h"

#include "cv4005_cmos_ex.h"
#include "cv4005_cmos_param.h"

#define DIV_0_TO_1(a)   ((0 == (a)) ? 1 : (a))
#define DIV_0_TO_1_FLOAT(a) ((((a) < 1E-10) && ((a) > -1E-10)) ? 1 : (a))
#define CV4005_ID 4005
#define CV4005_I2C_ADDR_1 0x35
#define CV4005_I2C_ADDR_2 0x36
#define CV4005_I2C_ADDR_IS_VALID(addr)      ((addr) == CV4005_I2C_ADDR_1 || (addr) == CV4005_I2C_ADDR_2)

/****************************************************************************
 * global variables                                                         *
 ***************************************************************************/

ISP_SNS_STATE_S *g_pastCV4005[VI_MAX_PIPE_NUM] = {CVI_NULL};

#define CV4005_SENSOR_GET_CTX(dev, pstCtx)   (pstCtx = g_pastCV4005[dev])
#define CV4005_SENSOR_SET_CTX(dev, pstCtx)   (g_pastCV4005[dev] = pstCtx)
#define CV4005_SENSOR_RESET_CTX(dev)         (g_pastCV4005[dev] = CVI_NULL)

ISP_SNS_COMMBUS_U g_aunCV4005_BusInfo[VI_MAX_PIPE_NUM] = {
	[0] = { .s8I2cDev = 3},
	[1 ... VI_MAX_PIPE_NUM - 1] = { .s8I2cDev = -1}
};

ISP_SNS_MIRRORFLIP_TYPE_E g_aeCV4005_MirrorFip[VI_MAX_PIPE_NUM] = {0};

CVI_U16 g_au16CV4005_GainMode[VI_MAX_PIPE_NUM] = {0};

/****************************************************************************
 * local variables and functions                                            *
 ***************************************************************************/
static CVI_U32 g_au32InitExposure[VI_MAX_PIPE_NUM]  = {0};
static CVI_U32 g_au32LinesPer500ms[VI_MAX_PIPE_NUM] = {0};
static CVI_U16 g_au16InitWBGain[VI_MAX_PIPE_NUM][3] = {{0} };
static CVI_U16 g_au16SampleRgain[VI_MAX_PIPE_NUM] = {0};
static CVI_U16 g_au16SampleBgain[VI_MAX_PIPE_NUM] = {0};
static CVI_S32 cmos_get_isp_size(VI_PIPE ViPipe, ISP_SNS_ISP_INFO_S *pstIspCfg);
static CVI_U32 sensor_cmp_isp_size(ISP_SNS_ISP_INFO_S *pstIsp1, ISP_SNS_ISP_INFO_S *pstIsp2);
/*****CV4005 Lines Range*****/
#define CV4005_FULL_LINES_MAX  (0xfffff)

/*****CV4005 Register Address*****/
#define CV4005_EXP1_ADDR0			0x304a
#define CV4005_EXP1_ADDR1			0x3049
#define CV4005_EXP1_ADDR2			0x3048
#define CV4005_AGAIN1_ADDR			0x3114 /* datasheet GAIN_0, not FAE 0x3154 */
#define CV4005_VTS_ADDR0			0x301e
#define CV4005_VTS_ADDR1			0x301d
#define CV4005_VTS_ADDR2			0x301c
#define CV4005_FLIP_MIRROR_ADDR		0x3028

#define CV4005_RES_IS_1440P(w, h)		((w) == 2560 && (h) == 1440)

static CVI_S32 cmos_get_ae_default(VI_PIPE ViPipe, AE_SENSOR_DEFAULT_S *pstAeSnsDft)
{
	const CV4005_MODE_S *pstMode;

	CVI_U32 FpsMax;
	ISP_SNS_STATE_S *pstSnsState = CVI_NULL;

	CMOS_CHECK_POINTER(pstAeSnsDft);
	CV4005_SENSOR_GET_CTX(ViPipe, pstSnsState);
	CMOS_CHECK_POINTER(pstSnsState);

	pstMode = &g_astCV4005_mode[pstSnsState->u8ImgMode];
	FpsMax = g_astCV4005_mode[pstSnsState->u8ImgMode].f32MaxFps;

	pstAeSnsDft->u32FullLinesStd = pstSnsState->u32FLStd;
	pstAeSnsDft->u32FlickerFreq = 50 * 256;
	pstAeSnsDft->u32FullLinesMax = CV4005_FULL_LINES_MAX;
	pstAeSnsDft->u32HmaxTimes = (1000000) / (pstSnsState->u32FLStd * FpsMax);

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
		pstAeSnsDft->u32LinesPer500ms = pstSnsState->u32FLStd * FpsMax / 2;
	else
		pstAeSnsDft->u32LinesPer500ms = g_au32LinesPer500ms[ViPipe];

	pstAeSnsDft->f32Fps = pstMode->f32MaxFps;
	pstAeSnsDft->f32MinFps = pstMode->f32MinFps;
	pstAeSnsDft->au8HistThresh[0] = 0xd;
	pstAeSnsDft->au8HistThresh[1] = 0x28;
	pstAeSnsDft->au8HistThresh[2] = 0x60;
	pstAeSnsDft->au8HistThresh[3] = 0x80;
	pstAeSnsDft->u32MaxIntTime = pstMode->stExp[0].u16Max;
	pstAeSnsDft->u32MinIntTime = pstMode->stExp[0].u16Min;
	pstAeSnsDft->u32MaxIntTimeTarget = 65535;
	pstAeSnsDft->u32MinIntTimeTarget = 6;
	pstAeSnsDft->u32MaxAgain = pstMode->stAgain[0].u32Max;
	pstAeSnsDft->u32MinAgain = pstMode->stAgain[0].u32Min;
	pstAeSnsDft->u32MaxAgainTarget = pstAeSnsDft->u32MaxAgain;
	pstAeSnsDft->u32MinAgainTarget = pstAeSnsDft->u32MinAgain;
	pstAeSnsDft->u32MaxDgain = pstMode->stDgain[0].u32Max;
	pstAeSnsDft->u32MinDgain = pstMode->stDgain[0].u32Min;
	pstAeSnsDft->u32MaxDgainTarget = pstAeSnsDft->u32MaxDgain;
	pstAeSnsDft->u32MinDgainTarget = pstAeSnsDft->u32MinDgain;
	pstAeSnsDft->u8AeCompensation = 40;
	pstAeSnsDft->u32InitAESpeed = 64;
	pstAeSnsDft->u32InitAETolerance = 2;
	pstAeSnsDft->u32AEResponseFrame = 4;
	pstAeSnsDft->enAeExpMode = AE_EXP_HIGHLIGHT_PRIOR;
	pstAeSnsDft->u32InitExposure = g_au32InitExposure[ViPipe] ?
		g_au32InitExposure[ViPipe] : pstMode->stExp[0].u16Def;
	CVI_TRACE_SNS(CVI_DBG_INFO, "again[%d, %d], dgain[%d, %d]\n",
		pstAeSnsDft->u32MinAgain, pstAeSnsDft->u32MaxAgain, pstAeSnsDft->u32MinDgain, pstAeSnsDft->u32MaxDgain);

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
	CV4005_SENSOR_GET_CTX(ViPipe, pstSnsState);
	CMOS_CHECK_POINTER(pstSnsState);

	pstSnsRegsInfo = &pstSnsState->astSyncInfo[0].snsCfg;
	u32Vts = g_astCV4005_mode[pstSnsState->u8ImgMode].u32VtsDef;
	f32MaxFps = g_astCV4005_mode[pstSnsState->u8ImgMode].f32MaxFps;
	f32MinFps = g_astCV4005_mode[pstSnsState->u8ImgMode].f32MinFps;

	if ((f32Fps <= f32MaxFps) && (f32Fps >= f32MinFps))
		u32VMAX = u32Vts * f32MaxFps / DIV_0_TO_1_FLOAT(f32Fps);
	else
		return CVI_FAILURE;

	u32VMAX = (u32VMAX > CV4005_FULL_LINES_MAX) ? CV4005_FULL_LINES_MAX : u32VMAX;
	pstSnsRegsInfo->astI2cData[LINEAR_VTS_0].u32Data = ((u32VMAX >> 16) & 0x0F);
	pstSnsRegsInfo->astI2cData[LINEAR_VTS_1].u32Data = ((u32VMAX >> 8) & 0xFF);
	pstSnsRegsInfo->astI2cData[LINEAR_VTS_2].u32Data = (u32VMAX & 0xFF);

	pstSnsState->u32FLStd = u32VMAX;
	pstAeSnsDft->f32Fps = f32Fps;
	pstAeSnsDft->u32LinesPer500ms = pstSnsState->u32FLStd * f32Fps / 2;
	pstAeSnsDft->u32FullLinesStd = pstSnsState->u32FLStd;
	pstAeSnsDft->u32MaxIntTime = pstSnsState->u32FLStd - 6;
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
	CVI_U32 exp;

	CV4005_SENSOR_GET_CTX(ViPipe, pstSnsState);
	CMOS_CHECK_POINTER(pstSnsState);
	CMOS_CHECK_POINTER(u32IntTime);
	pstSnsRegsInfo = &pstSnsState->astSyncInfo[0].snsCfg;

	exp = u32IntTime[0];
	if (exp < 6)
		exp = 6;
	if (exp > pstSnsState->u32FLStd - 6)
		exp = pstSnsState->u32FLStd - 6;

	/* SHUTTER0 is complementary: integration = FRAME_LENGTH - SHUTTER0
	 * (same as CV4001/CV4002, without *2 because CV4005 VTS is 1:1)
	 */
	exp = pstSnsState->u32FLStd - exp;

	pstSnsRegsInfo->astI2cData[LINEAR_EXP_0].u32Data = ((exp >> 16) & 0x0F);
	pstSnsRegsInfo->astI2cData[LINEAR_EXP_1].u32Data = ((exp >> 8) & 0xFF);
	pstSnsRegsInfo->astI2cData[LINEAR_EXP_2].u32Data = (exp & 0xFF);

	return CVI_SUCCESS;
}

/* GAIN_0: reg * 0.3dB, 0x00~0xC8 (1024x max) */
static CVI_U32 gain_table[201] = {
	1024, 1060, 1097, 1136, 1176, 1217, 1260, 1304, 1350, 1397,
	1446, 1497, 1550, 1604, 1661, 1719, 1780, 1842, 1907, 1974,
	2043, 2115, 2189, 2266, 2346, 2428, 2514, 2602, 2693, 2788,
	2886, 2987, 3092, 3201, 3314, 3430, 3551, 3675, 3805, 3938,
	4077, 4220, 4368, 4522, 4681, 4845, 5015, 5192, 5374, 5563,
	5758, 5961, 6170, 6387, 6611, 6844, 7084, 7333, 7591, 7858,
	8134, 8420, 8716, 9022, 9339, 9667, 10007, 10359, 10723, 11099,
	11489, 11893, 12311, 12744, 13192, 13655, 14135, 14632, 15146, 15678,
	16229, 16800, 17390, 18001, 18634, 19289, 19966, 20668, 21394, 22146,
	22925, 23730, 24564, 25427, 26321, 27246, 28203, 29194, 30220, 31282,
	32382, 33520, 34698, 35917, 37179, 38486, 39838, 41238, 42687, 44188,
	45740, 47348, 49012, 50734, 52517, 54363, 56273, 58251, 60298, 62417,
	64610, 66881, 69231, 71664, 74182, 76789, 79488, 82281, 85173, 88166,
	91264, 94471, 97791, 101228, 104785, 108468, 112279, 116225, 120310, 124537,
	128914, 133444, 138134, 142988, 148013, 153215, 158599, 164172, 169942, 175914,
	182096, 188495, 195119, 201976, 209074, 216421, 224027, 231900, 240049, 248485,
	257217, 266256, 275613, 285299, 295325, 305703, 316446, 327567, 339078, 350994,
	363329, 376097, 389314, 402995, 417157, 431817, 446992, 462700, 478961, 495793,
	513216, 531251, 549921, 569246, 589250, 609958, 631393, 653582, 676550, 700326,
	724936, 750412, 776783, 804081, 832338, 861589, 891867, 923209, 955652, 989236,
	1024000,
};

static CVI_U32 cv4005_gain_lin_to_reg(CVI_U32 u32GainLin)
{
	CVI_U32 i, total = sizeof(gain_table) / sizeof(gain_table[0]);

	if (u32GainLin >= gain_table[total - 1])
		return total - 1;

	for (i = 1; i < total; i++) {
		if (u32GainLin < gain_table[i])
			break;
	}

	return i - 1;
}

static CVI_S32 cmos_again_calc_table(VI_PIPE ViPipe, CVI_U32 *pu32AgainLin, CVI_U32 *pu32AgainDb)
{
	CVI_U32 u32Reg;

	CMOS_CHECK_POINTER(pu32AgainLin);
	CMOS_CHECK_POINTER(pu32AgainDb);
	UNUSED(ViPipe);

	u32Reg = cv4005_gain_lin_to_reg(*pu32AgainLin);
	*pu32AgainLin = gain_table[u32Reg];
	*pu32AgainDb = gain_table[u32Reg];
	return CVI_SUCCESS;
}

static CVI_S32 cmos_dgain_calc_table(VI_PIPE ViPipe, CVI_U32 *pu32DgainLin, CVI_U32 *pu32DgainDb)
{
	CVI_U32 u32Reg;

	CMOS_CHECK_POINTER(pu32DgainLin);
	CMOS_CHECK_POINTER(pu32DgainDb);
	UNUSED(ViPipe);

	u32Reg = cv4005_gain_lin_to_reg(*pu32DgainLin);
	*pu32DgainLin = gain_table[u32Reg];
	*pu32DgainDb = gain_table[u32Reg];
	return CVI_SUCCESS;
}

static CVI_S32 cmos_gains_update(VI_PIPE ViPipe, CVI_U32 *pu32Again, CVI_U32 *pu32Dgain)
{
	ISP_SNS_STATE_S *pstSnsState = CVI_NULL;
	ISP_SNS_REGS_INFO_S *pstSnsRegsInfo = CVI_NULL;
	CVI_U32 u32Reg;
	CVI_U32 u32RegMax = sizeof(gain_table) / sizeof(gain_table[0]) - 1;

	CV4005_SENSOR_GET_CTX(ViPipe, pstSnsState);
	CMOS_CHECK_POINTER(pstSnsState);
	CMOS_CHECK_POINTER(pu32Again);
	CMOS_CHECK_POINTER(pu32Dgain);
	pstSnsRegsInfo = &pstSnsState->astSyncInfo[0].snsCfg;

	/* GAIN_0 is 0.3dB/step: add again/dgain indices to avoid again*dgain truncation */
	u32Reg = cv4005_gain_lin_to_reg(pu32Again[0]) + cv4005_gain_lin_to_reg(pu32Dgain[0]);
	if (u32Reg > u32RegMax)
		u32Reg = u32RegMax;
	pstSnsRegsInfo->astI2cData[LINEAR_AGAIN].u32Data = u32Reg;

	return CVI_SUCCESS;
}

static CVI_S32 cmos_get_inttime_max(VI_PIPE ViPipe, CVI_U16 u16ManRatioEnable, CVI_U32 *au32Ratio,
				    CVI_U32 *au32IntTimeMax, CVI_U32 *au32IntTimeMin, CVI_U32 *pu32LFMaxIntTime)
{
	ISP_SNS_STATE_S *pstSnsState = CVI_NULL;

	CMOS_CHECK_POINTER(au32IntTimeMax);
	CMOS_CHECK_POINTER(au32IntTimeMin);
	CMOS_CHECK_POINTER(pu32LFMaxIntTime);
	CV4005_SENSOR_GET_CTX(ViPipe, pstSnsState);
	CMOS_CHECK_POINTER(pstSnsState);
	UNUSED(u16ManRatioEnable);
	UNUSED(au32Ratio);

	au32IntTimeMax[0] = pstSnsState->u32FLStd - 6;
	au32IntTimeMin[0] = 6;
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

	return CVI_SUCCESS;
}

static CVI_S32 cmos_get_blc_default(VI_PIPE ViPipe, ISP_CMOS_BLACK_LEVEL_S *pstBlc)
{
	UNUSED(ViPipe);
	CMOS_CHECK_POINTER(pstBlc);
	memcpy(pstBlc, &g_stIspBlcCalibratio, sizeof(ISP_CMOS_BLACK_LEVEL_S));
	return CVI_SUCCESS;
}

static CVI_S32 cmos_get_isp_size(VI_PIPE ViPipe, ISP_SNS_ISP_INFO_S *pstIspCfg)
{
	const CV4005_MODE_S *pstMode = CVI_NULL;
	ISP_SNS_STATE_S *pstSnsState = CVI_NULL;

	CV4005_SENSOR_GET_CTX(ViPipe, pstSnsState);
	CMOS_CHECK_POINTER(pstSnsState);
	pstMode = &g_astCV4005_mode[pstSnsState->u8ImgMode];
	pstIspCfg->frm_num = 1;
	memcpy(&pstIspCfg->img_size[0], &pstMode->astImg[0], sizeof(ISP_WDR_SIZE_S));
	return CVI_SUCCESS;
}

static CVI_U32 sensor_cmp_isp_size(ISP_SNS_ISP_INFO_S *pstIsp1, ISP_SNS_ISP_INFO_S *pstIsp2)
{
	if (pstIsp1->frm_num != pstIsp2->frm_num)
		return 1;
	if (pstIsp1->img_size[0].stSnsSize.u32Width != pstIsp2->img_size[0].stSnsSize.u32Width)
		return 1;
	if (pstIsp1->img_size[0].stSnsSize.u32Height != pstIsp2->img_size[0].stSnsSize.u32Height)
		return 1;
	if (pstIsp1->img_size[0].stWndRect.s32X != pstIsp2->img_size[0].stWndRect.s32X)
		return 1;
	if (pstIsp1->img_size[0].stWndRect.s32Y != pstIsp2->img_size[0].stWndRect.s32Y)
		return 1;
	if (pstIsp1->img_size[0].stWndRect.u32Width != pstIsp2->img_size[0].stWndRect.u32Width)
		return 1;
	if (pstIsp1->img_size[0].stWndRect.u32Height != pstIsp2->img_size[0].stWndRect.u32Height)
		return 1;

	return 0;
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
	CV4005_SENSOR_GET_CTX(ViPipe, pstSnsState);
	CMOS_CHECK_POINTER(pstSnsState);
	pstSnsRegsInfo = &pstSnsSyncInfo->snsCfg;
	pstCfg0 = &pstSnsState->astSyncInfo[0];
	pstCfg1 = &pstSnsState->astSyncInfo[1];
	pstI2c_data = pstCfg0->snsCfg.astI2cData;

	if ((pstSnsState->bSyncInit == CVI_FALSE) || (pstSnsRegsInfo->bConfig == CVI_FALSE)) {
		pstCfg0->snsCfg.enSnsType = SNS_I2C_TYPE;
		pstCfg0->snsCfg.unComBus.s8I2cDev = g_aunCV4005_BusInfo[ViPipe].s8I2cDev;
		pstCfg0->snsCfg.u8Cfg2ValidDelayMax = 0;
		pstCfg0->snsCfg.use_snsr_sram = CVI_TRUE;
		pstCfg0->snsCfg.u32RegNum = LINEAR_REGS_NUM;

		for (i = 0; i < pstCfg0->snsCfg.u32RegNum; i++) {
			pstI2c_data[i].bUpdate = CVI_TRUE;
			pstI2c_data[i].u8DevAddr = cv4005_i2c_addr;
			pstI2c_data[i].u32AddrByteNum = cv4005_addr_byte;
			pstI2c_data[i].u32DataByteNum = cv4005_data_byte;
		}

		pstI2c_data[LINEAR_EXP_0].u32RegAddr = CV4005_EXP1_ADDR0;
		pstI2c_data[LINEAR_EXP_1].u32RegAddr = CV4005_EXP1_ADDR1;
		pstI2c_data[LINEAR_EXP_2].u32RegAddr = CV4005_EXP1_ADDR2;
		pstI2c_data[LINEAR_AGAIN].u32RegAddr = CV4005_AGAIN1_ADDR;
		pstI2c_data[LINEAR_VTS_0].u32RegAddr = CV4005_VTS_ADDR0;
		pstI2c_data[LINEAR_VTS_1].u32RegAddr = CV4005_VTS_ADDR1;
		pstI2c_data[LINEAR_VTS_2].u32RegAddr = CV4005_VTS_ADDR2;
		pstI2c_data[LINEAR_FLIP_MIRROR].u32RegAddr = CV4005_FLIP_MIRROR_ADDR;
		pstSnsState->bSyncInit = CVI_TRUE;
		pstCfg0->snsCfg.need_update = CVI_TRUE;
		cmos_get_isp_size(ViPipe, &pstCfg0->ispCfg);
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

		pstCfg0->ispCfg.need_update = sensor_cmp_isp_size(&pstCfg0->ispCfg, &pstCfg1->ispCfg) ?
			CVI_TRUE : CVI_FALSE;
	}

	pstSnsRegsInfo->bConfig = CVI_FALSE;
	memcpy(pstSnsSyncInfo, &pstSnsState->astSyncInfo[0], sizeof(ISP_SNS_SYNC_INFO_S));
	memcpy(&pstSnsState->astSyncInfo[1], &pstSnsState->astSyncInfo[0], sizeof(ISP_SNS_SYNC_INFO_S));
	pstSnsState->au32FL[1] = pstSnsState->au32FL[0];

	pstCfg0->snsCfg.astI2cData[LINEAR_FLIP_MIRROR].bDropFrm = CVI_FALSE;

	return CVI_SUCCESS;
}

static CVI_S32 cmos_set_image_mode(VI_PIPE ViPipe, ISP_CMOS_SENSOR_IMAGE_MODE_S *pstSensorImageMode)
{
	CVI_U8 u8SensorImageMode = 0;
	ISP_SNS_STATE_S *pstSnsState = CVI_NULL;

	CMOS_CHECK_POINTER(pstSensorImageMode);
	CV4005_SENSOR_GET_CTX(ViPipe, pstSnsState);
	CMOS_CHECK_POINTER(pstSnsState);

	u8SensorImageMode = pstSnsState->u8ImgMode;
	pstSnsState->bSyncInit = CVI_FALSE;

	if (pstSensorImageMode->f32Fps <= 30 &&
	    CV4005_RES_IS_1440P(pstSensorImageMode->u16Width, pstSensorImageMode->u16Height))
		u8SensorImageMode = CV4005_MODE_2560X1440P30;
	else
		return CVI_FAILURE;

	if ((pstSnsState->bInit == CVI_TRUE) && (u8SensorImageMode == pstSnsState->u8ImgMode))
		return CVI_FAILURE;

	pstSnsState->u8ImgMode = u8SensorImageMode;
	return CVI_SUCCESS;
}

static CVI_VOID sensor_mirror_flip(VI_PIPE ViPipe, ISP_SNS_MIRRORFLIP_TYPE_E eSnsMirrorFlip)
{
	CVI_U8 flip_val = 0;
	CVI_U8 x_crop_sta = 4; /* X_CROP_STA, datasheet 0x3038/0x3039 */
	CVI_U8 y_crop_sta = 6; /* Y_DCROP_STA, datasheet 0x3034/0x3035 */

	ISP_SNS_STATE_S *pstSnsState = CVI_NULL;
	ISP_SNS_REGS_INFO_S *pstSnsRegsInfo = CVI_NULL;

	CV4005_SENSOR_GET_CTX(ViPipe, pstSnsState);
	CMOS_CHECK_POINTER_VOID(pstSnsState);

	pstSnsRegsInfo = &pstSnsState->astSyncInfo[0].snsCfg;

	if (pstSnsState->bInit == CVI_TRUE && g_aeCV4005_MirrorFip[ViPipe] != eSnsMirrorFlip) {
		switch (eSnsMirrorFlip) {
		case ISP_SNS_NORMAL:
			flip_val = 0x0;
			x_crop_sta = 4;
			y_crop_sta = 6;
			break;
		case ISP_SNS_MIRROR:
			flip_val = 0x1;
			x_crop_sta = 5;
			y_crop_sta = 6;
			break;
		case ISP_SNS_FLIP:
			flip_val = 0x2;
			x_crop_sta = 4;
			y_crop_sta = 7;
			break;
		case ISP_SNS_MIRROR_FLIP:
			flip_val = 0x3;
			x_crop_sta = 5;
			y_crop_sta = 7;
			break;
		default:
			return;
		}

		pstSnsRegsInfo->astI2cData[LINEAR_FLIP_MIRROR].u32Data = flip_val;
		pstSnsRegsInfo->astI2cData[LINEAR_FLIP_MIRROR].bDropFrm = 1;
		pstSnsRegsInfo->astI2cData[LINEAR_FLIP_MIRROR].u8DropFrmNum = 2;

		/* shift crop start by 1px to keep Bayer phase after mirror/flip */
		cv4005_write_register(ViPipe, 0x3034, y_crop_sta);
		cv4005_write_register(ViPipe, 0x3035, 0x00);
		cv4005_write_register(ViPipe, 0x3038, x_crop_sta);
		cv4005_write_register(ViPipe, 0x3039, 0x00);

		g_aeCV4005_MirrorFip[ViPipe] = eSnsMirrorFlip;
	}
}

static CVI_VOID sensor_global_init(VI_PIPE ViPipe)
{
	ISP_SNS_STATE_S *pstSnsState = CVI_NULL;

	CV4005_SENSOR_GET_CTX(ViPipe, pstSnsState);
	CMOS_CHECK_POINTER_VOID(pstSnsState);

	pstSnsState->bInit = CVI_FALSE;
	pstSnsState->bSyncInit = CVI_FALSE;
	pstSnsState->u8ImgMode = CV4005_MODE_2560X1440P30;
	pstSnsState->u32FLStd  = g_astCV4005_mode[pstSnsState->u8ImgMode].u32VtsDef;
	pstSnsState->au32FL[0] = g_astCV4005_mode[pstSnsState->u8ImgMode].u32VtsDef;
	pstSnsState->au32FL[1] = g_astCV4005_mode[pstSnsState->u8ImgMode].u32VtsDef;

	memset(&pstSnsState->astSyncInfo[0], 0, sizeof(ISP_SNS_SYNC_INFO_S));
	memset(&pstSnsState->astSyncInfo[1], 0, sizeof(ISP_SNS_SYNC_INFO_S));
}

static CVI_S32 sensor_rx_attr(VI_PIPE ViPipe, SNS_COMBO_DEV_ATTR_S *pstRxAttr)
{
	ISP_SNS_STATE_S *pstSnsState = CVI_NULL;

	CV4005_SENSOR_GET_CTX(ViPipe, pstSnsState);
	CMOS_CHECK_POINTER(pstSnsState);
	CMOS_CHECK_POINTER(pstRxAttr);

	memcpy(pstRxAttr, &cv4005_rx_attr, sizeof(*pstRxAttr));

	pstRxAttr->img_size.width = g_astCV4005_mode[pstSnsState->u8ImgMode].astImg[0].stSnsSize.u32Width;
	pstRxAttr->img_size.height = g_astCV4005_mode[pstSnsState->u8ImgMode].astImg[0].stSnsSize.u32Height;
	pstRxAttr->mipi_attr.wdr_mode = CVI_MIPI_WDR_MODE_NONE;

	return CVI_SUCCESS;

}

static CVI_S32 sensor_patch_rx_attr(RX_INIT_ATTR_S *pstRxInitAttr)
{
	SNS_COMBO_DEV_ATTR_S *pstRxAttr = &cv4005_rx_attr;
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

	pstSensorExpFunc->pfn_cmos_sensor_init = cv4005_init;
	pstSensorExpFunc->pfn_cmos_sensor_exit = cv4005_exit;
	pstSensorExpFunc->pfn_cmos_sensor_global_init = sensor_global_init;
	pstSensorExpFunc->pfn_cmos_set_image_mode = cmos_set_image_mode;
	pstSensorExpFunc->pfn_cmos_get_isp_default = cmos_get_isp_default;
	pstSensorExpFunc->pfn_cmos_get_isp_black_level = cmos_get_blc_default;
	pstSensorExpFunc->pfn_cmos_get_sns_reg_info = cmos_get_sns_regs_info;

	return CVI_SUCCESS;
}

/****************************************************************************
 * callback structure                                                       *
 ****************************************************************************/
static CVI_VOID sensor_patch_i2c_addr(CVI_S32 s32I2cAddr)
{
	if (CV4005_I2C_ADDR_IS_VALID(s32I2cAddr))
		cv4005_i2c_addr = s32I2cAddr;
}

static CVI_S32 cv4005_set_bus_info(VI_PIPE ViPipe, ISP_SNS_COMMBUS_U unSNSBusInfo)
{
	g_aunCV4005_BusInfo[ViPipe].s8I2cDev = unSNSBusInfo.s8I2cDev;

	return CVI_SUCCESS;
}

static CVI_S32 sensor_ctx_init(VI_PIPE ViPipe)
{
	ISP_SNS_STATE_S *pastSnsStateCtx = CVI_NULL;

	CV4005_SENSOR_GET_CTX(ViPipe, pastSnsStateCtx);

	if (pastSnsStateCtx == CVI_NULL) {
		pastSnsStateCtx = (ISP_SNS_STATE_S *)malloc(sizeof(ISP_SNS_STATE_S));
		if (pastSnsStateCtx == CVI_NULL) {
			CVI_TRACE_SNS(CVI_DBG_ERR, "Isp[%d] SnsCtx malloc memory failed!\n", ViPipe);
			return -ENOMEM;
		}
	}

	memset(pastSnsStateCtx, 0, sizeof(ISP_SNS_STATE_S));

	CV4005_SENSOR_SET_CTX(ViPipe, pastSnsStateCtx);

	return CVI_SUCCESS;
}

static CVI_VOID sensor_ctx_exit(VI_PIPE ViPipe)
{
	ISP_SNS_STATE_S *pastSnsStateCtx = CVI_NULL;

	CV4005_SENSOR_GET_CTX(ViPipe, pastSnsStateCtx);
	SENSOR_FREE(pastSnsStateCtx);
	CV4005_SENSOR_RESET_CTX(ViPipe);
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

	stSnsAttrInfo.eSensorId = CV4005_ID;

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

	s32Ret = CVI_ISP_SensorUnRegCallBack(ViPipe, CV4005_ID);
	if (s32Ret != CVI_SUCCESS) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "sensor unregister callback function failed!\n");
		return s32Ret;
	}

	s32Ret = CVI_AE_SensorUnRegCallBack(ViPipe, pstAeLib, CV4005_ID);
	if (s32Ret != CVI_SUCCESS) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "sensor unregister callback function to ae lib failed!\n");
		return s32Ret;
	}

	s32Ret = CVI_AWB_SensorUnRegCallBack(ViPipe, pstAwbLib, CV4005_ID);
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
	g_au16CV4005_GainMode[ViPipe] = pstInitAttr->enGainMode;

	return CVI_SUCCESS;
}

static CVI_S32 sensor_probe(VI_PIPE ViPipe)
{
	return cv4005_probe(ViPipe);
}

ISP_SNS_OBJ_S stSnsCV4005_Obj = {
	.pfnRegisterCallback    = sensor_register_callback,
	.pfnUnRegisterCallback  = sensor_unregister_callback,
	.pfnStandby             = cv4005_standby,
	.pfnRestart             = cv4005_restart,
	.pfnWriteReg            = cv4005_write_register,
	.pfnReadReg             = cv4005_read_register,
	.pfnSetBusInfo          = cv4005_set_bus_info,
	.pfnSetInit             = sensor_set_init,
	.pfnMirrorFlip          = sensor_mirror_flip,
	.pfnPatchRxAttr         = sensor_patch_rx_attr,
	.pfnPatchI2cAddr        = sensor_patch_i2c_addr,
	.pfnGetRxAttr           = sensor_rx_attr,
	.pfnExpSensorCb         = cmos_init_sensor_exp_function,
	.pfnExpAeCb             = cmos_init_ae_exp_function,
	.pfnSnsProbe            = sensor_probe,
};
