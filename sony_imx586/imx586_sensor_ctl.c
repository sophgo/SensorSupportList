#include <unistd.h>
#include <cvi_comm_video.h>
#include "cvi_debug.h"
#include "cvi_sns_ctrl.h"
#include "imx586_cmos_ex.h"
#include "sensor_i2c.h"

static void imx586_init_setting(VI_PIPE ViPipe);
static void imx586_linear_12M30_init(VI_PIPE ViPipe);
static void imx586_linear_8M60_init(VI_PIPE ViPipe);
static void imx586_linear_2M120_init(VI_PIPE ViPipe);

CVI_U8 imx586_i2c_addr = 0x1a;
static int g_init_error[VI_MAX_PIPE_NUM];
static CVI_BOOL g_init_in_progress[VI_MAX_PIPE_NUM];
const CVI_U32 imx586_addr_byte = 2;
const CVI_U32 imx586_data_byte = 1;

int imx586_i2c_init(VI_PIPE ViPipe)
{
	return sensor_i2c_init(ViPipe, (CVI_U8)g_aunImx586_BusInfo[ViPipe].s8I2cDev,
							(CVI_U8)g_aunImx586_AddrInfo[ViPipe].s8I2cAddr);
}

int imx586_i2c_exit(VI_PIPE ViPipe)
{
	return sensor_i2c_exit(ViPipe, (CVI_U8)g_aunImx586_BusInfo[ViPipe].s8I2cDev);
}

int imx586_read_register(VI_PIPE ViPipe, int addr)
{
	return sensor_i2c_read(ViPipe, (CVI_U8)g_aunImx586_BusInfo[ViPipe].s8I2cDev,
							(CVI_U8)g_aunImx586_AddrInfo[ViPipe].s8I2cAddr, (CVI_U32)addr,
							imx586_addr_byte, imx586_data_byte);
}

int imx586_write_register(VI_PIPE ViPipe, int addr, int data)
{
	/* Keep the first I2C error until initialization completes. */
	int ret;

	if (g_init_in_progress[ViPipe] && g_init_error[ViPipe] < 0)
		return g_init_error[ViPipe];
	ret = sensor_i2c_write(ViPipe, (CVI_U8)g_aunImx586_BusInfo[ViPipe].s8I2cDev,
							(CVI_U8)g_aunImx586_AddrInfo[ViPipe].s8I2cAddr, (CVI_U32)addr,
							imx586_addr_byte, (CVI_U32)data, imx586_data_byte);
	if (g_init_in_progress[ViPipe] && ret < 0)
		g_init_error[ViPipe] = ret;
	return ret;
}

static void delay_ms(int ms)
{
	usleep(ms * 1000);
}

/* Sensor ID: 0x0016/0x0017 = 0x05/0x86. */
int imx586_probe(VI_PIPE ViPipe)
{
	int ret;
	int id_h;
	int id_l;

	delay_ms(2);
	ret = imx586_i2c_init(ViPipe);
	if (ret != CVI_SUCCESS)
		return ret;
	id_h = imx586_read_register(ViPipe, 0x0016);
	id_l = imx586_read_register(ViPipe, 0x0017);
	if (id_h != 0x05 || id_l != 0x86) {
		CVI_TRACE_SNS(CVI_DBG_ERR,
			"IMX586 probe failed: pipe=%d, bus=%d, addr=0x%02x, id_h=0x%x, id_l=0x%x\n",
			ViPipe, g_aunImx586_BusInfo[ViPipe].s8I2cDev,
			g_aunImx586_AddrInfo[ViPipe].s8I2cAddr, id_h, id_l);
		imx586_i2c_exit(ViPipe);
		return CVI_FAILURE;
	}
	return CVI_SUCCESS;
}

void imx586_standby(VI_PIPE ViPipe)
{
	imx586_write_register(ViPipe, 0x0100, 0x00);
}

void imx586_restart(VI_PIPE ViPipe)
{
	imx586_write_register(ViPipe, 0x0100, 0x01);
	delay_ms(20);
}

void imx586_default_reg_init(VI_PIPE ViPipe)
{
	CVI_U32 i;

	for (i = 0; i < g_pastImx586[ViPipe]->astSyncInfo[0].snsCfg.u32RegNum; i++) {
		imx586_write_register(ViPipe,
				g_pastImx586[ViPipe]->astSyncInfo[0].snsCfg.astI2cData[i].u32RegAddr,
				g_pastImx586[ViPipe]->astSyncInfo[0].snsCfg.astI2cData[i].u32Data);
	}
}

void imx586_mirror_flip(VI_PIPE ViPipe, ISP_SNS_MIRRORFLIP_TYPE_E eSnsMirrorFlip)
{
	CVI_U8 value = 0x0;
	CVI_U8 start_x = 0;
	CVI_U8 start_y = 0;
	ISP_SNS_STATE_S *pstSnsState = CVI_NULL;
	ISP_SNS_REGS_INFO_S *pstSnsRegsInfo = CVI_NULL;
	ISP_SNS_ISP_INFO_S *pstIspCfg0 = CVI_NULL;

	pstSnsState = g_pastImx586[ViPipe];
	CMOS_CHECK_POINTER_VOID(pstSnsState);

	pstSnsRegsInfo = &pstSnsState->astSyncInfo[0].snsCfg;
	pstIspCfg0 = &pstSnsState->astSyncInfo[0].ispCfg;

	switch (eSnsMirrorFlip) {
	case ISP_SNS_NORMAL:
		value = 0x0;
		start_x = 0;
		start_y = 0;
		break;
	case ISP_SNS_MIRROR:
		value = 0x1;
		start_x = 1;
		start_y = 0;
		break;
	case ISP_SNS_FLIP:
		value = 0x2;
		start_x = 0;
		start_y = 1;
		break;
	case ISP_SNS_MIRROR_FLIP:
		value = 0x3;
		start_x = 2;
		start_y = 2;
		break;
	default:
		return;
	}

	if (pstSnsState->enWDRMode == WDR_MODE_NONE) {
		pstSnsRegsInfo->astI2cData[LINEAR_FLIP_MIRROR].u32Data = value;
		pstSnsRegsInfo->astI2cData[LINEAR_FLIP_MIRROR].bDropFrm = 1;
		pstSnsRegsInfo->astI2cData[LINEAR_FLIP_MIRROR].u8DropFrmNum = 2;
	}

	pstIspCfg0->img_size[0].stWndRect.s32X = start_x;
	pstIspCfg0->img_size[0].stWndRect.s32Y = start_y;
}

void imx586_init(VI_PIPE ViPipe)
{
	CVI_U8 u8ImgMode;
	int ret;

	/* Validate initialization and streaming before reporting success. */
	CMOS_CHECK_POINTER_VOID(g_pastImx586[ViPipe]);
	g_pastImx586[ViPipe]->bInit = CVI_FALSE;
	u8ImgMode = g_pastImx586[ViPipe]->u8ImgMode;

	ret = imx586_probe(ViPipe);
	if (ret != CVI_SUCCESS)
		return;
	delay_ms(10);
	g_init_error[ViPipe] = CVI_SUCCESS;
	g_init_in_progress[ViPipe] = CVI_TRUE;
	imx586_init_setting(ViPipe);
	if (u8ImgMode == IMX586_MODE_12M30)
		imx586_linear_12M30_init(ViPipe);
	else if (u8ImgMode == IMX586_MODE_8M60)
		imx586_linear_8M60_init(ViPipe);
	else if (u8ImgMode == IMX586_MODE_2M120)
		imx586_linear_2M120_init(ViPipe);
	else
		g_init_error[ViPipe] = CVI_FAILURE;

	g_init_in_progress[ViPipe] = CVI_FALSE;
	ret = g_init_error[ViPipe];
	if (ret == CVI_SUCCESS) {
		ret = imx586_read_register(ViPipe, 0x0100);
		if (ret == 1)
			ret = CVI_SUCCESS;
		else
			ret = CVI_FAILURE;
	}
	if (ret != CVI_SUCCESS) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "IMX586 init failed: pipe=%d, mode=%d, ret=%d\n",
			ViPipe, u8ImgMode, ret);
		imx586_i2c_exit(ViPipe);
		return;
	}
	g_pastImx586[ViPipe]->bInit = CVI_TRUE;
	switch (u8ImgMode) {
	case IMX586_MODE_12M30:
		printf("ViPipe:%d,===IMX586 12M 30fps 10bit LINEAR Init OK!===\n", ViPipe);
		break;
	case IMX586_MODE_8M60:
		printf("ViPipe:%d,===IMX586 3840x2160 60fps 10bit LINEAR Init OK!===\n", ViPipe);
		break;
	case IMX586_MODE_2M120:
		printf("ViPipe:%d,===IMX586 2M 120fps 10bit LINEAR Init OK!===\n", ViPipe);
		break;
	default:
		break;
	}
}

static void imx586_init_setting(VI_PIPE ViPipe)
{
    imx586_write_register(ViPipe, 0x0136, 0x18);
    imx586_write_register(ViPipe, 0x0137, 0x00);
    imx586_write_register(ViPipe, 0x3C7E, 0x0A);
    imx586_write_register(ViPipe, 0x3C7F, 0x0A);
    imx586_write_register(ViPipe, 0x0111, 0x02);
    imx586_write_register(ViPipe, 0x380C, 0x00);
    imx586_write_register(ViPipe, 0x3C00, 0x10);
    imx586_write_register(ViPipe, 0x3C01, 0x10);
    imx586_write_register(ViPipe, 0x3C02, 0x10);
    imx586_write_register(ViPipe, 0x3C03, 0x10);
    imx586_write_register(ViPipe, 0x3C04, 0x10);
    imx586_write_register(ViPipe, 0x3C05, 0x01);
    imx586_write_register(ViPipe, 0x3C06, 0x00);
    imx586_write_register(ViPipe, 0x3C07, 0x00);
    imx586_write_register(ViPipe, 0x3C08, 0x03);
    imx586_write_register(ViPipe, 0x3C09, 0xFF);
    imx586_write_register(ViPipe, 0x3C0A, 0x01);
    imx586_write_register(ViPipe, 0x3C0B, 0x00);
    imx586_write_register(ViPipe, 0x3C0C, 0x00);
    imx586_write_register(ViPipe, 0x3C0D, 0x03);
    imx586_write_register(ViPipe, 0x3C0E, 0xFF);
    imx586_write_register(ViPipe, 0x3C0F, 0x20);
    imx586_write_register(ViPipe, 0x3F88, 0x00);
    imx586_write_register(ViPipe, 0x3F8E, 0x00);
    imx586_write_register(ViPipe, 0x5282, 0x01);
    imx586_write_register(ViPipe, 0x9004, 0x14);
    imx586_write_register(ViPipe, 0x9200, 0xF4);
    imx586_write_register(ViPipe, 0x9201, 0xA7);
    imx586_write_register(ViPipe, 0x9202, 0xF4);
    imx586_write_register(ViPipe, 0x9203, 0xAA);
    imx586_write_register(ViPipe, 0x9204, 0xF4);
    imx586_write_register(ViPipe, 0x9205, 0xAD);
    imx586_write_register(ViPipe, 0x9206, 0xF4);
    imx586_write_register(ViPipe, 0x9207, 0xB0);
    imx586_write_register(ViPipe, 0x9208, 0xF4);
    imx586_write_register(ViPipe, 0x9209, 0xB3);
    imx586_write_register(ViPipe, 0x920A, 0xB7);
    imx586_write_register(ViPipe, 0x920B, 0x34);
    imx586_write_register(ViPipe, 0x920C, 0xB7);
    imx586_write_register(ViPipe, 0x920D, 0x36);
    imx586_write_register(ViPipe, 0x920E, 0xB7);
    imx586_write_register(ViPipe, 0x920F, 0x37);
    imx586_write_register(ViPipe, 0x9210, 0xB7);
    imx586_write_register(ViPipe, 0x9211, 0x38);
    imx586_write_register(ViPipe, 0x9212, 0xB7);
    imx586_write_register(ViPipe, 0x9213, 0x39);
    imx586_write_register(ViPipe, 0x9214, 0xB7);
    imx586_write_register(ViPipe, 0x9215, 0x3A);
    imx586_write_register(ViPipe, 0x9216, 0xB7);
    imx586_write_register(ViPipe, 0x9217, 0x3C);
    imx586_write_register(ViPipe, 0x9218, 0xB7);
    imx586_write_register(ViPipe, 0x9219, 0x3D);
    imx586_write_register(ViPipe, 0x921A, 0xB7);
    imx586_write_register(ViPipe, 0x921B, 0x3E);
    imx586_write_register(ViPipe, 0x921C, 0xB7);
    imx586_write_register(ViPipe, 0x921D, 0x3F);
    imx586_write_register(ViPipe, 0x921E, 0x77);
    imx586_write_register(ViPipe, 0x921F, 0x77);
    imx586_write_register(ViPipe, 0x9222, 0xC4);
    imx586_write_register(ViPipe, 0x9223, 0x4B);
    imx586_write_register(ViPipe, 0x9224, 0xC4);
    imx586_write_register(ViPipe, 0x9225, 0x4C);
    imx586_write_register(ViPipe, 0x9226, 0xC4);
    imx586_write_register(ViPipe, 0x9227, 0x4D);
    imx586_write_register(ViPipe, 0x9810, 0x14);
    imx586_write_register(ViPipe, 0x9814, 0x14);
    imx586_write_register(ViPipe, 0x99B2, 0x20);
    imx586_write_register(ViPipe, 0x99B3, 0x0F);
    imx586_write_register(ViPipe, 0x99B4, 0x0F);
    imx586_write_register(ViPipe, 0x99B5, 0x0F);
    imx586_write_register(ViPipe, 0x99B6, 0x0F);
    imx586_write_register(ViPipe, 0x99E4, 0x0F);
    imx586_write_register(ViPipe, 0x99E5, 0x0F);
    imx586_write_register(ViPipe, 0x99E6, 0x0F);
    imx586_write_register(ViPipe, 0x99E7, 0x0F);
    imx586_write_register(ViPipe, 0x99E8, 0x0F);
    imx586_write_register(ViPipe, 0x99E9, 0x0F);
    imx586_write_register(ViPipe, 0x99EA, 0x0F);
    imx586_write_register(ViPipe, 0x99EB, 0x0F);
    imx586_write_register(ViPipe, 0x99EC, 0x0F);
    imx586_write_register(ViPipe, 0x99ED, 0x0F);
    imx586_write_register(ViPipe, 0xA569, 0x06);
    imx586_write_register(ViPipe, 0xA56A, 0x13);
    imx586_write_register(ViPipe, 0xA56B, 0x13);
    imx586_write_register(ViPipe, 0xA679, 0x20);
    imx586_write_register(ViPipe, 0xA830, 0x68);
    imx586_write_register(ViPipe, 0xA831, 0x56);
    imx586_write_register(ViPipe, 0xA832, 0x2B);
    imx586_write_register(ViPipe, 0xA833, 0x55);
    imx586_write_register(ViPipe, 0xA834, 0x55);
    imx586_write_register(ViPipe, 0xA835, 0x16);
    imx586_write_register(ViPipe, 0xA837, 0x51);
    imx586_write_register(ViPipe, 0xA838, 0x34);
    imx586_write_register(ViPipe, 0xA854, 0x4F);
    imx586_write_register(ViPipe, 0xA855, 0x48);
    imx586_write_register(ViPipe, 0xA856, 0x45);
    imx586_write_register(ViPipe, 0xA857, 0x02);
    imx586_write_register(ViPipe, 0xA85A, 0x23);
    imx586_write_register(ViPipe, 0xA85B, 0x16);
    imx586_write_register(ViPipe, 0xA85C, 0x12);
    imx586_write_register(ViPipe, 0xA85D, 0x02);
    imx586_write_register(ViPipe, 0xAC72, 0x01);
    imx586_write_register(ViPipe, 0xAC73, 0x26);
    imx586_write_register(ViPipe, 0xAC74, 0x01);
    imx586_write_register(ViPipe, 0xAC75, 0x26);
    imx586_write_register(ViPipe, 0xAC76, 0x00);
    imx586_write_register(ViPipe, 0xAC77, 0xC4);
    imx586_write_register(ViPipe, 0xB051, 0x02);
    imx586_write_register(ViPipe, 0xC020, 0x01);
    imx586_write_register(ViPipe, 0xC61D, 0x00);
    imx586_write_register(ViPipe, 0xC625, 0x00);
    imx586_write_register(ViPipe, 0xC638, 0x03);
    imx586_write_register(ViPipe, 0xC63B, 0x01);
    imx586_write_register(ViPipe, 0xE286, 0x31);
    imx586_write_register(ViPipe, 0xE2A6, 0x32);
    imx586_write_register(ViPipe, 0xE2C6, 0x33);
    imx586_write_register(ViPipe, 0xEA4B, 0x00);
    imx586_write_register(ViPipe, 0xEA4C, 0x00);
    imx586_write_register(ViPipe, 0xEA4D, 0x00);
    imx586_write_register(ViPipe, 0xEA4E, 0x00);
    imx586_write_register(ViPipe, 0xF000, 0x00);
    imx586_write_register(ViPipe, 0xF001, 0x10);
    imx586_write_register(ViPipe, 0xF00C, 0x00);
    imx586_write_register(ViPipe, 0xF00D, 0x40);
    imx586_write_register(ViPipe, 0xF030, 0x00);
    imx586_write_register(ViPipe, 0xF031, 0x10);
    imx586_write_register(ViPipe, 0xF03C, 0x00);
    imx586_write_register(ViPipe, 0xF03D, 0x40);
    imx586_write_register(ViPipe, 0xF44B, 0x80);
    imx586_write_register(ViPipe, 0xF44C, 0x10);
    imx586_write_register(ViPipe, 0xF44D, 0x06);
    imx586_write_register(ViPipe, 0xF44E, 0x80);
    imx586_write_register(ViPipe, 0xF44F, 0x10);
    imx586_write_register(ViPipe, 0xF450, 0x06);
    imx586_write_register(ViPipe, 0xF451, 0x80);
    imx586_write_register(ViPipe, 0xF452, 0x10);
    imx586_write_register(ViPipe, 0xF453, 0x06);
    imx586_write_register(ViPipe, 0xF454, 0x80);
    imx586_write_register(ViPipe, 0xF455, 0x10);
    imx586_write_register(ViPipe, 0xF456, 0x06);
    imx586_write_register(ViPipe, 0xF457, 0x80);
    imx586_write_register(ViPipe, 0xF458, 0x10);
    imx586_write_register(ViPipe, 0xF459, 0x06);
    imx586_write_register(ViPipe, 0xF478, 0x20);
    imx586_write_register(ViPipe, 0xF479, 0x80);
    imx586_write_register(ViPipe, 0xF47A, 0x80);
    imx586_write_register(ViPipe, 0xF47B, 0x20);
    imx586_write_register(ViPipe, 0xF47C, 0x80);
    imx586_write_register(ViPipe, 0xF47D, 0x80);
    imx586_write_register(ViPipe, 0xF47E, 0x20);
    imx586_write_register(ViPipe, 0xF47F, 0x80);
    imx586_write_register(ViPipe, 0xF480, 0x80);
    imx586_write_register(ViPipe, 0xF481, 0x20);
    imx586_write_register(ViPipe, 0xF482, 0x60);
    imx586_write_register(ViPipe, 0xF483, 0x80);
    imx586_write_register(ViPipe, 0xF484, 0x20);
    imx586_write_register(ViPipe, 0xF485, 0x60);
    imx586_write_register(ViPipe, 0xF486, 0x80);
    imx586_write_register(ViPipe, 0x9852, 0x00);
    imx586_write_register(ViPipe, 0x9954, 0x0F);
    imx586_write_register(ViPipe, 0xA7AD, 0x01);
    imx586_write_register(ViPipe, 0xA7CB, 0x01);
    imx586_write_register(ViPipe, 0xAE09, 0xFF);
    imx586_write_register(ViPipe, 0xAE0A, 0xFF);
    imx586_write_register(ViPipe, 0xAE12, 0x58);
    imx586_write_register(ViPipe, 0xAE13, 0x58);
    imx586_write_register(ViPipe, 0xAE15, 0x10);
    imx586_write_register(ViPipe, 0xAE16, 0x10);
    imx586_write_register(ViPipe, 0xAF05, 0x48);
    imx586_write_register(ViPipe, 0xB07C, 0x02);
}

static void imx586_linear_12M30_init(VI_PIPE ViPipe)
{
    imx586_write_register(ViPipe, 0x0112, 0x0A);
    imx586_write_register(ViPipe, 0x0113, 0x0A);
    imx586_write_register(ViPipe, 0x0114, 0x03);
    imx586_write_register(ViPipe, 0x0342, 0x23);
    imx586_write_register(ViPipe, 0x0343, 0x10);
    imx586_write_register(ViPipe, 0x0340, 0x0B);
    imx586_write_register(ViPipe, 0x0341, 0xF8);
    imx586_write_register(ViPipe, 0x0344, 0x00);
    imx586_write_register(ViPipe, 0x0345, 0x00);
    imx586_write_register(ViPipe, 0x0346, 0x00);
    imx586_write_register(ViPipe, 0x0347, 0x00);
    imx586_write_register(ViPipe, 0x0348, 0x1F);
    imx586_write_register(ViPipe, 0x0349, 0x3F);
    imx586_write_register(ViPipe, 0x034A, 0x17);
    imx586_write_register(ViPipe, 0x034B, 0x6F);
    imx586_write_register(ViPipe, 0x0220, 0x62);
    imx586_write_register(ViPipe, 0x0222, 0x01);
    imx586_write_register(ViPipe, 0x0900, 0x01);
    imx586_write_register(ViPipe, 0x0901, 0x22);
    imx586_write_register(ViPipe, 0x0902, 0x08);
    imx586_write_register(ViPipe, 0x3140, 0x00);
    imx586_write_register(ViPipe, 0x3246, 0x81);
    imx586_write_register(ViPipe, 0x3247, 0x81);
    imx586_write_register(ViPipe, 0x3F15, 0x00);
    imx586_write_register(ViPipe, 0x0401, 0x00);
    imx586_write_register(ViPipe, 0x0404, 0x00);
    imx586_write_register(ViPipe, 0x0405, 0x10);
    imx586_write_register(ViPipe, 0x0408, 0x00);
    imx586_write_register(ViPipe, 0x0409, 0x00);
    imx586_write_register(ViPipe, 0x040A, 0x00);
    imx586_write_register(ViPipe, 0x040B, 0x00);
    imx586_write_register(ViPipe, 0x040C, 0x0F);
    imx586_write_register(ViPipe, 0x040D, 0xA0);
    imx586_write_register(ViPipe, 0x040E, 0x0B);
    imx586_write_register(ViPipe, 0x040F, 0xB8);
    imx586_write_register(ViPipe, 0x034C, 0x0F);
    imx586_write_register(ViPipe, 0x034D, 0xA0);
    imx586_write_register(ViPipe, 0x034E, 0x0B);
    imx586_write_register(ViPipe, 0x034F, 0xB8);
    imx586_write_register(ViPipe, 0x0301, 0x05);
    imx586_write_register(ViPipe, 0x0303, 0x04);
    imx586_write_register(ViPipe, 0x0305, 0x04);
    imx586_write_register(ViPipe, 0x0306, 0x01);
    imx586_write_register(ViPipe, 0x0307, 0x58);
    imx586_write_register(ViPipe, 0x030B, 0x02);
    imx586_write_register(ViPipe, 0x030D, 0x03);
    imx586_write_register(ViPipe, 0x030E, 0x01);
    imx586_write_register(ViPipe, 0x030F, 0x1F);
    imx586_write_register(ViPipe, 0x0310, 0x01);
    imx586_write_register(ViPipe, 0x3620, 0x00);
    imx586_write_register(ViPipe, 0x3621, 0x00);
    imx586_write_register(ViPipe, 0x3C11, 0x04);
    imx586_write_register(ViPipe, 0x3C12, 0x03);
    imx586_write_register(ViPipe, 0x3C13, 0x2D);
    imx586_write_register(ViPipe, 0x3F0C, 0x01);
    imx586_write_register(ViPipe, 0x3F14, 0x00);
    imx586_write_register(ViPipe, 0x3F80, 0x00);
    imx586_write_register(ViPipe, 0x3F81, 0x14);
    imx586_write_register(ViPipe, 0x3F8C, 0x00);
    imx586_write_register(ViPipe, 0x3F8D, 0x14);
    imx586_write_register(ViPipe, 0x3FF8, 0x00);
    imx586_write_register(ViPipe, 0x3FF9, 0x3C);
    imx586_write_register(ViPipe, 0x3FFE, 0x01);
    imx586_write_register(ViPipe, 0x3FFF, 0x8C);
    imx586_write_register(ViPipe, 0x0202, 0x0B);
    imx586_write_register(ViPipe, 0x0203, 0xC8);
    imx586_write_register(ViPipe, 0x0224, 0x01);
    imx586_write_register(ViPipe, 0x0225, 0xF4);
    imx586_write_register(ViPipe, 0x3FE0, 0x01);
    imx586_write_register(ViPipe, 0x3FE1, 0xF4);
    imx586_write_register(ViPipe, 0x0204, 0x00);
    imx586_write_register(ViPipe, 0x0205, 0x70);
    imx586_write_register(ViPipe, 0x0216, 0x00);
    imx586_write_register(ViPipe, 0x0217, 0x70);
    imx586_write_register(ViPipe, 0x0218, 0x01);
    imx586_write_register(ViPipe, 0x0219, 0x00);
    imx586_write_register(ViPipe, 0x020E, 0x01);
    imx586_write_register(ViPipe, 0x020F, 0x00);
    imx586_write_register(ViPipe, 0x0210, 0x01);
    imx586_write_register(ViPipe, 0x0211, 0x00);
    imx586_write_register(ViPipe, 0x0212, 0x01);
    imx586_write_register(ViPipe, 0x0213, 0x00);
    imx586_write_register(ViPipe, 0x0214, 0x01);
    imx586_write_register(ViPipe, 0x0215, 0x00);
    imx586_write_register(ViPipe, 0x3FE2, 0x00);
    imx586_write_register(ViPipe, 0x3FE3, 0x70);
    imx586_write_register(ViPipe, 0x3FE4, 0x01);
    imx586_write_register(ViPipe, 0x3FE5, 0x00);
    imx586_write_register(ViPipe, 0x323B, 0x00);
    imx586_write_register(ViPipe, 0x323C, 0x00);
    imx586_write_register(ViPipe, 0x3E20, 0x02);
    imx586_write_register(ViPipe, 0x3E3B, 0x01);
    imx586_write_register(ViPipe, 0x4434, 0x01);
    imx586_write_register(ViPipe, 0x4435, 0xF0);
    imx586_write_register(ViPipe, 0x0100, 0x01);// - Streaming
    imx586_default_reg_init(ViPipe);

    delay_ms(200);
    imx586_write_register(ViPipe, 0x0100, 0x01);// - Streaming
}

static void imx586_linear_8M60_init(VI_PIPE ViPipe)
{
    imx586_write_register(ViPipe, 0x0112, 0x0A); /* CSI_DT_FMT_H */
    imx586_write_register(ViPipe, 0x0113, 0x0A); /* CSI_DT_FMT_L */
    imx586_write_register(ViPipe, 0x0114, 0x03); /* CSI_LANE_MODE */

    imx586_write_register(ViPipe, 0x0342, 0x23); /* LINE_LENGTH_PCK = 0x2310 = 8976 */
    imx586_write_register(ViPipe, 0x0343, 0x10);
    imx586_write_register(ViPipe, 0x0340, 0x09); /* FRM_LENGTH_LINES = 0x090C = 2316 */
    imx586_write_register(ViPipe, 0x0341, 0x0C);

    imx586_write_register(ViPipe, 0x0344, 0x00); /* X_ADD_STA = 0 */
    imx586_write_register(ViPipe, 0x0345, 0x00);
    imx586_write_register(ViPipe, 0x0346, 0x02); /* Y_ADD_STA = 0x02E8 = 744 */
    imx586_write_register(ViPipe, 0x0347, 0xE8);
    imx586_write_register(ViPipe, 0x0348, 0x1F); /* X_ADD_END = 0x1F3F = 7999 */
    imx586_write_register(ViPipe, 0x0349, 0x3F);
    imx586_write_register(ViPipe, 0x034A, 0x14); /* Y_ADD_END = 0x1487 = 5255 */
    imx586_write_register(ViPipe, 0x034B, 0x87);

    imx586_write_register(ViPipe, 0x0220, 0x62); /* HDR_MODE = linear */
    imx586_write_register(ViPipe, 0x0222, 0x01); /* EXPO_RATIO */

    imx586_write_register(ViPipe, 0x0900, 0x01); /* BINNING_MODE */
    imx586_write_register(ViPipe, 0x0901, 0x22); /* 2x2 binning */
    imx586_write_register(ViPipe, 0x0902, 0x08); /* BINNING_WEIGHTING */

    imx586_write_register(ViPipe, 0x3140, 0x00);
    imx586_write_register(ViPipe, 0x3246, 0x81);
    imx586_write_register(ViPipe, 0x3247, 0x81);
    imx586_write_register(ViPipe, 0x3F15, 0x00);

    imx586_write_register(ViPipe, 0x0401, 0x00); /* SCALE_MODE */
    imx586_write_register(ViPipe, 0x0404, 0x00); /* SCALE_M[8] */
    imx586_write_register(ViPipe, 0x0405, 0x10); /* SCALE_M[7:0] */
    imx586_write_register(ViPipe, 0x0408, 0x00); /* DIG_CROP_X_OFFSET[12:8] */
    imx586_write_register(ViPipe, 0x0409, 0x4C); /* DIG_CROP_X_OFFSET[7:0] = 0x4C = 76 (centered for 3848 crop) */
    imx586_write_register(ViPipe, 0x040A, 0x00); /* DIG_CROP_Y_OFFSET[12:8] */
    imx586_write_register(ViPipe, 0x040B, 0x2C); /* DIG_CROP_Y_OFFSET[7:0] = 0x2C = 44 (centered for 2168 crop) */
    imx586_write_register(ViPipe, 0x040C, 0x0F); /* DIG_CROP_IMAGE_WIDTH = 0x0F08 = 3848 (8px flip margin) */
    imx586_write_register(ViPipe, 0x040D, 0x08);
    imx586_write_register(ViPipe, 0x040E, 0x08); /* DIG_CROP_IMAGE_HEIGHT = 0x0878 = 2168 (8px flip margin) */
    imx586_write_register(ViPipe, 0x040F, 0x78);

    imx586_write_register(ViPipe, 0x034C, 0x0F); /* X_OUT_SIZE = 0x0F00 = 3840 */
    imx586_write_register(ViPipe, 0x034D, 0x08);
    imx586_write_register(ViPipe, 0x034E, 0x08); /* Y_OUT_SIZE = 0x0870 = 2160 */
    imx586_write_register(ViPipe, 0x034F, 0x78);

    /* 24 MHz INCK: about 60 fps and 1736 Mbps per lane. */
    imx586_write_register(ViPipe, 0x0301, 0x05); /* IVT_PXCK_DIV */
    imx586_write_register(ViPipe, 0x0303, 0x02); /* IVT_SYCK_DIV */
    imx586_write_register(ViPipe, 0x0305, 0x04); /* IVT_PREPLLCK_DIV */
    imx586_write_register(ViPipe, 0x0306, 0x01); /* IVT_PLL_MPY[10:8] */
    imx586_write_register(ViPipe, 0x0307, 0x04); /* IVT_PLL_MPY[7:0] */
    imx586_write_register(ViPipe, 0x030B, 0x01); /* IOP_SYCK_DIV */
    imx586_write_register(ViPipe, 0x030D, 0x03); /* IOP_PREPLLCK_DIV */
    imx586_write_register(ViPipe, 0x030E, 0x00); /* IOP_PLL_MPY[11:8] */
    imx586_write_register(ViPipe, 0x030F, 0xD9); /* IOP_PLL_MPY[7:0] */
    imx586_write_register(ViPipe, 0x0310, 0x01); /* PLL_MULT_DRIV */

    imx586_write_register(ViPipe, 0x3620, 0x00);
    imx586_write_register(ViPipe, 0x3621, 0x00);

    imx586_write_register(ViPipe, 0x3C11, 0x04);
    imx586_write_register(ViPipe, 0x3C12, 0x03);
    imx586_write_register(ViPipe, 0x3C13, 0x2D);

    imx586_write_register(ViPipe, 0x3F0C, 0x01);
    imx586_write_register(ViPipe, 0x3F14, 0x00);
    imx586_write_register(ViPipe, 0x3F80, 0x00);
    imx586_write_register(ViPipe, 0x3F81, 0x14);
    imx586_write_register(ViPipe, 0x3F8C, 0x00);
    imx586_write_register(ViPipe, 0x3F8D, 0x14);
    imx586_write_register(ViPipe, 0x3FF8, 0x00);
    imx586_write_register(ViPipe, 0x3FF9, 0x3C);
    imx586_write_register(ViPipe, 0x3FFE, 0x01);
    imx586_write_register(ViPipe, 0x3FFF, 0x8C);

    imx586_write_register(ViPipe, 0x0202, 0x08); /* COARSE_INTEG_TIME = 0x08DC = 2268 */
    imx586_write_register(ViPipe, 0x0203, 0xDC);
    imx586_write_register(ViPipe, 0x0224, 0x01); /* ST_COARSE_INTEG_TIME = 0x01F4 = 500 */
    imx586_write_register(ViPipe, 0x0225, 0xF4);
    imx586_write_register(ViPipe, 0x3FE0, 0x01);
    imx586_write_register(ViPipe, 0x3FE1, 0xF4);

    imx586_write_register(ViPipe, 0x0204, 0x00); /* ANA_GAIN_GLOBAL = 0x70 = 112 */
    imx586_write_register(ViPipe, 0x0205, 0x70);
    imx586_write_register(ViPipe, 0x0216, 0x00); /* ST_ANA_GAIN_GLOBAL */
    imx586_write_register(ViPipe, 0x0217, 0x70);
    imx586_write_register(ViPipe, 0x0218, 0x01); /* ST_DIG_GAIN_GLOBAL = 0x0100 = 256 */
    imx586_write_register(ViPipe, 0x0219, 0x00);
    imx586_write_register(ViPipe, 0x020E, 0x01); /* DIG_GAIN_GLOBAL = 0x0100 */
    imx586_write_register(ViPipe, 0x020F, 0x00);
    imx586_write_register(ViPipe, 0x0210, 0x01); /* DIG_GAIN_R */
    imx586_write_register(ViPipe, 0x0211, 0x00);
    imx586_write_register(ViPipe, 0x0212, 0x01); /* DIG_GAIN_B */
    imx586_write_register(ViPipe, 0x0213, 0x00);
    imx586_write_register(ViPipe, 0x0214, 0x01); /* DIG_GAIN_GB */
    imx586_write_register(ViPipe, 0x0215, 0x00);

    imx586_write_register(ViPipe, 0x3FE2, 0x00);
    imx586_write_register(ViPipe, 0x3FE3, 0x70);
    imx586_write_register(ViPipe, 0x3FE4, 0x01);
    imx586_write_register(ViPipe, 0x3FE5, 0x00);

    imx586_write_register(ViPipe, 0x323B, 0x00);
    imx586_write_register(ViPipe, 0x323C, 0x00);
    imx586_write_register(ViPipe, 0x3E20, 0x02);
    imx586_write_register(ViPipe, 0x3E3B, 0x01);
    imx586_write_register(ViPipe, 0x4434, 0x01);
    imx586_write_register(ViPipe, 0x4435, 0xF0);

    imx586_write_register(ViPipe, 0x0100, 0x01); /* Streaming */
    imx586_default_reg_init(ViPipe);

    delay_ms(200);
    imx586_write_register(ViPipe, 0x0100, 0x01); /* Streaming */

}

static void imx586_linear_2M120_init(VI_PIPE ViPipe)
{
    imx586_write_register(ViPipe, 0x0112, 0x0A);
    imx586_write_register(ViPipe, 0x0113, 0x0A);
    imx586_write_register(ViPipe, 0x0114, 0x03);
    imx586_write_register(ViPipe, 0x0342, 0x15);
    imx586_write_register(ViPipe, 0x0343, 0x00);
    /* FRM_LENGTH_LINES = 1142 for the 120 fps reference timing. */
    imx586_write_register(ViPipe, 0x0340, 0x04);
    imx586_write_register(ViPipe, 0x0341, 0x76);
    imx586_write_register(ViPipe, 0x0344, 0x00);
    imx586_write_register(ViPipe, 0x0345, 0x00);
    imx586_write_register(ViPipe, 0x0346, 0x03);
    imx586_write_register(ViPipe, 0x0347, 0x48);
    imx586_write_register(ViPipe, 0x0348, 0x1F);
    imx586_write_register(ViPipe, 0x0349, 0x3F);
    imx586_write_register(ViPipe, 0x034A, 0x14);
    imx586_write_register(ViPipe, 0x034B, 0x27);
    imx586_write_register(ViPipe, 0x0220, 0x62);
    imx586_write_register(ViPipe, 0x0222, 0x01);
    imx586_write_register(ViPipe, 0x0900, 0x01);
    imx586_write_register(ViPipe, 0x0901, 0x44);
    imx586_write_register(ViPipe, 0x0902, 0x08);
    imx586_write_register(ViPipe, 0x3140, 0x00);
    imx586_write_register(ViPipe, 0x3246, 0x89);
    imx586_write_register(ViPipe, 0x3247, 0x89);
    imx586_write_register(ViPipe, 0x3F15, 0x00);
    imx586_write_register(ViPipe, 0x0401, 0x00);
    imx586_write_register(ViPipe, 0x0404, 0x00);
    imx586_write_register(ViPipe, 0x0405, 0x10);
    imx586_write_register(ViPipe, 0x0408, 0x00);
    imx586_write_register(ViPipe, 0x0409, 0x28);
    imx586_write_register(ViPipe, 0x040A, 0x00);
    imx586_write_register(ViPipe, 0x040B, 0x00);
    imx586_write_register(ViPipe, 0x040C, 0x07);
    imx586_write_register(ViPipe, 0x040D, 0x80);
    imx586_write_register(ViPipe, 0x040E, 0x04);
    imx586_write_register(ViPipe, 0x040F, 0x38);
    imx586_write_register(ViPipe, 0x034C, 0x07);
    imx586_write_register(ViPipe, 0x034D, 0x80);
    imx586_write_register(ViPipe, 0x034E, 0x04);
    imx586_write_register(ViPipe, 0x034F, 0x38);
    imx586_write_register(ViPipe, 0x0301, 0x05);
    imx586_write_register(ViPipe, 0x0303, 0x04);
    imx586_write_register(ViPipe, 0x0305, 0x04);
    imx586_write_register(ViPipe, 0x0306, 0x01);
    imx586_write_register(ViPipe, 0x0307, 0x33);
    imx586_write_register(ViPipe, 0x030B, 0x02);
    imx586_write_register(ViPipe, 0x030D, 0x0C);
    imx586_write_register(ViPipe, 0x030E, 0x02);
    imx586_write_register(ViPipe, 0x030F, 0xD6);
    imx586_write_register(ViPipe, 0x0310, 0x01);
    imx586_write_register(ViPipe, 0x3620, 0x00);
    imx586_write_register(ViPipe, 0x3621, 0x00);
    imx586_write_register(ViPipe, 0x3C11, 0x0C);
    imx586_write_register(ViPipe, 0x3C12, 0x05);
    imx586_write_register(ViPipe, 0x3C13, 0x2C);
    imx586_write_register(ViPipe, 0x3F0C, 0x00);
    imx586_write_register(ViPipe, 0x3F14, 0x00);
    imx586_write_register(ViPipe, 0x3F80, 0x02);
    imx586_write_register(ViPipe, 0x3F81, 0x67);
    imx586_write_register(ViPipe, 0x3F8C, 0x02);
    imx586_write_register(ViPipe, 0x3F8D, 0x44);
    imx586_write_register(ViPipe, 0x3FF8, 0x00);
    imx586_write_register(ViPipe, 0x3FF9, 0x00);
    imx586_write_register(ViPipe, 0x3FFE, 0x01);
    imx586_write_register(ViPipe, 0x3FFF, 0x90);
    imx586_write_register(ViPipe, 0x0202, 0x04);
    imx586_write_register(ViPipe, 0x0203, 0x46);
    imx586_write_register(ViPipe, 0x0224, 0x01);
    imx586_write_register(ViPipe, 0x0225, 0xF4);
    imx586_write_register(ViPipe, 0x3FE0, 0x01);
    imx586_write_register(ViPipe, 0x3FE1, 0xF4);
    imx586_write_register(ViPipe, 0x0204, 0x00);
    imx586_write_register(ViPipe, 0x0205, 0x70);
    imx586_write_register(ViPipe, 0x0216, 0x00);
    imx586_write_register(ViPipe, 0x0217, 0x70);
    imx586_write_register(ViPipe, 0x0218, 0x01);
    imx586_write_register(ViPipe, 0x0219, 0x00);
    imx586_write_register(ViPipe, 0x020E, 0x01);
    imx586_write_register(ViPipe, 0x020F, 0x00);
    imx586_write_register(ViPipe, 0x0210, 0x01);
    imx586_write_register(ViPipe, 0x0211, 0x00);
    imx586_write_register(ViPipe, 0x0212, 0x01);
    imx586_write_register(ViPipe, 0x0213, 0x00);
    imx586_write_register(ViPipe, 0x0214, 0x01);
    imx586_write_register(ViPipe, 0x0215, 0x00);
    imx586_write_register(ViPipe, 0x3FE2, 0x00);
    imx586_write_register(ViPipe, 0x3FE3, 0x70);
    imx586_write_register(ViPipe, 0x3FE4, 0x01);
    imx586_write_register(ViPipe, 0x3FE5, 0x00);
    imx586_write_register(ViPipe, 0x3E20, 0x01);
    imx586_write_register(ViPipe, 0x3E37, 0x00);
    imx586_write_register(ViPipe, 0x3E20, 0x01);
    imx586_write_register(ViPipe, 0x3E3B, 0x00);
    imx586_write_register(ViPipe, 0x4434, 0x00);
    imx586_write_register(ViPipe, 0x4435, 0xF8);
    imx586_write_register(ViPipe, 0x0350, 0x00);
    imx586_write_register(ViPipe, 0x0100, 0x01);// - Streaming
    imx586_default_reg_init(ViPipe);

    delay_ms(200);
    imx586_write_register(ViPipe, 0x0100, 0x01);// - Streaming
}
