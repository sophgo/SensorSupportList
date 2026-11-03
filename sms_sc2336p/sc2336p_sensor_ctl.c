#include <unistd.h>
#include <stdlib.h>
#include <cvi_comm_video.h>
#include "cvi_sns_ctrl.h"
#include "sc2336p_cmos_ex.h"
#include "sensor_i2c.h"

static void sc2336p_linear_1080p30_2l_init(VI_PIPE ViPipe);
static void sc2336p_linear_1080p30_1l_init(VI_PIPE ViPipe);

static void delay_ms(int ms)
{
	usleep(ms * 1000);
}

#define SC2336P_CHIP_ID_HI_ADDR		0x3107
#define SC2336P_CHIP_ID_LO_ADDR		0x3108
#define SC2336P_CHIP_ID			0xcb3a
#define SC2336P_CHIP_ID_ALT		0x9b3a

const CVI_U8 sc2336p_i2c_addr = 0x30;
const CVI_U32 sc2336p_addr_byte = 2;
const CVI_U32 sc2336p_data_byte = 1;

int sc2336p_i2c_init(VI_PIPE ViPipe)
{
	return sensor_i2c_init(ViPipe, (CVI_U8)g_aunSC2336P_BusInfo[ViPipe].s8I2cDev,
				(CVI_U8)g_aunSC2336P_AddrInfo[ViPipe].s8I2cAddr);
}

int sc2336p_i2c_exit(VI_PIPE ViPipe)
{
	return sensor_i2c_exit(ViPipe, (CVI_U8)g_aunSC2336P_BusInfo[ViPipe].s8I2cDev);
}

int sc2336p_write_register(VI_PIPE ViPipe, int addr, int data)
{
	return sensor_i2c_write(ViPipe, (CVI_U8)g_aunSC2336P_BusInfo[ViPipe].s8I2cDev,
				(CVI_U8)g_aunSC2336P_AddrInfo[ViPipe].s8I2cAddr,
				(CVI_U32)addr, sc2336p_addr_byte,
				(CVI_U32)data, sc2336p_data_byte);
}

int sc2336p_read_register(VI_PIPE ViPipe, int addr)
{
	return sensor_i2c_read(ViPipe, (CVI_U8)g_aunSC2336P_BusInfo[ViPipe].s8I2cDev,
			       (CVI_U8)g_aunSC2336P_AddrInfo[ViPipe].s8I2cAddr,
			       (CVI_U32)addr, sc2336p_addr_byte, sc2336p_data_byte);
}

void sc2336p_standby(VI_PIPE ViPipe)
{
	sc2336p_i2c_init(ViPipe);
	sc2336p_write_register(ViPipe, 0x0100, 0x00);
	delay_ms(10);
	sc2336p_write_register(ViPipe, 0x0103, 0x01);
	delay_ms(20);
}

static CVI_BOOL sc2336p_chip_id_valid(CVI_U16 chip_id)
{
	return chip_id == SC2336P_CHIP_ID || chip_id == SC2336P_CHIP_ID_ALT;
}

static void sc2336p_soft_reset(VI_PIPE ViPipe)
{
	sc2336p_write_register(ViPipe, 0x0100, 0x00);
	delay_ms(10);
	sc2336p_write_register(ViPipe, 0x0103, 0x01);
	delay_ms(20);
}

static CVI_S16 sc2336p_read_chip_id(VI_PIPE ViPipe, CVI_U16 *chip_id)
{
	int nVal;

	nVal = sc2336p_read_register(ViPipe, SC2336P_CHIP_ID_HI_ADDR);
	if (nVal < 0)
		return (CVI_S16)nVal;
	*chip_id = (nVal & 0xFF) << 8;
	nVal = sc2336p_read_register(ViPipe, SC2336P_CHIP_ID_LO_ADDR);
	if (nVal < 0)
		return (CVI_S16)nVal;
	*chip_id |= (nVal & 0xFF);

	return CVI_SUCCESS;
}

static void sc2336p_probe_prepare(VI_PIPE ViPipe)
{
	sc2336p_soft_reset(ViPipe);
	sc2336p_write_register(ViPipe, 0x301a, 0xf8);
	sc2336p_write_register(ViPipe, 0x0100, 0x01);
	delay_ms(5);
}

void sc2336p_restart(VI_PIPE ViPipe)
{
	sc2336p_write_register(ViPipe, 0x0100, 0x00);
	delay_ms(20);
	sc2336p_write_register(ViPipe, 0x0100, 0x01);
}

void sc2336p_mirror_flip(VI_PIPE ViPipe, ISP_SNS_MIRRORFLIP_TYPE_E eSnsMirrorFlip)
{
	CVI_U8 val = 0;

	switch (eSnsMirrorFlip) {
	case ISP_SNS_NORMAL: break;
	case ISP_SNS_MIRROR: val |= 0x6; break;
	case ISP_SNS_FLIP: val |= 0x60; break;
	case ISP_SNS_MIRROR_FLIP: val |= 0x66; break;
	default: return;
	}
	sc2336p_write_register(ViPipe, 0x3221, val);
}

int sc2336p_probe(VI_PIPE ViPipe)
{
	CVI_U16 chip_id = 0;
	int retry;
	CVI_S16 s32Ret;

	usleep(4 * 1000);
	if (sc2336p_i2c_init(ViPipe) != CVI_SUCCESS)
		return CVI_FAILURE;

	delay_ms(5);

	s32Ret = sc2336p_read_chip_id(ViPipe, &chip_id);
	if (s32Ret == CVI_SUCCESS && sc2336p_chip_id_valid(chip_id))
		return CVI_SUCCESS;

	for (retry = 0; retry < 3; retry++) {
		sc2336p_soft_reset(ViPipe);
		s32Ret = sc2336p_read_chip_id(ViPipe, &chip_id);
		if (s32Ret == CVI_SUCCESS && sc2336p_chip_id_valid(chip_id))
			return CVI_SUCCESS;

		sc2336p_probe_prepare(ViPipe);
		s32Ret = sc2336p_read_chip_id(ViPipe, &chip_id);
		if (s32Ret == CVI_SUCCESS && sc2336p_chip_id_valid(chip_id))
			return CVI_SUCCESS;
		delay_ms(10);
	}

	return CVI_FAILURE;
}

void sc2336p_init(VI_PIPE ViPipe)
{
	CVI_U8 u8ImgMode = g_pastSC2336P[ViPipe]->u8ImgMode;
	sc2336p_i2c_init(ViPipe);
	if (u8ImgMode == SC2336P_MODE_1080P30_1L)
		sc2336p_linear_1080p30_1l_init(ViPipe);
	else
		sc2336p_linear_1080p30_2l_init(ViPipe);
	g_pastSC2336P[ViPipe]->bInit = CVI_TRUE;
}

void sc2336p_exit(VI_PIPE ViPipe)
{
	sc2336p_standby(ViPipe);
	if (g_pastSC2336PComboDevArray[ViPipe]) {
		free(g_pastSC2336PComboDevArray[ViPipe]);
		g_pastSC2336PComboDevArray[ViPipe] = CVI_NULL;
	}
	sc2336p_i2c_exit(ViPipe);
}

static void sc2336p_linear_1080p30_2l_init(VI_PIPE ViPipe)
{
	sc2336p_write_register(ViPipe, 0x0103, 0x01);
	sc2336p_write_register(ViPipe, 0x0100, 0x00);
	sc2336p_write_register(ViPipe, 0x36e9, 0x80);
	sc2336p_write_register(ViPipe, 0x37f9, 0x80);
	sc2336p_write_register(ViPipe, 0x301f, 0x02);
	sc2336p_write_register(ViPipe, 0x3106, 0x05);
	sc2336p_write_register(ViPipe, 0x3248, 0x04);
	sc2336p_write_register(ViPipe, 0x3249, 0x0b);
	sc2336p_write_register(ViPipe, 0x3253, 0x08);
	sc2336p_write_register(ViPipe, 0x3301, 0x09);
	sc2336p_write_register(ViPipe, 0x3302, 0xff);
	sc2336p_write_register(ViPipe, 0x3303, 0x10);
	sc2336p_write_register(ViPipe, 0x3306, 0x80);
	sc2336p_write_register(ViPipe, 0x3307, 0x02);
	sc2336p_write_register(ViPipe, 0x3309, 0xc8);
	sc2336p_write_register(ViPipe, 0x330a, 0x01);
	sc2336p_write_register(ViPipe, 0x330b, 0x30);
	sc2336p_write_register(ViPipe, 0x330c, 0x16);
	sc2336p_write_register(ViPipe, 0x330d, 0xff);
	sc2336p_write_register(ViPipe, 0x3318, 0x02);
	sc2336p_write_register(ViPipe, 0x331f, 0xb9);
	sc2336p_write_register(ViPipe, 0x3321, 0x0a);
	sc2336p_write_register(ViPipe, 0x3327, 0x0e);
	sc2336p_write_register(ViPipe, 0x332b, 0x12);
	sc2336p_write_register(ViPipe, 0x3333, 0x10);
	sc2336p_write_register(ViPipe, 0x3334, 0x40);
	sc2336p_write_register(ViPipe, 0x335e, 0x06);
	sc2336p_write_register(ViPipe, 0x335f, 0x0a);
	sc2336p_write_register(ViPipe, 0x3364, 0x1f);
	sc2336p_write_register(ViPipe, 0x337c, 0x02);
	sc2336p_write_register(ViPipe, 0x337d, 0x0e);
	sc2336p_write_register(ViPipe, 0x3390, 0x09);
	sc2336p_write_register(ViPipe, 0x3391, 0x0f);
	sc2336p_write_register(ViPipe, 0x3392, 0x1f);
	sc2336p_write_register(ViPipe, 0x3393, 0x20);
	sc2336p_write_register(ViPipe, 0x3394, 0x20);
	sc2336p_write_register(ViPipe, 0x3395, 0xe0);
	sc2336p_write_register(ViPipe, 0x33a2, 0x04);
	sc2336p_write_register(ViPipe, 0x33b1, 0x80);
	sc2336p_write_register(ViPipe, 0x33b2, 0x68);
	sc2336p_write_register(ViPipe, 0x33b3, 0x42);
	sc2336p_write_register(ViPipe, 0x33f9, 0x90);
	sc2336p_write_register(ViPipe, 0x33fb, 0xd0);
	sc2336p_write_register(ViPipe, 0x33fc, 0x0f);
	sc2336p_write_register(ViPipe, 0x33fd, 0x1f);
	sc2336p_write_register(ViPipe, 0x349f, 0x03);
	sc2336p_write_register(ViPipe, 0x34a6, 0x0f);
	sc2336p_write_register(ViPipe, 0x34a7, 0x1f);
	sc2336p_write_register(ViPipe, 0x34a8, 0x42);
	sc2336p_write_register(ViPipe, 0x34a9, 0x18);
	sc2336p_write_register(ViPipe, 0x34aa, 0x01);
	sc2336p_write_register(ViPipe, 0x34ab, 0x43);
	sc2336p_write_register(ViPipe, 0x34ac, 0x01);
	sc2336p_write_register(ViPipe, 0x34ad, 0x80);
	sc2336p_write_register(ViPipe, 0x3630, 0xf4);
	sc2336p_write_register(ViPipe, 0x3632, 0x44);
	sc2336p_write_register(ViPipe, 0x3633, 0x22);
	sc2336p_write_register(ViPipe, 0x3639, 0xf4);
	sc2336p_write_register(ViPipe, 0x363c, 0x47);
	sc2336p_write_register(ViPipe, 0x3670, 0x09);
	sc2336p_write_register(ViPipe, 0x3674, 0xf4);
	sc2336p_write_register(ViPipe, 0x3675, 0xfb);
	sc2336p_write_register(ViPipe, 0x3676, 0xed);
	sc2336p_write_register(ViPipe, 0x367c, 0x09);
	sc2336p_write_register(ViPipe, 0x367d, 0x0f);
	sc2336p_write_register(ViPipe, 0x3690, 0x22);
	sc2336p_write_register(ViPipe, 0x3691, 0x22);
	sc2336p_write_register(ViPipe, 0x3692, 0x22);
	sc2336p_write_register(ViPipe, 0x3698, 0x89);
	sc2336p_write_register(ViPipe, 0x3699, 0x96);
	sc2336p_write_register(ViPipe, 0x369a, 0xd0);
	sc2336p_write_register(ViPipe, 0x369b, 0xd0);
	sc2336p_write_register(ViPipe, 0x369c, 0x09);
	sc2336p_write_register(ViPipe, 0x369d, 0x0f);
	sc2336p_write_register(ViPipe, 0x36a2, 0x09);
	sc2336p_write_register(ViPipe, 0x36a3, 0x0f);
	sc2336p_write_register(ViPipe, 0x36a4, 0x1f);
	sc2336p_write_register(ViPipe, 0x36d0, 0x01);
	sc2336p_write_register(ViPipe, 0x3722, 0x81);
	sc2336p_write_register(ViPipe, 0x3724, 0x41);
	sc2336p_write_register(ViPipe, 0x3725, 0xc1);
	sc2336p_write_register(ViPipe, 0x3728, 0x20);
	sc2336p_write_register(ViPipe, 0x3900, 0x0d);
	sc2336p_write_register(ViPipe, 0x3905, 0x98);
	sc2336p_write_register(ViPipe, 0x391b, 0x81);
	sc2336p_write_register(ViPipe, 0x391c, 0x10);
	sc2336p_write_register(ViPipe, 0x3933, 0x81);
	sc2336p_write_register(ViPipe, 0x3934, 0xd0);
	sc2336p_write_register(ViPipe, 0x3940, 0x75);
	sc2336p_write_register(ViPipe, 0x3941, 0x00);
	sc2336p_write_register(ViPipe, 0x3942, 0x01);
	sc2336p_write_register(ViPipe, 0x3943, 0xd1);
	sc2336p_write_register(ViPipe, 0x3952, 0x02);
	sc2336p_write_register(ViPipe, 0x3953, 0x0f);
	sc2336p_write_register(ViPipe, 0x3e01, 0x45);
	sc2336p_write_register(ViPipe, 0x3e02, 0xf0);
	sc2336p_write_register(ViPipe, 0x3e08, 0x1f);
	sc2336p_write_register(ViPipe, 0x3e1b, 0x14);
	sc2336p_write_register(ViPipe, 0x440e, 0x02);
	sc2336p_write_register(ViPipe, 0x4509, 0x38);
	sc2336p_write_register(ViPipe, 0x5799, 0x06);
	sc2336p_write_register(ViPipe, 0x5ae0, 0xfe);
	sc2336p_write_register(ViPipe, 0x5ae1, 0x40);
	sc2336p_write_register(ViPipe, 0x5ae2, 0x30);
	sc2336p_write_register(ViPipe, 0x5ae3, 0x28);
	sc2336p_write_register(ViPipe, 0x5ae4, 0x20);
	sc2336p_write_register(ViPipe, 0x5ae5, 0x30);
	sc2336p_write_register(ViPipe, 0x5ae6, 0x28);
	sc2336p_write_register(ViPipe, 0x5ae7, 0x20);
	sc2336p_write_register(ViPipe, 0x5ae8, 0x3c);
	sc2336p_write_register(ViPipe, 0x5ae9, 0x30);
	sc2336p_write_register(ViPipe, 0x5aea, 0x28);
	sc2336p_write_register(ViPipe, 0x5aeb, 0x3c);
	sc2336p_write_register(ViPipe, 0x5aec, 0x30);
	sc2336p_write_register(ViPipe, 0x5aed, 0x28);
	sc2336p_write_register(ViPipe, 0x5aee, 0xfe);
	sc2336p_write_register(ViPipe, 0x5aef, 0x40);
	sc2336p_write_register(ViPipe, 0x5af4, 0x30);
	sc2336p_write_register(ViPipe, 0x5af5, 0x28);
	sc2336p_write_register(ViPipe, 0x5af6, 0x20);
	sc2336p_write_register(ViPipe, 0x5af7, 0x30);
	sc2336p_write_register(ViPipe, 0x5af8, 0x28);
	sc2336p_write_register(ViPipe, 0x5af9, 0x20);
	sc2336p_write_register(ViPipe, 0x5afa, 0x3c);
	sc2336p_write_register(ViPipe, 0x5afb, 0x30);
	sc2336p_write_register(ViPipe, 0x5afc, 0x28);
	sc2336p_write_register(ViPipe, 0x5afd, 0x3c);
	sc2336p_write_register(ViPipe, 0x5afe, 0x30);
	sc2336p_write_register(ViPipe, 0x5aff, 0x28);
	sc2336p_write_register(ViPipe, 0x36e9, 0x20);
	sc2336p_write_register(ViPipe, 0x37f9, 0x27);


	sc2336p_write_register(ViPipe, 0x0100, 0x01);

}

static void sc2336p_linear_1080p30_1l_init(VI_PIPE ViPipe)
{
	sc2336p_write_register(ViPipe, 0x0103,0x01);
	sc2336p_write_register(ViPipe, 0x0100,0x00);
	sc2336p_write_register(ViPipe, 0x36e9,0x80);
	sc2336p_write_register(ViPipe, 0x37f9,0x80);
	sc2336p_write_register(ViPipe, 0x3018,0x12);
	sc2336p_write_register(ViPipe, 0x3019,0x0e);
	sc2336p_write_register(ViPipe, 0x301f,0x17);
	sc2336p_write_register(ViPipe, 0x3106,0x05);
	sc2336p_write_register(ViPipe, 0x3248,0x04);
	sc2336p_write_register(ViPipe, 0x3249,0x0b);
	sc2336p_write_register(ViPipe, 0x3253,0x08);
	sc2336p_write_register(ViPipe, 0x3301,0x09);
	sc2336p_write_register(ViPipe, 0x3302,0xff);
	sc2336p_write_register(ViPipe, 0x3303,0x10);
	sc2336p_write_register(ViPipe, 0x3306,0x80);
	sc2336p_write_register(ViPipe, 0x3307,0x02);
	sc2336p_write_register(ViPipe, 0x3309,0xc8);
	sc2336p_write_register(ViPipe, 0x330a,0x01);
	sc2336p_write_register(ViPipe, 0x330b,0x30);
	sc2336p_write_register(ViPipe, 0x330c,0x16);
	sc2336p_write_register(ViPipe, 0x330d,0xff);
	sc2336p_write_register(ViPipe, 0x3318,0x02);
	sc2336p_write_register(ViPipe, 0x331f,0xb9);
	sc2336p_write_register(ViPipe, 0x3321,0x0a);
	sc2336p_write_register(ViPipe, 0x3327,0x0e);
	sc2336p_write_register(ViPipe, 0x332b,0x12);
	sc2336p_write_register(ViPipe, 0x3333,0x10);
	sc2336p_write_register(ViPipe, 0x3334,0x40);
	sc2336p_write_register(ViPipe, 0x335e,0x06);
	sc2336p_write_register(ViPipe, 0x335f,0x0a);
	sc2336p_write_register(ViPipe, 0x3364,0x1f);
	sc2336p_write_register(ViPipe, 0x337c,0x02);
	sc2336p_write_register(ViPipe, 0x337d,0x0e);
	sc2336p_write_register(ViPipe, 0x3390,0x09);
	sc2336p_write_register(ViPipe, 0x3391,0x0f);
	sc2336p_write_register(ViPipe, 0x3392,0x1f);
	sc2336p_write_register(ViPipe, 0x3393,0x20);
	sc2336p_write_register(ViPipe, 0x3394,0x20);
	sc2336p_write_register(ViPipe, 0x3395,0xe0);
	sc2336p_write_register(ViPipe, 0x33a2,0x04);
	sc2336p_write_register(ViPipe, 0x33b1,0x80);
	sc2336p_write_register(ViPipe, 0x33b2,0x68);
	sc2336p_write_register(ViPipe, 0x33b3,0x42);
	sc2336p_write_register(ViPipe, 0x33f9,0x90);
	sc2336p_write_register(ViPipe, 0x33fb,0xd0);
	sc2336p_write_register(ViPipe, 0x33fc,0x0f);
	sc2336p_write_register(ViPipe, 0x33fd,0x1f);
	sc2336p_write_register(ViPipe, 0x349f,0x03);
	sc2336p_write_register(ViPipe, 0x34a6,0x0f);
	sc2336p_write_register(ViPipe, 0x34a7,0x1f);
	sc2336p_write_register(ViPipe, 0x34a8,0x42);
	sc2336p_write_register(ViPipe, 0x34a9,0x18);
	sc2336p_write_register(ViPipe, 0x34aa,0x01);
	sc2336p_write_register(ViPipe, 0x34ab,0x43);
	sc2336p_write_register(ViPipe, 0x34ac,0x01);
	sc2336p_write_register(ViPipe, 0x34ad,0x80);
	sc2336p_write_register(ViPipe, 0x3630,0xf4);
	sc2336p_write_register(ViPipe, 0x3632,0x44);
	sc2336p_write_register(ViPipe, 0x3633,0x22);
	sc2336p_write_register(ViPipe, 0x3639,0xf4);
	sc2336p_write_register(ViPipe, 0x363c,0x47);
	sc2336p_write_register(ViPipe, 0x3670,0x09);
	sc2336p_write_register(ViPipe, 0x3674,0xf4);
	sc2336p_write_register(ViPipe, 0x3675,0xfb);
	sc2336p_write_register(ViPipe, 0x3676,0xed);
	sc2336p_write_register(ViPipe, 0x367c,0x09);
	sc2336p_write_register(ViPipe, 0x367d,0x0f);
	sc2336p_write_register(ViPipe, 0x3690,0x22);
	sc2336p_write_register(ViPipe, 0x3691,0x22);
	sc2336p_write_register(ViPipe, 0x3692,0x22);
	sc2336p_write_register(ViPipe, 0x3698,0x89);
	sc2336p_write_register(ViPipe, 0x3699,0x96);
	sc2336p_write_register(ViPipe, 0x369a,0xd0);
	sc2336p_write_register(ViPipe, 0x369b,0xd0);
	sc2336p_write_register(ViPipe, 0x369c,0x09);
	sc2336p_write_register(ViPipe, 0x369d,0x0f);
	sc2336p_write_register(ViPipe, 0x36a2,0x09);
	sc2336p_write_register(ViPipe, 0x36a3,0x0f);
	sc2336p_write_register(ViPipe, 0x36a4,0x1f);
	sc2336p_write_register(ViPipe, 0x36d0,0x01);
	sc2336p_write_register(ViPipe, 0x36ec,0x0c);
	sc2336p_write_register(ViPipe, 0x3722,0xc1);
	sc2336p_write_register(ViPipe, 0x3724,0x41);
	sc2336p_write_register(ViPipe, 0x3725,0xc1);
	sc2336p_write_register(ViPipe, 0x3728,0x20);
	sc2336p_write_register(ViPipe, 0x3900,0x0d);
	sc2336p_write_register(ViPipe, 0x3905,0x98);
	sc2336p_write_register(ViPipe, 0x3919,0x04);
	sc2336p_write_register(ViPipe, 0x391b,0x81);
	sc2336p_write_register(ViPipe, 0x391c,0x10);
	sc2336p_write_register(ViPipe, 0x3933,0x81);
	sc2336p_write_register(ViPipe, 0x3934,0xd0);
	sc2336p_write_register(ViPipe, 0x3940,0x75);
	sc2336p_write_register(ViPipe, 0x3941,0x00);
	sc2336p_write_register(ViPipe, 0x3942,0x01);
	sc2336p_write_register(ViPipe, 0x3943,0xd1);
	sc2336p_write_register(ViPipe, 0x3952,0x02);
	sc2336p_write_register(ViPipe, 0x3953,0x0f);
	sc2336p_write_register(ViPipe, 0x3e01,0x45);
	sc2336p_write_register(ViPipe, 0x3e02,0xf0);
	sc2336p_write_register(ViPipe, 0x3e08,0x1f);
	sc2336p_write_register(ViPipe, 0x3e1b,0x14);
	sc2336p_write_register(ViPipe, 0x440e,0x02);
	sc2336p_write_register(ViPipe, 0x4509,0x38);
	sc2336p_write_register(ViPipe, 0x5799,0x06);
	sc2336p_write_register(ViPipe, 0x5ae0,0xfe);
	sc2336p_write_register(ViPipe, 0x5ae1,0x40);
	sc2336p_write_register(ViPipe, 0x5ae2,0x30);
	sc2336p_write_register(ViPipe, 0x5ae3,0x28);
	sc2336p_write_register(ViPipe, 0x5ae4,0x20);
	sc2336p_write_register(ViPipe, 0x5ae5,0x30);
	sc2336p_write_register(ViPipe, 0x5ae6,0x28);
	sc2336p_write_register(ViPipe, 0x5ae7,0x20);
	sc2336p_write_register(ViPipe, 0x5ae8,0x3c);
	sc2336p_write_register(ViPipe, 0x5ae9,0x30);
	sc2336p_write_register(ViPipe, 0x5aea,0x28);
	sc2336p_write_register(ViPipe, 0x5aeb,0x3c);
	sc2336p_write_register(ViPipe, 0x5aec,0x30);
	sc2336p_write_register(ViPipe, 0x5aed,0x28);
	sc2336p_write_register(ViPipe, 0x5aee,0xfe);
	sc2336p_write_register(ViPipe, 0x5aef,0x40);
	sc2336p_write_register(ViPipe, 0x5af4,0x30);
	sc2336p_write_register(ViPipe, 0x5af5,0x28);
	sc2336p_write_register(ViPipe, 0x5af6,0x20);
	sc2336p_write_register(ViPipe, 0x5af7,0x30);
	sc2336p_write_register(ViPipe, 0x5af8,0x28);
	sc2336p_write_register(ViPipe, 0x5af9,0x20);
	sc2336p_write_register(ViPipe, 0x5afa,0x3c);
	sc2336p_write_register(ViPipe, 0x5afb,0x30);
	sc2336p_write_register(ViPipe, 0x5afc,0x28);
	sc2336p_write_register(ViPipe, 0x5afd,0x3c);
	sc2336p_write_register(ViPipe, 0x5afe,0x30);
	sc2336p_write_register(ViPipe, 0x5aff,0x28);
	sc2336p_write_register(ViPipe, 0x36e9,0x20);
	sc2336p_write_register(ViPipe, 0x37f9,0x27);


	sc2336p_write_register(ViPipe, 0x0100, 0x01);

}
