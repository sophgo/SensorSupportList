#include <unistd.h>
#include <stdio.h>
#include <cvi_comm_video.h>
#include "cvi_sns_ctrl.h"
#include "mis4001_cmos_ex.h"
#include "sensor_i2c.h"

static void mis4001_linear_1440p30_init(VI_PIPE ViPipe);

CVI_U8 mis4001_i2c_addr = 0x30;
const CVI_U32 mis4001_addr_byte = 2;
const CVI_U32 mis4001_data_byte = 1;

int mis4001_i2c_init(VI_PIPE ViPipe)
{
	return sensor_i2c_init(ViPipe, (CVI_U8)g_aunMIS4001_BusInfo[ViPipe].s8I2cDev,
				(CVI_U8)g_aunMIS4001_AddrInfo[ViPipe].s8I2cAddr);
}

int mis4001_i2c_exit(VI_PIPE ViPipe)
{
	return sensor_i2c_exit(ViPipe, (CVI_U8)g_aunMIS4001_BusInfo[ViPipe].s8I2cDev);
}

int mis4001_read_register(VI_PIPE ViPipe, int addr)
{
	return sensor_i2c_read(ViPipe, (CVI_U8)g_aunMIS4001_BusInfo[ViPipe].s8I2cDev,
				(CVI_U8)g_aunMIS4001_AddrInfo[ViPipe].s8I2cAddr, (CVI_U32)addr,
				mis4001_addr_byte, mis4001_data_byte);
}

int mis4001_write_register(VI_PIPE ViPipe, int addr, int data)
{
	return sensor_i2c_write(ViPipe, (CVI_U8)g_aunMIS4001_BusInfo[ViPipe].s8I2cDev,
				(CVI_U8)g_aunMIS4001_AddrInfo[ViPipe].s8I2cAddr, (CVI_U32)addr,
				mis4001_addr_byte, (CVI_U32)data, mis4001_data_byte);
}

static void delay_ms(int ms)
{
	usleep(ms * 1000);
}

void mis4001_standby(VI_PIPE ViPipe)
{
	mis4001_write_register(ViPipe, 0x3006, 0x02);
}

void mis4001_restart(VI_PIPE ViPipe)
{
	mis4001_write_register(ViPipe, 0x3006, 0x00);
}

void mis4001_default_reg_init(VI_PIPE ViPipe)
{
	CVI_U32 i;

	for (i = 0; i < g_pastMIS4001[ViPipe]->astSyncInfo[0].snsCfg.u32RegNum; i++) {
		mis4001_write_register(ViPipe,
			g_pastMIS4001[ViPipe]->astSyncInfo[0].snsCfg.astI2cData[i].u32RegAddr,
			g_pastMIS4001[ViPipe]->astSyncInfo[0].snsCfg.astI2cData[i].u32Data);
	}
}

#define MIS4001_CHIP_ID_HI_ADDR		0x3000
#define MIS4001_CHIP_ID_LO_ADDR		0x3001
#define MIS4001_CHIP_ID			0x1311
#define MIS4001_CHIP_ID_ALT		0x4001

int mis4001_probe(VI_PIPE ViPipe)
{
	int nVal;
	CVI_U16 chip_id;

	if (mis4001_i2c_init(ViPipe) != CVI_SUCCESS)
		return CVI_FAILURE;

	delay_ms(5);

	nVal = mis4001_read_register(ViPipe, MIS4001_CHIP_ID_HI_ADDR);
	if (nVal < 0) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "read sensor id error.\n");
		return nVal;
	}
	chip_id = (nVal & 0xFF) << 8;
	nVal = mis4001_read_register(ViPipe, MIS4001_CHIP_ID_LO_ADDR);
	if (nVal < 0) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "read sensor id error.\n");
		return nVal;
	}
	chip_id |= (nVal & 0xFF);

	if (chip_id != MIS4001_CHIP_ID && chip_id != MIS4001_CHIP_ID_ALT) {
		CVI_TRACE_SNS(CVI_DBG_ERR,
			"MIS4001 Sensor ID Wrong! (0x%x)\n", chip_id);
		return CVI_FAILURE;
	}

	return CVI_SUCCESS;
}

void mis4001_init(VI_PIPE ViPipe)
{
	mis4001_i2c_init(ViPipe);
	mis4001_linear_1440p30_init(ViPipe);
	g_pastMIS4001[ViPipe]->bInit = CVI_TRUE;
}

void mis4001_exit(VI_PIPE ViPipe)
{
	mis4001_i2c_exit(ViPipe);
}

static void mis4001_linear_1440p30_init(VI_PIPE ViPipe)
{
	mis4001_write_register(ViPipe, 0x300a, 0x01);
	mis4001_write_register(ViPipe, 0x3006, 0x02);
	delay_ms(100);
	mis4001_write_register(ViPipe, 0x4220, 0x2b);
	mis4001_write_register(ViPipe, 0x4221, 0x6b);
	mis4001_write_register(ViPipe, 0x4222, 0xab);
	mis4001_write_register(ViPipe, 0x4223, 0xeb);
	mis4001_write_register(ViPipe, 0x3011, 0x2b);
	mis4001_write_register(ViPipe, 0x3302, 0x02);
	mis4001_write_register(ViPipe, 0x3307, 0x64);
	mis4001_write_register(ViPipe, 0x3306, 0x01);
	mis4001_write_register(ViPipe, 0x3309, 0x01);
	mis4001_write_register(ViPipe, 0x3308, 0x04);
	mis4001_write_register(ViPipe, 0x330a, 0x04);
	mis4001_write_register(ViPipe, 0x330b, 0x09);
	mis4001_write_register(ViPipe, 0x310f, 0xb8);
	mis4001_write_register(ViPipe, 0x310e, 0x0b);
	mis4001_write_register(ViPipe, 0x310d, 0xdc);
	mis4001_write_register(ViPipe, 0x310c, 0x05);
	mis4001_write_register(ViPipe, 0x3102, 0x00);
	mis4001_write_register(ViPipe, 0x3115, 0x00);
	mis4001_write_register(ViPipe, 0x3114, 0x00);
	mis4001_write_register(ViPipe, 0x3117, 0x0f);
	mis4001_write_register(ViPipe, 0x3116, 0x0a);
	mis4001_write_register(ViPipe, 0x3111, 0xf4);
	mis4001_write_register(ViPipe, 0x3110, 0x00);
	mis4001_write_register(ViPipe, 0x3113, 0xa5);
	mis4001_write_register(ViPipe, 0x3112, 0x06);
	mis4001_write_register(ViPipe, 0x3128, 0x0f);
	mis4001_write_register(ViPipe, 0x3129, 0xff);
	mis4001_write_register(ViPipe, 0x3012, 0x03);
	mis4001_write_register(ViPipe, 0x3f00, 0x01);
	mis4001_write_register(ViPipe, 0x3f02, 0x07);
	mis4001_write_register(ViPipe, 0x3f01, 0x00);
	mis4001_write_register(ViPipe, 0x3f04, 0x2a);
	mis4001_write_register(ViPipe, 0x3f03, 0x00);
	mis4001_write_register(ViPipe, 0x3f06, 0xa5);
	mis4001_write_register(ViPipe, 0x3f05, 0x04);
	mis4001_write_register(ViPipe, 0x3f08, 0xff);
	mis4001_write_register(ViPipe, 0x3f07, 0x1f);
	mis4001_write_register(ViPipe, 0x3f0a, 0xa4);
	mis4001_write_register(ViPipe, 0x3f09, 0x01);
	mis4001_write_register(ViPipe, 0x3f0c, 0x38);
	mis4001_write_register(ViPipe, 0x3f0b, 0x00);
	mis4001_write_register(ViPipe, 0x3f0e, 0xff);
	mis4001_write_register(ViPipe, 0x3f0d, 0x1f);
	mis4001_write_register(ViPipe, 0x3f10, 0xff);
	mis4001_write_register(ViPipe, 0x3f0f, 0x1f);
	mis4001_write_register(ViPipe, 0x3f13, 0x07);
	mis4001_write_register(ViPipe, 0x3f12, 0x00);
	mis4001_write_register(ViPipe, 0x3f15, 0x9d);
	mis4001_write_register(ViPipe, 0x3f14, 0x01);
	mis4001_write_register(ViPipe, 0x3f17, 0x31);
	mis4001_write_register(ViPipe, 0x3f16, 0x00);
	mis4001_write_register(ViPipe, 0x3f19, 0x73);
	mis4001_write_register(ViPipe, 0x3f18, 0x01);
	mis4001_write_register(ViPipe, 0x3f1b, 0x00);
	mis4001_write_register(ViPipe, 0x3f1a, 0x00);
	mis4001_write_register(ViPipe, 0x3f1d, 0xa9);
	mis4001_write_register(ViPipe, 0x3f1c, 0x04);
	mis4001_write_register(ViPipe, 0x3f1f, 0xff);
	mis4001_write_register(ViPipe, 0x3f1e, 0x1f);
	mis4001_write_register(ViPipe, 0x3f21, 0xff);
	mis4001_write_register(ViPipe, 0x3f20, 0x1f);
	mis4001_write_register(ViPipe, 0x3f23, 0x85);
	mis4001_write_register(ViPipe, 0x3f22, 0x00);
	mis4001_write_register(ViPipe, 0x3f25, 0x27);
	mis4001_write_register(ViPipe, 0x3f24, 0x01);
	mis4001_write_register(ViPipe, 0x3f28, 0x46);
	mis4001_write_register(ViPipe, 0x3f27, 0x00);
	mis4001_write_register(ViPipe, 0x3f2a, 0x07);
	mis4001_write_register(ViPipe, 0x3f29, 0x00);
	mis4001_write_register(ViPipe, 0x3f2c, 0x3f);
	mis4001_write_register(ViPipe, 0x3f2b, 0x00);
	mis4001_write_register(ViPipe, 0x3f2e, 0x70);
	mis4001_write_register(ViPipe, 0x3f2d, 0x01);
	mis4001_write_register(ViPipe, 0x3f30, 0x38);
	mis4001_write_register(ViPipe, 0x3f2f, 0x00);
	mis4001_write_register(ViPipe, 0x3f32, 0x3f);
	mis4001_write_register(ViPipe, 0x3f31, 0x00);
	mis4001_write_register(ViPipe, 0x3f34, 0xd1);
	mis4001_write_register(ViPipe, 0x3f33, 0x00);
	mis4001_write_register(ViPipe, 0x3f36, 0xc0);
	mis4001_write_register(ViPipe, 0x3f35, 0x00);
	mis4001_write_register(ViPipe, 0x3f38, 0x2f);
	mis4001_write_register(ViPipe, 0x3f37, 0x02);
	mis4001_write_register(ViPipe, 0x3f3a, 0x5d);
	mis4001_write_register(ViPipe, 0x3f39, 0x02);
	mis4001_write_register(ViPipe, 0x3f4f, 0x5d);
	mis4001_write_register(ViPipe, 0x3f4e, 0x02);
	mis4001_write_register(ViPipe, 0x3f51, 0x5d);
	mis4001_write_register(ViPipe, 0x3f50, 0x02);
	mis4001_write_register(ViPipe, 0x3f53, 0x5d);
	mis4001_write_register(ViPipe, 0x3f52, 0x02);
	mis4001_write_register(ViPipe, 0x3f55, 0x50);
	mis4001_write_register(ViPipe, 0x3f54, 0x02);
	mis4001_write_register(ViPipe, 0x3f3c, 0x9a);
	mis4001_write_register(ViPipe, 0x3f3b, 0x00);
	mis4001_write_register(ViPipe, 0x3f3e, 0x09);
	mis4001_write_register(ViPipe, 0x3f3d, 0x04);
	mis4001_write_register(ViPipe, 0x3f40, 0x93);
	mis4001_write_register(ViPipe, 0x3f3f, 0x01);
	mis4001_write_register(ViPipe, 0x3f42, 0x8f);
	mis4001_write_register(ViPipe, 0x3f41, 0x00);
	mis4001_write_register(ViPipe, 0x3f44, 0xb0);
	mis4001_write_register(ViPipe, 0x3f43, 0x04);
	mis4001_write_register(ViPipe, 0x312b, 0x4a);
	mis4001_write_register(ViPipe, 0x312a, 0x00);
	mis4001_write_register(ViPipe, 0x312f, 0xb2);
	mis4001_write_register(ViPipe, 0x312e, 0x00);
	mis4001_write_register(ViPipe, 0x3124, 0x09);
	mis4001_write_register(ViPipe, 0x4200, 0x09);
	mis4001_write_register(ViPipe, 0x4201, 0x00);
	mis4001_write_register(ViPipe, 0x4204, 0xff);
	mis4001_write_register(ViPipe, 0x4205, 0x3f);
	mis4001_write_register(ViPipe, 0x4214, 0x60);
	mis4001_write_register(ViPipe, 0x420c, 0x50);
	mis4001_write_register(ViPipe, 0x420e, 0x94);
	mis4001_write_register(ViPipe, 0x4216, 0x6c);
	mis4001_write_register(ViPipe, 0x4217, 0xdc);
	mis4001_write_register(ViPipe, 0x4218, 0x02);
	mis4001_write_register(ViPipe, 0x4240, 0x8d);
	mis4001_write_register(ViPipe, 0x4242, 0x03);
	mis4001_write_register(ViPipe, 0x4224, 0x10);
	mis4001_write_register(ViPipe, 0x4225, 0x0a);
	mis4001_write_register(ViPipe, 0x4226, 0xb0);
	mis4001_write_register(ViPipe, 0x4227, 0x05);
	mis4001_write_register(ViPipe, 0x4228, 0x10);
	mis4001_write_register(ViPipe, 0x4229, 0x0a);
	mis4001_write_register(ViPipe, 0x422a, 0xb0);
	mis4001_write_register(ViPipe, 0x422b, 0x05);
	mis4001_write_register(ViPipe, 0x422c, 0x10);
	mis4001_write_register(ViPipe, 0x422d, 0x0a);
	mis4001_write_register(ViPipe, 0x422e, 0xb0);
	mis4001_write_register(ViPipe, 0x422f, 0x05);
	mis4001_write_register(ViPipe, 0x4230, 0x10);
	mis4001_write_register(ViPipe, 0x4231, 0x0a);
	mis4001_write_register(ViPipe, 0x4232, 0xb0);
	mis4001_write_register(ViPipe, 0x4233, 0x05);
	mis4001_write_register(ViPipe, 0x4509, 0x0f);
	mis4001_write_register(ViPipe, 0x4505, 0x00);
	mis4001_write_register(ViPipe, 0x4501, 0xff);
	mis4001_write_register(ViPipe, 0x4502, 0x33);
	mis4001_write_register(ViPipe, 0x4503, 0x11);
	mis4001_write_register(ViPipe, 0x4501, 0xf0);
	mis4001_write_register(ViPipe, 0x4502, 0x30);
	mis4001_write_register(ViPipe, 0x4503, 0x10);
	mis4001_write_register(ViPipe, 0x3a00, 0x00);
	mis4001_write_register(ViPipe, 0x3a01, 0x80);
	mis4001_write_register(ViPipe, 0x401e, 0x3c);
	mis4001_write_register(ViPipe, 0x401d, 0xa0);
	mis4001_write_register(ViPipe, 0x3012, 0x03);
	mis4001_write_register(ViPipe, 0x3e00, 0x00);
	mis4001_write_register(ViPipe, 0x3e01, 0x10);
	mis4001_write_register(ViPipe, 0x400d, 0x30);
	mis4001_write_register(ViPipe, 0x3500, 0x1b);
	mis4001_write_register(ViPipe, 0x3501, 0x03);
	mis4001_write_register(ViPipe, 0x3508, 0x0a);
	mis4001_write_register(ViPipe, 0x3508, 0x04);
	mis4001_write_register(ViPipe, 0x3513, 0x01);
	mis4001_write_register(ViPipe, 0x3514, 0x09);
	mis4001_write_register(ViPipe, 0x3515, 0x0b);
	mis4001_write_register(ViPipe, 0x3702, 0x80);
	mis4001_write_register(ViPipe, 0x3704, 0x80);
	mis4001_write_register(ViPipe, 0x3706, 0x80);
	mis4001_write_register(ViPipe, 0x3708, 0x80);
	mis4001_write_register(ViPipe, 0x400d, 0x30);
	mis4001_write_register(ViPipe, 0x4004, 0x00);
	mis4001_write_register(ViPipe, 0x4005, 0x30);
	mis4001_write_register(ViPipe, 0x4009, 0x09);
	mis4001_write_register(ViPipe, 0x400a, 0x48);
	mis4001_write_register(ViPipe, 0x4006, 0x86);
	mis4001_write_register(ViPipe, 0x4019, 0x08);
	mis4001_write_register(ViPipe, 0x401b, 0x00);
	mis4001_write_register(ViPipe, 0x3f42, 0x58);
	mis4001_write_register(ViPipe, 0x3f49, 0x60);
	mis4001_write_register(ViPipe, 0x3f38, 0x38);
	mis4001_write_register(ViPipe, 0x4501, 0xff);
	mis4001_write_register(ViPipe, 0x3006, 0x00);
	delay_ms(3);
	mis4001_write_register(ViPipe, 0x4501, 0xf0);

	mis4001_default_reg_init(ViPipe);

	delay_ms(100);

	printf("ViPipe:%d,===MIS4001 2560x1440P 30fps 10bit LINE Init OK!===\n", ViPipe);
}
