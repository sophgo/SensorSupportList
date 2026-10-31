#include <unistd.h>
#include <cvi_comm_video.h>
#include "cvi_sns_ctrl.h"
#include "gc05a3_cmos_ex.h"
#include "sensor_i2c.h"

#define GC05A3_CHIP_ID_ADDR_H	0x03f0
#define GC05A3_CHIP_ID_ADDR_L	0x03f1
#define GC05A3_CHIP_ID		0x05a3

static void gc05a3_linear_1944p20_init(VI_PIPE ViPipe);

const CVI_U32 gc05a3_addr_byte = 2;
const CVI_U32 gc05a3_data_byte = 1;

int gc05a3_i2c_init(VI_PIPE ViPipe)
{
	return sensor_i2c_init(ViPipe, (CVI_U8)g_aunGc05a3_BusInfo[ViPipe].s8I2cDev,
							(CVI_U8)g_aunGc05a3_AddrInfo[ViPipe].s8I2cAddr);
}

int gc05a3_i2c_exit(VI_PIPE ViPipe)
{
	return sensor_i2c_exit(ViPipe, (CVI_U8)g_aunGc05a3_BusInfo[ViPipe].s8I2cDev);
}

int gc05a3_read_register(VI_PIPE ViPipe, int addr)
{
	return sensor_i2c_read(ViPipe, (CVI_U8)g_aunGc05a3_BusInfo[ViPipe].s8I2cDev,
							(CVI_U8)g_aunGc05a3_AddrInfo[ViPipe].s8I2cAddr, (CVI_U32)addr,
							gc05a3_addr_byte, gc05a3_data_byte);
}

int gc05a3_write_register(VI_PIPE ViPipe, int addr, int data)
{
	return sensor_i2c_write(ViPipe, (CVI_U8)g_aunGc05a3_BusInfo[ViPipe].s8I2cDev,
							(CVI_U8)g_aunGc05a3_AddrInfo[ViPipe].s8I2cAddr, (CVI_U32)addr,
							gc05a3_addr_byte, (CVI_U32)data, gc05a3_data_byte);
}

static void delay_ms(int ms)
{
	usleep(ms * 1000);
}

void gc05a3_standby(VI_PIPE ViPipe)
{
	/* Stop streaming without changing the initialized clock/calibration state. */
	int ret = gc05a3_write_register(ViPipe, 0x0100, 0x00);

	if (ret != CVI_SUCCESS) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "gc05a3 standby pipe%d write 0x0100 failed: %d\n", ViPipe, ret);
		return;
	}

	printf("gc05a3 standby\n");
}

void gc05a3_restart(VI_PIPE ViPipe)
{
	/* Resume with the same stream-on value used by GC05A3 initialization. */
	int ret = gc05a3_write_register(ViPipe, 0x0100, 0x01);

	if (ret != CVI_SUCCESS) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "gc05a3 restart pipe%d write 0x0100 failed: %d\n", ViPipe, ret);
		return;
	}

	printf("gc05a3 restart\n");
}

void gc05a3_default_reg_init(VI_PIPE ViPipe)
{
	CVI_U32 i;

	for (i = 0; i < g_pastGc05a3[ViPipe]->astSyncInfo[0].snsCfg.u32RegNum; i++) {
		gc05a3_write_register(ViPipe,
				g_pastGc05a3[ViPipe]->astSyncInfo[0].snsCfg.astI2cData[i].u32RegAddr,
				g_pastGc05a3[ViPipe]->astSyncInfo[0].snsCfg.astI2cData[i].u32Data);
	}
}

int gc05a3_probe(VI_PIPE ViPipe)
{
	int nVal;
	int nVal2;

	usleep(50);
	if (gc05a3_i2c_init(ViPipe) != CVI_SUCCESS)
		return CVI_FAILURE;

	nVal  = gc05a3_read_register(ViPipe, GC05A3_CHIP_ID_ADDR_H);
	nVal2 = gc05a3_read_register(ViPipe, GC05A3_CHIP_ID_ADDR_L);
	if (nVal < 0 || nVal2 < 0) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "read sensor id error.\n");
		return nVal;
	}

	if ((((nVal & 0xFF) << 8) | (nVal2 & 0xFF)) != GC05A3_CHIP_ID) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "Sensor ID Mismatch! Use the wrong sensor??\n");
		return CVI_FAILURE;
	}

	return CVI_SUCCESS;
}

void gc05a3_init(VI_PIPE ViPipe)
{
	CVI_U8 u8ImgMode;

	u8ImgMode = g_pastGc05a3[ViPipe]->u8ImgMode;
	gc05a3_i2c_init(ViPipe);

	switch (u8ImgMode) {
	case GC05A3_MODE_2592X1944P20:
		gc05a3_linear_1944p20_init(ViPipe);
		break;
	default:
		CVI_TRACE_SNS(CVI_DBG_ERR, "Sensor ImageMode[%d] invalid!\n", u8ImgMode);
		break;
	}

	g_pastGc05a3[ViPipe]->bInit = CVI_TRUE;
}

static void gc05a3_linear_1944p20_init(VI_PIPE ViPipe)
{
	gc05a3_write_register(ViPipe, 0x0315, 0x80);
	gc05a3_write_register(ViPipe, 0x0315, 0xd4);
	gc05a3_write_register(ViPipe, 0x0d06, 0x01);
	gc05a3_write_register(ViPipe, 0x0a70, 0x80);
	gc05a3_write_register(ViPipe, 0x031a, 0x00);
	gc05a3_write_register(ViPipe, 0x0314, 0x00);
	gc05a3_write_register(ViPipe, 0x0130, 0x08);
	gc05a3_write_register(ViPipe, 0x0132, 0x01);
	gc05a3_write_register(ViPipe, 0x0135, 0x01);
	gc05a3_write_register(ViPipe, 0x0136, 0x36);
	gc05a3_write_register(ViPipe, 0x0137, 0x03);
	gc05a3_write_register(ViPipe, 0x0134, 0x5b);
	gc05a3_write_register(ViPipe, 0x031c, 0xe0);
	gc05a3_write_register(ViPipe, 0x0d82, 0x14);
	gc05a3_write_register(ViPipe, 0x0dd1, 0x56);
	gc05a3_write_register(ViPipe, 0x0af4, 0x01);
	gc05a3_write_register(ViPipe, 0x0002, 0x10);
	gc05a3_write_register(ViPipe, 0x00c3, 0x34);
	gc05a3_write_register(ViPipe, 0x0084, 0x21);
	gc05a3_write_register(ViPipe, 0x0d05, 0xcc);
	gc05a3_write_register(ViPipe, 0x0218, 0x00);
	gc05a3_write_register(ViPipe, 0x005e, 0x48);
	gc05a3_write_register(ViPipe, 0x0d06, 0x01);
	gc05a3_write_register(ViPipe, 0x0007, 0x16);
	gc05a3_write_register(ViPipe, 0x0101, 0x00);
	gc05a3_write_register(ViPipe, 0x0342, 0x06);
	gc05a3_write_register(ViPipe, 0x0343, 0xe8);
	gc05a3_write_register(ViPipe, 0x0220, 0x07);
	gc05a3_write_register(ViPipe, 0x0221, 0xd0);
	gc05a3_write_register(ViPipe, 0x0202, 0x07);
	gc05a3_write_register(ViPipe, 0x0203, 0x32);
	gc05a3_write_register(ViPipe, 0x0340, 0x0b);
	gc05a3_write_register(ViPipe, 0x0341, 0xe8);
	gc05a3_write_register(ViPipe, 0x0219, 0x00);
	gc05a3_write_register(ViPipe, 0x0210, 0x10);
	gc05a3_write_register(ViPipe, 0x0346, 0x00);
	gc05a3_write_register(ViPipe, 0x0347, 0x04);
	gc05a3_write_register(ViPipe, 0x0d14, 0x00);
	gc05a3_write_register(ViPipe, 0x0d13, 0x05);
	gc05a3_write_register(ViPipe, 0x0d16, 0x05);
	gc05a3_write_register(ViPipe, 0x0d15, 0x1d);
	gc05a3_write_register(ViPipe, 0x00c0, 0x0a);
	gc05a3_write_register(ViPipe, 0x00c1, 0x30);
	gc05a3_write_register(ViPipe, 0x034a, 0x07);
	gc05a3_write_register(ViPipe, 0x034b, 0xa8);
	gc05a3_write_register(ViPipe, 0x0e01, 0x00);
	gc05a3_write_register(ViPipe, 0x0e02, 0x03);
	gc05a3_write_register(ViPipe, 0x0e03, 0x01);
	gc05a3_write_register(ViPipe, 0x0e04, 0x00);
	gc05a3_write_register(ViPipe, 0x0e05, 0x01);
	gc05a3_write_register(ViPipe, 0x0e06, 0x0a);
	gc05a3_write_register(ViPipe, 0x0e07, 0x00);
	gc05a3_write_register(ViPipe, 0x0e1d, 0x03);
	gc05a3_write_register(ViPipe, 0x0e1e, 0x08);
	gc05a3_write_register(ViPipe, 0x0e1f, 0x03);
	gc05a3_write_register(ViPipe, 0x0e19, 0x00);
	gc05a3_write_register(ViPipe, 0x0e3b, 0x00);
	gc05a3_write_register(ViPipe, 0x0e3c, 0x00);
	gc05a3_write_register(ViPipe, 0x0e3d, 0x00);
	gc05a3_write_register(ViPipe, 0x0e2a, 0x0a);
	gc05a3_write_register(ViPipe, 0x0e2b, 0x0a);
	gc05a3_write_register(ViPipe, 0x0e2c, 0x1c);
	gc05a3_write_register(ViPipe, 0x0e2d, 0xb4);
	gc05a3_write_register(ViPipe, 0x0e24, 0x1d);
	gc05a3_write_register(ViPipe, 0x0e26, 0x01);
	gc05a3_write_register(ViPipe, 0x0e52, 0x15);
	gc05a3_write_register(ViPipe, 0x0e17, 0x2a);
	gc05a3_write_register(ViPipe, 0x0e3a, 0x3a);
	gc05a3_write_register(ViPipe, 0x0d28, 0x2a);
	gc05a3_write_register(ViPipe, 0x0d11, 0x80);
	gc05a3_write_register(ViPipe, 0x0d27, 0x09);
	gc05a3_write_register(ViPipe, 0x0d21, 0x80);
	gc05a3_write_register(ViPipe, 0x0d30, 0x02);
	gc05a3_write_register(ViPipe, 0x03f6, 0x0d);
	gc05a3_write_register(ViPipe, 0x0d08, 0x01);
	gc05a3_write_register(ViPipe, 0x0d03, 0xcf);
	gc05a3_write_register(ViPipe, 0x0d04, 0x0a);
	gc05a3_write_register(ViPipe, 0x03f3, 0xd4);
	gc05a3_write_register(ViPipe, 0x03f8, 0x59);
	gc05a3_write_register(ViPipe, 0x03f5, 0xbc);
	gc05a3_write_register(ViPipe, 0x0d07, 0x54);
	gc05a3_write_register(ViPipe, 0x00c4, 0x00);
	gc05a3_write_register(ViPipe, 0x00c5, 0x01);
	gc05a3_write_register(ViPipe, 0x0af6, 0x00);
	gc05a3_write_register(ViPipe, 0x0ba0, 0x17);
	gc05a3_write_register(ViPipe, 0x0ba1, 0x00);
	gc05a3_write_register(ViPipe, 0x0ba2, 0x00);
	gc05a3_write_register(ViPipe, 0x0ba3, 0x00);
	gc05a3_write_register(ViPipe, 0x0ba4, 0x03);
	gc05a3_write_register(ViPipe, 0x0ba5, 0x00);
	gc05a3_write_register(ViPipe, 0x0ba6, 0x00);
	gc05a3_write_register(ViPipe, 0x0ba7, 0x00);
	gc05a3_write_register(ViPipe, 0x0ba8, 0x40);
	gc05a3_write_register(ViPipe, 0x0ba9, 0x00);
	gc05a3_write_register(ViPipe, 0x0baa, 0x00);
	gc05a3_write_register(ViPipe, 0x0bab, 0x00);
	gc05a3_write_register(ViPipe, 0x0bac, 0x40);
	gc05a3_write_register(ViPipe, 0x0bad, 0x00);
	gc05a3_write_register(ViPipe, 0x0bae, 0x00);
	gc05a3_write_register(ViPipe, 0x0baf, 0x00);
	gc05a3_write_register(ViPipe, 0x0bb0, 0x02);
	gc05a3_write_register(ViPipe, 0x0bb1, 0x00);
	gc05a3_write_register(ViPipe, 0x0bb2, 0x00);
	gc05a3_write_register(ViPipe, 0x0bb3, 0x00);
	gc05a3_write_register(ViPipe, 0x0bb8, 0x02);
	gc05a3_write_register(ViPipe, 0x0bb9, 0x00);
	gc05a3_write_register(ViPipe, 0x0bba, 0x00);
	gc05a3_write_register(ViPipe, 0x0bbb, 0x00);
	gc05a3_write_register(ViPipe, 0x0a70, 0x80);
	gc05a3_write_register(ViPipe, 0x0a71, 0x00);
	gc05a3_write_register(ViPipe, 0x0a72, 0x00);
	gc05a3_write_register(ViPipe, 0x0a66, 0x00);
	gc05a3_write_register(ViPipe, 0x0a67, 0x80);
	gc05a3_write_register(ViPipe, 0x0a4d, 0x4e);
	gc05a3_write_register(ViPipe, 0x0a50, 0x00);
	gc05a3_write_register(ViPipe, 0x0a4f, 0x0c);
	gc05a3_write_register(ViPipe, 0x0a66, 0x00);
	gc05a3_write_register(ViPipe, 0x00c6, 0x07);
	gc05a3_write_register(ViPipe, 0x00c7, 0xa8);
	gc05a3_write_register(ViPipe, 0x00c8, 0x0a);
	gc05a3_write_register(ViPipe, 0x00c9, 0x30);
	gc05a3_write_register(ViPipe, 0x00ca, 0x00);
	gc05a3_write_register(ViPipe, 0x00cb, 0x00);
	gc05a3_write_register(ViPipe, 0x00cc, 0x00);
	gc05a3_write_register(ViPipe, 0x00cd, 0x00);
	gc05a3_write_register(ViPipe, 0x0aa1, 0x00);
	gc05a3_write_register(ViPipe, 0x0aa2, 0xe0);
	gc05a3_write_register(ViPipe, 0x0aa3, 0x00);
	gc05a3_write_register(ViPipe, 0x0aa4, 0x40);
	gc05a3_write_register(ViPipe, 0x0a90, 0x03);
	gc05a3_write_register(ViPipe, 0x0a91, 0x0e);
	gc05a3_write_register(ViPipe, 0x0a94, 0x80);
	gc05a3_write_register(ViPipe, 0x0af6, 0x20);
	gc05a3_write_register(ViPipe, 0x0b00, 0x91);
	gc05a3_write_register(ViPipe, 0x0b01, 0x17);
	gc05a3_write_register(ViPipe, 0x0b02, 0x01);
	gc05a3_write_register(ViPipe, 0x0b03, 0x00);
	gc05a3_write_register(ViPipe, 0x0b04, 0x01);
	gc05a3_write_register(ViPipe, 0x0b05, 0x17);
	gc05a3_write_register(ViPipe, 0x0b06, 0x01);
	gc05a3_write_register(ViPipe, 0x0b07, 0x00);
	gc05a3_write_register(ViPipe, 0x0ae9, 0x01);
	gc05a3_write_register(ViPipe, 0x0aea, 0x02);
	gc05a3_write_register(ViPipe, 0x0ae8, 0x53);
	gc05a3_write_register(ViPipe, 0x0ae8, 0x43);
	gc05a3_write_register(ViPipe, 0x0af6, 0x30);
	gc05a3_write_register(ViPipe, 0x0b00, 0x05);
	gc05a3_write_register(ViPipe, 0x0b01, 0x08);
	gc05a3_write_register(ViPipe, 0x0b02, 0x00);
	gc05a3_write_register(ViPipe, 0x0b03, 0x00);
	gc05a3_write_register(ViPipe, 0x0b04, 0x0f);
	gc05a3_write_register(ViPipe, 0x0b05, 0x1c);
	gc05a3_write_register(ViPipe, 0x0b06, 0x00);
	gc05a3_write_register(ViPipe, 0x0b07, 0x00);
	gc05a3_write_register(ViPipe, 0x0b08, 0x24);
	gc05a3_write_register(ViPipe, 0x0b09, 0x30);
	gc05a3_write_register(ViPipe, 0x0b0a, 0x00);
	gc05a3_write_register(ViPipe, 0x0b0b, 0x00);
	gc05a3_write_register(ViPipe, 0x0b0c, 0x0e);
	gc05a3_write_register(ViPipe, 0x0b0d, 0x2a);
	gc05a3_write_register(ViPipe, 0x0b0e, 0x00);
	gc05a3_write_register(ViPipe, 0x0b0f, 0x00);
	gc05a3_write_register(ViPipe, 0x0b10, 0x0e);
	gc05a3_write_register(ViPipe, 0x0b11, 0x2b);
	gc05a3_write_register(ViPipe, 0x0b12, 0x00);
	gc05a3_write_register(ViPipe, 0x0b13, 0x00);
	gc05a3_write_register(ViPipe, 0x0b14, 0x0d);
	gc05a3_write_register(ViPipe, 0x0b15, 0x21);
	gc05a3_write_register(ViPipe, 0x0b16, 0x00);
	gc05a3_write_register(ViPipe, 0x0b17, 0x00);
	gc05a3_write_register(ViPipe, 0x0b18, 0x0d);
	gc05a3_write_register(ViPipe, 0x0b19, 0x29);
	gc05a3_write_register(ViPipe, 0x0b1a, 0x00);
	gc05a3_write_register(ViPipe, 0x0b1b, 0x00);
	gc05a3_write_register(ViPipe, 0x0b1c, 0x0e);
	gc05a3_write_register(ViPipe, 0x0b1d, 0x23);
	gc05a3_write_register(ViPipe, 0x0b1e, 0x00);
	gc05a3_write_register(ViPipe, 0x0b1f, 0x00);
	gc05a3_write_register(ViPipe, 0x0b20, 0x0e);
	gc05a3_write_register(ViPipe, 0x0b21, 0x24);
	gc05a3_write_register(ViPipe, 0x0b22, 0x00);
	gc05a3_write_register(ViPipe, 0x0b23, 0x00);
	gc05a3_write_register(ViPipe, 0x0b24, 0x08);
	gc05a3_write_register(ViPipe, 0x0b25, 0x08);
	gc05a3_write_register(ViPipe, 0x0b26, 0x00);
	gc05a3_write_register(ViPipe, 0x0b27, 0x00);
	gc05a3_write_register(ViPipe, 0x0b28, 0x00);
	gc05a3_write_register(ViPipe, 0x0b29, 0x30);
	gc05a3_write_register(ViPipe, 0x0b2a, 0x00);
	gc05a3_write_register(ViPipe, 0x0b2b, 0x00);
	gc05a3_write_register(ViPipe, 0x0b2c, 0x01);
	gc05a3_write_register(ViPipe, 0x0b2d, 0x01);
	gc05a3_write_register(ViPipe, 0x0b2e, 0x00);
	gc05a3_write_register(ViPipe, 0x0b2f, 0x00);
	gc05a3_write_register(ViPipe, 0x0b30, 0x0b);
	gc05a3_write_register(ViPipe, 0x0b31, 0x0b);
	gc05a3_write_register(ViPipe, 0x0b32, 0x00);
	gc05a3_write_register(ViPipe, 0x0b33, 0x00);
	gc05a3_write_register(ViPipe, 0x0b34, 0x00);
	gc05a3_write_register(ViPipe, 0x0b35, 0x2d);
	gc05a3_write_register(ViPipe, 0x0b36, 0x00);
	gc05a3_write_register(ViPipe, 0x0b37, 0x00);
	gc05a3_write_register(ViPipe, 0x0b38, 0x01);
	gc05a3_write_register(ViPipe, 0x0b39, 0x01);
	gc05a3_write_register(ViPipe, 0x0b3a, 0x00);
	gc05a3_write_register(ViPipe, 0x0b3b, 0x00);
	gc05a3_write_register(ViPipe, 0x0b3c, 0x0e);
	gc05a3_write_register(ViPipe, 0x0b3d, 0x0e);
	gc05a3_write_register(ViPipe, 0x0b3e, 0x00);
	gc05a3_write_register(ViPipe, 0x0b3f, 0x00);
	gc05a3_write_register(ViPipe, 0x0b40, 0x00);
	gc05a3_write_register(ViPipe, 0x0b41, 0x2b);
	gc05a3_write_register(ViPipe, 0x0b42, 0x00);
	gc05a3_write_register(ViPipe, 0x0b43, 0x00);
	gc05a3_write_register(ViPipe, 0x0b44, 0x02);
	gc05a3_write_register(ViPipe, 0x0b45, 0x02);
	gc05a3_write_register(ViPipe, 0x0b46, 0x00);
	gc05a3_write_register(ViPipe, 0x0b47, 0x00);
	gc05a3_write_register(ViPipe, 0x0b48, 0x12);
	gc05a3_write_register(ViPipe, 0x0b49, 0x12);
	gc05a3_write_register(ViPipe, 0x0b4a, 0x00);
	gc05a3_write_register(ViPipe, 0x0b4b, 0x00);
	gc05a3_write_register(ViPipe, 0x0b4c, 0x80);
	gc05a3_write_register(ViPipe, 0x0b4d, 0x5d);
	gc05a3_write_register(ViPipe, 0x0b4e, 0x00);
	gc05a3_write_register(ViPipe, 0x0b4f, 0x00);
	gc05a3_write_register(ViPipe, 0x0b50, 0x05);
	gc05a3_write_register(ViPipe, 0x0b51, 0x05);
	gc05a3_write_register(ViPipe, 0x0b52, 0x00);
	gc05a3_write_register(ViPipe, 0x0b53, 0x00);
	gc05a3_write_register(ViPipe, 0x0b54, 0x16);
	gc05a3_write_register(ViPipe, 0x0b55, 0x16);
	gc05a3_write_register(ViPipe, 0x0b56, 0x00);
	gc05a3_write_register(ViPipe, 0x0b57, 0x00);
	gc05a3_write_register(ViPipe, 0x0b58, 0x80);
	gc05a3_write_register(ViPipe, 0x0b59, 0x5d);
	gc05a3_write_register(ViPipe, 0x0b5a, 0x00);
	gc05a3_write_register(ViPipe, 0x0b5b, 0x00);
	gc05a3_write_register(ViPipe, 0x0b5c, 0x0b);
	gc05a3_write_register(ViPipe, 0x0b5d, 0x0b);
	gc05a3_write_register(ViPipe, 0x0b5e, 0x00);
	gc05a3_write_register(ViPipe, 0x0b5f, 0x00);
	gc05a3_write_register(ViPipe, 0x0b60, 0x17);
	gc05a3_write_register(ViPipe, 0x0b61, 0x17);
	gc05a3_write_register(ViPipe, 0x0b62, 0x00);
	gc05a3_write_register(ViPipe, 0x0b63, 0x00);
	gc05a3_write_register(ViPipe, 0x0b64, 0x80);
	gc05a3_write_register(ViPipe, 0x0b65, 0x5d);
	gc05a3_write_register(ViPipe, 0x0b66, 0x00);
	gc05a3_write_register(ViPipe, 0x0b67, 0x00);
	gc05a3_write_register(ViPipe, 0x0b68, 0x10);
	gc05a3_write_register(ViPipe, 0x0b69, 0x10);
	gc05a3_write_register(ViPipe, 0x0b6a, 0x00);
	gc05a3_write_register(ViPipe, 0x0b6b, 0x00);
	gc05a3_write_register(ViPipe, 0x0b6c, 0x17);
	gc05a3_write_register(ViPipe, 0x0b6d, 0x17);
	gc05a3_write_register(ViPipe, 0x0b6e, 0x00);
	gc05a3_write_register(ViPipe, 0x0b6f, 0x00);
	gc05a3_write_register(ViPipe, 0x0b70, 0x80);
	gc05a3_write_register(ViPipe, 0x0b71, 0x5d);
	gc05a3_write_register(ViPipe, 0x0b72, 0x00);
	gc05a3_write_register(ViPipe, 0x0b73, 0x00);
	gc05a3_write_register(ViPipe, 0x0b74, 0x1f);
	gc05a3_write_register(ViPipe, 0x0b75, 0x1f);
	gc05a3_write_register(ViPipe, 0x0b76, 0x00);
	gc05a3_write_register(ViPipe, 0x0b77, 0x00);
	gc05a3_write_register(ViPipe, 0x0b80, 0x01);
	gc05a3_write_register(ViPipe, 0x0b81, 0x00);
	gc05a3_write_register(ViPipe, 0x0b82, 0x1b);
	gc05a3_write_register(ViPipe, 0x0b83, 0x00);
	gc05a3_write_register(ViPipe, 0x0b84, 0x00);
	gc05a3_write_register(ViPipe, 0x0b85, 0x00);
	gc05a3_write_register(ViPipe, 0x0b86, 0x1a);
	gc05a3_write_register(ViPipe, 0x0b87, 0x00);
	gc05a3_write_register(ViPipe, 0x0b88, 0x01);
	gc05a3_write_register(ViPipe, 0x0b89, 0x6b);
	gc05a3_write_register(ViPipe, 0x0b8a, 0x34);
	gc05a3_write_register(ViPipe, 0x0b8b, 0x00);
	gc05a3_write_register(ViPipe, 0x0b8c, 0x00);
	gc05a3_write_register(ViPipe, 0x0b8d, 0x01);
	gc05a3_write_register(ViPipe, 0x0b8e, 0x30);
	gc05a3_write_register(ViPipe, 0x0b8f, 0x00);
	gc05a3_write_register(ViPipe, 0x0b90, 0x01);
	gc05a3_write_register(ViPipe, 0x0b91, 0xfb);
	gc05a3_write_register(ViPipe, 0x0b92, 0x15);
	gc05a3_write_register(ViPipe, 0x0b93, 0x00);
	gc05a3_write_register(ViPipe, 0x0b94, 0x00);
	gc05a3_write_register(ViPipe, 0x0b95, 0x02);
	gc05a3_write_register(ViPipe, 0x0b96, 0x33);
	gc05a3_write_register(ViPipe, 0x0b97, 0x00);
	gc05a3_write_register(ViPipe, 0x0b98, 0x02);
	gc05a3_write_register(ViPipe, 0x0b99, 0xd3);
	gc05a3_write_register(ViPipe, 0x0b9a, 0x1d);
	gc05a3_write_register(ViPipe, 0x0b9b, 0x00);
	gc05a3_write_register(ViPipe, 0x0b9c, 0x00);
	gc05a3_write_register(ViPipe, 0x0b9d, 0x03);
	gc05a3_write_register(ViPipe, 0x0b9e, 0x20);
	gc05a3_write_register(ViPipe, 0x0b9f, 0x00);
	gc05a3_write_register(ViPipe, 0x0ba0, 0x03);
	gc05a3_write_register(ViPipe, 0x0ba1, 0xf0);
	gc05a3_write_register(ViPipe, 0x0ba2, 0x27);
	gc05a3_write_register(ViPipe, 0x0ba3, 0x00);
	gc05a3_write_register(ViPipe, 0x0ba4, 0x00);
	gc05a3_write_register(ViPipe, 0x0ba5, 0x04);
	gc05a3_write_register(ViPipe, 0x0ba6, 0x12);
	gc05a3_write_register(ViPipe, 0x0ba7, 0x00);
	gc05a3_write_register(ViPipe, 0x0ba8, 0x05);
	gc05a3_write_register(ViPipe, 0x0ba9, 0x7e);
	gc05a3_write_register(ViPipe, 0x0baa, 0x37);
	gc05a3_write_register(ViPipe, 0x0bab, 0x00);
	gc05a3_write_register(ViPipe, 0x0bac, 0x00);
	gc05a3_write_register(ViPipe, 0x0bad, 0x05);
	gc05a3_write_register(ViPipe, 0x0bae, 0x1a);
	gc05a3_write_register(ViPipe, 0x0baf, 0x01);
	gc05a3_write_register(ViPipe, 0x0bb0, 0x07);
	gc05a3_write_register(ViPipe, 0x0bb1, 0xac);
	gc05a3_write_register(ViPipe, 0x0bb2, 0x50);
	gc05a3_write_register(ViPipe, 0x0bb3, 0x01);
	gc05a3_write_register(ViPipe, 0x0bb4, 0x00);
	gc05a3_write_register(ViPipe, 0x0bb5, 0x06);
	gc05a3_write_register(ViPipe, 0x0bb6, 0x24);
	gc05a3_write_register(ViPipe, 0x0bb7, 0x01);
	gc05a3_write_register(ViPipe, 0x0bb8, 0x0a);
	gc05a3_write_register(ViPipe, 0x0bb9, 0xca);
	gc05a3_write_register(ViPipe, 0x0bba, 0x67);
	gc05a3_write_register(ViPipe, 0x0bbb, 0x01);
	gc05a3_write_register(ViPipe, 0x0bbc, 0x09);
	gc05a3_write_register(ViPipe, 0x0bbd, 0x36);
	gc05a3_write_register(ViPipe, 0x0bbe, 0x30);
	gc05a3_write_register(ViPipe, 0x0bbf, 0x01);
	gc05a3_write_register(ViPipe, 0x0bc0, 0x0f);
	gc05a3_write_register(ViPipe, 0x0bc1, 0x9a);
	gc05a3_write_register(ViPipe, 0x0bc2, 0x8d);
	gc05a3_write_register(ViPipe, 0x0bc3, 0x01);
	gc05a3_write_register(ViPipe, 0x0bc4, 0x10);
	gc05a3_write_register(ViPipe, 0x0bc5, 0x06);
	gc05a3_write_register(ViPipe, 0x0bc6, 0x40);
	gc05a3_write_register(ViPipe, 0x0bc7, 0x01);
	gc05a3_write_register(ViPipe, 0x02c3, 0x76);
	gc05a3_write_register(ViPipe, 0x02c2, 0x10);
	gc05a3_write_register(ViPipe, 0x02c1, 0xe0);
	gc05a3_write_register(ViPipe, 0x02c5, 0x09);
	gc05a3_write_register(ViPipe, 0x0207, 0x04);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0050, 0x00);
	gc05a3_write_register(ViPipe, 0x0089, 0x03);
	gc05a3_write_register(ViPipe, 0x005a, 0x40);
	gc05a3_write_register(ViPipe, 0x00c3, 0x35);
	gc05a3_write_register(ViPipe, 0x00c4, 0x80);
	gc05a3_write_register(ViPipe, 0x0080, 0x10);
	gc05a3_write_register(ViPipe, 0x0058, 0x89);
	gc05a3_write_register(ViPipe, 0x00d0, 0x45);
	gc05a3_write_register(ViPipe, 0x009b, 0x07);
	gc05a3_write_register(ViPipe, 0x004a, 0x03);
	gc05a3_write_register(ViPipe, 0x0048, 0xf0);
	gc05a3_write_register(ViPipe, 0x0049, 0x0f);
	gc05a3_write_register(ViPipe, 0x0041, 0x20);
	gc05a3_write_register(ViPipe, 0x0043, 0x0a);
	gc05a3_write_register(ViPipe, 0x009d, 0x0a);
	gc05a3_write_register(ViPipe, 0x0236, 0x40);
	gc05a3_write_register(ViPipe, 0x0204, 0x04);
	gc05a3_write_register(ViPipe, 0x0205, 0x00);
	gc05a3_write_register(ViPipe, 0x02b3, 0x00);
	gc05a3_write_register(ViPipe, 0x02b4, 0x00);
	gc05a3_write_register(ViPipe, 0x009e, 0x01);
	gc05a3_write_register(ViPipe, 0x009f, 0x86);
	gc05a3_write_register(ViPipe, 0x0aa1, 0x15);
	gc05a3_write_register(ViPipe, 0x0aa2, 0x50);
	gc05a3_write_register(ViPipe, 0x0aa3, 0x00);
	gc05a3_write_register(ViPipe, 0x0aa4, 0x09);
	gc05a3_write_register(ViPipe, 0x0a90, 0x25);
	gc05a3_write_register(ViPipe, 0x0a91, 0x0e);
	gc05a3_write_register(ViPipe, 0x0a94, 0x80);
	gc05a3_write_register(ViPipe, 0x0350, 0x01);
	gc05a3_write_register(ViPipe, 0x0353, 0x00);
	gc05a3_write_register(ViPipe, 0x0354, 0x08);
	gc05a3_write_register(ViPipe, 0x034c, 0x0a);
	gc05a3_write_register(ViPipe, 0x034d, 0x20);
	gc05a3_write_register(ViPipe, 0x021f, 0x14);
	gc05a3_write_register(ViPipe, 0x0aa1, 0x10);
	gc05a3_write_register(ViPipe, 0x0aa2, 0xf8);
	gc05a3_write_register(ViPipe, 0x0aa3, 0x00);
	gc05a3_write_register(ViPipe, 0x0aa4, 0x1f);
	gc05a3_write_register(ViPipe, 0x0a90, 0x11);
	gc05a3_write_register(ViPipe, 0x0a91, 0x0e);
	gc05a3_write_register(ViPipe, 0x0a94, 0x80);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0aff, 0x00);
	gc05a3_write_register(ViPipe, 0x0a90, 0x00);
	gc05a3_write_register(ViPipe, 0x0a70, 0x00);
	gc05a3_write_register(ViPipe, 0x0a67, 0x00);
	gc05a3_write_register(ViPipe, 0x0af4, 0x29);
	gc05a3_write_register(ViPipe, 0x0d80, 0x07);
	gc05a3_write_register(ViPipe, 0x0dd3, 0x04);
	gc05a3_write_register(ViPipe, 0x0dd4, 0x10);
	gc05a3_write_register(ViPipe, 0x0ddc, 0x04);
	gc05a3_write_register(ViPipe, 0x0de2, 0x04);
	gc05a3_write_register(ViPipe, 0x0107, 0x05);
	gc05a3_write_register(ViPipe, 0x0117, 0x01);
	gc05a3_write_register(ViPipe, 0x0d81, 0x00);
	gc05a3_write_register(ViPipe, 0x0223, 0x01);
	gc05a3_write_register(ViPipe, 0x000b, 0x06);
	gc05a3_write_register(ViPipe, 0x031c, 0x80);
	gc05a3_write_register(ViPipe, 0x03fe, 0x30);
	gc05a3_write_register(ViPipe, 0x0d17, 0x06);
	gc05a3_write_register(ViPipe, 0x03fe, 0x00);
	gc05a3_write_register(ViPipe, 0x0d17, 0x00);
	gc05a3_write_register(ViPipe, 0x031c, 0x93);
	gc05a3_write_register(ViPipe, 0x03fe, 0x00);
	gc05a3_write_register(ViPipe, 0x03fe, 0x00);
	gc05a3_write_register(ViPipe, 0x03fe, 0x00);
	gc05a3_write_register(ViPipe, 0x03fe, 0x00);
	gc05a3_write_register(ViPipe, 0x03fe, 0x00);
	gc05a3_write_register(ViPipe, 0x03fe, 0x00);
	gc05a3_write_register(ViPipe, 0x03fe, 0x00);
	gc05a3_write_register(ViPipe, 0x03fe, 0x00);
	gc05a3_write_register(ViPipe, 0x03fe, 0x00);
	gc05a3_write_register(ViPipe, 0x03fe, 0x00);
	gc05a3_write_register(ViPipe, 0x03fe, 0x00);
	gc05a3_write_register(ViPipe, 0x03fe, 0x00);
	gc05a3_write_register(ViPipe, 0x031c, 0x80);
	gc05a3_write_register(ViPipe, 0x03fe, 0x30);
	gc05a3_write_register(ViPipe, 0x0d17, 0x06);
	gc05a3_write_register(ViPipe, 0x03fe, 0x00);
	gc05a3_write_register(ViPipe, 0x0d17, 0x00);
	gc05a3_write_register(ViPipe, 0x031c, 0x93);
	gc05a3_write_register(ViPipe, 0x0315, 0x80);
	gc05a3_write_register(ViPipe, 0x0315, 0xd4);
	gc05a3_write_register(ViPipe, 0x0d06, 0x01);
	gc05a3_write_register(ViPipe, 0x0a70, 0x80);
	gc05a3_write_register(ViPipe, 0x031a, 0x00);
	gc05a3_write_register(ViPipe, 0x0314, 0x00);
	gc05a3_write_register(ViPipe, 0x0130, 0x08);
	gc05a3_write_register(ViPipe, 0x0132, 0x01);
	gc05a3_write_register(ViPipe, 0x0135, 0x01);
	gc05a3_write_register(ViPipe, 0x0136, 0x36);
	gc05a3_write_register(ViPipe, 0x0137, 0x03);
	gc05a3_write_register(ViPipe, 0x0134, 0x5b);
	gc05a3_write_register(ViPipe, 0x031c, 0xe0);
	gc05a3_write_register(ViPipe, 0x0d82, 0x14);
	gc05a3_write_register(ViPipe, 0x0dd1, 0x56);
	gc05a3_write_register(ViPipe, 0x0af4, 0x01);
	gc05a3_write_register(ViPipe, 0x0002, 0x10);
	gc05a3_write_register(ViPipe, 0x00c3, 0x34);
	gc05a3_write_register(ViPipe, 0x0084, 0x21);
	gc05a3_write_register(ViPipe, 0x0d05, 0xcc);
	gc05a3_write_register(ViPipe, 0x0218, 0x00);
	gc05a3_write_register(ViPipe, 0x005e, 0x48);
	gc05a3_write_register(ViPipe, 0x0d06, 0x01);
	gc05a3_write_register(ViPipe, 0x0007, 0x16);
	gc05a3_write_register(ViPipe, 0x0342, 0x06);
	gc05a3_write_register(ViPipe, 0x0343, 0xe8);
	gc05a3_write_register(ViPipe, 0x0220, 0x07);
	gc05a3_write_register(ViPipe, 0x0221, 0xd0);
	gc05a3_write_register(ViPipe, 0x0202, 0x07);
	gc05a3_write_register(ViPipe, 0x0203, 0x32);
	gc05a3_write_register(ViPipe, 0x0340, 0x0b);
	gc05a3_write_register(ViPipe, 0x0341, 0xe8);
	gc05a3_write_register(ViPipe, 0x0219, 0x00);
	gc05a3_write_register(ViPipe, 0x0210, 0x10);
	gc05a3_write_register(ViPipe, 0x0346, 0x00);
	gc05a3_write_register(ViPipe, 0x0347, 0x04);
	gc05a3_write_register(ViPipe, 0x0d14, 0x00);
	gc05a3_write_register(ViPipe, 0x0d13, 0x05);
	gc05a3_write_register(ViPipe, 0x0d16, 0x05);
	gc05a3_write_register(ViPipe, 0x0d15, 0x1d);
	gc05a3_write_register(ViPipe, 0x00c0, 0x0a);
	gc05a3_write_register(ViPipe, 0x00c1, 0x30);
	gc05a3_write_register(ViPipe, 0x034a, 0x07);
	gc05a3_write_register(ViPipe, 0x034b, 0xa8);
	gc05a3_write_register(ViPipe, 0x00ca, 0x00);
	gc05a3_write_register(ViPipe, 0x00cb, 0x00);
	gc05a3_write_register(ViPipe, 0x00cc, 0x00);
	gc05a3_write_register(ViPipe, 0x00cd, 0x00);
	gc05a3_write_register(ViPipe, 0x00c3, 0x35);
	gc05a3_write_register(ViPipe, 0x0350, 0x01);
	gc05a3_write_register(ViPipe, 0x0353, 0x00);
	gc05a3_write_register(ViPipe, 0x0354, 0x08);
	gc05a3_write_register(ViPipe, 0x034c, 0x0a);
	gc05a3_write_register(ViPipe, 0x034d, 0x20);
	gc05a3_write_register(ViPipe, 0x021f, 0x14);
	gc05a3_write_register(ViPipe, 0x0d84, 0x0c);
	gc05a3_write_register(ViPipe, 0x0d85, 0xa8);
	gc05a3_write_register(ViPipe, 0x0d86, 0x06);
	gc05a3_write_register(ViPipe, 0x0d87, 0x55);
	gc05a3_write_register(ViPipe, 0x0db3, 0x12);
	gc05a3_write_register(ViPipe, 0x0db4, 0x08);
	gc05a3_write_register(ViPipe, 0x0db5, 0x1c);
	gc05a3_write_register(ViPipe, 0x0db6, 0x07);
	gc05a3_write_register(ViPipe, 0x0db8, 0x26);
	gc05a3_write_register(ViPipe, 0x0db9, 0x0a);
	gc05a3_write_register(ViPipe, 0x0d93, 0x06);
	gc05a3_write_register(ViPipe, 0x0d94, 0x08);
	gc05a3_write_register(ViPipe, 0x0d95, 0x11);
	gc05a3_write_register(ViPipe, 0x0d99, 0x0b);
	gc05a3_write_register(ViPipe, 0x0223, 0x01);
	gc05a3_write_register(ViPipe, 0x000b, 0x06);
	gc05a3_write_register(ViPipe, 0x0084, 0x01);
	gc05a3_write_register(ViPipe, 0x031c, 0x80);
	gc05a3_write_register(ViPipe, 0x03fe, 0x30);
	gc05a3_write_register(ViPipe, 0x0d17, 0x06);
	gc05a3_write_register(ViPipe, 0x03fe, 0x00);
	gc05a3_write_register(ViPipe, 0x0d17, 0x00);
	gc05a3_write_register(ViPipe, 0x031c, 0x93);
	gc05a3_write_register(ViPipe, 0x03fe, 0x00);
	gc05a3_write_register(ViPipe, 0x03fe, 0x00);
	gc05a3_write_register(ViPipe, 0x03fe, 0x00);
	gc05a3_write_register(ViPipe, 0x03fe, 0x00);
	gc05a3_write_register(ViPipe, 0x03fe, 0x00);
	gc05a3_write_register(ViPipe, 0x03fe, 0x00);
	gc05a3_write_register(ViPipe, 0x03fe, 0x00);
	gc05a3_write_register(ViPipe, 0x03fe, 0x00);
	gc05a3_write_register(ViPipe, 0x03fe, 0x00);
	gc05a3_write_register(ViPipe, 0x03fe, 0x00);
	gc05a3_write_register(ViPipe, 0x03fe, 0x00);
	gc05a3_write_register(ViPipe, 0x03fe, 0x00);
	gc05a3_write_register(ViPipe, 0x031c, 0x80);
	gc05a3_write_register(ViPipe, 0x03fe, 0x30);
	gc05a3_write_register(ViPipe, 0x0d17, 0x06);
	gc05a3_write_register(ViPipe, 0x03fe, 0x00);
	gc05a3_write_register(ViPipe, 0x0d17, 0x00);
	gc05a3_write_register(ViPipe, 0x031c, 0x93);
	gc05a3_write_register(ViPipe, 0x0110, 0x01);
	gc05a3_write_register(ViPipe, 0x0100, 0x01);

	gc05a3_default_reg_init(ViPipe);
	delay_ms(10);

	printf("ViPipe:%d,===GC05A3 2592x1944 22.5fps 10bit LINEAR Init OK!===\n", ViPipe);
}
