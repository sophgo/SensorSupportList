#include <unistd.h>
#include <cvi_comm_video.h>
#include "cvi_sns_ctrl.h"
#include "s5k3p3st_cmos_ex.h"
#include "sensor_i2c.h"

#define S5K3P3ST_PATTERN_ENABLE	0

static void s5k3p3st_linear_1744p30_init(VI_PIPE ViPipe);

CVI_U8 s5k3p3st_i2c_addr = 0x30;        /* I2C Address of S5K3P3ST */
const CVI_U32 s5k3p3st_addr_byte = 2;
const CVI_U32 s5k3p3st_data_byte = 2;

int s5k3p3st_i2c_init(VI_PIPE ViPipe)
{
	return sensor_i2c_init(ViPipe, (CVI_U8)g_aunS5K3P3ST_BusInfo[ViPipe].s8I2cDev,
							(CVI_U8)g_aunS5K3P3ST_AddrInfo[ViPipe].s8I2cAddr);
}

int s5k3p3st_i2c_exit(VI_PIPE ViPipe)
{
	return sensor_i2c_exit(ViPipe, (CVI_U8)g_aunS5K3P3ST_BusInfo[ViPipe].s8I2cDev);
}

int s5k3p3st_read_register(VI_PIPE ViPipe, int addr)
{
	return sensor_i2c_read(ViPipe, (CVI_U8)g_aunS5K3P3ST_BusInfo[ViPipe].s8I2cDev,
							(CVI_U8)g_aunS5K3P3ST_AddrInfo[ViPipe].s8I2cAddr, (CVI_U32)addr,
							s5k3p3st_addr_byte, s5k3p3st_data_byte);
}

int s5k3p3st_write_register(VI_PIPE ViPipe, int addr, int data)
{
	return sensor_i2c_write(ViPipe, (CVI_U8)g_aunS5K3P3ST_BusInfo[ViPipe].s8I2cDev,
							(CVI_U8)g_aunS5K3P3ST_AddrInfo[ViPipe].s8I2cAddr, (CVI_U32)addr,
							s5k3p3st_addr_byte, (CVI_U32)data, s5k3p3st_data_byte);
}

static void delay_ms(int ms)
{
	usleep(ms * 1000);
}

void s5k3p3st_standby(VI_PIPE ViPipe)
{
	s5k3p3st_write_register(ViPipe, 0x0100, 0x00);
}

void s5k3p3st_restart(VI_PIPE ViPipe)
{
	s5k3p3st_write_register(ViPipe, 0x0100, 0x00);
	delay_ms(20);
	s5k3p3st_write_register(ViPipe, 0x0100, 0x01);
}

void s5k3p3st_default_reg_init(VI_PIPE ViPipe)
{
	CVI_U32 i;

	for (i = 0; i < g_pastS5K3P3ST[ViPipe]->astSyncInfo[0].snsCfg.u32RegNum; i++) {
		s5k3p3st_write_register(ViPipe,
				g_pastS5K3P3ST[ViPipe]->astSyncInfo[0].snsCfg.astI2cData[i].u32RegAddr,
				g_pastS5K3P3ST[ViPipe]->astSyncInfo[0].snsCfg.astI2cData[i].u32Data);
	}
}

#define S5K3P3ST_CHIP_ID_ADDR		0x0000
#define S5K3P3ST_CHIP_ID			0x3103

void s5k3p3st_mirror_flip(VI_PIPE ViPipe, ISP_SNS_MIRRORFLIP_TYPE_E eSnsMirrorFlip)
{
	CVI_U8 val = 0;

	switch (eSnsMirrorFlip) {
	case ISP_SNS_NORMAL:
		break;
	case ISP_SNS_MIRROR:
		val |= 0x1;
		break;
	case ISP_SNS_FLIP:
		val |= 0x2;
		break;
	case ISP_SNS_MIRROR_FLIP:
		val |= 0x3;
		break;
	default:
		return;
	}

	s5k3p3st_write_register(ViPipe, 0x0101, val);
}

int s5k3p3st_probe(VI_PIPE ViPipe)
{
	int nVal;
	CVI_U16 chip_id;

	delay_ms(4);
	if (s5k3p3st_i2c_init(ViPipe) != CVI_SUCCESS)
		return CVI_FAILURE;

	nVal = s5k3p3st_read_register(ViPipe, S5K3P3ST_CHIP_ID_ADDR);
	if (nVal < 0) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "read sensor id error.\n");
		return nVal;
	}
	chip_id = nVal;

	if (chip_id != S5K3P3ST_CHIP_ID) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "Sensor ID Mismatch! Use the wrong sensor?? chip_value = %x\n", chip_id);
		return CVI_FAILURE;
	}

	return CVI_SUCCESS;
}


void s5k3p3st_init(VI_PIPE ViPipe)
{
	CVI_U8 u8ImgMode = g_pastS5K3P3ST[ViPipe]->u8ImgMode;

	s5k3p3st_i2c_init(ViPipe);

	if (u8ImgMode == S5K3P3ST_MODE_1744P30) {
		s5k3p3st_linear_1744p30_init(ViPipe);
	} else {
		s5k3p3st_linear_1744p30_init(ViPipe);
		CVI_TRACE_SNS(CVI_DBG_ERR, "Sensor imagemode[%d] unsupport!, use default mode\n", u8ImgMode);
	}

	g_pastS5K3P3ST[ViPipe]->bInit = CVI_TRUE;
}

static void s5k3p3st_linear_1744p30_init(VI_PIPE ViPipe)
{
	s5k3p3st_write_register(ViPipe, 0x6028, 0x4000);
	s5k3p3st_write_register(ViPipe, 0x602A, 0x6010);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x0001);
	s5k3p3st_write_register(ViPipe, 0x6028, 0x4000);
	s5k3p3st_write_register(ViPipe, 0x602A, 0x0114);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x0100);//2lane
	s5k3p3st_write_register(ViPipe, 0x6028, 0x4000);
	s5k3p3st_write_register(ViPipe, 0x602A, 0x6214);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x7971);
	s5k3p3st_write_register(ViPipe, 0x602A, 0x6218);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x0100);
	s5k3p3st_write_register(ViPipe, 0x602A, 0xF408);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x0048);
	s5k3p3st_write_register(ViPipe, 0x602A, 0xF40C);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x0000);
	s5k3p3st_write_register(ViPipe, 0x602A, 0xF4AA);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x0060);
	s5k3p3st_write_register(ViPipe, 0x602A, 0xF442);
	delay_ms(5);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x0800);
	s5k3p3st_write_register(ViPipe, 0x602A, 0xF43E);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x0400);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x0000);
	s5k3p3st_write_register(ViPipe, 0x602A, 0xF4A4);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x0010);
	s5k3p3st_write_register(ViPipe, 0x602A, 0xF4AC);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x0056);
	s5k3p3st_write_register(ViPipe, 0x602A, 0xF480);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x0008);
	s5k3p3st_write_register(ViPipe, 0x602A, 0xF492);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x0016);
	s5k3p3st_write_register(ViPipe, 0x602A, 0x3E58);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x0056);
	s5k3p3st_write_register(ViPipe, 0x602A, 0x39EE);
	delay_ms(2);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x0206);
	delay_ms(2);
	s5k3p3st_write_register(ViPipe, 0x602A, 0x39E8);
	delay_ms(2);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x0205);
	delay_ms(2);
	s5k3p3st_write_register(ViPipe, 0x6028, 0x2000);
	s5k3p3st_write_register(ViPipe, 0x602A, 0x14B0);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0xF412);
	s5k3p3st_write_register(ViPipe, 0x6028, 0x4000);
	s5k3p3st_write_register(ViPipe, 0x602A, 0x3A36);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x33F0);
	s5k3p3st_write_register(ViPipe, 0x602A, 0x32B2);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x0132);
	s5k3p3st_write_register(ViPipe, 0x602A, 0x3A38);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x006C);
	s5k3p3st_write_register(ViPipe, 0x602A, 0x3552);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x00D0);
	s5k3p3st_write_register(ViPipe, 0x602A, 0x3195);
	delay_ms(2);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x0101);
	delay_ms(2);
	s5k3p3st_write_register(ViPipe, 0x6028, 0x2000);
	s5k3p3st_write_register(ViPipe, 0x602A, 0x13EC);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x8011);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x8011);
	s5k3p3st_write_register(ViPipe, 0x6028, 0x4000);
	s5k3p3st_write_register(ViPipe, 0x602A, 0x3002);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x0001);
	delay_ms(2);
	s5k3p3st_write_register(ViPipe, 0x602A, 0x0136);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x1800);
	s5k3p3st_write_register(ViPipe, 0x602A, 0x0304);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x0006);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x008C);
	s5k3p3st_write_register(ViPipe, 0x602A, 0x030C);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x0004);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x0078);
	s5k3p3st_write_register(ViPipe, 0x602A, 0x3008);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x0001);
	s5k3p3st_write_register(ViPipe, 0x602A, 0x0302);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x0001);
	s5k3p3st_write_register(ViPipe, 0x602A, 0x0300);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x0008);
	s5k3p3st_write_register(ViPipe, 0x602A, 0x030A);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x0001);
	s5k3p3st_write_register(ViPipe, 0x602A, 0x0308);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x0008);
	s5k3p3st_write_register(ViPipe, 0x602A, 0x0202);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x0100);
	s5k3p3st_write_register(ViPipe, 0x602A, 0x0200);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x0000);
	s5k3p3st_write_register(ViPipe, 0x602A, 0x021E);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x0100);
	s5k3p3st_write_register(ViPipe, 0x602A, 0x021C);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x0200);
	s5k3p3st_write_register(ViPipe, 0x602A, 0x0344);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x0000);
	s5k3p3st_write_register(ViPipe, 0x602A, 0x0348);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x090F);
	s5k3p3st_write_register(ViPipe, 0x602A, 0x0346);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x0000);
	s5k3p3st_write_register(ViPipe, 0x602A, 0x034A);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x06D3);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x0910);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x06D0);
	s5k3p3st_write_register(ViPipe, 0x602A, 0x0342);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x1400);
	s5k3p3st_write_register(ViPipe, 0x602A, 0x0340);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x071E);
	s5k3p3st_write_register(ViPipe, 0x602A, 0x39BA);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x0001);
	s5k3p3st_write_register(ViPipe, 0x602A, 0x3005);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x0801);
	s5k3p3st_write_register(ViPipe, 0x602A, 0x39AB);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x0001);
	delay_ms(2);
	s5k3p3st_write_register(ViPipe, 0x6028, 0x2000);
	s5k3p3st_write_register(ViPipe, 0x602A, 0x026C);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x41F0);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x0000);
	s5k3p3st_write_register(ViPipe, 0x6028, 0x4000);
	s5k3p3st_write_register(ViPipe, 0x602A, 0x37D4);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x002D);
	s5k3p3st_write_register(ViPipe, 0x602A, 0x37DA);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x005D);
	s5k3p3st_write_register(ViPipe, 0x602A, 0x37E0);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x008D);
	s5k3p3st_write_register(ViPipe, 0x602A, 0x37E6);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x00BD);
	s5k3p3st_write_register(ViPipe, 0x602A, 0x37EC);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x00ED);
	s5k3p3st_write_register(ViPipe, 0x602A, 0x37F2);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x011D);
	s5k3p3st_write_register(ViPipe, 0x602A, 0x37F8);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x014D);
	s5k3p3st_write_register(ViPipe, 0x602A, 0x37FE);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x017D);
	s5k3p3st_write_register(ViPipe, 0x602A, 0x3804);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x01AD);
	s5k3p3st_write_register(ViPipe, 0x602A, 0x380A);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x01DD);
	s5k3p3st_write_register(ViPipe, 0x602A, 0x3810);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x020D);
	s5k3p3st_write_register(ViPipe, 0x602A, 0x32A6);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x0006);
	s5k3p3st_write_register(ViPipe, 0x602A, 0x32BE);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x0006);
	s5k3p3st_write_register(ViPipe, 0x602A, 0x3210);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x0006);
	s5k3p3st_write_register(ViPipe, 0x602A, 0x3072);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x03C0);
	s5k3p3st_write_register(ViPipe, 0x602A, 0x6214);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x7970);
	s5k3p3st_write_register(ViPipe, 0x602A, 0x0100);
	s5k3p3st_write_register(ViPipe, 0x6F12, 0x0100);
	delay_ms(200);
	s5k3p3st_write_register(ViPipe, 0x0202, 0x0400);
	s5k3p3st_write_register(ViPipe, 0x0204, 0x0020);

	s5k3p3st_default_reg_init(ViPipe);

	printf("ViPipe:%d,===S5K3P3ST 2320*1744 30fps 10bit LINE Init OK!===\n", ViPipe);
}
