#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <syslog.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/cvi_comm_video.h>
#include "cvi_sns_ctrl.h"
#include "gc20c3_cmos_ex.h"

#define GC20C3_CHIP_ID_ADDR_H	0x03f0
#define GC20C3_CHIP_ID_ADDR_L	0x03f1
#define GC20C3_CHIP_ID		0x20c3

static void gc20c3_linear_1080p30_init(VI_PIPE ViPipe);

CVI_U8 gc20c3_i2c_addr = 0x31;
const CVI_U32 gc20c3_addr_byte = 2;
const CVI_U32 gc20c3_data_byte = 1;
static int g_fd[VI_MAX_PIPE_NUM] = {[0 ... (VI_MAX_PIPE_NUM - 1)] = -1};

int gc20c3_i2c_init(VI_PIPE ViPipe)
{
	char acDevFile[16] = {0};
	CVI_U8 u8DevNum;

	if (g_fd[ViPipe] >= 0)
		return CVI_SUCCESS;
	int ret;

	u8DevNum = g_aunGc20c3_BusInfo[ViPipe].s8I2cDev;
	snprintf(acDevFile, sizeof(acDevFile),  "/dev/i2c-%u", u8DevNum);

	g_fd[ViPipe] = open(acDevFile, O_RDWR, 0600);

	if (g_fd[ViPipe] < 0) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "Open /dev/i2c-%u error!\n", u8DevNum);
		return CVI_FAILURE;
	}

	ret = ioctl(g_fd[ViPipe], I2C_SLAVE_FORCE, gc20c3_i2c_addr);
	if (ret < 0) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "I2C_SLAVE_FORCE error!\n");
		close(g_fd[ViPipe]);
		g_fd[ViPipe] = -1;
		return ret;
	}

	return CVI_SUCCESS;
}

int gc20c3_i2c_exit(VI_PIPE ViPipe)
{
	if (g_fd[ViPipe] >= 0) {
		close(g_fd[ViPipe]);
		g_fd[ViPipe] = -1;
		return CVI_SUCCESS;
	}
	return CVI_FAILURE;
}

int gc20c3_read_register(VI_PIPE ViPipe, int addr)
{
	int ret, data;
	CVI_U8 buf[8];
	CVI_U8 idx = 0;

	if (g_fd[ViPipe] < 0)
		return CVI_FAILURE;

	if (gc20c3_addr_byte == 2)
		buf[idx++] = (addr >> 8) & 0xff;

	buf[idx++] = addr & 0xff;

	ret = write(g_fd[ViPipe], buf, gc20c3_addr_byte);
	if (ret < 0) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "I2C_WRITE error!\n");
		return ret;
	}

	buf[0] = 0;
	buf[1] = 0;
	ret = read(g_fd[ViPipe], buf, gc20c3_data_byte);
	if (ret < 0) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "I2C_READ error!\n");
		return ret;
	}

	data = 0;
	if (gc20c3_data_byte == 2) {
		data = buf[0] << 8;
		data += buf[1];
	} else {
		data = buf[0];
	}

	syslog(LOG_DEBUG, "i2c r 0x%x = 0x%x\n", addr, data);
	return data;
}

int gc20c3_write_register(VI_PIPE ViPipe, int addr, int data)
{
	CVI_U8 idx = 0;
	int ret;
	CVI_U8 buf[8];

	if (g_fd[ViPipe] < 0)
		return CVI_SUCCESS;

	if (gc20c3_addr_byte == 2) {
		buf[idx] = (addr >> 8) & 0xff;
		idx++;
		buf[idx] = addr & 0xff;
		idx++;
	}
	if (gc20c3_data_byte == 1) {
		buf[idx] = data & 0xff;
		idx++;
	}

	ret = write(g_fd[ViPipe], buf, gc20c3_addr_byte + gc20c3_data_byte);
	if (ret < 0) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "I2C_WRITE error!\n");
		return CVI_FAILURE;
	}
	ret = read(g_fd[ViPipe], buf, gc20c3_addr_byte + gc20c3_data_byte);
	syslog(LOG_DEBUG, "i2c w 0x%x 0x%x\n", addr, data);
	return CVI_SUCCESS;
}

static void delay_ms(int ms)
{
	usleep(ms * 1000);
}

void gc20c3_standby(VI_PIPE ViPipe)
{
	gc20c3_write_register(ViPipe, 0x0100, 0x00);
	gc20c3_write_register(ViPipe, 0x031c, 0xc7);
	gc20c3_write_register(ViPipe, 0x0317, 0x01);

	printf("gc20c3_standby\n");
}

void gc20c3_restart(VI_PIPE ViPipe)
{
	gc20c3_write_register(ViPipe, 0x0317, 0x00);
	gc20c3_write_register(ViPipe, 0x031c, 0xc6);
	gc20c3_write_register(ViPipe, 0x0100, 0x09);

	printf("gc20c3_restart\n");
}

void gc20c3_default_reg_init(VI_PIPE ViPipe)
{
	CVI_U32 i;

	for (i = 0; i < g_pastGc20c3[ViPipe]->astSyncInfo[0].snsCfg.u32RegNum; i++) {
		gc20c3_write_register(ViPipe,
				g_pastGc20c3[ViPipe]->astSyncInfo[0].snsCfg.astI2cData[i].u32RegAddr,
				g_pastGc20c3[ViPipe]->astSyncInfo[0].snsCfg.astI2cData[i].u32Data);
	}
}

int gc20c3_probe(VI_PIPE ViPipe)
{
	int nVal;
	int nVal2;

	usleep(50);
	if (gc20c3_i2c_init(ViPipe) != CVI_SUCCESS)
		return CVI_FAILURE;

	nVal  = gc20c3_read_register(ViPipe, GC20C3_CHIP_ID_ADDR_H);
	nVal2 = gc20c3_read_register(ViPipe, GC20C3_CHIP_ID_ADDR_L);
	if (nVal < 0 || nVal2 < 0) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "read sensor id error.\n");
		return nVal;
	}

	if ((((nVal & 0xFF) << 8) | (nVal2 & 0xFF)) != GC20C3_CHIP_ID) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "Sensor ID Mismatch! Use the wrong sensor??\n");
		return CVI_FAILURE;
	}

	return CVI_SUCCESS;
}

void gc20c3_init(VI_PIPE ViPipe)
{
	gc20c3_i2c_init(ViPipe);

	gc20c3_linear_1080p30_init(ViPipe);

	g_pastGc20c3[ViPipe]->bInit = CVI_TRUE;
}

void gc20c3_exit(VI_PIPE ViPipe)
{
	gc20c3_i2c_exit(ViPipe);
}

static void gc20c3_linear_1080p30_init(VI_PIPE ViPipe)
{
	delay_ms(10);
	/****system****/
	gc20c3_write_register(ViPipe, 0x03fe, 0xff);
	gc20c3_write_register(ViPipe, 0x03fe, 0x00);
	gc20c3_write_register(ViPipe, 0x03fe, 0x10);
	gc20c3_write_register(ViPipe, 0x0190, 0x03);
	gc20c3_write_register(ViPipe, 0x0b4d, 0x02);
	gc20c3_write_register(ViPipe, 0x0d40, 0x01);
	gc20c3_write_register(ViPipe, 0x0087, 0x50);
	gc20c3_write_register(ViPipe, 0x0209, 0x00);
	gc20c3_write_register(ViPipe, 0x03b5, 0x11);
	gc20c3_write_register(ViPipe, 0x031c, 0x18);
	gc20c3_write_register(ViPipe, 0x03b2, 0x03);
	gc20c3_write_register(ViPipe, 0x03bb, 0xff);
	gc20c3_write_register(ViPipe, 0x03be, 0xff);
	gc20c3_write_register(ViPipe, 0x0d11, 0x0b);
	gc20c3_write_register(ViPipe, 0x0d15, 0x03);
	gc20c3_write_register(ViPipe, 0x0d12, 0x05);
	gc20c3_write_register(ViPipe, 0x0d16, 0x00);
	gc20c3_write_register(ViPipe, 0x0d17, 0x78);
	gc20c3_write_register(ViPipe, 0x0d10, 0x06);
	gc20c3_write_register(ViPipe, 0x0d10, 0x07);
	gc20c3_write_register(ViPipe, 0x0d1a, 0x02);
	gc20c3_write_register(ViPipe, 0x0d18, 0x01);
	gc20c3_write_register(ViPipe, 0x0d19, 0x48);
	gc20c3_write_register(ViPipe, 0x0145, 0x0d);
	gc20c3_write_register(ViPipe, 0x0144, 0x02);
	gc20c3_write_register(ViPipe, 0x0142, 0x04);
	gc20c3_write_register(ViPipe, 0x0143, 0x14);
	gc20c3_write_register(ViPipe, 0x0146, 0x05);
	gc20c3_write_register(ViPipe, 0x0141, 0x05);
	gc20c3_write_register(ViPipe, 0x0149, 0x05);
	gc20c3_write_register(ViPipe, 0x014a, 0x07);
	gc20c3_write_register(ViPipe, 0x014b, 0x06);
	gc20c3_write_register(ViPipe, 0x0b4e, 0x88);
	gc20c3_write_register(ViPipe, 0x0e0c, 0x00);
	gc20c3_write_register(ViPipe, 0x0e0f, 0x00);
	gc20c3_write_register(ViPipe, 0x0e3a, 0x98);
	gc20c3_write_register(ViPipe, 0x0b45, 0x06);
	gc20c3_write_register(ViPipe, 0x0b47, 0xf0);
	gc20c3_write_register(ViPipe, 0x0d30, 0x06);
	gc20c3_write_register(ViPipe, 0x0d2f, 0x05);
	gc20c3_write_register(ViPipe, 0x0b40, 0x57);
	gc20c3_write_register(ViPipe, 0x0b43, 0x00);
	gc20c3_write_register(ViPipe, 0x0b41, 0x0c);
	gc20c3_write_register(ViPipe, 0x0d31, 0x03);
	gc20c3_write_register(ViPipe, 0x0b4f, 0x00);
	gc20c3_write_register(ViPipe, 0x0c23, 0x40);
	gc20c3_write_register(ViPipe, 0x0e23, 0x1a);
	gc20c3_write_register(ViPipe, 0x0e2a, 0x09);
	gc20c3_write_register(ViPipe, 0x0e2b, 0xc9);
	gc20c3_write_register(ViPipe, 0x0e41, 0x87);
	gc20c3_write_register(ViPipe, 0x0e3b, 0xb5);
	gc20c3_write_register(ViPipe, 0x0e3a, 0x15);
	gc20c3_write_register(ViPipe, 0x0e37, 0x1f);
	gc20c3_write_register(ViPipe, 0x0e0f, 0x00);
	gc20c3_write_register(ViPipe, 0x0e13, 0x00);
	gc20c3_write_register(ViPipe, 0x03bd, 0x40);
	gc20c3_write_register(ViPipe, 0x0d41, 0x00);
	gc20c3_write_register(ViPipe, 0x0217, 0x02);
	gc20c3_write_register(ViPipe, 0x0213, 0x04);
	gc20c3_write_register(ViPipe, 0x0219, 0xc2);
	gc20c3_write_register(ViPipe, 0x0259, 0x04);
	gc20c3_write_register(ViPipe, 0x025a, 0x5e);
	gc20c3_write_register(ViPipe, 0x0211, 0x01);
	gc20c3_write_register(ViPipe, 0x0340, 0x04);
	gc20c3_write_register(ViPipe, 0x0341, 0x65);
	gc20c3_write_register(ViPipe, 0x0342, 0x03);
	gc20c3_write_register(ViPipe, 0x0343, 0xe8);
	gc20c3_write_register(ViPipe, 0x0212, 0x14);
	gc20c3_write_register(ViPipe, 0x0350, 0x06);
	gc20c3_write_register(ViPipe, 0x0348, 0x07);
	gc20c3_write_register(ViPipe, 0x0349, 0x88);
	gc20c3_write_register(ViPipe, 0x034a, 0x04);
	gc20c3_write_register(ViPipe, 0x034b, 0x40);
	gc20c3_write_register(ViPipe, 0x0347, 0x00);
	gc20c3_write_register(ViPipe, 0x0b0c, 0x00);
	gc20c3_write_register(ViPipe, 0x0b0d, 0x02);
	gc20c3_write_register(ViPipe, 0x0b0e, 0x07);
	gc20c3_write_register(ViPipe, 0x0b0f, 0x8a);
	gc20c3_write_register(ViPipe, 0x034e, 0x07);
	gc20c3_write_register(ViPipe, 0x034f, 0xa8);
	gc20c3_write_register(ViPipe, 0x0004, 0x0f);
	gc20c3_write_register(ViPipe, 0x0444, 0x00);
	gc20c3_write_register(ViPipe, 0x0038, 0x20);
	gc20c3_write_register(ViPipe, 0x0039, 0x20);
	gc20c3_write_register(ViPipe, 0x003a, 0x20);
	gc20c3_write_register(ViPipe, 0x003b, 0x20);
	gc20c3_write_register(ViPipe, 0x0492, 0x00);
	gc20c3_write_register(ViPipe, 0x0493, 0x00);
	gc20c3_write_register(ViPipe, 0x0070, 0x00);
	gc20c3_write_register(ViPipe, 0x0094, 0x07);
	gc20c3_write_register(ViPipe, 0x0095, 0x80);
	gc20c3_write_register(ViPipe, 0x0096, 0x04);
	gc20c3_write_register(ViPipe, 0x0097, 0x38);
	gc20c3_write_register(ViPipe, 0x0099, 0x04);
	gc20c3_write_register(ViPipe, 0x009b, 0x04);
	gc20c3_write_register(ViPipe, 0x0438, 0x0f);
	gc20c3_write_register(ViPipe, 0x0439, 0xf0);
	gc20c3_write_register(ViPipe, 0x021a, 0x10);
	gc20c3_write_register(ViPipe, 0x0476, 0x01);
	gc20c3_write_register(ViPipe, 0x0430, 0x23);
	gc20c3_write_register(ViPipe, 0x0443, 0x02);
	gc20c3_write_register(ViPipe, 0x0038, 0x00);
	gc20c3_write_register(ViPipe, 0x0039, 0x00);
	gc20c3_write_register(ViPipe, 0x003a, 0x00);
	gc20c3_write_register(ViPipe, 0x003b, 0x00);
	gc20c3_write_register(ViPipe, 0x0070, 0x80);
	gc20c3_write_register(ViPipe, 0x0448, 0x0d);
	gc20c3_write_register(ViPipe, 0x0449, 0x0d);
	gc20c3_write_register(ViPipe, 0x044a, 0x0d);
	gc20c3_write_register(ViPipe, 0x044b, 0x0d);
	gc20c3_write_register(ViPipe, 0x044c, 0x74);
	gc20c3_write_register(ViPipe, 0x044d, 0x74);
	gc20c3_write_register(ViPipe, 0x044e, 0x74);
	gc20c3_write_register(ViPipe, 0x044f, 0x74);
	gc20c3_write_register(ViPipe, 0x0485, 0x68);
	gc20c3_write_register(ViPipe, 0x0d38, 0x07);
	gc20c3_write_register(ViPipe, 0x0d39, 0x57);
	gc20c3_write_register(ViPipe, 0x0e4e, 0xa9);
	gc20c3_write_register(ViPipe, 0x0072, 0x09);
	gc20c3_write_register(ViPipe, 0x0073, 0x05);
	gc20c3_write_register(ViPipe, 0x0c20, 0x09);
	gc20c3_write_register(ViPipe, 0x0c1d, 0x02);
	gc20c3_write_register(ViPipe, 0x0c1e, 0x2c);
	gc20c3_write_register(ViPipe, 0x0c1f, 0xe6);
	gc20c3_write_register(ViPipe, 0x0c19, 0x00);
	gc20c3_write_register(ViPipe, 0x0c1a, 0x11);
	gc20c3_write_register(ViPipe, 0x0c1b, 0x00);
	gc20c3_write_register(ViPipe, 0x0c1c, 0x80);
	gc20c3_write_register(ViPipe, 0x0261, 0x13);
	gc20c3_write_register(ViPipe, 0x0004, 0x0f);
	gc20c3_write_register(ViPipe, 0x0040, 0x2c);
	gc20c3_write_register(ViPipe, 0x004c, 0x10);
	gc20c3_write_register(ViPipe, 0x004d, 0x40);
	gc20c3_write_register(ViPipe, 0x004e, 0xc0);
	gc20c3_write_register(ViPipe, 0x0043, 0x03);
	gc20c3_write_register(ViPipe, 0x0044, 0x11);
	gc20c3_write_register(ViPipe, 0x0045, 0x58);
	gc20c3_write_register(ViPipe, 0x0046, 0x57);
	gc20c3_write_register(ViPipe, 0x0052, 0x84);
	gc20c3_write_register(ViPipe, 0x0053, 0xa0);
	gc20c3_write_register(ViPipe, 0x0055, 0x20);
	gc20c3_write_register(ViPipe, 0x0152, 0x14);
	gc20c3_write_register(ViPipe, 0x0100, 0x03);
	gc20c3_write_register(ViPipe, 0x031c, 0x1f);
	gc20c3_write_register(ViPipe, 0x0336, 0x01);
	gc20c3_write_register(ViPipe, 0x0336, 0x00);
	gc20c3_write_register(ViPipe, 0x03fe, 0x00);

	gc20c3_default_reg_init(ViPipe);
	delay_ms(10);

	printf("ViPipe:%d,===GC20C3 1080P 30fps 10bit LINEAR Init OK!===\n", ViPipe);
}