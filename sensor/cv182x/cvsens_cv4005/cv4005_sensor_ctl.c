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
#include "cv4005_cmos_ex.h"

#define CV4005_CHIP_ID_ADDR_H	0x3003
#define CV4005_CHIP_ID_ADDR_L	0x3002
#define CV4005_CHIP_ID		0x4005

static void cv4005_linear_1440p30_init(VI_PIPE ViPipe);

CVI_U8 cv4005_i2c_addr = 0x35;
const CVI_U32 cv4005_addr_byte = 2;
const CVI_U32 cv4005_data_byte = 1;
static int g_fd[VI_MAX_PIPE_NUM] = {[0 ... (VI_MAX_PIPE_NUM - 1)] = -1};

int cv4005_i2c_init(VI_PIPE ViPipe)
{
	char acDevFile[16] = {0};
	CVI_U8 u8DevNum;

	if (g_fd[ViPipe] >= 0)
		return CVI_SUCCESS;
	int ret;

	u8DevNum = g_aunCV4005_BusInfo[ViPipe].s8I2cDev;
	snprintf(acDevFile, sizeof(acDevFile),  "/dev/i2c-%u", u8DevNum);

	g_fd[ViPipe] = open(acDevFile, O_RDWR, 0600);

	if (g_fd[ViPipe] < 0) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "Open /dev/i2c-%u error!\n", u8DevNum);
		return CVI_FAILURE;
	}

	ret = ioctl(g_fd[ViPipe], I2C_SLAVE_FORCE, cv4005_i2c_addr);
	if (ret < 0) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "I2C_SLAVE_FORCE error!\n");
		close(g_fd[ViPipe]);
		g_fd[ViPipe] = -1;
		return ret;
	}

	return CVI_SUCCESS;
}

int cv4005_i2c_exit(VI_PIPE ViPipe)
{
	if (g_fd[ViPipe] >= 0) {
		close(g_fd[ViPipe]);
		g_fd[ViPipe] = -1;
		return CVI_SUCCESS;
	}
	return CVI_FAILURE;
}

int cv4005_read_register(VI_PIPE ViPipe, int addr)
{
	int ret, data;
	CVI_U8 buf[8];
	CVI_U8 idx = 0;

	if (g_fd[ViPipe] < 0)
		return CVI_FAILURE;

	if (cv4005_addr_byte == 2)
		buf[idx++] = (addr >> 8) & 0xff;

	buf[idx++] = addr & 0xff;

	ret = write(g_fd[ViPipe], buf, cv4005_addr_byte);
	if (ret < 0) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "I2C_WRITE error!\n");
		return ret;
	}

	buf[0] = 0;
	buf[1] = 0;
	ret = read(g_fd[ViPipe], buf, cv4005_data_byte);
	if (ret < 0) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "I2C_READ error!\n");
		return ret;
	}

	data = buf[0];
	syslog(LOG_DEBUG, "i2c r 0x%x = 0x%x\n", addr, data);
	return data;
}

int cv4005_write_register(VI_PIPE ViPipe, int addr, int data)
{
	CVI_U8 idx = 0;
	int ret;
	CVI_U8 buf[8];

	if (g_fd[ViPipe] < 0)
		return CVI_SUCCESS;

	if (cv4005_addr_byte == 2) {
		buf[idx] = (addr >> 8) & 0xff;
		idx++;
		buf[idx] = addr & 0xff;
		idx++;
	}
	if (cv4005_data_byte == 1) {
		buf[idx] = data & 0xff;
		idx++;
	}

	ret = write(g_fd[ViPipe], buf, cv4005_addr_byte + cv4005_data_byte);
	if (ret < 0) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "I2C_WRITE error!\n");
		return CVI_FAILURE;
	}
	syslog(LOG_DEBUG, "i2c w 0x%x 0x%x\n", addr, data);
	return CVI_SUCCESS;
}

static void delay_ms(int ms)
{
	usleep(ms * 1000);
}

void cv4005_standby(VI_PIPE ViPipe)
{
	cv4005_write_register(ViPipe, 0x3000, 0x1);
}

void cv4005_restart(VI_PIPE ViPipe)
{
	cv4005_write_register(ViPipe, 0x3000, 0x01);
	delay_ms(20);
	cv4005_write_register(ViPipe, 0x3000, 0x00);
}

void cv4005_default_reg_init(VI_PIPE ViPipe)
{
	CVI_U32 i;

	for (i = 0; i < g_pastCV4005[ViPipe]->astSyncInfo[0].snsCfg.u32RegNum; i++) {
		cv4005_write_register(ViPipe,
			g_pastCV4005[ViPipe]->astSyncInfo[0].snsCfg.astI2cData[i].u32RegAddr,
			g_pastCV4005[ViPipe]->astSyncInfo[0].snsCfg.astI2cData[i].u32Data);
	}
}

int cv4005_probe(VI_PIPE ViPipe)
{
	int nVal;
	int nVal2;

	usleep(50);
	if (cv4005_i2c_init(ViPipe) != CVI_SUCCESS)
		return CVI_FAILURE;

	cv4005_write_register(ViPipe, 0x3c07, 0x01);
	delay_ms(5);

	nVal  = cv4005_read_register(ViPipe, CV4005_CHIP_ID_ADDR_H);
	nVal2 = cv4005_read_register(ViPipe, CV4005_CHIP_ID_ADDR_L);
	if (nVal < 0 || nVal2 < 0) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "read sensor id error.\n");
		return nVal;
	}

	if ((((nVal & 0xFF) << 8) | (nVal2 & 0xFF)) != CV4005_CHIP_ID) {
		CVI_TRACE_SNS(CVI_DBG_WARN, "Sensor ID 0x%04x, expect 0x%04x\n",
			(((nVal & 0xFF) << 8) | (nVal2 & 0xFF)), CV4005_CHIP_ID);
	}

	return CVI_SUCCESS;
}

void cv4005_init(VI_PIPE ViPipe)
{
	cv4005_i2c_init(ViPipe);
	cv4005_linear_1440p30_init(ViPipe);
	g_pastCV4005[ViPipe]->bInit = CVI_TRUE;
}

void cv4005_exit(VI_PIPE ViPipe)
{
	cv4005_i2c_exit(ViPipe);
}

static void cv4005_linear_1440p30_init(VI_PIPE ViPipe)
{
	delay_ms(10);

	/* FAE ini + datasheet 8.3: DCROP/WCROP output 2560x1440 on MIPI */
	cv4005_write_register(ViPipe, 0x3020, 0xc6);
	cv4005_write_register(ViPipe, 0x3021, 0x02);
	cv4005_write_register(ViPipe, 0x302d, 0x00);
	cv4005_write_register(ViPipe, 0x3030, 0x01); /* DCROP_MODE=1 */
	cv4005_write_register(ViPipe, 0x3034, 0x06); /* Y_DCROP_STA[7:0]=6 */
	cv4005_write_register(ViPipe, 0x3035, 0x00); /* Y_DCROP_STA[11:8] */
	cv4005_write_register(ViPipe, 0x3036, 0xa0); /* Y_DCROP_HEIGHT[7:0]=1440 */
	cv4005_write_register(ViPipe, 0x3037, 0x05); /* Y_DCROP_HEIGHT[11:8] */
	cv4005_write_register(ViPipe, 0x3038, 0x04); /* X_CROP_STA[7:0]=4 */
	cv4005_write_register(ViPipe, 0x3039, 0x00); /* X_CROP_STA[11:8] */
	cv4005_write_register(ViPipe, 0x303a, 0x00); /* X_CROP_WIDTH[7:0]=2560 */
	cv4005_write_register(ViPipe, 0x303b, 0x0a); /* X_CROP_WIDTH[11:8] */
	cv4005_write_register(ViPipe, 0x305c, 0x00);
	cv4005_write_register(ViPipe, 0x3138, 0x00);
	cv4005_write_register(ViPipe, 0x3139, 0x03);
	cv4005_write_register(ViPipe, 0x3150, 0x00);
	cv4005_write_register(ViPipe, 0x3151, 0x03);
	cv4005_write_register(ViPipe, 0x3401, 0x01);
	cv4005_write_register(ViPipe, 0x3950, 0x0f);
	cv4005_write_register(ViPipe, 0x3000, 0x00);
	cv4005_write_register(ViPipe, 0x3c0d, 0x01);
	cv4005_write_register(ViPipe, 0x3c07, 0x01);
	cv4005_write_register(ViPipe, 0x3c07, 0x00);
	cv4005_write_register(ViPipe, 0x3418, 0x6f);
	cv4005_write_register(ViPipe, 0x341a, 0x2f);
	cv4005_write_register(ViPipe, 0x341c, 0x2f);
	cv4005_write_register(ViPipe, 0x341e, 0xbf);
	cv4005_write_register(ViPipe, 0x341f, 0x00);
	cv4005_write_register(ViPipe, 0x3420, 0x2f);
	cv4005_write_register(ViPipe, 0x3422, 0x57);
	cv4005_write_register(ViPipe, 0x3424, 0x2f);
	cv4005_write_register(ViPipe, 0x3426, 0x4f);
	cv4005_write_register(ViPipe, 0x3428, 0x27);
	cv4005_write_register(ViPipe, 0x3000, 0x01);
	cv4005_write_register(ViPipe, 0x3950, 0x0b);
	cv4005_write_register(ViPipe, 0x3262, 0x0a);
	cv4005_write_register(ViPipe, 0x3263, 0x01);
	cv4005_write_register(ViPipe, 0x3440, 0x03);
	cv4005_write_register(ViPipe, 0x3442, 0x00);
	cv4005_write_register(ViPipe, 0x3703, 0x00);
	cv4005_write_register(ViPipe, 0x3808, 0x37);
	cv4005_write_register(ViPipe, 0x380a, 0x02);
	cv4005_write_register(ViPipe, 0x3031, 0x00);
	cv4005_write_register(ViPipe, 0x3204, 0x40);
	cv4005_write_register(ViPipe, 0x3134, 0x01);
	cv4005_write_register(ViPipe, 0x326e, 0x80);
	cv4005_write_register(ViPipe, 0x3510, 0x03);
	cv4005_write_register(ViPipe, 0x3512, 0x50);
	cv4005_write_register(ViPipe, 0x38dc, 0x02);
	cv4005_write_register(ViPipe, 0x3856, 0x0a);
	cv4005_write_register(ViPipe, 0x3857, 0x01);
	cv4005_write_register(ViPipe, 0x301c, 0xda);
	cv4005_write_register(ViPipe, 0x301d, 0x0d);
	cv4005_write_register(ViPipe, 0x3808, 0x3f);
	cv4005_write_register(ViPipe, 0x356a, 0x00);
	cv4005_write_register(ViPipe, 0x359f, 0x15);
	cv4005_write_register(ViPipe, 0x366c, 0x14);
	cv4005_write_register(ViPipe, 0x35e1, 0x03);
	cv4005_write_register(ViPipe, 0x3854, 0x00);
	cv4005_write_register(ViPipe, 0x3154, 0x11);
	cv4005_write_register(ViPipe, 0x3114, 0x00); /* GAIN_0 = 0dB / 1x */
	cv4005_write_register(ViPipe, 0x356f, 0x01);
	cv4005_write_register(ViPipe, 0x310c, 0x64);
	cv4005_write_register(ViPipe, 0x38cd, 0x26);
	cv4005_write_register(ViPipe, 0x38ce, 0x00);
	cv4005_write_register(ViPipe, 0x38cf, 0xff);
	cv4005_write_register(ViPipe, 0x36d8, 0x00);
	cv4005_write_register(ViPipe, 0x36d9, 0x00);
	cv4005_write_register(ViPipe, 0x356e, 0x00);
	cv4005_write_register(ViPipe, 0x31b6, 0x1e);
	cv4005_write_register(ViPipe, 0x31bc, 0x20);
	cv4005_write_register(ViPipe, 0x3000, 0x00);

	cv4005_default_reg_init(ViPipe);
	delay_ms(100);

	printf("ViPipe:%d,===CV4005 1440P 30fps 10bit LINEAR Init OK!===\n", ViPipe);
}
