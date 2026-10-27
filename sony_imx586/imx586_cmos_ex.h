#ifndef __IMX586_CMOS_EX_H_
#define __IMX586_CMOS_EX_H_

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif

#include <cvi_comm_cif.h>
#include <cvi_type.h>
#include "cvi_sns_ctrl.h"

enum imx586_linear_regs_e {
	LINEAR_HOLD = 0,
	LINEAR_CIT_H,         //0x0202 COARSE_INTEG_TIME[15:8]
	LINEAR_CIT_L,         //0x0203 COARSE_INTEG_TIME[7:0]
	LINEAR_GAIN_H,        //0x0204 ANA_GAIN_GLOBAL[9:8]
	LINEAR_GAIN_L,        //0x0205 ANA_GAIN_GLOBAL[7:0]
	LINEAR_DGAIN_H,       //0x020E DIG_GAIN_GLOBAL[15:8]
	LINEAR_DGAIN_L,       //0x020F DIG_GAIN_GLOBAL[7:0]
	LINEAR_VMAX_H,
	LINEAR_VMAX_L,
	LINEAR_REL,
	LINEAR_FLIP_MIRROR,
	LINEAR_REGS_NUM
};

typedef enum _IMX586_MODE_E {
	IMX586_MODE_12M30 = 0,
	IMX586_MODE_8M60,
	IMX586_MODE_2M120,
	IMX586_MODE_LINEAR_NUM,
	IMX586_MODE_NUM = IMX586_MODE_LINEAR_NUM,
} IMX586_MODE_E;

typedef struct _IMX586_MODE_S {
	ISP_WDR_SIZE_S astImg[2];
	CVI_FLOAT f32MaxFps;
	CVI_FLOAT f32MinFps;
	CVI_U32 u32HtsDef;
	CVI_U32 u32VtsDef;
	SNS_ATTR_S stExp[2];
	SNS_ATTR_LARGE_S stAgain[2];
	SNS_ATTR_LARGE_S stDgain[2];
	char name[64];
} IMX586_MODE_S;

/****************************************************************************
 * external variables and functions                                         *
 ****************************************************************************/

extern ISP_SNS_STATE_S *g_pastImx586[VI_MAX_PIPE_NUM];
extern ISP_SNS_COMMBUS_U g_aunImx586_BusInfo[];
extern ISP_SNS_COMMADDR_U g_aunImx586_AddrInfo[];
extern CVI_U16 g_au16Imx586_GainMode[];
extern CVI_U8 imx586_i2c_addr;
extern const CVI_U32 imx586_addr_byte;
extern const CVI_U32 imx586_data_byte;
extern void imx586_init(VI_PIPE ViPipe);
extern void imx586_exit(VI_PIPE ViPipe);
extern int imx586_i2c_exit(VI_PIPE ViPipe);
extern int imx586_probe(VI_PIPE ViPipe);
extern void imx586_standby(VI_PIPE ViPipe);
extern void imx586_restart(VI_PIPE ViPipe);
extern int  imx586_write_register(VI_PIPE ViPipe, int addr, int data);
extern int  imx586_read_register(VI_PIPE ViPipe, int addr);
extern void imx586_mirror_flip(VI_PIPE ViPipe, ISP_SNS_MIRRORFLIP_TYPE_E eSnsMirrorFlip);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */

#endif /* __IMX586_CMOS_EX_H_ */
