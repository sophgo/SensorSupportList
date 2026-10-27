#ifndef __S5K3P3ST_CMOS_EX_H_
#define __S5K3P3ST_CMOS_EX_H_

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif

#include <cvi_comm_cif.h>
#include <cvi_type.h>
#include "cvi_sns_ctrl.h"

enum s5k3p3st_linear_regs_e {
	LINEAR_SHS_COARSE_ADDR = 0,
	LINEAR_AGAIN_ADDR,
	LINEAR_DGAIN_GREENR_ADDR,
	LINEAR_DGAIN_RED_ADDR,
	LINEAR_DGAIN_BLUE_ADDR,
	LINEAR_DGAIN_GREENB_ADDR,
	LINEAR_VMAX_ADDR,
	LINEAR_REGS_NUM
};


typedef enum _S5K3P3ST_MODE_E {
	S5K3P3ST_MODE_1744P30 = 0,
	S5K3P3ST_LINEAR_NUM,
	S5K3P3ST_MODE_NUM
} S5K3P3ST_MODE_E;

typedef struct _SC533hAI_STATE_S {
	CVI_U32		u32Sexp_MAX;
} S5K3P3ST_STATE_S;

typedef struct _S5K3P3ST_MODE_S {
	ISP_WDR_SIZE_S astImg[2];
	CVI_FLOAT f32MaxFps;
	CVI_FLOAT f32MinFps;
	CVI_U32 u32HtsDef;
	CVI_U32 u32VtsDef;
	SNS_ATTR_LARGE_S stExp[2];
	SNS_ATTR_LARGE_S stAgain[2];
	SNS_ATTR_LARGE_S stDgain[2];
	CVI_U16 u16SexpMaxReg;
	char name[64];
} S5K3P3ST_MODE_S;

/****************************************************************************
 * external variables and functions                                         *
 ****************************************************************************/

extern ISP_SNS_STATE_S *g_pastS5K3P3ST[VI_MAX_PIPE_NUM];
extern ISP_SNS_COMMBUS_U g_aunS5K3P3ST_BusInfo[];
extern ISP_SNS_COMMADDR_U g_aunS5K3P3ST_AddrInfo[];
extern CVI_U16 g_au16S5K3P3ST_GainMode[];
extern CVI_U16 g_au16S5K3P3ST_L2SMode[];
extern CVI_U8 s5k3p3st_i2c_addr;
extern const CVI_U32 s5k3p3st_addr_byte;
extern const CVI_U32 s5k3p3st_data_byte;
extern void s5k3p3st_init(VI_PIPE ViPipe);
extern void s5k3p3st_exit(VI_PIPE ViPipe);
extern int  s5k3p3st_i2c_exit(VI_PIPE ViPipe);
extern void s5k3p3st_standby(VI_PIPE ViPipe);
extern void s5k3p3st_restart(VI_PIPE ViPipe);
extern int  s5k3p3st_write_register(VI_PIPE ViPipe, int addr, int data);
extern int  s5k3p3st_read_register(VI_PIPE ViPipe, int addr);
extern void s5k3p3st_mirror_flip(VI_PIPE ViPipe, ISP_SNS_MIRRORFLIP_TYPE_E eSnsMirrorFlip);
extern int  s5k3p3st_probe(VI_PIPE ViPipe);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */


#endif /* __S5K3P3ST_CMOS_EX_H_ */
