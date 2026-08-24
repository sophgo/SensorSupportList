#ifndef __MIS4001_CMOS_EX_H_
#define __MIS4001_CMOS_EX_H_

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif

#include <linux/cvi_type.h>
#include "cvi_sns_ctrl.h"

enum mis4001_linear_regs_e {
	LINEAR_SHS1_0_ADDR,
	LINEAR_SHS1_1_ADDR,
	LINEAR_AGAIN_ADDR,
	LINEAR_DGAIN_H_ADDR,
	LINEAR_DGAIN_L_ADDR,
	LINEAR_VMAX_0_ADDR,
	LINEAR_VMAX_1_ADDR,
	LINEAR_FLIP_MIRROR_ADDR,
	LINEAR_REGS_NUM
};

typedef enum _MIS4001_MODE_E {
	MIS4001_MODE_2560X1440P30 = 0,
	MIS4001_MODE_NUM
} MIS4001_MODE_E;

typedef struct _MIS4001_MODE_S {
	ISP_WDR_SIZE_S astImg[2];
	CVI_FLOAT f32MaxFps;
	CVI_FLOAT f32MinFps;
	CVI_U32 u32HtsDef;
	CVI_U32 u32VtsDef;
	SNS_ATTR_LARGE_S stExp[2];
	SNS_ATTR_LARGE_S stAgain[2];
	SNS_ATTR_LARGE_S stDgain[2];
	char name[64];
} MIS4001_MODE_S;

extern ISP_SNS_STATE_S *g_pastMIS4001[VI_MAX_PIPE_NUM];
extern ISP_SNS_COMMBUS_U g_aunMIS4001_BusInfo[];
extern CVI_U16 g_au16MIS4001_GainMode[];
extern CVI_U16 g_au16MIS4001_L2SMode[];
extern CVI_U8 mis4001_i2c_addr;
extern const CVI_U32 mis4001_addr_byte;
extern const CVI_U32 mis4001_data_byte;
extern void mis4001_init(VI_PIPE ViPipe);
extern void mis4001_exit(VI_PIPE ViPipe);
extern void mis4001_standby(VI_PIPE ViPipe);
extern void mis4001_restart(VI_PIPE ViPipe);
extern int  mis4001_write_register(VI_PIPE ViPipe, int addr, int data);
extern int  mis4001_read_register(VI_PIPE ViPipe, int addr);
extern void mis4001_mirror_flip(VI_PIPE ViPipe, ISP_SNS_MIRRORFLIP_TYPE_E eSnsMirrorFlip);
extern int  mis4001_probe(VI_PIPE ViPipe);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif

#endif /* __MIS4001_CMOS_EX_H_ */
