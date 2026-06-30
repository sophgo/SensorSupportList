#ifndef __GC20C3_CMOS_EX_H_
#define __GC20C3_CMOS_EX_H_

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif

#include <linux/cvi_type.h>
#include "cvi_sns_ctrl.h"

#ifndef UNUSED
#define UNUSED(x) ((void)(x))
#endif

enum gc20c3_linear_regs_e {
	LINEAR_EXP_H,
	LINEAR_EXP_L,
	LINEAR_AGAIN_L,
	LINEAR_AGAIN_H,
	LINEAR_COL_AGAIN_H,
	LINEAR_COL_AGAIN_L,
	LINEAR_AGAIN_MAG1,
	LINEAR_AGAIN_MAG2,
	LINEAR_AGAIN_MAG3,
	LINEAR_DGAIN_H,
	LINEAR_DGAIN_L,
	LINEAR_VTS_H,
	LINEAR_VTS_L,
	LINEAR_FLIP_MIRROR,
	LINEAR_REGS_NUM
};

typedef enum _GC20C3_MODE_E {
	GC20C3_MODE_1920X1080P30 = 0,
	GC20C3_MODE_LINEAR_NUM,
	GC20C3_MODE_NUM
} GC20C3_MODE_E;

typedef struct _GC20C3_STATE_S {
	CVI_U32		u32Sexp_MAX;
} GC20C3_STATE_S;

typedef struct _GC20C3_MODE_S {
	ISP_WDR_SIZE_S astImg[2];
	CVI_FLOAT f32MaxFps;
	CVI_FLOAT f32MinFps;
	CVI_U32 u32HtsDef;
	CVI_U32 u32VtsDef;
	SNS_ATTR_S stExp[2];
	SNS_ATTR_LARGE_S stAgain[2];
	SNS_ATTR_LARGE_S stDgain[2];
	char name[64];
} GC20C3_MODE_S;

/****************************************************************************
 * external variables and functions                                         *
 ****************************************************************************/

extern ISP_SNS_STATE_S *g_pastGc20c3[VI_MAX_PIPE_NUM];
extern ISP_SNS_COMMBUS_U g_aunGc20c3_BusInfo[];
extern ISP_SNS_MIRRORFLIP_TYPE_E g_aeGc20c3_MirrorFip[VI_MAX_PIPE_NUM];
extern CVI_U8 gc20c3_i2c_addr;
extern const CVI_U32 gc20c3_addr_byte;
extern const CVI_U32 gc20c3_data_byte;
extern void gc20c3_init(VI_PIPE ViPipe);
extern void gc20c3_exit(VI_PIPE ViPipe);
extern void gc20c3_standby(VI_PIPE ViPipe);
extern void gc20c3_restart(VI_PIPE ViPipe);
extern int  gc20c3_write_register(VI_PIPE ViPipe, int addr, int data);
extern int  gc20c3_read_register(VI_PIPE ViPipe, int addr);
extern int  gc20c3_probe(VI_PIPE ViPipe);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif

#endif /* __GC20C3_CMOS_EX_H_ */