#ifndef __GC05A3_CMOS_EX_H_
#define __GC05A3_CMOS_EX_H_

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif

#include <cvi_comm_cif.h>
#include <cvi_type.h>
#include "cvi_sns_ctrl.h"

#ifndef UNUSED
#define UNUSED(x) ((void)(x))
#endif

enum gc05a3_linear_regs_e {
	LINEAR_EXP_H,		/* 0x0202 */
	LINEAR_EXP_L,		/* 0x0203 */
	LINEAR_AGAIN_H,		/* 0x0204 */
	LINEAR_AGAIN_L,		/* 0x0205 */
	LINEAR_VTS_H,
	LINEAR_VTS_L,
	LINEAR_FRAME_BUF_ON,
	LINEAR_FLIP_MIRROR,
	LINEAR_FRAME_BUF_OFF,
	LINEAR_REGS_NUM
};

typedef enum _GC05A3_MODE_E {
	GC05A3_MODE_2592X1944P20 = 0,
	GC05A3_MODE_LINEAR_NUM,
	GC05A3_MODE_NUM
} GC05A3_MODE_E;

typedef struct _GC05A3_STATE_S {
	CVI_U32		u32Sexp_MAX;
} GC05A3_STATE_S;

typedef struct _GC05A3_MODE_S {
	ISP_WDR_SIZE_S astImg[2];
	CVI_FLOAT f32MaxFps;
	CVI_FLOAT f32MinFps;
	CVI_U32 u32HtsDef;
	CVI_U32 u32VtsDef;
	CVI_U32 u32VtsMax;
	SNS_ATTR_S stExp[2];
	SNS_ATTR_LARGE_S stAgain[2];
	SNS_ATTR_LARGE_S stDgain[2];
	char name[64];
} GC05A3_MODE_S;

/****************************************************************************
 * external variables and functions                                         *
 ****************************************************************************/

extern ISP_SNS_STATE_S *g_pastGc05a3[VI_MAX_PIPE_NUM];
extern ISP_SNS_COMMBUS_U g_aunGc05a3_BusInfo[];
extern ISP_SNS_COMMADDR_U g_aunGc05a3_AddrInfo[];
extern ISP_SNS_MIRRORFLIP_TYPE_E g_aeGc05a3_MirrorFip[VI_MAX_PIPE_NUM];
extern CVI_U8 gc05a3_i2c_addr;
extern const CVI_U32 gc05a3_addr_byte;
extern const CVI_U32 gc05a3_data_byte;
extern void gc05a3_init(VI_PIPE ViPipe);
extern void gc05a3_exit(VI_PIPE ViPipe);
extern int gc05a3_i2c_exit(VI_PIPE ViPipe);
extern void gc05a3_standby(VI_PIPE ViPipe);
extern void gc05a3_restart(VI_PIPE ViPipe);
extern int  gc05a3_write_register(VI_PIPE ViPipe, int addr, int data);
extern int  gc05a3_read_register(VI_PIPE ViPipe, int addr);
extern int  gc05a3_probe(VI_PIPE ViPipe);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */

#endif /* __GC05A3_CMOS_EX_H_ */
