#ifndef __GC4683_CMOS_EX_H_
#define __GC4683_CMOS_EX_H_

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

enum gc4683_linear_regs_e {
	LINEAR_EXP_H,
	LINEAR_EXP_L,
	LINEAR_AGAIN_H,
	LINEAR_AGAIN_L,
	LINEAR_AGAIN_PGA,
	LINEAR_AGAIN_MAG1,
	LINEAR_AGAIN_MAG2,
	LINEAR_AGAIN_MAG3,
	LINEAR_AGAIN_MAG4,
	LINEAR_COL_AGAIN_H,
	LINEAR_COL_AGAIN_L,
	LINEAR_DGAIN_H,
	LINEAR_DGAIN_L,
	LINEAR_VTS_H,
	LINEAR_VTS_L,
	LINEAR_FLIP_MIRROR,
	LINEAR_REGS_NUM
};

enum gc4683_wdr_regs_e {
	WDR2_SEXP_H,		// 0x0200
	WDR2_SEXP_L,		// 0x0201
	WDR2_LEXP_H,		// 0x0202
	WDR2_LEXP_L,		// 0x0203
	WDR2_LAGAIN_H,		// 0x02aa
	WDR2_LAGAIN_L,		// 0x02ab
	WDR2_LAGAIN_PGA,	// 0x02a9
	WDR2_LAGAIN_MAG1,	// 0x14ac
	WDR2_LAGAIN_MAG2,	// 0x1467
	WDR2_LAGAIN_MAG3,	// 0x1468
	WDR2_LCOL_AGAIN_H,	// 0x00a8
	WDR2_LCOL_AGAIN_L,	// 0x00a9
	WDR2_LDGAIN_H,		// 0x0064
	WDR2_LDGAIN_L,		// 0x0065
	WDR2_SAGAIN_H,		// 0x0293
	WDR2_SAGAIN_L,		// 0x0294
	WDR2_SAGAIN_MAG1,	// 0x14ab
	WDR2_SAGAIN_MAG2,	// 0x1457
	WDR2_SAGAIN_MAG3,	// 0x1458
	WDR2_SCOL_AGAIN_H,	// 0x00aa
	WDR2_SCOL_AGAIN_L,	// 0x00ab
	WDR2_SDGAIN_H,		// 0x007b
	WDR2_SDGAIN_L,		// 0x007c
	WDR2_VTS_H,		// 0x0340
	WDR2_VTS_L,		// 0x0341
	WDR2_FLIP_MIRROR,	// 0x022c
	WDR2_REGS_NUM
};

typedef enum _GC4683_MODE_E {
	GC4683_MODE_2560X1440P60 = 0,
	GC4683_MODE_1080P60,
	GC4683_MODE_1280X720P120,
	GC4683_MODE_LINEAR_NUM,
	GC4683_MODE_2560X1440P30_WDR = GC4683_MODE_LINEAR_NUM,
	GC4683_MODE_NUM
} GC4683_MODE_E;

typedef struct _GC4683_STATE_S {
	CVI_U32		u32Sexp_MAX;
} GC4683_STATE_S;

typedef struct _GC4683_MODE_S {
	ISP_WDR_SIZE_S astImg[2];
	CVI_FLOAT f32MaxFps;
	CVI_FLOAT f32MinFps;
	CVI_U32 u32HtsDef;
	CVI_U32 u32VtsDef;
	SNS_ATTR_S stExp[2];
	SNS_ATTR_LARGE_S stAgain[2];
	SNS_ATTR_LARGE_S stDgain[2];
	CVI_U16 u16SexpMaxReg;
	char name[64];
} GC4683_MODE_S;

/****************************************************************************
 * external variables and functions                                         *
 ****************************************************************************/

extern ISP_SNS_STATE_S *g_pastGc4683[VI_MAX_PIPE_NUM];
extern ISP_SNS_COMMBUS_U g_aunGc4683_BusInfo[];
extern ISP_SNS_COMMADDR_U g_aunGc4683_AddrInfo[];
extern ISP_SNS_MIRRORFLIP_TYPE_E g_aeGc4683_MirrorFip[VI_MAX_PIPE_NUM];
extern CVI_U8 gc4683_i2c_addr;
extern const CVI_U32 gc4683_addr_byte;
extern const CVI_U32 gc4683_data_byte;
extern void gc4683_init(VI_PIPE ViPipe);
extern void gc4683_exit(VI_PIPE ViPipe);
extern int gc4683_i2c_exit(VI_PIPE ViPipe);
extern void gc4683_standby(VI_PIPE ViPipe);
extern void gc4683_restart(VI_PIPE ViPipe);
extern int  gc4683_write_register(VI_PIPE ViPipe, int addr, int data);
extern int  gc4683_read_register(VI_PIPE ViPipe, int addr);
extern int  gc4683_probe(VI_PIPE ViPipe);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */


#endif /* __GC4683_CMOS_EX_H_ */

