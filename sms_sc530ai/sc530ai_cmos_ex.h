#ifndef __SC530AI_CMOS_EX_H_
#define __SC530AI_CMOS_EX_H_

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif

#include <cvi_comm_cif.h>
#include "cvi_sns_ctrl.h"
#include "cvi_type.h"

#ifndef UNUSED
#define UNUSED(x) ((void)(x))
#endif

#define SC530AI_I2C_ADDR 0x30

enum sc530ai_linear_regs_e {
	LINEAR_EXP_ADDR_0,
	LINEAR_EXP_ADDR_1,
	LINEAR_EXP_ADDR_2,
	LINEAR_AGAIN_ADDR,
	LINEAR_DGAIN_ADDR,
	LINEAR_DGAIN_FINE_ADDR,
	LINEAR_FLIP_MIRROR,
	LINEAR_VTS_ADDR_H,
	LINEAR_VTS_ADDR_L,
	LINEAR_REGS_NUM
};

enum sc530ai_wdr_regs_e {
	WDR_LEXP_ADDR_0,
	WDR_LEXP_ADDR_1,
	WDR_LEXP_ADDR_2,
	WDR_SEXP_ADDR_0,
	WDR_SEXP_ADDR_1,
	WDR_SEXP_ADDR_2,
	WDR_LEXP_AGAIN_ADDR,
	WDR_LEXP_DGAIN_ADDR,
	WDR_LEXP_DGAIN_FINE_ADDR,
	WDR_SEXP_AGAIN_ADDR,
	WDR_SEXP_DGAIN_ADDR,
	WDR_SEXP_DGAIN_FINE_ADDR,
	WDR_FLIP_MIRROR,
	WDR_VTS_ADDR_H,
	WDR_VTS_ADDR_L,
	WDR_MAXSEP_ADDR_H,
	WDR_MAXSEP_ADDR_L,
	WDR_REGS_NUM
};

typedef enum _SC530AI_MODE_E {
	SC530AI_MODE_1620P30_LINEAR = 0,
	SC530AI_MODE_1620P30_WDR,
	SC530AI_MODE_NUM
} SC530AI_MODE_E;

typedef struct _SC530AI_STATE_S {
	CVI_U32 u32Sexp_MAX;
} SC530AI_STATE_S;

typedef struct _SC530AI_MODE_S {
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
} SC530AI_MODE_S;

/****************************************************************************
 * external variables and functions                                         *
 ****************************************************************************/

extern ISP_SNS_STATE_S *g_pastSC530AI[VI_MAX_PIPE_NUM];
extern ISP_SNS_COMMBUS_U g_aunSC530AI_BusInfo[];
extern ISP_SNS_COMMADDR_U g_aunSC530AI_AddrInfo[];
extern CVI_U16 g_au16SC530AI_GainMode[];
extern CVI_U16 g_au16SC530AI_L2SMode[];
extern const CVI_U32 sc530ai_addr_byte;
extern const CVI_U32 sc530ai_data_byte;
extern void sc530ai_init(VI_PIPE ViPipe);
extern void sc530ai_exit(VI_PIPE ViPipe);
extern void sc530ai_standby(VI_PIPE ViPipe);
extern void sc530ai_restart(VI_PIPE ViPipe);
extern int sc530ai_write_register(VI_PIPE ViPipe, int addr, int data);
extern int sc530ai_read_register(VI_PIPE ViPipe, int addr);
extern int sc530ai_probe(VI_PIPE ViPipe);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */

#endif /* __SC530AI_CMOS_EX_H_ */
