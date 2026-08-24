#ifndef __OS08C10_CMOS_EX_H_
#define __OS08C10_CMOS_EX_H_

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif


#include <linux/cvi_type.h>
#include "cvi_sns_ctrl.h"

#define UNUSED(x) ((void)(x))

enum os08c10_WDR_regs_e {
	WDR_HOLD_START = 0,
	WDR_EXPS_0,
	WDR_EXPS_1,
	WDR_EXPL_0,
	WDR_EXPL_1,
	WDR_AGAINS_0,
	WDR_AGAINS_1,
	WDR_AGAINL_0,
	WDR_AGAINL_1,
	WDR_DGAINS_0,
	WDR_DGAINS_1,
	WDR_DGAINL_0,
	WDR_DGAINL_1,
	WDR_VTS_0,
	WDR_VTS_1,
	WDR_HOLD_END,
	WDR_LAUNCH_1,
	WDR_REGS_NUM
};

typedef enum _OS08C10_MODE_E {
	OS08C10_MODE_1920X1080P60_WDR = 0,
	OS08C10_MODE_1280X720P120_WDR,
	OS08C10_MODE_NUM
} OS08C10_MODE_E;

typedef struct _OS08C10_STATE_S {
	CVI_U32		u32Sexp_MAX;
} OS08C10_STATE_S;

typedef struct _OS08C10_MODE_S {
	ISP_WDR_SIZE_S astImg[2];
	CVI_FLOAT f32MaxFps;
	CVI_FLOAT f32MinFps;
	CVI_U32 u32HtsDef;
	CVI_U32 u32VtsDef;
	SNS_ATTR_S stExp[2];
	SNS_ATTR_S stAgain[2];
	SNS_ATTR_S stDgain[2];
	CVI_U32 u32Sexp_MAX;
	char name[64];
} OS08C10_MODE_S;

/****************************************************************************
 * external variables and functions                                         *
 ****************************************************************************/

extern ISP_SNS_STATE_S *g_pastOs08c10[VI_MAX_PIPE_NUM];
extern ISP_SNS_COMMBUS_U g_aunOs08c10_BusInfo[];
extern CVI_U16 g_au16Os08c10_GainMode[];
extern CVI_U16 g_au16Os08c10_L2SMode[VI_MAX_PIPE_NUM];
#define OS08C10_I2C_ADDR		0x36
#define OS08C10_I2C_ADDR_IS_VALID(addr)	((addr) == OS08C10_I2C_ADDR)

extern CVI_U8 os08c10_i2c_addr;
extern const CVI_U32 os08c10_addr_byte;
extern const CVI_U32 os08c10_data_byte;
extern void os08c10_init(VI_PIPE ViPipe);
extern void os08c10_exit(VI_PIPE ViPipe);
extern void os08c10_standby(VI_PIPE ViPipe);
extern void os08c10_restart(VI_PIPE ViPipe);
extern int  os08c10_write_register(VI_PIPE ViPipe, int addr, int data);
extern int  os08c10_read_register(VI_PIPE ViPipe, int addr);
extern int  os08c10_probe(VI_PIPE ViPipe);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */


#endif /* __OS08C10_CMOS_EX_H_ */