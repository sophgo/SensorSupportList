#ifndef __OG01A1B_CMOS_EX_H_
#define __OG01A1B_CMOS_EX_H_

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif

#include <cvi_comm_cif.h>
#include <cvi_type.h>
#include "cvi_sns_ctrl.h"


enum og01a1b_linear_regs_e {
	LINEAR_HOLD_START = 0,
	LINEAR_EXP_0,
	LINEAR_EXP_1,
	LINEAR_EXP_2,
	LINEAR_AGAIN_0,
	LINEAR_AGAIN_1,
	LINEAR_DGAIN_0,
	LINEAR_DGAIN_1,
	LINEAR_DGAIN_2,
	LINEAR_VTS_0,
	LINEAR_VTS_1,
	LINEAR_HOLD_END,
	LINEAR_LAUNCH_0,
	LINEAR_LAUNCH_1,
	LINEAR_REGS_NUM
};

typedef enum _OG01A1B_MODE_E {
	OG01A1B_MODE_640P50_8bit = 0,
	OG01A1B_MODE_LINEAR_NUM,
	OG01A1B_MODE_NUM = OG01A1B_MODE_LINEAR_NUM
} OG01A1B_MODE_E;

typedef struct _OG01A1B_MODE_S {
	ISP_WDR_SIZE_S astImg[1];
	CVI_FLOAT f32MaxFps;
	CVI_FLOAT f32MinFps;
	CVI_U32 u32HtsDef;
	CVI_U32 u32VtsDef;
	SNS_ATTR_S stExp[1];
	SNS_ATTR_LARGE_S stAgain[1];
	SNS_ATTR_LARGE_S stDgain[1];
	char name[64];
} OG01A1B_MODE_S;

/****************************************************************************
 * external variables and functions                                         *
 ****************************************************************************/

extern ISP_SNS_STATE_S *g_pastOg01a1b[VI_MAX_PIPE_NUM];
extern ISP_SNS_COMMBUS_U g_aunOg01a1b_BusInfo[];
extern ISP_SNS_COMMADDR_U g_aunOg01a1b_AddrInfo[];
extern CVI_U16 g_au16Og01a1b_GainMode[];
extern CVI_U16 g_au16Og01a1b_UseHwSync[VI_MAX_PIPE_NUM];
extern const CVI_U32 og01a1b_addr_byte;
extern const CVI_U32 og01a1b_data_byte;
extern void og01a1b_init(VI_PIPE ViPipe);
extern void og01a1b_exit(VI_PIPE ViPipe);
extern int og01a1b_i2c_exit(VI_PIPE ViPipe);
extern void og01a1b_standby(VI_PIPE ViPipe);
extern void og01a1b_restart(VI_PIPE ViPipe);
extern int  og01a1b_write_register(VI_PIPE ViPipe, int addr, int data);
extern int  og01a1b_read_register(VI_PIPE ViPipe, int addr);
extern int  og01a1b_probe(VI_PIPE ViPipe);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */


#endif /* __OG01A1B_CMOS_EX_H_ */
