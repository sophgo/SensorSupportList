#ifndef __CV4005_CMOS_EX_H_
#define __CV4005_CMOS_EX_H_

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif

#ifdef ARCH_CV182X
#include <linux/cvi_vip_cif.h>
#include <linux/cvi_vip_snsr.h>
#include "cvi_type.h"
#else
#include <linux/cif_uapi.h>
#include <linux/vi_snsr.h>
#include <linux/cvi_type.h>
#endif
#include "cvi_sns_ctrl.h"

#ifndef UNUSED
#define UNUSED(x) ((void)(x))
#endif

enum cv4005_linear_regs_e {
	LINEAR_EXP_0,       /* 0x304a [19:16] */
	LINEAR_EXP_1,       /* 0x3049 */
	LINEAR_EXP_2,       /* 0x3048 */
	LINEAR_AGAIN,       /* 0x3114 GAIN_0 */
	LINEAR_VTS_0,       /* 0x301e [19:16] */
	LINEAR_VTS_1,       /* 0x301d */
	LINEAR_VTS_2,       /* 0x301c */
	LINEAR_FLIP_MIRROR, /* 0x3028 */
	LINEAR_REGS_NUM
};

typedef enum _CV4005_MODE_E {
	CV4005_MODE_2560X1440P30 = 0,
	CV4005_MODE_NUM
} CV4005_MODE_E;

typedef struct _CV4005_MODE_S {
	ISP_WDR_SIZE_S astImg[2];
	CVI_FLOAT f32MaxFps;
	CVI_FLOAT f32MinFps;
	CVI_U32 u32HtsDef;
	CVI_U32 u32VtsDef;
	SNS_ATTR_S stExp[2];
	SNS_ATTR_LARGE_S stAgain[2];
	SNS_ATTR_LARGE_S stDgain[2];
	char name[64];
} CV4005_MODE_S;

extern ISP_SNS_STATE_S *g_pastCV4005[VI_MAX_PIPE_NUM];
extern ISP_SNS_COMMBUS_U g_aunCV4005_BusInfo[];
extern ISP_SNS_MIRRORFLIP_TYPE_E g_aeCV4005_MirrorFip[VI_MAX_PIPE_NUM];
extern CVI_U8 cv4005_i2c_addr;
extern const CVI_U32 cv4005_addr_byte;
extern const CVI_U32 cv4005_data_byte;
extern void cv4005_init(VI_PIPE ViPipe);
extern void cv4005_exit(VI_PIPE ViPipe);
extern void cv4005_standby(VI_PIPE ViPipe);
extern void cv4005_restart(VI_PIPE ViPipe);
extern int  cv4005_write_register(VI_PIPE ViPipe, int addr, int data);
extern int  cv4005_read_register(VI_PIPE ViPipe, int addr);
extern int  cv4005_probe(VI_PIPE ViPipe);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif

#endif /* __CV4005_CMOS_EX_H_ */
