/*
  Copyright (c), 2001-2022, Shenshu Tech. Co., Ltd.
 */

#ifndef __OT_SAMPLE_GYRO_DIS_H__
#define __OT_SAMPLE_GYRO_DIS_H__

#include "hi_type.h"
#include "hi_common_vo.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* __cplusplus */

hi_s32 sample_dis_ipc_gyro(hi_vo_intf_type vo_intf_type);

hi_s32 sample_dis_dv_gyro(hi_vo_intf_type vo_intf_type);

hi_s32 sample_dis_gyro_switch(hi_vo_intf_type vo_intf_type, hi_ldc_version ldc_version);

hi_s32 sample_dis_ipc_gyro_two_sensor(hi_size *size);

hi_void sample_dis_stop_gyro(hi_void);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of __cplusplus */

#endif /* __OT_SAMPLE_GYRO_DIS_H__ */

