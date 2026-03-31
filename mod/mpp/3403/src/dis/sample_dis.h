/*
  Copyright (c), 2001-2022, Shenshu Tech. Co., Ltd.
 */

#ifndef __OT_SAMPLE_DIS_H__
#define __OT_SAMPLE_DIS_H__

#include "hi_type.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

hi_void sample_dis_pause(hi_void);
hi_s32 sample_dis_start_sample(sample_vi_cfg *vi_cfg, sample_vo_cfg *vo_cfg, hi_size *img_size);
hi_s32 sample_dis_stop_sample(sample_vi_cfg *vi_cfg, sample_vo_cfg *vo_cfg);
hi_void sample_dis_stop_sample_without_sys_exit(sample_vi_cfg *vi_cfg, sample_vo_cfg *vo_cfg);

#ifdef __cplusplus
}
#endif /* End of __cplusplus */

#endif /* __OT_SAMPLE_DIS_H__ */