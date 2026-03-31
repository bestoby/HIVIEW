/*
  Copyright (c), 2001-2022, Shenshu Tech. Co., Ltd.
 */

#ifndef __OT_SAMPLE_FOV2LDC_H__
#define __OT_SAMPLE_FOV2LDC_H__

#include "hi_type.h"
#include "hi_common_video.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* __cplusplus */

#define FOV_PREC_BITS 20

typedef enum {
    HI_FOV_TYPE_DIAGONAL = 0,
    HI_FOV_TYPE_HOROZONTAL,
    HI_FOV_TYPE_VERTICAL,
    HI_FOV_TYPE_BUTT
} hi_fov_type;

typedef struct {
    hi_u32 width;
    hi_u32 height;
    hi_fov_type type; /* 0--diagonal,1--horizontal,2--vertical */
    hi_u32      fov; /* decimal bits 20bit */
} hi_fov_attr;

hi_s32 hi_sample_fov_to_ldcv2(const hi_fov_attr *fov_attr, hi_ldc_v2_attr *ldcv2_attr);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of __cplusplus */

#endif /* __OT_SAMPLE_FOV2LDC_H__ */

