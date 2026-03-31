/*
  Copyright (c), 2001-2022, Shenshu Tech. Co., Ltd.
 */

#include "sample_fov2ldc.h"

#include <math.h>
#include <stdio.h>
#include "securec.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* #ifdef __cplusplus */

#define PI   3.141593
#define MAX_OF_FOV  90

static hi_void sample_fov_to_ldcv2_print(const hi_fov_attr *fov_attr, hi_ldc_v2_attr *ldcv2_attr, hi_float fov)
{
    printf("\nInput:");
    printf("\n\tw=%d,h=%d,fovtype=%d (%s),fov=%f",
           fov_attr->width, fov_attr->height, fov_attr->type,
           (fov_attr->type == 0) ? ("diagonal") : ((fov_attr->type == 1) ? ("horizontal") : ("vertical")), fov);
    printf("\n\nOutput:");
    printf("\n\tfocal_len:%d,%d", ldcv2_attr->focal_len_x, ldcv2_attr->focal_len_y);
    printf("\n\tcoor_shift:%d,%d", ldcv2_attr->coord_shift_x, ldcv2_attr->coord_shift_y);

    printf("\n\tsrc_calibration_ratio:");
    printf("\n\t\t%d,%d,%d,%d",
           ldcv2_attr->src_calibration_ratio[0], /* index 0 */
           ldcv2_attr->src_calibration_ratio[1], /* index 1 */
           ldcv2_attr->src_calibration_ratio[2], /* index 2 */
           ldcv2_attr->src_calibration_ratio[3]); /* index 3 */
    printf("\n\t\t%d,%d,%d,%d",
           ldcv2_attr->src_calibration_ratio[4], /* index 4 */
           ldcv2_attr->src_calibration_ratio[5], /* index 5 */
           ldcv2_attr->src_calibration_ratio[6], /* index 6 */
           ldcv2_attr->src_calibration_ratio[7]); /* index 7 */
    printf("\n\t\t%d",
           ldcv2_attr->src_calibration_ratio[8]); /* index 8 */

    printf("\n\tdst_calibration_ratio:");
    printf("\n\t\t%d,%d,%d,%d",
           ldcv2_attr->dst_calibration_ratio[0], /* index 0 */
           ldcv2_attr->dst_calibration_ratio[1], /* index 1 */
           ldcv2_attr->dst_calibration_ratio[2], /* index 2 */
           ldcv2_attr->dst_calibration_ratio[3]); /* index 3 */
    printf("\n\t\t%d,%d,%d,%d",
           ldcv2_attr->dst_calibration_ratio[4], /* index 4 */
           ldcv2_attr->dst_calibration_ratio[5], /* index 5 */
           ldcv2_attr->dst_calibration_ratio[6], /* index 6 */
           ldcv2_attr->dst_calibration_ratio[7]); /* index 7 */
    printf("\n\t\t%d,%d,%d,%d",
           ldcv2_attr->dst_calibration_ratio[8], /* index 8 */
           ldcv2_attr->dst_calibration_ratio[9], /* index 9 */
           ldcv2_attr->dst_calibration_ratio[10], /* index 10 */
           ldcv2_attr->dst_calibration_ratio[11]); /* index 11 */
    printf("\n\t\t%d,%d",
           ldcv2_attr->dst_calibration_ratio[12],  /* index 12 */
           ldcv2_attr->dst_calibration_ratio[13]); /* index 13 */

    printf("\n\tmax_du:%d", ldcv2_attr->max_du);
    printf("\n\n");
}

static hi_void sample_fov_to_ldcv2_assign(const hi_fov_attr *fov_attr, hi_ldc_v2_attr *ldcv2_attr,
    hi_float fov_horizontal, hi_float fov_vertical)
{
    const hi_s32 dist_coef_prec = 100000;

    ldcv2_attr->focal_len_x = (hi_s32)(round((fov_attr->width / 2) /    /* 2 half */
        tan(fov_horizontal * PI / 180 / 2) * 100));                     /* 180 degree, 2 half, 100 gradient */
    ldcv2_attr->focal_len_y = (hi_s32)(round((fov_attr->height / 2) /   /* 2 half */
        tan(fov_vertical * PI / 180 / 2) * 100));                       /* 180 degree, 2 half, 100 gradient */

    ldcv2_attr->coord_shift_x = (hi_s32)(round(fov_attr->width / 2 * 100)); /* 2 half, 100 gradient */
    ldcv2_attr->coord_shift_y = (hi_s32)(round(fov_attr->height / 2 * 100)); /* 2 half, 100 gradient */

    ldcv2_attr->src_calibration_ratio[0] = 1 * dist_coef_prec; /* index 0 */
    ldcv2_attr->src_calibration_ratio[1] = 0; /* index 1 */
    ldcv2_attr->src_calibration_ratio[2] = 0; /* index 2 */
    ldcv2_attr->src_calibration_ratio[3] = 0; /* index 3 */
    ldcv2_attr->src_calibration_ratio[4] = 0; /* index 4 */
    ldcv2_attr->src_calibration_ratio[5] = 0; /* index 5 */
    ldcv2_attr->src_calibration_ratio[6] = 0; /* index 6 */
    ldcv2_attr->src_calibration_ratio[7] = 0; /* index 7 */
    ldcv2_attr->src_calibration_ratio[8] = 800000; /* index 8 : 800000 */

    ldcv2_attr->dst_calibration_ratio[0] = 1 * dist_coef_prec; /* index 0 */
    ldcv2_attr->dst_calibration_ratio[1] = 0; /* index 1 */
    ldcv2_attr->dst_calibration_ratio[2] = 0; /* index 2 */
    ldcv2_attr->dst_calibration_ratio[3] = 0; /* index 3 */
    ldcv2_attr->dst_calibration_ratio[4] = 0; /* index 4 */
    ldcv2_attr->dst_calibration_ratio[5] = 0; /* index 5 */
    ldcv2_attr->dst_calibration_ratio[6] = 0; /* index 6 */
    ldcv2_attr->dst_calibration_ratio[7] = 0; /* index 7 */
    ldcv2_attr->dst_calibration_ratio[8] = 0; /* index 8 */
    ldcv2_attr->dst_calibration_ratio[9] = 0; /* index 9 */
    ldcv2_attr->dst_calibration_ratio[10] = 0; /* index 10 */
    ldcv2_attr->dst_calibration_ratio[11] = 0; /* index 11 */
    ldcv2_attr->dst_calibration_ratio[12] = 800000; /* index 12 : 800000 */
    ldcv2_attr->dst_calibration_ratio[13] = 800000; /* index 13 : 800000 */

    ldcv2_attr->max_du = 1048576;     /* max value 1048576 */
}

hi_s32 hi_sample_fov_to_ldcv2(const hi_fov_attr *fov_attr, hi_ldc_v2_attr *ldcv2_attr)
{
    hi_float fov_horizontal = 0.0;
    hi_float fov_vertical = 0.0;
    hi_float fov;

    if ((fov_attr == HI_NULL) || (ldcv2_attr == HI_NULL)) {
        return HI_FAILURE;
    }

    (hi_void)memset_s(ldcv2_attr, sizeof(hi_ldc_v2_attr), 0, sizeof(hi_ldc_v2_attr));
    fov = (hi_float)fov_attr->fov / (hi_float)(1 << FOV_PREC_BITS);

    switch (fov_attr->type) {
        case HI_FOV_TYPE_DIAGONAL: /* input diagonal field of view angle */
            fov_horizontal = fov * fov_attr->width /
                sqrt(fov_attr->width * fov_attr->width + fov_attr->height * fov_attr->height);
            fov_vertical   = fov * fov_attr->height /
                sqrt(fov_attr->width * fov_attr->width + fov_attr->height * fov_attr->height);
            break;
        case HI_FOV_TYPE_HOROZONTAL: /* input horizontal field of view angle */
            fov_horizontal = fov;
            fov_vertical   = fov * fov_attr->height / fov_attr->width;
            break;
        case HI_FOV_TYPE_VERTICAL: /* input vertical field of view angle */
            fov_horizontal = fov * fov_attr->width / fov_attr->height;
            fov_vertical   = fov;
            break;
        default:
            break;
    }

    sample_fov_to_ldcv2_assign(fov_attr, ldcv2_attr, fov_horizontal, fov_vertical);
    sample_fov_to_ldcv2_print(fov_attr, ldcv2_attr, fov);
    return HI_SUCCESS;
}

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */

