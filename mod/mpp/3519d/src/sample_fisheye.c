/*
  Copyright (c), 2001-2024, Shenshu Tech. Co., Ltd.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/prctl.h>
#include <sys/ioctl.h>

#include "hi_mpi_vgs.h"
#include "hi_mpi_gdc.h"
#include "hi_common_gdc.h"
#include "hi_common_vgs.h"
#include "sample_comm.h"

#define X_ALIGN 2
#define Y_ALIGN 2
#define ADDR_ALIGN 16
#define out_ratio_1(x) ((x) / 3)
#define out_ratio_2(x) ((x) * 2 / 3)
#define out_ratio_3(x) ((x) / 2)

static volatile sig_atomic_t g_fisheye_sample_sig_flag = 0;

hi_payload_type g_venc_type = HI_PT_H265;
sample_rc g_rc_mode = SAMPLE_RC_CBR;
hi_size g_size;
hi_pic_size g_pic_size = PIC_1080P;

hi_u16 g_lmf_coef[128] = {
    0, 15, 31, 47, 63, 79, 95, 111, 127, 143, 159, 175,
    191, 207, 223, 239, 255, 271, 286, 302, 318, 334, 350, 365, 381, 397, 412,
    428, 443, 459, 474, 490, 505, 520, 536, 551, 566, 581, 596, 611, 626, 641,
    656, 670, 685, 699, 713, 728, 742, 756, 769, 783, 797, 810, 823, 836, 848,
    861, 873, 885, 896, 908, 919, 929, 940, 950, 959, 969, 984, 998, 1013, 1027,
    1042, 1056, 1071, 1085, 1100, 1114, 1129, 1143, 1158, 1172, 1187, 1201, 1215,
    1230, 1244, 1259, 1273, 1288, 1302, 1317, 1331, 1346, 1360, 1375, 1389, 1404,
    1418, 1433, 1447, 1462, 1476, 1491, 1505, 1519, 1534, 1548, 1563, 1577, 1592,
    1606, 1621, 1635, 1650, 1664, 1679, 1693, 1708, 1722, 1737, 1751, 1766, 1780, 1795, 1809, 1823, 1838
};

static sample_comm_venc_chn_param g_venc_chn_param = {
    .frame_rate           = 30,
    .stats_time           = 2,
    .gop                  = 60,
    .venc_size            = {1920, 1080},
    .size                 = -1,
    .profile              = 0,
    .is_rcn_ref_share_buf = HI_FALSE,
    .gop_attr             = {
        .gop_mode = HI_VENC_GOP_MODE_NORMAL_P,
        .normal_p = {2},
    },
    .type                 = HI_PT_H265,
    .rc_mode              = SAMPLE_RC_CBR,
};

pthread_t g_thread_id;
hi_bool g_set_fisheye_attr = HI_FALSE;

typedef struct {
    hi_vi_pipe vi_pipe;
    hi_vi_chn vi_chn;
} fisheye_set_attr_thread_info;

sample_vo_cfg g_vo_cfg;

/* function : show usage */
hi_void sample_fisheye_usage(const char *prg_name)
{
    printf("Usage : %s <index> <vo intf> <venc type>\n", prg_name);
    printf("index:\n");
    printf("\t 0) fisheye 360 panorama 2 half with ceiling mount.\n");
    printf("\t 1) fisheye 360 panorama and 2 normal PTZ with desktop mount.\n");
    printf("\t 2) fisheye 180 panorama and 2 normal dynamic PTZ with wall mount.\n");
    printf("\t 3) fisheye source picture and 3 normal PTZ with wall mount.\n");
    printf("\t 4) nine_lattice preview(Only images larger than or equal to 8M are supported).\n");
    printf("\t 5) nine_lattice_vpss preview(Only images larger than or equal to 8M are supported).\n");

    printf("vo intf:\n");
    printf("\t 0) vo BT1120 output, default.\n");

    printf("venc type:\n");
    printf("\t 0) H265, default.\n");
    printf("\t 1) H264.\n");
    return;
}

hi_void sample_fisheye_stop_switch_mode_thread(hi_void)
{
    if (g_set_fisheye_attr != HI_FALSE) {
        g_set_fisheye_attr = HI_FALSE;
        pthread_join(g_thread_id, HI_NULL);
    }
}

static hi_void sample_fisheye_pause(hi_void)
{
    if (g_fisheye_sample_sig_flag == 0) {
        sample_pause();
    }
}

static hi_void sample_fisheye_getchar(hi_void)
{
    if (g_fisheye_sample_sig_flag == 0) {
        getchar();
    }
}

/* function : to process abnormal case */
hi_void sample_fisheye_handle_sig(hi_s32 signo)
{
    if (signo == SIGINT || signo == SIGTERM) {
        g_fisheye_sample_sig_flag = 1;
    }
}

static hi_s32 sample_fisheye_start_venc(hi_venc_chn venc_chn)
{
    hi_s32 ret;
    ret = sample_comm_venc_start(venc_chn, &g_venc_chn_param);
    if (ret != HI_SUCCESS) {
        sample_print("sample fisheye start venc failed with %#x!\n", ret);
        return ret;
    }
    return HI_SUCCESS;
}

static hi_s32 sample_fisheye_vpss_start(hi_vpss_grp vpss_grp, hi_vpss_chn vpss_chn, const hi_bool *chn_enable)
{
    hi_s32 ret;
    hi_gdc_param gdc_param;
    hi_vpss_grp_cfg grp_cfg;
    hi_vpss_grp_attr grp_attr;
    sample_vpss_chn_attr vpss_chn_attr = {0};
    gdc_param.in_size.width = g_size.width;
    gdc_param.in_size.height = g_size.height;
    gdc_param.cell_size = OT_LUT_CELL_SIZE_16;
    sample_comm_vpss_get_default_grp_attr(&grp_attr);
    grp_attr.max_width = g_size.width;
    grp_attr.max_height = g_size.height;
    sample_comm_vpss_get_default_chn_attr(&vpss_chn_attr.chn_attr[vpss_chn]);
    vpss_chn_attr.chn_attr[vpss_chn].width = g_size.width;
    vpss_chn_attr.chn_attr[vpss_chn].height = g_size.height;
    vpss_chn_attr.chn_attr[vpss_chn].compress_mode = HI_COMPRESS_MODE_NONE;
    vpss_chn_attr.chn_enable[vpss_chn] = chn_enable[vpss_chn];
    vpss_chn_attr.chn_array_size = HI_VPSS_MAX_PHYS_CHN_NUM;

    hi_mpi_vpss_get_grp_cfg(vpss_grp, &grp_cfg);
    grp_cfg.max_out_rgn_num = OT_FISHEYE_MAX_RGN_NUM;
    hi_mpi_vpss_set_grp_cfg(vpss_grp, &grp_cfg);
    ret = sample_common_vpss_start(vpss_grp, &grp_attr, &vpss_chn_attr);
    if (ret != HI_SUCCESS) {
        sample_print("SAMPLE_VPSS_Start_Fisheye failed with %#x!\n", ret);
        return ret;
    }
    hi_mpi_vpss_set_grp_gdc_param(vpss_grp, &gdc_param);
    return HI_SUCCESS;
}

static hi_s32 sample_fisheye_start_bind(
    sample_vi_cfg *vi_cfg, hi_vi_pipe vi_pipe, hi_vi_chn vi_chn, hi_venc_chn venc_chn, hi_vo_chn vo_chn)
{
    const hi_vpss_grp vpss_grp = 0;
    const hi_vpss_chn vpss_chn = 0;
    hi_s32 ret;
    hi_vo_layer vo_layer = g_vo_cfg.vo_dev;
    ret = sample_comm_vi_bind_vpss(vi_pipe, vi_chn, vpss_grp, vpss_chn);
    if (ret != HI_SUCCESS) {
        sample_print("sample vi bind vpss failed with %#x!\n", ret);
        return ret;
    }
    ret = sample_comm_vpss_bind_venc(vpss_grp, vpss_chn, venc_chn);
    if (ret != HI_SUCCESS) {
        sample_print("sample vpss bind venc failed with %#x!\n", ret);
        sample_comm_vi_un_bind_vpss(vi_pipe, vi_chn, vpss_grp, vpss_chn);
        return ret;
    }
    ret = sample_comm_vpss_bind_vo(vpss_grp, vpss_chn, vo_layer, vo_chn);
    if (ret != HI_SUCCESS) {
        sample_print("sample vpss bind vo failed with %#x!\n", ret);
        sample_comm_vi_un_bind_vpss(vi_pipe, vi_chn, vpss_grp, vpss_chn);
        sample_comm_vpss_un_bind_venc(vpss_grp, vpss_chn, venc_chn);
        return ret;
    }
    return HI_SUCCESS;
}

/*
 * Function:    SAMPLE_VIO_FISHEYE_Start Vi Vpss Vo Venc
 * Description: offline mode. Embedded isp, phychn preview
 */
static hi_s32 sample_fisheye_start_vi_vpss_vo_venc(
    sample_vi_cfg *vi_cfg, hi_vi_pipe vi_pipe, hi_vi_chn vi_chn, hi_venc_chn venc_chn, hi_vo_chn vo_chn)
{
    hi_s32 ret;
    const hi_vpss_grp vpss_grp = 0;
    const hi_vpss_chn vpss_chn = 0;
    hi_bool chn_enable[HI_VPSS_MAX_PHYS_CHN_NUM] = {0};
    chn_enable[vpss_chn] = HI_TRUE;

    if (vi_cfg == HI_NULL) {
        sample_print("vi_cfg is null\n");
        return HI_FAILURE;
    }
    vi_cfg->pipe_info[vi_pipe].chn_info[vi_chn].chn_attr.video_format = HI_VIDEO_FORMAT_TILE_32x4;
    vi_cfg->pipe_info[vi_pipe].chn_info[vi_chn].chn_attr.compress_mode = HI_COMPRESS_MODE_TILE;

    ret = sample_comm_vi_start_vi(vi_cfg);
    if (ret != HI_SUCCESS) {
        sample_print("start vi failed!\n");
        goto exit;
    }
    ret = sample_fisheye_vpss_start(vpss_grp, vpss_chn, &chn_enable[vpss_chn]);
    if (ret != HI_SUCCESS) {
        sample_print("start vpss failed!\n");
        goto stop_vpss;
    }
    ret = sample_fisheye_start_venc(venc_chn);
    if (ret != HI_SUCCESS) {
        sample_print("sample fisheye start venc failed with %#x!\n", ret);
        goto stop_vi;
    }
    ret = sample_comm_vo_start_vo(&g_vo_cfg);
    if (ret != HI_SUCCESS) {
        sample_print("sample vio start VO failed with %#x!\n", ret);
        goto stop_venc;
    }
    ret = sample_fisheye_start_bind(vi_cfg, vi_pipe, vi_chn, venc_chn, vo_chn);
    if (ret != HI_SUCCESS) {
        sample_print("sample fisheye bind failed with %#x!\n", ret);
        goto stop_vo;
    }
    return HI_SUCCESS;
stop_vo:
    sample_comm_vo_stop_vo(&g_vo_cfg);
stop_venc:
    sample_comm_venc_stop(venc_chn);
stop_vpss:
    sample_common_vpss_stop(vpss_grp, &chn_enable[vpss_chn], HI_VPSS_MAX_PHYS_CHN_NUM);
stop_vi:
    sample_comm_vi_stop_vi(vi_cfg);
exit:
    return ret;
}

hi_void sample_fisheye_stop_vi_vpss_vo_venc(
    sample_vi_cfg *vi_cfg, hi_vi_pipe vi_pipe, hi_vi_chn vi_chn, hi_vo_chn vo_chn, hi_venc_chn venc_chn)
{
    hi_vo_layer vo_layer = g_vo_cfg.vo_dev;
    const hi_vpss_grp vpss_grp = 0;
    const hi_vpss_chn vpss_chn = 0;
    hi_bool chn_enable[HI_VPSS_MAX_PHYS_CHN_NUM] = {0};
    chn_enable[vpss_chn] = HI_FALSE;
    sample_comm_vpss_un_bind_venc(vpss_grp, vpss_chn, venc_chn);
    sample_comm_vpss_un_bind_vo(vpss_grp, vpss_chn, vo_layer, vo_chn);
    sample_comm_vi_un_bind_vpss(vi_pipe, vi_chn, vpss_grp, vpss_chn);
    sample_comm_vo_stop_vo(&g_vo_cfg);
    sample_common_vpss_stop(vpss_grp, &chn_enable[vpss_chn], HI_VPSS_MAX_PHYS_CHN_NUM);
    sample_comm_vi_stop_vi(vi_cfg);
    sample_comm_venc_stop(venc_chn);
}

hi_s32 sample_fisheye_start_vi_vo(sample_vi_cfg *vi_cfg, sample_vo_cfg *vo_cfg)
{
    hi_s32 ret;

    ret = sample_comm_vi_start_vi(vi_cfg);
    if (ret != HI_SUCCESS) {
        sample_print("start vi failed!\n");
        return ret;
    }

    ret = sample_comm_vo_start_vo(vo_cfg);
    if (ret != HI_SUCCESS) {
        sample_print("SAMPLE_VIO start VO failed with %#x!\n", ret);
        goto exit;
    }

    return ret;

exit:
    sample_comm_vi_stop_vi(vi_cfg);

    return ret;
}

hi_void sample_fisheye_stop_vi_vo(sample_vi_cfg *vi_cfg, sample_vo_cfg *vo_cfg)
{
    sample_comm_vo_stop_vo(vo_cfg);

    sample_comm_vi_stop_vi(vi_cfg);
}

static hi_void *sample_proc_set_fisheye_attr_thread(hi_void *arg)
{
    hi_s32 i;
    hi_vpss_grp vpss_grp;
    hi_s32 ret;
    hi_fisheye_correction_attr correction_attr;
    fisheye_set_attr_thread_info *thread_info = HI_NULL;

    if (arg == HI_NULL) {
        printf("arg is NULL\n");
        return HI_NULL;
    }

    prctl(PR_SET_NAME, "FISHEYE_Cruise", 0, 0, 0);

    (hi_void)memset_s(&correction_attr, sizeof(correction_attr), 0, sizeof(correction_attr));

    thread_info = (fisheye_set_attr_thread_info *)arg;

    vpss_grp = thread_info->vi_pipe;

    while (g_set_fisheye_attr == HI_TRUE) {
        ret = hi_mpi_vpss_get_grp_fisheye(vpss_grp, &correction_attr);
        if (ret != HI_SUCCESS) {
            return HI_NULL;
        }

        for (i = 1; i < 3; i++) { /* 3:set 3 rgn attr */
            if (correction_attr.fisheye_attr.fisheye_rgn_attr[i].pan == 360) { /* 360:pan max value */
                correction_attr.fisheye_attr.fisheye_rgn_attr[i].pan = 0;
            } else {
                correction_attr.fisheye_attr.fisheye_rgn_attr[i].pan++;
            }

            if (correction_attr.fisheye_attr.fisheye_rgn_attr[i].tilt == 360) { /* 360:tilt max value */
                correction_attr.fisheye_attr.fisheye_rgn_attr[i].tilt = 0;
            } else {
                correction_attr.fisheye_attr.fisheye_rgn_attr[i].tilt++;
            }
        }

        ret = hi_mpi_vpss_set_grp_fisheye(vpss_grp, &correction_attr);
        if (ret != HI_SUCCESS) {
            return HI_NULL;
        }

        sleep(1);
    }

    return HI_NULL;
}

hi_void sample_fisheye_start_set_fisheye_attr_thread(hi_vi_pipe vi_pipe, hi_vi_chn vi_chn)
{
    fisheye_set_attr_thread_info fisheye_attr_thread_info;

    fisheye_attr_thread_info.vi_pipe = vi_pipe;
    fisheye_attr_thread_info.vi_chn = vi_chn;

    g_set_fisheye_attr = HI_TRUE;

    pthread_create(&g_thread_id, HI_NULL, sample_proc_set_fisheye_attr_thread, &fisheye_attr_thread_info);

    sleep(1);
}

hi_void sample_fisheye_stop_set_fisheye_attr_thread(hi_void)
{
    if (g_set_fisheye_attr != HI_FALSE) {
        g_set_fisheye_attr = HI_FALSE;
        pthread_join(g_thread_id, HI_NULL);
    }
}

hi_void sample_fisheye_get_default_rgn_attr(hi_fisheye_rgn_attr *rgn_attr, hi_fisheye_view_mode view_mode)
{
    rgn_attr->view_mode = view_mode;
    rgn_attr->in_radius = 0;
    rgn_attr->out_radius = 1638; /* maohw 1200:out radius value */
    rgn_attr->pan = 180; /* 180:pan value */
    rgn_attr->tilt = 180; /* 180:tilt value */
    rgn_attr->hor_zoom = 4095; /* 4095: default hor_zoom value */
    rgn_attr->ver_zoom = 4095; /* 4095: default ver_zoom value */
    rgn_attr->out_rect.x = 0;
    rgn_attr->out_rect.y = 0;
    rgn_attr->out_rect.width = HI_ALIGN_DOWN(out_ratio_1(g_size.width), X_ALIGN);
    rgn_attr->out_rect.height = HI_ALIGN_DOWN(out_ratio_1(g_size.height), Y_ALIGN);
}

static hi_void sample_fisheye_get_default_attr(hi_fisheye_attr *fisheye_attr,
    hi_u32 rgn_num, hi_fisheye_view_mode view_mode)
{
    hi_u32 i;

    fisheye_attr->lmf_en = HI_FALSE;
    fisheye_attr->bg_color = COLOR_RGB_BLUE;
    fisheye_attr->hor_offset = 0;
    fisheye_attr->ver_offset = 0;
    fisheye_attr->trapezoid_coef = 0;
    fisheye_attr->fan_strength = 0;
    fisheye_attr->mount_mode = HI_FISHEYE_MOUNT_MODE_WALL;
    fisheye_attr->rgn_num = rgn_num;
    for (i = 0; i < fisheye_attr->rgn_num; i++) {
        sample_fisheye_get_default_rgn_attr(&fisheye_attr->fisheye_rgn_attr[i], view_mode);
    }
}

static hi_void sample_fisheye_get_out_rect_attr(hi_fisheye_rgn_attr *rgn_attr)
{
    rgn_attr[0].out_rect.x = 0;                                                  /* 0:rgn index */
    rgn_attr[0].out_rect.y = 0;                                                  /* 0:rgn index */
    rgn_attr[1].out_rect.x = HI_ALIGN_DOWN(out_ratio_1(g_size.width), X_ALIGN);  /* 1:rgn index */
    rgn_attr[1].out_rect.y = 0;                                                  /* 1:rgn index */
    rgn_attr[2].out_rect.x = HI_ALIGN_DOWN(out_ratio_2(g_size.width), X_ALIGN);  /* 2:rgn index */
    rgn_attr[2].out_rect.y = 0;                                                  /* 2:rgn index */
    rgn_attr[3].out_rect.x = 0;                                                  /* 3:rgn index */
    rgn_attr[3].out_rect.y = HI_ALIGN_DOWN(out_ratio_1(g_size.height), Y_ALIGN); /* 3:rgn index */
    rgn_attr[4].out_rect.x = HI_ALIGN_DOWN(out_ratio_1(g_size.width), X_ALIGN);  /* 4:rgn index */
    rgn_attr[4].out_rect.y = HI_ALIGN_DOWN(out_ratio_1(g_size.height), Y_ALIGN); /* 4:rgn index */
    rgn_attr[5].out_rect.x = HI_ALIGN_DOWN(out_ratio_2(g_size.width), X_ALIGN);  /* 5:rgn index */
    rgn_attr[5].out_rect.y = HI_ALIGN_DOWN(out_ratio_1(g_size.height), Y_ALIGN); /* 5:rgn index */
    rgn_attr[6].out_rect.x = 0;                                                  /* 6:rgn index */
    rgn_attr[6].out_rect.y = HI_ALIGN_DOWN(out_ratio_2(g_size.height), Y_ALIGN); /* 6:rgn index */
    rgn_attr[7].out_rect.x = HI_ALIGN_DOWN(out_ratio_1(g_size.width), X_ALIGN);  /* 7:rgn index */
    rgn_attr[7].out_rect.y = HI_ALIGN_DOWN(out_ratio_2(g_size.height), Y_ALIGN); /* 7:rgn index */
    rgn_attr[8].out_rect.x = HI_ALIGN_DOWN(out_ratio_2(g_size.width), X_ALIGN);  /* 8:rgn index */
    rgn_attr[8].out_rect.y = HI_ALIGN_DOWN(out_ratio_2(g_size.height), Y_ALIGN); /* 8:rgn index */
}

static hi_void sample_fisheye_get_buff_attr(hi_pic_buf_attr *buf_attr)
{
    buf_attr->width = g_size.width;
    buf_attr->height = g_size.height;
    buf_attr->align = 0;
    buf_attr->video_format  = HI_VIDEO_FORMAT_LINEAR;
    buf_attr->bit_width = HI_DATA_BIT_WIDTH_8;
    buf_attr->pixel_format = SAMPLE_PIXEL_FORMAT;
    buf_attr->compress_mode = HI_COMPRESS_MODE_NONE;
}

static hi_s32 sample_fisheye_set_vb(hi_u64 *out_phys_addr,
    hi_u8 **out_virt_addr, hi_vb_blk *vb_out_blk, hi_u32 *buf_size)
{
    hi_char *mmz_name = HI_NULL;
    hi_pic_buf_attr buf_attr;

    sample_fisheye_get_buff_attr(&buf_attr);
    *buf_size = hi_common_get_pic_buf_size(&buf_attr);

    *vb_out_blk = hi_mpi_vb_get_blk(HI_VB_INVALID_POOL_ID, *buf_size, mmz_name);
    if (*vb_out_blk == HI_VB_INVALID_HANDLE) {
        sample_print("Info:mpi_vb_get_blk(size:%u) fail\n", *buf_size);
        return HI_FAILURE;
    }

    *out_phys_addr = hi_mpi_vb_handle_to_phys_addr(*vb_out_blk);
    if (*out_phys_addr == 0) {
        sample_print("Info:mpi_vb_handle_to_phys_addr fail, u32OutPhyAddr:0x%llx\n", *out_phys_addr);
        hi_mpi_vb_release_blk(*vb_out_blk);
        return HI_FAILURE;
    }

    *out_virt_addr = (hi_u8 *)hi_mpi_sys_mmap(*out_phys_addr, *buf_size);
    if (*out_virt_addr == HI_NULL) {
        sample_print("Info:mpi_sys_mmap fail\n");
        hi_mpi_vb_release_blk(*vb_out_blk);
        return HI_FAILURE;
    }
    return HI_SUCCESS;
}

static hi_void sample_fisheye_set_task(hi_gdc_task_attr *task, const hi_u64 *out_phys_addr,
    const hi_u8 *out_virt_addr, const hi_vb_blk *vb_out_blk)
{
    hi_u32 out_width;
    hi_u32 out_height;
    hi_u32 out_stride;

    out_width = g_size.width;
    out_height = g_size.height;
    out_stride = HI_ALIGN_UP(out_width, ADDR_ALIGN);

    (hi_void)memcpy_s(&task->img_out, sizeof(hi_video_frame_info), &task->img_in, sizeof(hi_video_frame_info));

    task->img_out.pool_id = hi_mpi_vb_handle_to_pool_id(*vb_out_blk);
    task->img_out.video_frame.phys_addr[0] = *out_phys_addr;
    task->img_out.video_frame.phys_addr[1] = *out_phys_addr + out_stride * out_height;
    task->img_out.video_frame.virt_addr[0] = (hi_void *)out_virt_addr;
    task->img_out.video_frame.virt_addr[1] = (hi_void *)out_virt_addr + out_stride * out_height;
    task->img_out.video_frame.stride[0] = out_stride;
    task->img_out.video_frame.stride[1] = out_stride;
    task->img_out.video_frame.width = out_width;
    task->img_out.video_frame.height = out_height;
    task->img_out.video_frame.header_stride[0] = ADDR_ALIGN;
    task->img_out.video_frame.header_stride[1] = ADDR_ALIGN;
    task->img_out.video_frame.compress_mode = HI_COMPRESS_MODE_NONE;
    task->img_out.video_frame.video_format = HI_VIDEO_FORMAT_LINEAR;
}

static void sample_fisheye_set_vgs_task(hi_vgs_task_attr *vgs_task, const hi_u64 *out_phys_addr,
    const hi_u8 *out_virt_addr, const hi_vb_blk *vb_out_blk)
{
    hi_u32 out_width;
    hi_u32 out_height;
    hi_u32 out_stride;

    out_width = g_size.width;
    out_height = g_size.height;
    out_stride = HI_ALIGN_UP(g_size.width, ADDR_ALIGN);

    vgs_task->img_out.pool_id = hi_mpi_vb_handle_to_pool_id(*vb_out_blk);
    vgs_task->img_out.video_frame.width = HI_ALIGN_DOWN(out_ratio_1(out_width), X_ALIGN);
    vgs_task->img_out.video_frame.height = HI_ALIGN_DOWN(out_ratio_1(out_height), Y_ALIGN);
    vgs_task->img_out.video_frame.field = HI_VIDEO_FIELD_FRAME;
    vgs_task->img_out.video_frame.pixel_format = HI_PIXEL_FORMAT_YVU_SEMIPLANAR_420;
    vgs_task->img_out.video_frame.stride[0] = out_stride;
    vgs_task->img_out.video_frame.stride[1] = out_stride;

    vgs_task->img_out.video_frame.phys_addr[0] = *out_phys_addr + (hi_u64)out_stride * out_ratio_2(out_height) +
        HI_ALIGN_DOWN(out_ratio_2(out_width), X_ALIGN);
    vgs_task->img_out.video_frame.phys_addr[1] = *out_phys_addr + (hi_u64)out_stride * out_height +
        ((hi_u64)out_stride * out_ratio_1(out_height)) + HI_ALIGN_DOWN(out_ratio_2(out_width), X_ALIGN);
    vgs_task->img_out.video_frame.virt_addr[0] = (hi_void *)out_virt_addr + (hi_u64)out_stride *
        out_ratio_2(out_height) + HI_ALIGN_DOWN(out_ratio_2(out_width), X_ALIGN);
    vgs_task->img_out.video_frame.virt_addr[1] = (hi_void *)out_virt_addr + (hi_u64)out_stride * out_height +
        ((hi_u64)out_stride * out_ratio_1(out_height)) + HI_ALIGN_DOWN(out_ratio_2(out_width), X_ALIGN);
}

static hi_s32 sample_fisheye_nine_lattice_add_gdc(const hi_gdc_task_attr *task, hi_gdc_handle handle)
{
    hi_fisheye_attr fisheye_attr;
    hi_s32 ret;
    hi_u32 rgn_width = HI_ALIGN_DOWN(out_ratio_1(g_size.width), X_ALIGN);
    hi_u32 rgn_height = HI_ALIGN_DOWN(out_ratio_1(g_size.height), X_ALIGN);
    hi_u32 width_missing = g_size.width - 3 * rgn_width;   /* 3:rgn num of hor */
    hi_u32 height_missing = g_size.height - 3 * rgn_height; /* 3:rgn num of ver */
    hi_fisheye_rgn_attr *rgn_attr = TD_NULL;

    rgn_attr = fisheye_attr.fisheye_rgn_attr;
    sample_fisheye_get_default_attr(&fisheye_attr, HI_FISHEYE_MAX_RGN_NUM, HI_FISHEYE_VIEW_MODE_180_PANORAMA);
    if (width_missing != 0) {
        /* 2:last rgn index of the first row; 3:rgn num of hor */
        for (int i = 2; i < HI_FISHEYE_MAX_RGN_NUM; i = i + 3) {
            rgn_attr[i].out_rect.width += width_missing;
        }
    }
    if (height_missing != 0) {
        /* 6:first rgn index at the bottom */
        for (int i = 6; i < HI_FISHEYE_MAX_RGN_NUM; i++) {
            rgn_attr[i].out_rect.height += height_missing;
        }
    }
    sample_fisheye_get_out_rect_attr(rgn_attr);
    ret = hi_mpi_gdc_add_correction_task(handle, task, &fisheye_attr);
    if (ret != HI_SUCCESS) {
        return ret;
    }
    ret = hi_mpi_gdc_end_job(handle);
    if (ret != HI_SUCCESS) {
        return ret;
    }
    return HI_SUCCESS;
}

static hi_s32 sample_fisheye_nine_lattice_add_vgs(const hi_gdc_task_attr *task, const hi_u64 *out_phys_addr,
    const hi_u8 *out_virt_addr, const hi_vb_blk *vb_out_blk)
{
    hi_vgs_handle vgs_handle;
    hi_vgs_task_attr vgs_task;
    hi_s32 ret;

    ret = hi_mpi_vgs_begin_job(&vgs_handle);
    if (ret != HI_SUCCESS) {
        return ret;
    }

    (hi_void)memcpy_s(&vgs_task.img_in, sizeof(hi_video_frame_info), &task->img_in, sizeof(hi_video_frame_info));
    (hi_void)memcpy_s(&vgs_task.img_out, sizeof(hi_video_frame_info), &task->img_out, sizeof(hi_video_frame_info));

    sample_fisheye_set_vgs_task(&vgs_task, out_phys_addr, out_virt_addr, vb_out_blk);
    ret = hi_mpi_vgs_add_scale_task(vgs_handle, &vgs_task, HI_VGS_SCALE_COEF_NORM);
    if (ret != HI_SUCCESS) {
        hi_mpi_vgs_cancel_job(vgs_handle);
        return ret;
    }

    ret = hi_mpi_vgs_end_job(vgs_handle);
    if (ret != HI_SUCCESS) {
        hi_mpi_vgs_cancel_job(vgs_handle);
        return ret;
    }

    return HI_SUCCESS;
}

static hi_void sample_fisheye_nine_lattice_thread_set_vi_chn_attr(hi_vi_chn_attr *chn_attr, hi_u32 *old_depth,
    hi_u32 depth)
{
    hi_s32 ret;
    const hi_vi_pipe vi_pipe = 0;
    const hi_vi_chn vi_chn = 0;

    ret = hi_mpi_vi_get_chn_attr(vi_pipe, vi_chn, chn_attr);
    if (ret != HI_SUCCESS) {
        sample_print("Err, ret:0x%x\n", ret);
    }

    *old_depth = chn_attr->depth;
    chn_attr->depth = depth;
    ret = hi_mpi_vi_set_chn_attr(vi_pipe, vi_chn, chn_attr);
    if (ret != HI_SUCCESS) {
        sample_print("Err, ret:0x%x\n", ret);
    }
}

static hi_void sample_fisheye_nine_lattice_thread_set_vi_chn_attr_1(hi_vi_chn_attr *chn_attr, hi_u32 old_depth)
{
    hi_s32 ret;
    const hi_vi_pipe vi_pipe = 0;
    const hi_vi_chn vi_chn = 0;

    chn_attr->depth = old_depth;
    ret = hi_mpi_vi_set_chn_attr(vi_pipe, vi_chn, chn_attr);
    if (ret != HI_SUCCESS) {
        sample_print("Err, ret:0x%x\n", ret);
    }
}

static hi_s32 sample_fisheye_nine_lattice_thread_frame(hi_u32 *buf_size, hi_gdc_task_attr *task,
    hi_u8 **out_virt_addr, hi_vb_blk *vb_out_blk)
{
    hi_s32 ret;
    const hi_vi_pipe vi_pipe = 0;
    const hi_vi_chn vi_chn = 0;
    const hi_s32 milli_sec = -1;
    hi_u64 out_phys_addr;
    hi_gdc_handle handle;

    ret = hi_mpi_gdc_begin_job(&handle);
    if (ret != HI_SUCCESS) {
        sample_print("Err, ret:0x%x\n", ret);
        return HI_NULL;
    }
    ret = sample_fisheye_set_vb(&out_phys_addr, out_virt_addr, vb_out_blk, buf_size);
    if (ret != HI_SUCCESS) {
        hi_mpi_gdc_cancel_job(handle);
        return HI_NULL;
    }
    ret = hi_mpi_vi_get_chn_frame(vi_pipe, vi_chn, &task->img_in, milli_sec);
    if (ret != HI_SUCCESS) {
        sample_print("Err, ret:0x%x\n", ret);
        goto gdc_out;
    }
    sample_fisheye_set_task(task, &out_phys_addr, *out_virt_addr, vb_out_blk);
    ret = sample_fisheye_nine_lattice_add_gdc(task, handle);
    if (ret != HI_SUCCESS) {
        sample_print("Err, ret:0x%x\n", ret);
        goto vi_out;
    }
    ret = sample_fisheye_nine_lattice_add_vgs(task, &out_phys_addr, *out_virt_addr, vb_out_blk);
    if (ret != HI_SUCCESS) {
        sample_print("Err, ret:0x%x\n", ret);
        goto vi_out;
    }
    return HI_SUCCESS;
vi_out:
    hi_mpi_vi_release_chn_frame(vi_pipe, vi_chn, &task->img_in);
gdc_out:
    hi_mpi_gdc_cancel_job(handle);
    hi_mpi_sys_munmap(*out_virt_addr, *buf_size);
    hi_mpi_vb_release_blk(*vb_out_blk);
    return HI_FAILURE;
}

static hi_s32 sample_fisheye_nine_lattice_thread_out(hi_u32 buf_size, const hi_gdc_task_attr *task,
    const hi_u8 *out_virt_addr, hi_vb_blk vb_out_blk)
{
    hi_s32 ret;
    const hi_vi_pipe vi_pipe = 0;
    const hi_vi_chn vi_chn = 0;

    ret = hi_mpi_vi_release_chn_frame(vi_pipe, vi_chn, &task->img_in);
    if (ret != HI_SUCCESS) {
        printf("Info:mpi_vi_release_chn_frame fail, ret:0x%x\n", ret);
        hi_mpi_sys_munmap(out_virt_addr, buf_size);
        hi_mpi_vb_release_blk(vb_out_blk);
        return HI_FAILURE;
    }
    ret = hi_mpi_sys_munmap(out_virt_addr, buf_size);
    if (ret != HI_SUCCESS) {
        printf("Info:mpi_sys_munmap fail,ret:0x%x\n", ret);
        hi_mpi_vb_release_blk(vb_out_blk);
        return HI_FAILURE;
    }

    ret = hi_mpi_vb_release_blk(vb_out_blk);
    if (ret != HI_SUCCESS) {
        printf("Info:mpi_vb_release_blk fail,ret:0x%x\n", ret);
        return HI_FAILURE;
    }
    return HI_SUCCESS;
}

static hi_void *sample_fisheye_nine_lattice_thread(hi_void *arg)
{
    hi_s32 ret;
    hi_gdc_task_attr task;
    hi_vi_chn_attr chn_attr = { 0 };
    hi_u32 buf_size = 0;
    hi_u8 *out_virt_addr = HI_NULL;
    hi_vb_blk vb_out_blk = 0;
    hi_u32 old_depth;
    const hi_u32 depth = 2; /* 2:default depth */
    hi_vo_layer vo_layer = g_vo_cfg.vo_dev;
    const hi_vo_chn vo_chn = 0;
    const hi_s32 milli_sec = -1;

    if (arg == HI_NULL) {
        sample_print("arg is NULL\n");
        return HI_NULL;
    }

    prctl(PR_SET_NAME, "FISHEYE_Frame", 0, 0, 0);

    sample_fisheye_nine_lattice_thread_set_vi_chn_attr(&chn_attr, &old_depth, depth);

    while (g_set_fisheye_attr == HI_TRUE) {
        ret = sample_fisheye_nine_lattice_thread_frame(&buf_size, &task, &out_virt_addr, &vb_out_blk);
        if (ret != HI_SUCCESS) {
            return HI_NULL;
        }

        ret = hi_mpi_vo_send_frame(vo_layer, vo_chn, &task.img_out, milli_sec);
        if (ret != HI_SUCCESS) {
            sample_print("mpi_vo_send_frame fail, ret:0x%x\n", ret);
        }

        ret = sample_fisheye_nine_lattice_thread_out(buf_size, &task, out_virt_addr, vb_out_blk);
        if (ret != HI_SUCCESS) {
            return HI_NULL;
        }
        usleep(20000); /* 20000:delay times */
    }

    sample_fisheye_nine_lattice_thread_set_vi_chn_attr_1(&chn_attr, old_depth);
    return HI_NULL;
}

/* define SAMPLE_MEM_SHARE_ENABLE, when use tools to dump YUV/RAW. */
#ifdef SAMPLE_MEM_SHARE_ENABLE
hi_void sample_gdc_init_mem_share(hi_void)
{
    hi_u32 i;
    hi_vb_common_pools_id pools_id = {0};

    if (hi_mpi_vb_get_common_pool_id(&pools_id) != HI_SUCCESS) {
        sample_print("get common pool_id failed!\n");
        return;
    }
    for (i = 0; i < pools_id.pool_cnt; ++i) {
        hi_mpi_vb_pool_share_all(pools_id.pool[i]);
    }
}
#endif

hi_s32 sample_fisheye_sys_init(hi_void)
{
    hi_s32 ret;
    hi_pic_buf_attr buf_attr;
    hi_u64 blk_size;
    hi_vb_cfg vb_cfg;

    /* step  1: mpp system init */
    (hi_void)memset_s(&vb_cfg, sizeof(hi_vb_cfg), 0, sizeof(hi_vb_cfg));
    vb_cfg.max_pool_cnt = 128; /* 128:pool cnt num */

    /* comm video buffer */
    sample_fisheye_get_buff_attr(&buf_attr);
    blk_size = hi_common_get_pic_buf_size(&buf_attr);

    vb_cfg.common_pool[0].blk_size = blk_size;
    vb_cfg.common_pool[0].blk_cnt = 15; /* 15:blk cnt 0 num */

    /* vb for vi raw */
    buf_attr.pixel_format = HI_PIXEL_FORMAT_YVU_SEMIPLANAR_422;
    blk_size = hi_common_get_pic_buf_size(&buf_attr);

    vb_cfg.common_pool[1].blk_size = blk_size;
    vb_cfg.common_pool[1].blk_cnt = 4; /* 4:blk cnt 1 num */

    ret = sample_comm_sys_vb_init(&vb_cfg);
    if (ret != HI_SUCCESS) {
        sample_print("system init failed with %d!\n", ret);
        sample_comm_sys_exit();
        return ret;
    }
#ifdef SAMPLE_MEM_SHARE_ENABLE
    sample_gdc_init_mem_share();
#endif
    return HI_SUCCESS;
}

hi_void sample_fisheye_get_360_correction_attr(hi_fisheye_correction_attr *correction_attr)
{
    hi_u32 width = g_size.width;
    hi_u32 height = g_size.height;

    correction_attr->enable = HI_TRUE;
    correction_attr->dst_size.width = width;
    correction_attr->dst_size.height = height;
    /* 2:rgn_num */
    sample_fisheye_get_default_attr(&correction_attr->fisheye_attr, 2, HI_FISHEYE_VIEW_MODE_360_PANORAMA);
    correction_attr->fisheye_attr.lmf_en = HI_TRUE;
    correction_attr->fisheye_attr.mount_mode = HI_FISHEYE_MOUNT_MODE_CEILING;

    correction_attr->fisheye_attr.fisheye_rgn_attr[0].out_rect.x = 0;
    correction_attr->fisheye_attr.fisheye_rgn_attr[0].out_rect.y = 0;
    correction_attr->fisheye_attr.fisheye_rgn_attr[0].out_rect.width = width;
    correction_attr->fisheye_attr.fisheye_rgn_attr[0].out_rect.height = HI_ALIGN_DOWN(out_ratio_3(height), Y_ALIGN);

    correction_attr->fisheye_attr.fisheye_rgn_attr[1].out_rect.x = 0;
    correction_attr->fisheye_attr.fisheye_rgn_attr[1].out_rect.y = HI_ALIGN_DOWN(out_ratio_3(height), Y_ALIGN);
    correction_attr->fisheye_attr.fisheye_rgn_attr[1].out_rect.width = width;
    correction_attr->fisheye_attr.fisheye_rgn_attr[1].out_rect.height = HI_ALIGN_DOWN(out_ratio_3(height), Y_ALIGN);
}

hi_s32 sample_fisheye_360_pannorama_set_chn_fisheye(hi_vpss_grp vpss_grp, hi_vi_chn vi_chn,
    hi_fisheye_correction_attr *correction_attr)
{
    hi_s32 ret;
    hi_unused(vi_chn);

    ret = hi_mpi_vpss_set_grp_fisheye(vpss_grp, correction_attr);
    if (ret != HI_SUCCESS) {
        sample_print("set fisheye attr failed with ret:0x%x!\n", ret);
        return ret;
    }

    printf("\nplease press enter, disable fisheye\n\n");
    sample_fisheye_getchar();

    correction_attr->enable = HI_FALSE;
    ret = hi_mpi_vpss_set_grp_fisheye(vpss_grp, correction_attr);
    if (ret != HI_SUCCESS) {
        sample_print("set fisheye attr failed with ret:0x%x!\n", ret);
        return ret;
    }

    printf("\nplease press enter, enable fisheye\n");
    sample_fisheye_getchar();

    correction_attr->enable = HI_TRUE;
    ret = hi_mpi_vpss_set_grp_fisheye(vpss_grp, correction_attr);
    if (ret != HI_SUCCESS) {
        sample_print("set fisheye attr failed with ret:0x%x!\n", ret);
        return ret;
    }
    return ret;
}

/* function : vi/vpss: offline/online fisheye mode VI-VO. Embedded isp, phychn channel preview. */
hi_s32 sample_fisheye_360_panorama_celing_2_half(sample_vi_cfg *vi_cfg, hi_vi_pipe vi_pipe)
{
    const hi_vi_chn vi_chn = 0;
    hi_venc_chn venc_chn = 0;
    const hi_vo_chn vo_chn = 0;
    const hi_s32 chn_num = 1;
    const hi_vpss_grp vpss_grp = 0;
    hi_s32 ret;
    hi_fisheye_cfg fisheye_cfg;
    hi_fisheye_correction_attr correction_attr;

    ret = sample_fisheye_sys_init();
    if (ret != HI_SUCCESS) {
        return ret;
    }
    /* step  1: start VI VPSS VO VENC */
    ret = sample_fisheye_start_vi_vpss_vo_venc(vi_cfg, vi_pipe, vi_chn, venc_chn, vo_chn);
    if (ret != HI_SUCCESS) {
        sample_print("sample_fisheye_start_vi_vpss_vo_venc failed with %d\n", ret);
        goto exit;
    }

    /* step   2: stream venc process -- get stream, then save it to file. */
    ret = sample_comm_venc_start_get_stream(&venc_chn, chn_num);
    if (ret != HI_SUCCESS) {
        sample_print("sample_comm_venc_start_get_stream failed with %d\n", ret);
        goto stop_vi_vpss_vo_venc;
    }

    /* step  3: set fisheye Attr */
    (hi_void)memcpy_s(fisheye_cfg.lmf_coef, sizeof(g_lmf_coef), g_lmf_coef, sizeof(g_lmf_coef));
    sample_fisheye_get_360_correction_attr(&correction_attr);

    printf("\nplease press enter, enable fisheye\n\n");
    sample_fisheye_getchar();
    ret = hi_mpi_vpss_set_grp_fisheye_cfg(vpss_grp, &fisheye_cfg);
    if (ret != HI_SUCCESS) {
        sample_print("set fisheye config failed with ret:0x%x!\n", ret);
        goto stop_get_stream;
    }
    ret = sample_fisheye_360_pannorama_set_chn_fisheye(vpss_grp, vi_chn, &correction_attr);
    if (ret != HI_SUCCESS) {
        goto stop_get_stream;
    }
    sample_fisheye_pause();

stop_get_stream:
    sample_comm_venc_stop_get_stream(chn_num);
stop_vi_vpss_vo_venc:
    sample_fisheye_stop_vi_vpss_vo_venc(vi_cfg, vi_pipe, vi_chn, venc_chn, vo_chn);
exit:
    sample_comm_sys_exit();
    return ret;
}

hi_void sample_fisheye_get_360_and_2_normal_correction_attr(hi_fisheye_correction_attr *correction_attr)
{
    hi_u32 width = g_size.width;
    hi_u32 height = g_size.height;

    correction_attr->enable = HI_TRUE;
    correction_attr->dst_size.width = width;
    correction_attr->dst_size.height = height;

    /* 3:rgn num */
    sample_fisheye_get_default_attr(&correction_attr->fisheye_attr, 3, HI_FISHEYE_VIEW_MODE_NORM);

    correction_attr->fisheye_attr.mount_mode = HI_FISHEYE_MOUNT_MODE_DESKTOP;

    correction_attr->fisheye_attr.fisheye_rgn_attr[0].view_mode = HI_FISHEYE_VIEW_MODE_360_PANORAMA;
    correction_attr->fisheye_attr.fisheye_rgn_attr[0].out_rect.width = width;
    correction_attr->fisheye_attr.fisheye_rgn_attr[0].out_rect.height = HI_ALIGN_DOWN(out_ratio_3(height), Y_ALIGN);

    correction_attr->fisheye_attr.fisheye_rgn_attr[1].pan = 0;
    correction_attr->fisheye_attr.fisheye_rgn_attr[1].tilt = 90; /* 90:tilt value */
    correction_attr->fisheye_attr.fisheye_rgn_attr[1].hor_zoom = 2048; /* 2048:hor zoom value */
    correction_attr->fisheye_attr.fisheye_rgn_attr[1].out_rect.y = HI_ALIGN_DOWN(out_ratio_3(height), Y_ALIGN);
    correction_attr->fisheye_attr.fisheye_rgn_attr[1].out_rect.width = HI_ALIGN_DOWN(out_ratio_3(width), X_ALIGN);
    correction_attr->fisheye_attr.fisheye_rgn_attr[1].out_rect.height = HI_ALIGN_DOWN(out_ratio_3(height), Y_ALIGN);

    correction_attr->fisheye_attr.fisheye_rgn_attr[2].tilt = 270; /* 2:rgn num; 270:tilt value */
    correction_attr->fisheye_attr.fisheye_rgn_attr[2].hor_zoom = 2048; /* 2:rgn num; 2048:hor zoom */
    /* 2:rgn num */
    correction_attr->fisheye_attr.fisheye_rgn_attr[2].out_rect.x = HI_ALIGN_DOWN(out_ratio_3(width), X_ALIGN);
    /* 2:rgn num */
    correction_attr->fisheye_attr.fisheye_rgn_attr[2].out_rect.y = HI_ALIGN_DOWN(out_ratio_3(height), Y_ALIGN);
    /* 2:rgn num */
    correction_attr->fisheye_attr.fisheye_rgn_attr[2].out_rect.width = HI_ALIGN_DOWN(out_ratio_3(width), X_ALIGN);
    /* 2:rgn num */
    correction_attr->fisheye_attr.fisheye_rgn_attr[2].out_rect.height = HI_ALIGN_DOWN(out_ratio_3(height), Y_ALIGN);
}

/* function : vi/vpss: offline/online fisheye mode VI-VO. Embedded isp, phychn channel preview. */
hi_s32 sample_fisheye_360_panorama_desktop_and_2_normal(sample_vi_cfg *vi_cfg, hi_vi_pipe vi_pipe)
{
    const hi_vi_chn vi_chn = 0;
    const hi_vo_chn vo_chn = 0;
    hi_venc_chn venc_chn = 0;
    const hi_s32 chn_num = 1;
    hi_s32 ret;
    hi_fisheye_correction_attr correction_attr;

    ret = sample_fisheye_sys_init();
    if (ret != HI_SUCCESS) {
        return ret;
    }

    /* step  1: start VI VPSS VO  VENC */
    ret = sample_fisheye_start_vi_vpss_vo_venc(vi_cfg, vi_pipe, vi_chn, venc_chn, vo_chn);
    if (ret != HI_SUCCESS) {
        sample_print("sample_fisheye_start_vi_vpss_vo_venc failed with  %d\n", ret);
        goto exit;
    }

    /* step   2: stream venc process -- get stream, then save it to file. */
    ret = sample_comm_venc_start_get_stream(&venc_chn, chn_num);
    if (ret != HI_SUCCESS) {
        sample_print("sample_comm_venc_start_get_stream failed with %d\n", ret);
        goto stop_vi_vpss_vo_venc;
    }

    /* step  3: set fisheye Attr */
    sample_fisheye_get_360_and_2_normal_correction_attr(&correction_attr);

    ret = sample_fisheye_360_pannorama_set_chn_fisheye(vi_pipe, vi_chn, &correction_attr);
    if (ret != HI_SUCCESS) {
        goto stop_get_stream;
    }

    sample_fisheye_pause();

stop_get_stream:
    sample_comm_venc_stop_get_stream(chn_num);

stop_vi_vpss_vo_venc:
    sample_fisheye_stop_vi_vpss_vo_venc(vi_cfg, vi_pipe, vi_chn, venc_chn, vo_chn);

exit:
    sample_comm_sys_exit();
    return ret;
}

hi_void sample_fisheye_get_180_and_2_normal_correction_attr(hi_fisheye_correction_attr *correction_attr)
{
    g_size.width = 3840; 
    g_size.height = 2160;
    sample_comm_vi_get_size_by_sns_type(SENSOR0_TYPE, &g_size);
   
    hi_u32 width = g_size.width;
    hi_u32 height = g_size.height;

    correction_attr->enable = HI_TRUE;
    correction_attr->dst_size.width = width;
    correction_attr->dst_size.height = height;
    
    printf("correction_attr dst_size.width:%d, height:%d\n", width, height);

    /* 3:rgn num */
    
    // ############ total rgn areas = dst_size  ############ 
    
    sample_fisheye_get_default_attr(&correction_attr->fisheye_attr, 1/*maohw 3*/, HI_FISHEYE_VIEW_MODE_NORM);
    correction_attr->fisheye_attr.trapezoid_coef = 0; /* 10:default value */
    correction_attr->fisheye_attr.fan_strength = 94; /* 300:default value */
    correction_attr->fisheye_attr.bg_color = COLOR_RGB_BLUE;
    correction_attr->fisheye_attr.hor_offset = 17;
    correction_attr->fisheye_attr.ver_offset = -17;
    correction_attr->fisheye_attr.trapezoid_coef = 0;
    correction_attr->fisheye_attr.fan_strength = 94;
    
    correction_attr->fisheye_attr.fisheye_rgn_attr[0].view_mode = HI_FISHEYE_VIEW_MODE_180_PANORAMA;
    correction_attr->fisheye_attr.fisheye_rgn_attr[0].out_rect.width = width;
    correction_attr->fisheye_attr.fisheye_rgn_attr[0].out_rect.height = height;
    
#if 0 //maohw
    correction_attr->fisheye_attr.fisheye_rgn_attr[0].view_mode = HI_FISHEYE_VIEW_MODE_180_PANORAMA;
    correction_attr->fisheye_attr.fisheye_rgn_attr[0].out_rect.width = width;
    correction_attr->fisheye_attr.fisheye_rgn_attr[0].out_rect.height = HI_ALIGN_DOWN(out_ratio_3(height), Y_ALIGN);

    correction_attr->fisheye_attr.fisheye_rgn_attr[1].out_rect.y = HI_ALIGN_DOWN(out_ratio_3(height), Y_ALIGN);
    correction_attr->fisheye_attr.fisheye_rgn_attr[1].out_rect.width = HI_ALIGN_DOWN(out_ratio_3(width), X_ALIGN);
    correction_attr->fisheye_attr.fisheye_rgn_attr[1].out_rect.height = HI_ALIGN_DOWN(out_ratio_3(height), Y_ALIGN);

    correction_attr->fisheye_attr.fisheye_rgn_attr[2].pan = 200; /* 2:rgn num; 200:pan value */
    correction_attr->fisheye_attr.fisheye_rgn_attr[2].tilt = 200; /* 2:rgn num; 200:tilt value */
    /* 2:rgn num */
    correction_attr->fisheye_attr.fisheye_rgn_attr[2].out_rect.x = HI_ALIGN_DOWN(out_ratio_3(width), X_ALIGN);
    /* 2:rgn num */
    correction_attr->fisheye_attr.fisheye_rgn_attr[2].out_rect.y = HI_ALIGN_DOWN(out_ratio_3(height), Y_ALIGN);
    /* 2:rgn num */
    correction_attr->fisheye_attr.fisheye_rgn_attr[2].out_rect.width = HI_ALIGN_DOWN(out_ratio_3(width), X_ALIGN);
    /* 2:rgn num */
    correction_attr->fisheye_attr.fisheye_rgn_attr[2].out_rect.height = HI_ALIGN_DOWN(out_ratio_3(height), Y_ALIGN);
#endif    
}
/* function : vi/vpss: offline/online fisheye mode VI-VO. Embedded isp, phychn channel preview. */
hi_s32 sample_fisheye_180_panorama_wall_and_2_dynamic_normal(sample_vi_cfg *vi_cfg, hi_vi_pipe vi_pipe)
{
    const hi_vi_chn vi_chn = 0;
    hi_venc_chn venc_chn = 0;
    const hi_vo_chn vo_chn = 0;
    const hi_s32 chn_num = 1;
    hi_s32 ret;
    hi_fisheye_correction_attr correction_attr;

    ret = sample_fisheye_sys_init();
    if (ret != HI_SUCCESS) {
        return ret;
    }

    /* step  1: start VI VPSS  VO  VENC */
    ret = sample_fisheye_start_vi_vpss_vo_venc(vi_cfg, vi_pipe, vi_chn, venc_chn, vo_chn);
    if (ret != HI_SUCCESS) {
        sample_print("sample_fisheye_start_vi_vpss_vo_venc %d\n", ret);
        goto exit;
    }
    /* step   2: stream venc process -- get stream, then save it to file. */
    ret = sample_comm_venc_start_get_stream(&venc_chn, chn_num);
    if (ret != HI_SUCCESS) {
        sample_print("sample_comm_venc_start_get_stream failed with %d\n", ret);
        goto stop_vi_vpss_vo_venc;
    }
    /* step  3: set fisheye Attr */
    sample_fisheye_get_180_and_2_normal_correction_attr(&correction_attr);

    ret = hi_mpi_vpss_set_grp_fisheye(vi_pipe, &correction_attr);
    if (ret != HI_SUCCESS) {
        sample_print("set fisheye attr failed with ret:0x%x!\n", ret);
        goto stop_get_stream;
    }
    /* create a pthread to change the fisheye attr */
    sample_fisheye_start_set_fisheye_attr_thread(vi_pipe, vi_chn);
    sample_fisheye_pause();
    sample_fisheye_stop_set_fisheye_attr_thread();

stop_get_stream:
    sample_comm_venc_stop_get_stream(chn_num);
stop_vi_vpss_vo_venc:
    sample_fisheye_stop_vi_vpss_vo_venc(vi_cfg, vi_pipe, vi_chn, venc_chn, vo_chn);
exit:
    sample_comm_sys_exit();
    return ret;
}

hi_void sample_fisheye_get_source_and_3_normal_correction_attr(hi_fisheye_correction_attr *correction_attr)
{
    hi_u32 width = g_size.width;
    hi_u32 height = g_size.height;

    correction_attr->enable = HI_TRUE;
    correction_attr->dst_size.width = width;
    correction_attr->dst_size.height = height;

    /* 4:rgn num */
    sample_fisheye_get_default_attr(&correction_attr->fisheye_attr, 4, HI_FISHEYE_VIEW_MODE_NORM);
    correction_attr->fisheye_attr.trapezoid_coef = 10; /* 10:trapezoid_coef strength */

    correction_attr->fisheye_attr.fisheye_rgn_attr[0].hor_zoom = 2048; /* 2048:hor zoom value */
    correction_attr->fisheye_attr.fisheye_rgn_attr[0].view_mode = HI_FISHEYE_VIEW_MODE_NO_TRANS;
    correction_attr->fisheye_attr.fisheye_rgn_attr[0].out_rect.width =  HI_ALIGN_DOWN(out_ratio_3(width), X_ALIGN);
    correction_attr->fisheye_attr.fisheye_rgn_attr[0].out_rect.height = HI_ALIGN_DOWN(out_ratio_3(height), Y_ALIGN);

    correction_attr->fisheye_attr.fisheye_rgn_attr[1].tilt = 135; /* 135:tilt value */
    correction_attr->fisheye_attr.fisheye_rgn_attr[1].hor_zoom = 2048; /* 2048:hor zoom value */
    correction_attr->fisheye_attr.fisheye_rgn_attr[1].out_rect.x = HI_ALIGN_DOWN(out_ratio_3(width), X_ALIGN);
    correction_attr->fisheye_attr.fisheye_rgn_attr[1].out_rect.width = HI_ALIGN_DOWN(out_ratio_3(width), X_ALIGN);
    correction_attr->fisheye_attr.fisheye_rgn_attr[1].out_rect.height = HI_ALIGN_DOWN(out_ratio_3(height), Y_ALIGN);

    correction_attr->fisheye_attr.fisheye_rgn_attr[2].pan = 135; /* 2:rgn num; 135:pan value */
    correction_attr->fisheye_attr.fisheye_rgn_attr[2].hor_zoom = 2048; /* 2:rgn num; 2048: hor zoom value */
    /* 2:rgn num */
    correction_attr->fisheye_attr.fisheye_rgn_attr[2].out_rect.y = HI_ALIGN_DOWN(out_ratio_3(height), Y_ALIGN);
    /* 2:rgn num */
    correction_attr->fisheye_attr.fisheye_rgn_attr[2].out_rect.width = HI_ALIGN_DOWN(out_ratio_3(width), X_ALIGN);
    /* 2:rgn num */
    correction_attr->fisheye_attr.fisheye_rgn_attr[2].out_rect.height = HI_ALIGN_DOWN(out_ratio_3(height), Y_ALIGN);

    correction_attr->fisheye_attr.fisheye_rgn_attr[3].pan = 215; /* 3:rgn num; 215:pan value */
    correction_attr->fisheye_attr.fisheye_rgn_attr[3].hor_zoom = 2048; /* 3:rgn num; 2048: hor zoom value */
    /* 3:rgn num */
    correction_attr->fisheye_attr.fisheye_rgn_attr[3].out_rect.x = HI_ALIGN_DOWN(out_ratio_3(width), X_ALIGN);
    /* 3:rgn num */
    correction_attr->fisheye_attr.fisheye_rgn_attr[3].out_rect.y = HI_ALIGN_DOWN(out_ratio_3(height), Y_ALIGN);
    /* 3:rgn num */
    correction_attr->fisheye_attr.fisheye_rgn_attr[3].out_rect.width = HI_ALIGN_DOWN(out_ratio_3(width), X_ALIGN);
    /* 3:rgn num */
    correction_attr->fisheye_attr.fisheye_rgn_attr[3].out_rect.height = HI_ALIGN_DOWN(out_ratio_3(height), Y_ALIGN);
}

hi_void sample_fisheye_nine_lattice_vpss_correction_attr(hi_fisheye_correction_attr *correction_attr)
{
    hi_u32 width = g_size.width;
    hi_u32 height = g_size.height;
    hi_u32 rgn_width = HI_ALIGN_DOWN(out_ratio_1(g_size.width), X_ALIGN);
    hi_u32 rgn_height = HI_ALIGN_DOWN(out_ratio_1(g_size.height), X_ALIGN);
    hi_u32 width_missing = width - 3 * rgn_width;    /* 3:rgn num of hor */
    hi_u32 height_missing = height - 3 * rgn_height; /* 3:rgn num of ver */
    hi_fisheye_rgn_attr *rgn_attr = TD_NULL;

    correction_attr->enable = HI_TRUE;
    correction_attr->dst_size.width = width;
    correction_attr->dst_size.height = height;
    rgn_attr = correction_attr->fisheye_attr.fisheye_rgn_attr;
    sample_fisheye_get_default_attr(&correction_attr->fisheye_attr, HI_FISHEYE_MAX_RGN_NUM, HI_FISHEYE_VIEW_MODE_NORM);
    if (width_missing != 0) {
        /* 2:last rgn index of the first row; 3:rgn num of hor */
        for (int i = 2; i < HI_FISHEYE_MAX_RGN_NUM; i = i + 3) {
            rgn_attr[i].out_rect.width += width_missing;
        }
    }
    if (height_missing != 0) {
        /* 6:first rgn index at the bottom */
        for (int i = 6; i < HI_FISHEYE_MAX_RGN_NUM; i++) {
            rgn_attr[i].out_rect.height += height_missing;
        }
    }

    sample_fisheye_get_out_rect_attr(rgn_attr);
}

static hi_s32 sample_fisheye_source_and_3_normal_switch(hi_vi_pipe vi_pipe, hi_vi_chn vi_chn,
    hi_fisheye_correction_attr *correction_attr)
{
    hi_s32 ret;
    hi_unused(vi_chn);

    printf("\nplease press enter, disable fisheye\n\n");
    sample_fisheye_getchar();

    correction_attr->enable = HI_FALSE;
    ret = hi_mpi_vpss_set_grp_fisheye(vi_pipe, correction_attr);
    if (ret != HI_SUCCESS) {
        sample_print("set fisheye attr failed with ret:0x%x!\n", ret);
        return ret;
    }
    printf("\nplease press enter, enable fisheye\n");
    sample_fisheye_getchar();

    correction_attr->enable = HI_TRUE;
    ret = hi_mpi_vpss_set_grp_fisheye(vi_pipe, correction_attr);
    if (ret != HI_SUCCESS) {
        sample_print("set fisheye attr failed with ret:0x%x!\n", ret);
        return ret;
    }

    return HI_SUCCESS;
}

/* function : vi/vpss: offline/online fisheye mode VI-VO. Embedded isp, phychn channel preview. */
hi_s32 sample_fisheye_source_and_3_normal(sample_vi_cfg *vi_cfg, hi_vi_pipe vi_pipe)
{
    const hi_vi_chn vi_chn = 0;
    hi_venc_chn venc_chn = 0;
    const hi_vo_chn vo_chn = 0;
    const hi_s32 chn_num = 1;
    hi_s32 ret;
    hi_fisheye_correction_attr correction_attr;

    /* mpp system init */
    ret = sample_fisheye_sys_init();
    if (ret != HI_SUCCESS) {
        return ret;
    }
    /* step  1: start VI VPSS  VO  VENC */
    ret = sample_fisheye_start_vi_vpss_vo_venc(vi_cfg, vi_pipe, vi_chn, venc_chn, vo_chn);
    if (ret != HI_SUCCESS) {
        sample_print("sample_fisheye_start_vi_vpss_vo_venc %d\n", ret);
        goto exit;
    }
    /* step   2: stream venc process -- get stream, then save it to file. */
    ret = sample_comm_venc_start_get_stream(&venc_chn, chn_num);
    if (ret != HI_SUCCESS) {
        sample_print("sample_comm_venc_start_get_stream failed with %d\n", ret);
        goto stop_vi_vpss_vo_venc;
    }
    /* step  3: set fisheye Attr */
    sample_fisheye_get_source_and_3_normal_correction_attr(&correction_attr);

    ret = hi_mpi_vpss_set_grp_fisheye(vi_pipe, &correction_attr);
    if (ret != HI_SUCCESS) {
        sample_print("set fisheye attr failed with ret:0x%x!\n", ret);
        goto stop_get_stream;
    }

    ret = sample_fisheye_source_and_3_normal_switch(vi_pipe, vi_chn, &correction_attr);
    if (ret != HI_SUCCESS) {
        goto stop_get_stream;
    }

    sample_fisheye_pause();

stop_get_stream:
    sample_comm_venc_stop_get_stream(chn_num);
stop_vi_vpss_vo_venc:
    sample_fisheye_stop_vi_vpss_vo_venc(vi_cfg, vi_pipe, vi_chn, venc_chn, vo_chn);
exit:
    sample_comm_sys_exit();
    return ret;
}

hi_s32 sample_fisheye_nine_lattice_vpss(sample_vi_cfg *vi_cfg, hi_vi_pipe vi_pipe)
{
    const hi_vi_chn vi_chn = 0;
    hi_venc_chn venc_chn = 0;
    const hi_vo_chn vo_chn = 0;
    const hi_s32 chn_num = 1;
    hi_s32 ret;
    hi_fisheye_correction_attr correction_attr;

    /* mpp system init */
    ret = sample_fisheye_sys_init();
    if (ret != HI_SUCCESS) {
        return ret;
    }
    /* step  1: start VI VPSS VO VENC */
    ret = sample_fisheye_start_vi_vpss_vo_venc(vi_cfg, vi_pipe, vi_chn, venc_chn, vo_chn);
    if (ret != HI_SUCCESS) {
        sample_print("sample_fisheye_start_vi_vpss_vo_venc %d\n", ret);
        goto exit;
    }
    /* step   2: stream venc process -- get stream, then save it to file. */
    ret = sample_comm_venc_start_get_stream(&venc_chn, chn_num);
    if (ret != HI_SUCCESS) {
        sample_print("sample_comm_venc_start_get_stream failed with %d\n", ret);
        goto stop_vi_vpss_vo_venc;
    }
    /* step  3: set fisheye Attr */
    sample_fisheye_nine_lattice_vpss_correction_attr(&correction_attr);

    ret = hi_mpi_vpss_set_grp_fisheye(vi_pipe, &correction_attr);
    if (ret != HI_SUCCESS) {
        sample_print("set fisheye attr failed with ret:0x%x!\n", ret);
        goto stop_get_stream;
    }
    sample_fisheye_pause();

stop_get_stream:
    sample_comm_venc_stop_get_stream(chn_num);
stop_vi_vpss_vo_venc:
    sample_fisheye_stop_vi_vpss_vo_venc(vi_cfg, vi_pipe, vi_chn, venc_chn, vo_chn);
exit:
    sample_comm_sys_exit();
    return ret;
}

/* function : vi/: online fisheye mode VI-VO. Embedded isp, phychn channel preview. */
hi_s32 sample_fisheye_nine_lattice(sample_vi_cfg *vi_cfg)
{
    hi_s32 ret;
    hi_vi_pipe vi_pipe = 0;
    hi_vi_chn vi_chn = 0;
    if (vi_cfg == HI_NULL) {
        sample_print("vi_cfg is NULL!\n");
        return HI_FAILURE;
    }

    /* step    1: mpp system init */
    ret = sample_fisheye_sys_init();
    if (ret != HI_SUCCESS) {
        return ret;
    }
    /* step    2: start vi vo */

    vi_cfg->pipe_info[vi_pipe].chn_info[vi_chn].chn_attr.video_format = HI_VIDEO_FORMAT_TILE_32x4;
    vi_cfg->pipe_info[vi_pipe].chn_info[vi_chn].chn_attr.compress_mode = HI_COMPRESS_MODE_TILE;
    sample_fisheye_start_vi_vo(vi_cfg, &g_vo_cfg);

    /* step    3: start a thread */
    g_set_fisheye_attr = HI_TRUE;
    pthread_create(&g_thread_id, HI_NULL, sample_fisheye_nine_lattice_thread, &g_size);

    sample_fisheye_pause();

    sample_fisheye_stop_switch_mode_thread();

    sample_fisheye_stop_vi_vo(vi_cfg, &g_vo_cfg);
    sample_comm_sys_exit();
    return ret;
}

static hi_s32 sample_fisheye_start(char *argv[])
{
    hi_s32 ret;
    sample_vi_cfg vi_cfg;
    const hi_vi_pipe vi_pipe = 0;

    sample_comm_vi_get_default_vi_cfg(SENSOR0_TYPE, &vi_cfg);
    /* step1:  Get  input size */
    sample_comm_vi_get_size_by_sns_type(vi_cfg.sns_info.sns_type, &g_size);

    switch (*argv[1]) {
        /* VI/VPSS - VO. Embedded isp, phychn channel preview. */
        case '0':
            ret = sample_fisheye_360_panorama_celing_2_half(&vi_cfg, vi_pipe);
            break;

        case '1':
            ret = sample_fisheye_360_panorama_desktop_and_2_normal(&vi_cfg, vi_pipe);
            break;

        case '2':
            ret = sample_fisheye_180_panorama_wall_and_2_dynamic_normal(&vi_cfg, vi_pipe);
            break;

        case '3':
            ret = sample_fisheye_source_and_3_normal(&vi_cfg, vi_pipe);
            break;

        case '4':
            ret = sample_fisheye_nine_lattice(&vi_cfg);
            break;

        case '5':
            ret = sample_fisheye_nine_lattice_vpss(&vi_cfg, vi_pipe);
            break;

        default:
            sample_print("the index is invalid!\n");
            sample_fisheye_usage(argv[0]);
            return HI_FAILURE;
    }
    return ret;
}
/*
 * function    : main()
 * Description : video fisheye preview sample
 */
#ifdef __LITEOS__
int app_main(int argc, char *argv[])
#else
int sample_fisheye_main(int argc, char *argv[])
#endif
{
    hi_s32 ret;

    if ((argc < 2) || (argc > 4) || (strlen(argv[1]) != 1)) { /* 2,4:arg num */
        goto arg_error;
    }

#ifndef __LITEOS__
    sample_sys_signal(&sample_fisheye_handle_sig);
#endif

    if (argc > 3) { /* 3:arg num */
        if ((strlen(argv[3]) != 1)) { /* 3 intf */
            goto arg_error;
        }
        switch (*argv[3]) { /* 3:array index  */
            case '0':
                break;
            case '1':
                g_venc_chn_param.type = HI_PT_H264;
                break;
            default:
                goto arg_error;
        }
    }

    sample_comm_vo_get_def_config(&g_vo_cfg);
    g_vo_cfg.vo_intf_type = HI_VO_INTF_BT1120;
    if (argc > 2) { /* 2:arg num */
        if ((strlen(argv[2]) != 1)) { /* 2 intf */
            goto arg_error;
        }
        switch (*argv[2]) { /* 2:array index */
            case '0':
                g_vo_cfg.vo_intf_type = HI_VO_INTF_BT1120;
                break;
            case '1':
                g_vo_cfg.vo_intf_type = HI_VO_INTF_BT1120;
                break;
            default:
                goto arg_error;
        }
    }
    ret = sample_fisheye_start(argv);
    if (ret == HI_SUCCESS && g_fisheye_sample_sig_flag == 0) {
        sample_print("program exit normally!\n");
    } else {
        sample_print("program exit abnormally!\n");
    }

    return ret;

arg_error:
    sample_fisheye_usage(argv[0]);
    return HI_FAILURE;
}