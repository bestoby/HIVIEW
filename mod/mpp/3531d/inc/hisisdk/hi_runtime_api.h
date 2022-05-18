/*
 * Copyright (C) Hisilicon Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description:
 * Author:
 * Create: 2019-05-19
 */

#ifndef __HI_RUNTIME_API__
#define __HI_RUNTIME_API__

#include "hi_type.h"
#include "hi_common_runtime.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* __cplusplus */

/*****************************************************************************
*   Prototype    : HiRTInit
*   Description  : Environment Init
*   Parameters   : hi_char *globalSetting             GlobalSetting for runtime
                                                        if NULL, use default
                   HiRTMemCtrl *memCtrl               Memory Controll by alloc,flush,and free
                                                        if NULL, use mmz
*   Return Value : HI_SUCCESS: Success; Error codes: Failure.
*****************************************************************************/
hi_s32 HiRTInit(IN const hi_char *globalSetting, IN HiRTMemCtrl *memCtrl);

/*****************************************************************************
*   Prototype    : HiRTLoadModelGroup
*   Description  : Load model
*   Parameters   : hi_char*           modelGroupConfig    Input Group Config
                   HiRTGroupInfo*     modelGroupAttr      Group Info, such as wk,cop,connector
                   HiRTGroupHandle*   groupHandle          output Group Handle
*   Return Value : HI_SUCCESS: Success; Error codes: Failure.
*****************************************************************************/
hi_s32 HiRTLoadModelGroup(IN const hi_char *modelGroupConfig,
                          IN HiRTGroupInfo *modelGroupAttr,
                          OUT HiRTGroupHandle *groupHandle);

/*****************************************************************************
*   Prototype    : HiRTForwardGroupSync
*   Description  : Perform prediction on input sample(s), and output responses for corresponding sample(s), Sync Fuction
*   Parameters   : HiRTGroupHandle            groupHandle   Group Handle generated by HiRTLoadModelGroup
*                  HiRTGroupSrcBlobArrayPtr   src           Input Blobs
*                  HiRTGroupDstBlobArrayPtr   dst           Output Blobs.
                   hi_u64                     srcId         the number of src
*   Return Value: HI_SUCCESS: Success; Error codes: Failure.
*****************************************************************************/
hi_s32 HiRTForwardGroupSync(IN const HiRTGroupHandle groupHandle,
                            IN const HiRTGroupSrcBlobArrayPtr src,
                            OUT HiRTGroupDstBlobArrayPtr dst,
                            IN hi_u64 srcId);

/*****************************************************************************
*   Prototype    : HiRTForwardGroupAsync
*   Description  : Perform prediction on input sample(s), and output responses for corresponding sample(s),
*                  ASync Function
*   Parameters   : HiRTGroupHandle            groupHandle   Group Handle generated by HiRTLoadModelGroup
*                  can't be NULL
*                  HiRTGroupSrcBlobArrayPtr   src           Input Blobs
*                  HiRTGroupDstBlobArrayPtr   dst           Output Blobs.
*                  hi_u64                     srcId         Frame id set by user, can be duplicate number
*                  HiRTForwardCallback        cbFun         Callback for ForwardGroup,
*                                                                    can't be null
*   Return Value: HI_SUCCESS: Success; Error codes: Failure.
*****************************************************************************/
hi_s32 HiRTForwardGroupAsync(IN const HiRTGroupHandle groupHandle,
                             IN const HiRTGroupSrcBlobArrayPtr src,
                             OUT HiRTGroupDstBlobArrayPtr dst,
                             IN hi_u64 srcId, IN HiRTForwardCallback callbackFunc);

/*****************************************************************************
*   Prototype    : HiRTUnloadModelGroup
*   Description  : Unload model
*   Parameters   : HiRTGroupHandle   groupHandle      Group Handle generated by HiRTLoadModelGroup
*                                                                can not be NULL
*
*   Return Value : HI_SUCCESS: Success; Error codes: Failure.
*****************************************************************************/
hi_s32 HiRTUnloadModelGroup(IN const HiRTGroupHandle groupHandle);

/*****************************************************************************
*   Prototype    : HiRTDeInit
*   Description  : Environment DeInit
*   Return Value : HI_SUCCESS: Success; Error codes: Failure.
*****************************************************************************/
hi_s32 HiRTDeInit();

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */

#endif /* __HI_RUNTIME_API__ */