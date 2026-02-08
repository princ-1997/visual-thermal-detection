#ifndef __RTSP_STREAMER_H__
#define __RTSP_STREAMER_H__

#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
#include "libircam.h"
#include "libiruvc.h"
#include "libirv4l2.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 热红外 RTSP 推流模块
 * 
 * 参考 reference 项目的 appSource 模式：
 *   GStreamer RTSP Server + appsrc → 硬件编码 → RTSP 输出
 *
 * 数据流:
 *   IR UVC Camera → ir_video_frame_get() [need_data回调中]
 *                 → appsrc → mpph264enc → rtph264pay → RTSP Client
 */

/* 帧格式枚举（与 IR SDK data.h 保持一致） */
typedef enum {
    IR_FMT_YUYV_IMAGE = 0,
    IR_FMT_NV12_IMAGE,
    IR_FMT_NV12_AND_TEMP,
    IR_FMT_YUYV_AND_TEMP,
    IR_FMT_UYVY_IMAGE,
} IrFrameFmt_t;

/* RTSP 推流上下文 */
typedef struct {
    /* IR 相机句柄 */
    IrVideoHandle_t*  video_handle;
    void*             driver_handle;    /* IruvcHandle_t* */

    /* 视频参数 */
    const char*       gst_format;       /* GStreamer 格式: "NV12" 或 "YUY2" */
    int               width;            /* 图像宽度 */
    int               height;           /* 图像高度 */
    int               framerate;        /* 帧率 */

    /* 帧缓冲 */
    uint8_t*          raw_buffer;       /* 原始帧缓冲(可能含温度数据) */
    uint32_t          raw_buffer_size;  /* 原始帧大小 */
    uint32_t          image_size;       /* 纯图像部分大小 */

    /* GStreamer 时间戳 */
    GstClockTime      timestamp;
    gboolean          stream_started;

    /* UVC 流参数(用于启动/停止流) */
    int               uvc_width;        /* UVC 采集宽度 */
    int               uvc_height;       /* UVC 采集高度 */
    int               uvc_fps;          /* UVC 采集帧率 */
    float             frame_size_ratio; /* 帧大小比率 */
    IrFrameFmt_t      frame_fmt;        /* 帧输出格式 */
    gboolean          is_v4l2;          /* TRUE=V4L2(DVP/MIPI), FALSE=UVC */
} RtspStreamerCtx_t;

/**
 * 初始化 RTSP 推流上下文
 * @param ctx           上下文指针
 * @param video_handle  IR 视频句柄
 * @param driver_handle UVC 驱动句柄(IruvcHandle_t*)
 * @param frame_fmt     帧输出格式
 * @param img_w         图像宽度
 * @param img_h         图像高度
 * @param raw_size      原始帧总大小(含温度等)
 * @param image_size    纯图像大小
 * @param uvc_w         UVC 采集宽度
 * @param uvc_h         UVC 采集高度
 * @param uvc_fps       UVC 帧率
 * @param frame_ratio   帧大小比率
 * @return 0 成功, -1 失败
 */
int rtsp_streamer_init(RtspStreamerCtx_t* ctx,
                       IrVideoHandle_t* video_handle,
                       void* driver_handle,
                       IrFrameFmt_t frame_fmt,
                       int img_w, int img_h,
                       uint32_t raw_size, uint32_t image_size,
                       int stream_w, int stream_h, int stream_fps,
                       float frame_ratio, int is_v4l2);

/**
 * 启动 RTSP 服务器（阻塞在 GMainLoop）
 * @param ctx      上下文指针
 * @param port     RTSP 端口 (如 "8554")
 * @param url_path URL 路径 (如 "/thermal")
 * @param loop     外部创建的 GMainLoop（便于信号处理器退出）
 * @return 0 成功
 */
int rtsp_streamer_run(RtspStreamerCtx_t* ctx, const char* port, const char* url_path, GMainLoop* loop);

/**
 * 释放 RTSP 推流资源
 */
void rtsp_streamer_cleanup(RtspStreamerCtx_t* ctx);

#ifdef __cplusplus
}
#endif

#endif /* __RTSP_STREAMER_H__ */
