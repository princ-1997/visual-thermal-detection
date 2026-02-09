/**
 * 热成像与可见光配准融合 RTSP 推流模块
 *
 * 数据流：IR 热成像 + MIPI 可见光 → 配准(warpPerspective) → 融合 → appsrc → RTSP
 */
#ifndef __FUSION_STREAMER_H__
#define __FUSION_STREAMER_H__

#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
#include "libircam.h"
#include "libiruvc.h"
#include "libirv4l2.h"
#include "fusion_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 帧格式（与 IR SDK 一致） */
typedef enum {
    IR_FMT_YUYV_IMAGE = 0,
    IR_FMT_NV12_IMAGE,
    IR_FMT_NV12_AND_TEMP,
    IR_FMT_YUYV_AND_TEMP,
    IR_FMT_UYVY_IMAGE,
} IrFrameFmt_t;

/* 融合推流上下文 */
typedef struct {
    /* === 热成像源 === */
    IrVideoHandle_t*  ir_video_handle;
    void*             ir_driver_handle;
    uint8_t*          thermal_raw_buffer;
    uint32_t          thermal_raw_size;
    uint32_t          thermal_image_size;
    int               thermal_width;
    int               thermal_height;
    int               thermal_stream_w;   /* IR 流采集宽高（可能与 image 不同）*/
    int               thermal_stream_h;
    IrFrameFmt_t      thermal_fmt;
    float             thermal_frame_ratio;
    gboolean          is_v4l2;

    /* === 可见光源 === */
    int               visible_cam_index;   /* mipicamera 索引，如 23 */
    int               visible_width;
    int               visible_height;
    uint8_t*          visible_buffer;      /* NV12 缓冲 */

    /* ===  fusion 参数 === */
    FusionParams_t    fusion_params;

    /* === 人体检测 === */
    DetectConfig_t    detect_config;

    /* === 输出 === */
    int               out_width;
    int               out_height;
    int               framerate;
    GstClockTime      timestamp;
    gboolean          stream_started;
} FusionStreamerCtx_t;

/**
 * 初始化融合推流上下文
 * @param ctx              上下文
 * @param ir_video_handle  IR 视频句柄
 * @param ir_driver_handle IR 驱动句柄
 * @param thermal_fmt      热成像格式
 * @param thermal_w/h     热成像图像宽高
 * @param thermal_stream_w/h 热成像流采集宽高
 * @param thermal_raw_size 热成像原始帧大小
 * @param thermal_image_size 热成像纯图像大小
 * @param thermal_ratio    帧大小比率
 * @param visible_cam_idx  可见光 camera 索引
 * @param visible_w/h      可见光宽高
 * @param fps              帧率
 * @param is_v4l2          IR 是否 V4L2
 */
int fusion_streamer_init(FusionStreamerCtx_t* ctx,
                         IrVideoHandle_t* ir_video_handle,
                         void* ir_driver_handle,
                         IrFrameFmt_t thermal_fmt,
                         int thermal_w, int thermal_h,
                         int thermal_stream_w, int thermal_stream_h,
                         uint32_t thermal_raw_size, uint32_t thermal_image_size,
                         float thermal_ratio,
                         int visible_cam_idx, int visible_w, int visible_h,
                         int fps, gboolean is_v4l2);

/**
 * 设置融合参数
 */
void fusion_streamer_set_params(FusionStreamerCtx_t* ctx, const FusionParams_t* params);

/**
 * 设置人体检测配置（需在 stream_init 前调用）
 */
void fusion_streamer_set_detect_config(FusionStreamerCtx_t* ctx, const DetectConfig_t* config);

/**
 * Snap：串行拍照并保存可见光、热成像
 * @param visible_path 可见光保存路径（如 visible.jpg）
 * @param thermal_path 热成像保存路径（如 thermal.jpg）
 */
int fusion_streamer_snap(FusionStreamerCtx_t* ctx, const char* visible_path, const char* thermal_path);

/**
 * 启动 RTSP 服务器（阻塞）
 */
int fusion_streamer_run(FusionStreamerCtx_t* ctx, const char* port, const char* url_path, GMainLoop* loop);

/**
 * 释放资源
 */
void fusion_streamer_cleanup(FusionStreamerCtx_t* ctx);

#ifdef __cplusplus
}
#endif

#endif /* __FUSION_STREAMER_H__ */
