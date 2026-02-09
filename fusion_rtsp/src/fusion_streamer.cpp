/**
 * 热成像与可见光配准融合 RTSP 推流
 *
 * 配准：warpPerspective(thermal, H, visible_size)，H 矩阵可手动填写
 * 融合：Alpha 叠加 / 边缘叠加（默认）
 */
#include "fusion_streamer.h"
#include "camera.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <opencv2/opencv.hpp>
using namespace cv;

/* ========== 配准矩阵 H (3x3) - 请手动填写 ==========
 * 将热成像坐标系映射到可见光坐标系
 * 可通过标定工具获取，或根据特征点计算
 * 单位矩阵表示无变换（仅用于测试）
 */
static double g_homography_H[3][3] = {
    { 1.81519250e+00, -6.03496110e-01,  8.52525363e+02},
    { 5.28769420e-01,  5.05579479e-01,  5.18922777e+02},
    { 8.80582379e-04, -6.93669550e-04,  1.00000000e+00}
};

/* ======================== IR 视频流启停 ======================== */

static int start_ir_stream(FusionStreamerCtx_t* ctx)
{
    int ret;
    int uvc_w = ctx->thermal_stream_w;
    int uvc_h = ctx->thermal_stream_h;

    if (ctx->is_v4l2) {
        CamStreamParams_t params;
        memset(&params, 0, sizeof(CamStreamParams_t));
        params.width  = uvc_w;
        params.height = uvc_h;
        ret = ctx->ir_video_handle->ir_video_start_stream(ctx->ir_driver_handle, &params);
    } else {
        IruvcCamStreamParams_t params;
        memset(&params, 0, sizeof(IruvcCamStreamParams_t));
        params.usr_callback.iruvc_handle = (IruvcHandle_t*)ctx->ir_driver_handle;
        params.usr_callback.usr_func = NULL;
        params.usr_callback.usr_param = NULL;
        params.camera_param.width  = uvc_w;
        params.camera_param.height = uvc_h;
        params.camera_param.frame_size = (unsigned int)(uvc_w * uvc_h * ctx->thermal_frame_ratio);
        params.camera_param.fps = ctx->framerate;
        params.camera_param.timeout_ms_delay = 2000;
        char data_type[10] = {0};
        if (ctx->thermal_fmt == IR_FMT_YUYV_IMAGE || ctx->thermal_fmt == IR_FMT_YUYV_AND_TEMP || ctx->thermal_fmt == IR_FMT_UYVY_IMAGE)
            strcpy(data_type, "YUYV");
        else
            strcpy(data_type, "NV12");
        params.camera_param.format = data_type;
        ret = ctx->ir_video_handle->ir_video_start_stream(ctx->ir_driver_handle, &params);
    }
    printf("[Fusion] start_ir_stream: ret=%d\n", ret);
    return ret;
}

static void stop_ir_stream(FusionStreamerCtx_t* ctx)
{
    if (ctx->is_v4l2) {
        CamStreamParams_t params;
        memset(&params, 0, sizeof(CamStreamParams_t));
        ctx->ir_video_handle->ir_video_stop_stream(ctx->ir_driver_handle, &params);
    } else {
        IruvcStreamStopParams_t stop_params;
        memset(&stop_params, 0, sizeof(IruvcStreamStopParams_t));
        stop_params.stop_mode = CLOSE_CAM_SIDE_PREVIEW;
        ctx->ir_video_handle->ir_video_stop_stream(ctx->ir_driver_handle, &stop_params);
    }
    printf("[Fusion] stop_ir_stream done\n");
}

/* ======================== 格式转换 ======================== */

static Mat nv12_to_bgr(const uint8_t* nv12, int w, int h)
{
Mat yuv(h * 3 / 2, w, CV_8UC1, (void*)nv12);
Mat bgr;
cvtColor(yuv, bgr, COLOR_YUV2BGR_NV12);
    return bgr;
}

static void bgr_to_nv12(const Mat& bgr, uint8_t* nv12)
{
Mat yuv;
cvtColor(bgr, yuv, COLOR_BGR2YUV_I420);
    int ySize = bgr.cols * bgr.rows;
    int uvSize = ySize / 4;
    memcpy(nv12, yuv.data, ySize);
    uint8_t* uPlane = yuv.data + ySize;
    uint8_t* vPlane = uPlane + uvSize;
    uint8_t* uvDst = nv12 + ySize;
    for (int i = 0; i < uvSize; i++) {
        uvDst[i * 2]     = uPlane[i];
        uvDst[i * 2 + 1] = vPlane[i];
    }
}

static Mat yuy2_to_bgr(const uint8_t* yuy2, int w, int h)
{
Mat yuy2Mat(h, w, CV_8UC2, (void*)yuy2);
Mat bgr;
cvtColor(yuy2Mat, bgr, COLOR_YUV2BGR_YUY2);
    return bgr;
}

static Mat thermal_to_bgr(FusionStreamerCtx_t* ctx)
{
    const uint8_t* raw = ctx->thermal_raw_buffer;
    int w = ctx->thermal_width;
    int h = ctx->thermal_height;
    if (ctx->thermal_fmt == IR_FMT_NV12_IMAGE || ctx->thermal_fmt == IR_FMT_NV12_AND_TEMP)
        return nv12_to_bgr(raw, w, h);
    else
        return yuy2_to_bgr(raw, w, h);
}

/* ======================== Snap：串行拍照并保存 ======================== */

#define SNAP_VISIBLE_WARMUP 5   /* 可见光预热帧数 */
#define SNAP_THERMAL_WARMUP 100  /* 热成像预热帧数 */

int fusion_streamer_snap(FusionStreamerCtx_t* ctx, const char* visible_path, const char* thermal_path)
{
    if (!ctx || !visible_path || !thermal_path) return -1;

    /* 1. 初始化可见光相机 */
    if (mipicamera_init(ctx->visible_cam_index, ctx->visible_width, ctx->visible_height, 0) != 0) {
        printf("[Snap] mipicamera_init failed\n");
        return -1;
    }
    mipicamera_set_format(ctx->visible_cam_index, RK_FORMAT_YCbCr_420_SP);

    /* 2. 丢弃预热帧后拍摄可见光 */
    for (int i = 0; i < SNAP_VISIBLE_WARMUP; i++) {
        if (mipicamera_getframe(ctx->visible_cam_index, (char*)ctx->visible_buffer) != 0) {
            printf("[Snap] mipicamera_getframe failed\n");
            mipicamera_exit(ctx->visible_cam_index);
            return -1;
        }
    }
    Mat visibleBGR = nv12_to_bgr(ctx->visible_buffer, ctx->visible_width, ctx->visible_height);
    if (!imwrite(visible_path, visibleBGR)) {
        printf("[Snap] imwrite visible failed: %s\n", visible_path);
        mipicamera_exit(ctx->visible_cam_index);
        return -1;
    }
    printf("[Snap] visible saved: %s\n", visible_path);
    mipicamera_exit(ctx->visible_cam_index);

    /* 3. 启动热成像流 */
    if (start_ir_stream(ctx) != 0) {
        printf("[Snap] start_ir_stream failed\n");
        return -1;
    }

    /* 4. 丢弃预热帧后拍摄热成像（首帧多为黑） */
    for (int i = 0; i < SNAP_THERMAL_WARMUP; i++) {
        ctx->ir_video_handle->ir_video_frame_get(ctx->ir_driver_handle, NULL,
                                                 ctx->thermal_raw_buffer, ctx->thermal_raw_size);
    }
    Mat thermalBGR = thermal_to_bgr(ctx);
    if (!imwrite(thermal_path, thermalBGR)) {
        printf("[Snap] imwrite thermal failed: %s\n", thermal_path);
        stop_ir_stream(ctx);
        return -1;
    }
    printf("[Snap] thermal saved: %s\n", thermal_path);
    stop_ir_stream(ctx);

    return 0;
}

/* ======================== 融合 ======================== */

static void fuse_edge_overlay(const Mat& visible, const Mat& thermalWarped,
Mat& out, int cannyLow, int cannyHigh)
{
    visible.copyTo(out);
Mat thermalGray;
cvtColor(thermalWarped, thermalGray, COLOR_BGR2GRAY);
Mat edges;
Canny(thermalGray, edges, cannyLow, cannyHigh);
    /* 红色边缘叠加 */
    out.setTo(Scalar(0, 0, 255), edges);
}

/* ======================== GStreamer 回调 ======================== */

static void stream_init(gpointer user_data);
static void stream_uninit(gpointer user_data);

static void on_need_data(GstElement* appsrc, guint unused, gpointer user_data)
{
    FusionStreamerCtx_t* ctx = (FusionStreamerCtx_t*)user_data;
    if (!ctx || !ctx->stream_started) return;

    /* 1. 获取双源帧 */
    ctx->ir_video_handle->ir_video_frame_get(ctx->ir_driver_handle, NULL,
                                             ctx->thermal_raw_buffer, ctx->thermal_raw_size);

    if (mipicamera_getframe(ctx->visible_cam_index, (char*)ctx->visible_buffer) != 0) {
        return;
    }

    /* 2. 转 BGR */
Mat thermalBGR = thermal_to_bgr(ctx);
Mat visibleBGR = nv12_to_bgr(ctx->visible_buffer, ctx->visible_width, ctx->visible_height);

    /* 3. 配准：warpPerspective(thermal -> visible 坐标系) */
Mat H = (Mat_<double>(3, 3) <<
        g_homography_H[0][0], g_homography_H[0][1], g_homography_H[0][2],
        g_homography_H[1][0], g_homography_H[1][1], g_homography_H[1][2],
        g_homography_H[2][0], g_homography_H[2][1], g_homography_H[2][2]);
Mat thermalWarped;
warpPerspective(thermalBGR, thermalWarped, H, Size(ctx->visible_width, ctx->visible_height));

    /* 4. 融合 */
Mat fused;
    if (ctx->fusion_params.mode == FUSION_MODE_ALPHA_BLEND) {
        float a = ctx->fusion_params.alpha;
        if (a < 0) a = 0;
        if (a > 1) a = 1;
addWeighted(visibleBGR, 1.0f - a, thermalWarped, a, 0, fused);
    } else {
        int cl = ctx->fusion_params.canny_low  > 0 ? ctx->fusion_params.canny_low  : 50;
        int ch = ctx->fusion_params.canny_high > 0 ? ctx->fusion_params.canny_high : 150;
        fuse_edge_overlay(visibleBGR, thermalWarped, fused, cl, ch);
    }

    /* 5. BGR -> NV12 -> GstBuffer */
    guint nv12Size = ctx->out_width * ctx->out_height * 3 / 2;
    GstBuffer* buffer = gst_buffer_new_allocate(NULL, nv12Size, NULL);
    GstMapInfo map;
    if (gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
        bgr_to_nv12(fused, map.data);
        gst_buffer_unmap(buffer, &map);
    }

    GST_BUFFER_PTS(buffer) = ctx->timestamp;
    GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale_int(1, GST_SECOND, ctx->framerate);
    ctx->timestamp += GST_BUFFER_DURATION(buffer);

    GstFlowReturn ret;
    g_signal_emit_by_name(appsrc, "push-buffer", buffer, &ret);
    gst_buffer_unref(buffer);
}

static void stream_init(gpointer user_data)
{
    FusionStreamerCtx_t* ctx = (FusionStreamerCtx_t*)user_data;
    if (!ctx || ctx->stream_started) return;
    if (mipicamera_init(ctx->visible_cam_index, ctx->visible_width, ctx->visible_height, 0) != 0) {
        printf("[Fusion] mipicamera_init failed\n");
        return;
    }
    mipicamera_set_format(ctx->visible_cam_index, RK_FORMAT_YCbCr_420_SP);
    start_ir_stream(ctx);
    ctx->stream_started = TRUE;
    ctx->timestamp = 0;
    printf("[Fusion] stream initialized\n");
}

static void stream_uninit(gpointer user_data)
{
    FusionStreamerCtx_t* ctx = (FusionStreamerCtx_t*)user_data;
    if (!ctx || !ctx->stream_started) return;
    stop_ir_stream(ctx);
    mipicamera_exit(ctx->visible_cam_index);
    ctx->stream_started = FALSE;
    printf("[Fusion] stream released\n");
}

static void on_media_configure(GstRTSPMediaFactory* factory, GstRTSPMedia* media, gpointer user_data)
{
    FusionStreamerCtx_t* ctx = (FusionStreamerCtx_t*)user_data;
    if (!ctx) return;

    GstElement* element = gst_rtsp_media_get_element(media);
    GstElement* appsrc = gst_bin_get_by_name_recurse_up(GST_BIN(element), "videosrc");
    if (!appsrc) {
        printf("[Fusion] ERROR: appsrc 'videosrc' not found\n");
        gst_object_unref(element);
        return;
    }

    gst_util_set_object_arg(G_OBJECT(appsrc), "format", "time");
    GstCaps* caps = gst_caps_new_simple("video/x-raw",
        "format", G_TYPE_STRING, "NV12",
        "width",  G_TYPE_INT, ctx->out_width,
        "height", G_TYPE_INT, ctx->out_height,
        "framerate", GST_TYPE_FRACTION, ctx->framerate, 1,
        NULL);
    if (caps) {
        g_object_set(G_OBJECT(appsrc), "caps", caps, NULL);
        gst_caps_unref(caps);
    }
    g_object_set(G_OBJECT(appsrc), "is-live", TRUE, "block", TRUE, NULL);

    stream_init(user_data);
    g_object_set_data_full(G_OBJECT(media), "fusion-ctx", user_data, (GDestroyNotify)stream_uninit);
    g_signal_connect(appsrc, "need-data", (GCallback)on_need_data, user_data);

    gst_object_unref(appsrc);
    gst_object_unref(element);
}

/* ======================== 公开接口 ======================== */

int fusion_streamer_init(FusionStreamerCtx_t* ctx,
                         IrVideoHandle_t* ir_video_handle,
                         void* ir_driver_handle,
                         IrFrameFmt_t thermal_fmt,
                         int thermal_w, int thermal_h,
                         int thermal_stream_w, int thermal_stream_h,
                         uint32_t thermal_raw_size, uint32_t thermal_image_size,
                         float thermal_ratio,
                         int visible_cam_idx, int visible_w, int visible_h,
                         int fps, gboolean is_v4l2)
{
    memset(ctx, 0, sizeof(FusionStreamerCtx_t));

    ctx->ir_video_handle   = ir_video_handle;
    ctx->ir_driver_handle  = ir_driver_handle;
    ctx->thermal_fmt       = thermal_fmt;
    ctx->thermal_width     = thermal_w;
    ctx->thermal_height    = thermal_h;
    ctx->thermal_stream_w  = thermal_stream_w;
    ctx->thermal_stream_h  = thermal_stream_h;
    ctx->thermal_raw_size  = thermal_raw_size;
    ctx->thermal_image_size = thermal_image_size;
    ctx->thermal_frame_ratio = thermal_ratio;
    ctx->visible_cam_index = visible_cam_idx;
    ctx->visible_width     = visible_w;
    ctx->visible_height    = visible_h;
    ctx->out_width         = visible_w;
    ctx->out_height        = visible_h;
    ctx->framerate         = fps;
    ctx->is_v4l2           = is_v4l2;

    ctx->thermal_raw_buffer = (uint8_t*)malloc(thermal_raw_size);
    if (!ctx->thermal_raw_buffer) {
        printf("[Fusion] ERROR: thermal raw buffer alloc failed\n");
        return -1;
    }

    ctx->visible_buffer = (uint8_t*)malloc((size_t)visible_w * visible_h * 3 / 2);
    if (!ctx->visible_buffer) {
        printf("[Fusion] ERROR: visible buffer alloc failed\n");
        free(ctx->thermal_raw_buffer);
        return -1;
    }

    ctx->fusion_params.mode = FUSION_MODE_EDGE_OVERLAY;
    ctx->fusion_params.alpha = 0.5f;
    ctx->fusion_params.canny_low = 50;
    ctx->fusion_params.canny_high = 150;

    printf("[Fusion] init ok: thermal=%dx%d, visible=%dx%d, fps=%d\n",
           thermal_w, thermal_h, visible_w, visible_h, fps);
    return 0;
}

void fusion_streamer_set_params(FusionStreamerCtx_t* ctx, const FusionParams_t* params)
{
    if (ctx && params) {
        memcpy(&ctx->fusion_params, params, sizeof(FusionParams_t));
    }
}

int fusion_streamer_run(FusionStreamerCtx_t* ctx, const char* port, const char* url_path, GMainLoop* loop)
{
    GstRTSPServer* server = gst_rtsp_server_new();
    g_object_set(server, "service", port, NULL);

    GstRTSPMountPoints* mounts = gst_rtsp_server_get_mount_points(server);
    GstRTSPMediaFactory* factory = gst_rtsp_media_factory_new();

    gchar* pipeline = g_strdup("appsrc name=videosrc ! mpph264enc ! rtph264pay name=pay0 pt=96");
    gst_rtsp_media_factory_set_launch(factory, pipeline);
    g_free(pipeline);

    gst_rtsp_media_factory_set_shared(factory, TRUE);
    g_signal_connect(factory, "media-configure", (GCallback)on_media_configure, ctx);

    gst_rtsp_mount_points_add_factory(mounts, url_path, factory);
    g_object_unref(mounts);

    gst_rtsp_server_attach(server, NULL);

    printf("=============================================\n");
    printf("[Fusion] Thermal+Visible RTSP Server Ready!\n");
    printf("[Fusion] URL: rtsp://<board_ip>:%s%s\n", port, url_path);
    printf("=============================================\n");

    g_main_loop_run(loop);
    return 0;
}

void fusion_streamer_cleanup(FusionStreamerCtx_t* ctx)
{
    if (!ctx) return;
    if (ctx->stream_started) {
        stop_ir_stream(ctx);
        mipicamera_exit(ctx->visible_cam_index);
        ctx->stream_started = FALSE;
    }
    if (ctx->thermal_raw_buffer) {
        free(ctx->thermal_raw_buffer);
        ctx->thermal_raw_buffer = NULL;
    }
    if (ctx->visible_buffer) {
        free(ctx->visible_buffer);
        ctx->visible_buffer = NULL;
    }
}
