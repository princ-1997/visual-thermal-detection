/**
 * 热红外 RTSP 推流模块
 *
 * 参考 reference/src/appSource.cpp 的 appsrc + GStreamer RTSP Server 模式，
 * 将热红外 UVC 相机的帧数据通过 RTSP 实时推流。
 *
 * 核心思路：
 *   1. RTSP 客户端连接时，media-configure 回调触发，启动 IR 视频流
 *   2. GStreamer 的 need-data 回调中，调用 ir_video_frame_get 获取一帧
 *   3. 将图像部分拷贝到 GstBuffer 推入 appsrc pipeline
 *   4. 客户端全部断开时，停止 IR 视频流
 */

#include "rtsp_streamer.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ======================== IR 视频流启停 ======================== */

/**
 * 启动 IR 视频流（根据 is_v4l2 区分 V4L2 和 UVC 路径）
 */
static int start_ir_stream(RtspStreamerCtx_t* ctx)
{
    int ret;

    if (ctx->is_v4l2) {
        /* V4L2 路径 */
        CamStreamParams_t params;
        memset(&params, 0, sizeof(CamStreamParams_t));
        params.width  = ctx->uvc_width;
        params.height = ctx->uvc_height;

        ret = ctx->video_handle->ir_video_start_stream(ctx->driver_handle, &params);
        printf("[RTSP] start_ir_stream (V4L2): ret=%d (%dx%d, fps=%d)\n",
               ret, ctx->uvc_width, ctx->uvc_height, ctx->uvc_fps);
    } else {
        /* UVC 路径 */
        IruvcCamStreamParams_t params;
        memset(&params, 0, sizeof(IruvcCamStreamParams_t));

        params.usr_callback.iruvc_handle = (IruvcHandle_t*)ctx->driver_handle;
        params.usr_callback.usr_func = NULL;
        params.usr_callback.usr_param = NULL;
        params.camera_param.width  = ctx->uvc_width;
        params.camera_param.height = ctx->uvc_height;
        params.camera_param.frame_size = (unsigned int)(ctx->uvc_width * ctx->uvc_height * ctx->frame_size_ratio);
        params.camera_param.fps = ctx->uvc_fps;
        params.camera_param.timeout_ms_delay = 2000;

        char data_type[10] = {0};
        if (ctx->frame_fmt == IR_FMT_YUYV_IMAGE ||
            ctx->frame_fmt == IR_FMT_YUYV_AND_TEMP ||
            ctx->frame_fmt == IR_FMT_UYVY_IMAGE) {
            strcpy(data_type, "YUYV");
        } else {
            strcpy(data_type, "NV12");
        }
        params.camera_param.format = data_type;

        ret = ctx->video_handle->ir_video_start_stream(ctx->driver_handle, &params);
        printf("[RTSP] start_ir_stream (UVC): ret=%d (%dx%d, fps=%d, fmt=%s)\n",
               ret, ctx->uvc_width, ctx->uvc_height, ctx->uvc_fps, data_type);
    }

    return ret;
}

/**
 * 停止 IR 视频流
 */
static void stop_ir_stream(RtspStreamerCtx_t* ctx)
{
    if (ctx->is_v4l2) {
        CamStreamParams_t params;
        memset(&params, 0, sizeof(CamStreamParams_t));
        ctx->video_handle->ir_video_stop_stream(ctx->driver_handle, &params);
    } else {
        IruvcStreamStopParams_t stop_params;
        memset(&stop_params, 0, sizeof(IruvcStreamStopParams_t));
        stop_params.stop_mode = CLOSE_CAM_SIDE_PREVIEW;
        ctx->video_handle->ir_video_stop_stream(ctx->driver_handle, &stop_params);
    }
    printf("[RTSP] stop_ir_stream done\n");
}

/* ======================== GStreamer 回调 ======================== */

/**
 * appsrc 的 need-data 回调
 * 每当 pipeline 需要数据时调用，从 IR 相机获取一帧并推送
 * （与 reference 中 mipicamera_getframe 在 need_data 中调用的模式一致）
 */
static void on_need_data(GstElement* appsrc, guint unused, gpointer user_data)
{
    RtspStreamerCtx_t* ctx = (RtspStreamerCtx_t*)user_data;
    if (!ctx || !ctx->stream_started) {
        return;
    }

    /* 从 IR 相机获取一帧原始数据 */
    ctx->video_handle->ir_video_frame_get(
        ctx->driver_handle, NULL,
        ctx->raw_buffer, ctx->raw_buffer_size);

    /* 分配 GstBuffer，只拷贝图像部分 */
    GstBuffer* buffer = gst_buffer_new_allocate(NULL, ctx->image_size, NULL);
    GstMapInfo map;
    if (gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
        memcpy(map.data, ctx->raw_buffer, ctx->image_size);
        gst_buffer_unmap(buffer, &map);
    }

    /* 设置时间戳 */
    GST_BUFFER_PTS(buffer) = ctx->timestamp;
    GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale_int(1, GST_SECOND, ctx->framerate);
    ctx->timestamp += GST_BUFFER_DURATION(buffer);

    /* 推送到 pipeline */
    GstFlowReturn ret;
    g_signal_emit_by_name(appsrc, "push-buffer", buffer, &ret);
    gst_buffer_unref(buffer);
}

/**
 * 流初始化 - 当 RTSP 客户端连接时调用
 */
static void stream_init(gpointer user_data)
{
    RtspStreamerCtx_t* ctx = (RtspStreamerCtx_t*)user_data;
    if (!ctx || ctx->stream_started) {
        return;
    }

    start_ir_stream(ctx);
    ctx->stream_started = TRUE;
    ctx->timestamp = 0;
    printf("[RTSP] stream initialized (client connected)\n");
}

/**
 * 流反初始化 - 当所有 RTSP 客户端断开时调用
 * 作为 g_object_set_data_full 的 GDestroyNotify 回调
 */
static void stream_uninit(gpointer user_data)
{
    RtspStreamerCtx_t* ctx = (RtspStreamerCtx_t*)user_data;
    if (!ctx || !ctx->stream_started) {
        return;
    }

    stop_ir_stream(ctx);
    ctx->stream_started = FALSE;
    printf("[RTSP] stream released (all clients disconnected)\n");
}

/**
 * media-configure 回调
 * 当新的 RTSP media 被创建时触发（参考 reference/src/appSource.cpp）
 */
static void on_media_configure(GstRTSPMediaFactory* factory,
                               GstRTSPMedia* media,
                               gpointer user_data)
{
    RtspStreamerCtx_t* ctx = (RtspStreamerCtx_t*)user_data;
    if (!ctx) {
        return;
    }

    /* 获取 pipeline 中的 appsrc 元素 */
    GstElement* element = gst_rtsp_media_get_element(media);
    GstElement* appsrc = gst_bin_get_by_name_recurse_up(GST_BIN(element), "videosrc");

    /* 配置 appsrc: 时间模式 */
    gst_util_set_object_arg(G_OBJECT(appsrc), "format", "time");

    /* 配置视频 caps */
    GstCaps* caps = gst_caps_new_simple("video/x-raw",
        "format",    G_TYPE_STRING,     ctx->gst_format,
        "width",     G_TYPE_INT,        ctx->width,
        "height",    G_TYPE_INT,        ctx->height,
        "framerate", GST_TYPE_FRACTION, ctx->framerate, 1,
        NULL);
    if (caps) {
        g_object_set(G_OBJECT(appsrc), "caps", caps, NULL);
        gst_caps_unref(caps);
    }
    g_object_set(G_OBJECT(appsrc), "is-live", TRUE, "block", TRUE, NULL);

    /* 初始化 IR 视频流 */
    stream_init(user_data);

    /* 注册清理回调：media 销毁时停止流 */
    g_object_set_data_full(G_OBJECT(media), "thermal-rtsp-ctx",
                           user_data, (GDestroyNotify)stream_uninit);

    /* 注册数据回调 */
    g_signal_connect(appsrc, "need-data", (GCallback)on_need_data, user_data);

    gst_object_unref(appsrc);
    gst_object_unref(element);
}

/* ======================== 公开接口 ======================== */

int rtsp_streamer_init(RtspStreamerCtx_t* ctx,
                       IrVideoHandle_t* video_handle,
                       void* driver_handle,
                       IrFrameFmt_t frame_fmt,
                       int img_w, int img_h,
                       uint32_t raw_size, uint32_t image_size,
                       int stream_w, int stream_h, int stream_fps,
                       float frame_ratio, int is_v4l2)
{
    memset(ctx, 0, sizeof(RtspStreamerCtx_t));

    ctx->video_handle  = video_handle;
    ctx->driver_handle = driver_handle;
    ctx->frame_fmt     = frame_fmt;
    ctx->is_v4l2       = is_v4l2 ? TRUE : FALSE;

    /* 确定 GStreamer 格式字符串 */
    if (frame_fmt == IR_FMT_NV12_IMAGE || frame_fmt == IR_FMT_NV12_AND_TEMP) {
        ctx->gst_format = "NV12";
    } else {
        ctx->gst_format = "YUY2";
    }

    ctx->width     = img_w;
    ctx->height    = img_h;
    ctx->framerate = stream_fps;
    ctx->image_size     = image_size;
    ctx->raw_buffer_size = raw_size;

    ctx->uvc_width  = stream_w;
    ctx->uvc_height = stream_h;
    ctx->uvc_fps    = stream_fps;
    ctx->frame_size_ratio = frame_ratio;

    /* 分配原始帧缓冲 */
    ctx->raw_buffer = (uint8_t*)malloc(raw_size);
    if (!ctx->raw_buffer) {
        printf("[RTSP] ERROR: failed to allocate raw buffer (%u bytes)\n", raw_size);
        return -1;
    }

    ctx->timestamp = 0;
    ctx->stream_started = FALSE;

    printf("[RTSP] init ok: image=%dx%d, fmt=%s, fps=%d, image_size=%u, raw_size=%u\n",
           img_w, img_h, ctx->gst_format, ctx->uvc_fps, image_size, raw_size);
    return 0;
}

int rtsp_streamer_run(RtspStreamerCtx_t* ctx, const char* port, const char* url_path, GMainLoop* loop)
{
    GstRTSPServer* server = gst_rtsp_server_new();
    g_object_set(server, "service", port, NULL);

    GstRTSPMountPoints* mounts = gst_rtsp_server_get_mount_points(server);
    GstRTSPMediaFactory* factory = gst_rtsp_media_factory_new();

    /*
     * 构建 GStreamer pipeline:
     *   NV12:  appsrc → mpph264enc → rtph264pay (RK 硬件编码器直接接受 NV12)
     *   YUY2:  appsrc → videoconvert → mpph264enc → rtph264pay
     */
    gchar* pipeline;
    if (strcmp(ctx->gst_format, "NV12") == 0) {
        pipeline = g_strdup("appsrc name=videosrc ! mpph264enc ! rtph264pay name=pay0 pt=96");
    } else {
        pipeline = g_strdup("appsrc name=videosrc ! videoconvert ! mpph264enc ! rtph264pay name=pay0 pt=96");
    }
    printf("[RTSP] pipeline: %s\n", pipeline);

    gst_rtsp_media_factory_set_launch(factory, pipeline);
    g_free(pipeline);

    /* 共享模式：多个客户端共享同一路流 */
    gst_rtsp_media_factory_set_shared(factory, TRUE);

    /* 注册 media-configure 回调 */
    g_signal_connect(factory, "media-configure", (GCallback)on_media_configure, ctx);

    gst_rtsp_mount_points_add_factory(mounts, url_path, factory);
    g_object_unref(mounts);

    gst_rtsp_server_attach(server, NULL);

    printf("=============================================\n");
    printf("[RTSP] Thermal IR RTSP Server Ready!\n");
    printf("[RTSP] URL: rtsp://<board_ip>:%s%s\n", port, url_path);
    printf("=============================================\n");

    g_main_loop_run(loop);

    return 0;
}

void rtsp_streamer_cleanup(RtspStreamerCtx_t* ctx)
{
    if (!ctx) return;

    if (ctx->stream_started) {
        stop_ir_stream(ctx);
        ctx->stream_started = FALSE;
    }

    if (ctx->raw_buffer) {
        free(ctx->raw_buffer);
        ctx->raw_buffer = NULL;
    }
}
