/**
 * 热成像与可见光配准融合 RTSP 推流
 *
 * 配准：warpPerspective(thermal, H, visible_size)，H 矩阵可手动填写
 * 融合：Alpha 叠加 / 边缘叠加（默认）
 */
#include "fusion_streamer.h"
#include "camera.h"
#include "person_detector.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <opencv2/opencv.hpp>
using namespace cv;

/* 检测结果缓存（用于跨帧保持绘制） */
static uint32_t g_lastDetectSeq = 0;
static DetectResult_t g_lastDetectResult = {0};
static int g_lastDetectCount = 0;
/* 检测提交用 NV12 缓冲（visibleCrop BGR 转 NV12） */
static uint8_t* g_detectNv12Buf = NULL;
static size_t g_detectNv12Size = 0;

/* BGR 上绘制检测框 */
static void drawDetectionBoxesOnBGR(Mat& bgr, const DetectResult_t* result)
{
    if (!result || result->count == 0) return;
    static const Scalar colors[] = {
        Scalar(0, 0, 255), Scalar(255, 0, 0), Scalar(0, 255, 0), Scalar(0, 255, 255),
        Scalar(255, 0, 255), Scalar(255, 255, 0), Scalar(0, 128, 255), Scalar(255, 128, 0)
    };
    const int nColors = sizeof(colors) / sizeof(colors[0]);
    for (int i = 0; i < result->count; i++) {
        const DetectBox_t* box = &result->boxes[i];
        Scalar color = colors[i % nColors];
        rectangle(bgr, Point(box->left, box->top), Point(box->right, box->bottom), color, 2);
        char label[32];
        snprintf(label, sizeof(label), "person %.1f%%", box->confidence * 100);
        int baseline = 0;
        double fontScale = 0.6;
        int thickness = 2;
        int textY = box->top - 5;
        if (textY < 20) textY = box->top + 20;
        putText(bgr, label, Point(box->left, textY), FONT_HERSHEY_SIMPLEX, fontScale, color, thickness);
    }
}

/* ========== 配准矩阵 H (3x3) - 请手动填写 ==========
 * 将热成像坐标系映射到可见光坐标系
 * 可通过标定工具获取，或根据特征点计算
 * H 标定时的可见光分辨率（若与当前不同则自动缩放）
 */
static double g_homography_H[3][3] = {
    { 5.91182905e-01,  5.71124538e-01,  8.82142738e+02},
    {-1.75684254e-01,  1.23285882e+00,  5.41628838e+02},
    {-2.69454378e-04,  5.66310896e-04,  1.00000000e+00}
};
static const int H_CALIB_W = 1920;
static const int H_CALIB_H = 1080;

/* 计算自适应裁剪区域：根据热红外在可见光中的位置计算裁剪框
 * 裁剪框尺寸为热红外的 sqrt(2) 倍，以热红外中心为中心 */
static Rect calculate_adaptive_crop_region(int thermal_w, int thermal_h, 
                                           int visible_w, int visible_h,
                                           const Mat& H)
{
    /* 计算热红外四角在可见光坐标系中的位置 */
    std::vector<Point2f> thermalCorners = {
        Point2f(0, 0),
        Point2f(thermal_w - 1, 0),
        Point2f(thermal_w - 1, thermal_h - 1),
        Point2f(0, thermal_h - 1)
    };
    std::vector<Point2f> thermalCornersWarped;
    perspectiveTransform(thermalCorners, thermalCornersWarped, H);
    
    /* 计算包围框 */
    float minX = visible_w, maxX = 0, minY = visible_h, maxY = 0;
    for (const auto& pt : thermalCornersWarped) {
        if (pt.x < minX) minX = pt.x;
        if (pt.x > maxX) maxX = pt.x;
        if (pt.y < minY) minY = pt.y;
        if (pt.y > maxY) maxY = pt.y;
    }
    
    /* 热红外的最大宽高 */
    int thermal_w_in_visible = (int)(maxX - minX);
    int thermal_h_in_visible = (int)(maxY - minY);
    
    /* 以热红外中心为中心，向外扩展到 sqrt(2) 倍 */
    int thermal_center_x = (int)((minX + maxX) / 2);
    int thermal_center_y = (int)((minY + maxY) / 2);
    const float SQRT2 = 1.414213562f;
    int crop_w = (int)(thermal_w_in_visible * SQRT2);
    int crop_h = (int)(thermal_h_in_visible * SQRT2);
    
    /* 对齐到 16 以满足 RV1126 MPP H.264 编码器要求 */
    crop_w = (crop_w + 15) & ~15;
    crop_h = (crop_h + 15) & ~15;
    
    /* 确定裁剪起始点（以中心扩展） */
    int crop_x = thermal_center_x - crop_w / 2;
    int crop_y = thermal_center_y - crop_h / 2;
    
    /* 边界检查和调整 */
    if (crop_x < 0) crop_x = 0;
    if (crop_y < 0) crop_y = 0;
    if (crop_x + crop_w > visible_w) crop_x = visible_w - crop_w;
    if (crop_y + crop_h > visible_h) crop_y = visible_h - crop_h;
    if (crop_w > visible_w) crop_w = visible_w;
    if (crop_h > visible_h) crop_h = visible_h;
    
    return Rect(crop_x, crop_y, crop_w, crop_h);
}

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
    /* 1. 先进行0.6参数的alpha_blend */
    addWeighted(visible, 0.4f, thermalWarped, 0.6f, 0, out);
    
    /* 2. 再叠加热成像的边缘检测结果 */
    Mat thermalGray;
    cvtColor(thermalWarped, thermalGray, COLOR_BGR2GRAY);
    /* 高斯模糊：平滑梯度、抑制噪声，减少多重平行边缘重影 */
    Mat blurred;
    GaussianBlur(thermalGray, blurred, Size(5, 5), 1.4);
    Mat edges;
    Canny(blurred, edges, cannyLow, cannyHigh);
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

    /* 3. 配准：warpPerspective(thermal -> visible 坐标系)
     * 若当前可见光分辨率与标定不同，则缩放 H */
    double sx = (double)ctx->visible_width / H_CALIB_W;
    double sy = (double)ctx->visible_height / H_CALIB_H;
    Mat H = (Mat_<double>(3, 3) <<
        g_homography_H[0][0] * sx, g_homography_H[0][1] * sx, g_homography_H[0][2] * sx,
        g_homography_H[1][0] * sy, g_homography_H[1][1] * sy, g_homography_H[1][2] * sy,
        g_homography_H[2][0],      g_homography_H[2][1],      g_homography_H[2][2]);
    Mat thermalWarped;
    warpPerspective(thermalBGR, thermalWarped, H, Size(ctx->visible_width, ctx->visible_height));

    /* 4. 自适应裁剪可见光：使用预先计算好的裁剪区域（在init时已计算） */
    Rect roi = calculate_adaptive_crop_region(ctx->thermal_width, ctx->thermal_height,
                                              ctx->visible_width, ctx->visible_height, H);
    Mat visibleCrop = visibleBGR(roi);
    Mat thermalCrop = thermalWarped(roi);
    
    /* 计算热红外四角在裁剪后的局部坐标系中的位置（用于绘制边框） */
    std::vector<Point2f> thermalCorners = {
        Point2f(0, 0),
        Point2f(ctx->thermal_width - 1, 0),
        Point2f(ctx->thermal_width - 1, ctx->thermal_height - 1),
        Point2f(0, ctx->thermal_height - 1)
    };
    std::vector<Point2f> thermalCornersWarped;
    perspectiveTransform(thermalCorners, thermalCornersWarped, H);
    
    /* 4.5 人体检测：对 visibleCrop 异步提交，获取最新结果 */
    if (ctx->detect_config.enabled && ctx->detect_config.model_path) {
        size_t nv12Size = (size_t)ctx->out_width * ctx->out_height * 3 / 2;
        if (g_detectNv12Size < nv12Size) {
            free(g_detectNv12Buf);
            g_detectNv12Buf = (uint8_t*)malloc(nv12Size);
            g_detectNv12Size = g_detectNv12Buf ? nv12Size : 0;
        }
        if (g_detectNv12Buf) {
            bgr_to_nv12(visibleCrop, g_detectNv12Buf);
            personDetector_submitFrame(g_detectNv12Buf, ctx->out_width, ctx->out_height);
            DetectResult_t res;
            uint32_t seq = 0;
            int cnt = personDetector_getLastResultWithSeq(&res, &seq);
            if (seq != g_lastDetectSeq) {
                g_lastDetectSeq = seq;
                g_lastDetectCount = cnt;
                if (cnt > 0) memcpy(&g_lastDetectResult, &res, sizeof(DetectResult_t));
            }
        }
    }

    /* 5. 融合 */
    Mat fused;
    if (ctx->fusion_params.mode == FUSION_MODE_ALPHA_BLEND) {
        float a = ctx->fusion_params.alpha;
        if (a < 0) a = 0;
        if (a > 1) a = 1;
        addWeighted(visibleCrop, 1.0f - a, thermalCrop, a, 0, fused);
    } else {
        int cl = ctx->fusion_params.canny_low  > 0 ? ctx->fusion_params.canny_low  : 50;
        int ch = ctx->fusion_params.canny_high > 0 ? ctx->fusion_params.canny_high : 150;
        fuse_edge_overlay(visibleCrop, thermalCrop, fused, cl, ch);
    }
    
    /* 5.1 给热红外画面增加红色边框以突出显示 */
    std::vector<Point> thermalCornersLocal;
    for (const auto& pt : thermalCornersWarped) {
        /* 将可见光全局坐标转换为裁剪后的局部坐标 */
        int localX = (int)pt.x - roi.x;
        int localY = (int)pt.y - roi.y;
        thermalCornersLocal.push_back(Point(localX, localY));
    }
    /* 绘制热红外边框四条边 */
    for (size_t i = 0; i < thermalCornersLocal.size(); i++) {
        Point p1 = thermalCornersLocal[i];
        Point p2 = thermalCornersLocal[(i + 1) % thermalCornersLocal.size()];
        line(fused, p1, p2, Scalar(0, 0, 255), 3);
    }

    /* 5.5 在融合图上绘制检测框 */
    if (ctx->detect_config.enabled && g_lastDetectCount > 0) {
        drawDetectionBoxesOnBGR(fused, &g_lastDetectResult);
    }

    /* 6. BGR -> NV12 -> GstBuffer */
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
    /* 人体检测初始化 */
    if (ctx->detect_config.enabled && ctx->detect_config.model_path) {
        DetectorConfig_t dcfg;
        memset(&dcfg, 0, sizeof(dcfg));
        dcfg.modelPath = ctx->detect_config.model_path;
        dcfg.detectFps = ctx->detect_config.detect_fps > 0 ? ctx->detect_config.detect_fps : 3;
        dcfg.imageWidth = ctx->out_width;
        dcfg.imageHeight = ctx->out_height;
        dcfg.confidenceThreshold = ctx->detect_config.confidence_threshold > 0 ? ctx->detect_config.confidence_threshold : 0.4f;
        dcfg.callback = NULL;
        dcfg.callbackUserData = NULL;
        if (personDetector_init(&dcfg) == 0) {
            personDetector_setSourceFps(ctx->framerate);
        } else {
            printf("[Fusion] person detector init failed, detection disabled\n");
        }
    }
    printf("[Fusion] stream initialized\n");
}

static void stream_uninit(gpointer user_data)
{
    FusionStreamerCtx_t* ctx = (FusionStreamerCtx_t*)user_data;
    if (!ctx || !ctx->stream_started) return;
    if (ctx->detect_config.enabled) {
        personDetector_release();
    }
    free(g_detectNv12Buf);
    g_detectNv12Buf = NULL;
    g_detectNv12Size = 0;
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
    ctx->framerate         = fps;
    ctx->is_v4l2           = is_v4l2;
    
    /* 根据单应性矩阵H计算自适应裁剪区域，确定输出尺寸 */
    double sx = (double)visible_w / H_CALIB_W;
    double sy = (double)visible_h / H_CALIB_H;
    Mat H = (Mat_<double>(3, 3) <<
        g_homography_H[0][0] * sx, g_homography_H[0][1] * sx, g_homography_H[0][2] * sx,
        g_homography_H[1][0] * sy, g_homography_H[1][1] * sy, g_homography_H[1][2] * sy,
        g_homography_H[2][0],      g_homography_H[2][1],      g_homography_H[2][2]);
    Rect cropRegion = calculate_adaptive_crop_region(thermal_w, thermal_h, visible_w, visible_h, H);
    ctx->out_width  = cropRegion.width;
    ctx->out_height = cropRegion.height;

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

    ctx->detect_config.enabled = FALSE;
    ctx->detect_config.model_path = NULL;
    ctx->detect_config.detect_fps = 3;
    ctx->detect_config.confidence_threshold = 0.4f;

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

void fusion_streamer_set_detect_config(FusionStreamerCtx_t* ctx, const DetectConfig_t* config)
{
    if (ctx && config) {
        memcpy(&ctx->detect_config, config, sizeof(DetectConfig_t));
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
    if (ctx->detect_config.enabled) {
        personDetector_release();
    }
    free(g_detectNv12Buf);
    g_detectNv12Buf = NULL;
    g_detectNv12Size = 0;
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
