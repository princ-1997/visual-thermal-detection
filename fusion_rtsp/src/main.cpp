/**
 * 热成像与可见光配准融合 RTSP 主程序
 *
 * 用法：./fusion_rtsp <thermal_config> [options]
 *   例：./fusion_rtsp stream_RV1126.conf -p 8554 -u /fusion -v 23
 *   例：./fusion_rtsp stream_RV1126.conf -c fusion.conf
 *
 * Alpha 叠加模式：./fusion_rtsp stream_RV1126.conf -c fusion_alpha.conf -v 23
 *   fusion_alpha.conf 中设置 "mode":"alpha_blend", "alpha":0.5
 */
#include "fusion_streamer.h"
#include "fusion_config.h"
#include "config.h"
#include "data.h"
#include "cmd.h"
#include "libircam.h"
#include "libiruvc.h"
#include "libirv4l2.h"
#include "libircmd.h"
#include "libiri2c.h"
#include "libiruart.h"
#include "cJSON.h"
#include <getopt.h>
#include <signal.h>
#include <atomic>
#include <cstring>
#include <string>
#include <fstream>

std::atomic_bool isRUNNING(true);
int temp_measure_on = 0;
IrVideoHandle_t* ir_image_video_handle = NULL;
IrVideoHandle_t* ir_temp_video_handle  = NULL;

#define DEFAULT_RTSP_PORT    "8554"
#define DEFAULT_URL_PATH     "/fusion"
#define DEFAULT_VISIBLE_CAM  23
#define DEFAULT_VISIBLE_W    1920
#define DEFAULT_VISIBLE_H    1080
#define DEFAULT_FPS         10     /* 双摄像头帧率，降低卡顿 */

static GMainLoop* g_loop = NULL;

static void signal_handler(int sig)
{
    printf("\n[Main] signal %d, shutting down...\n", sig);
    isRUNNING = false;
    if (g_loop) g_main_loop_quit(g_loop);
}

static void print_usage(const char* prog)
{
    printf("Usage: %s <thermal_config> [options]\n", prog);
    printf("Options:\n");
    printf("  -p <port>     RTSP port (default: %s)\n", DEFAULT_RTSP_PORT);
    printf("  -u <path>     URL path (default: %s)\n", DEFAULT_URL_PATH);
    printf("  -v <index>    Visible camera index, e.g. 23 for /dev/video23 (default: %d)\n", DEFAULT_VISIBLE_CAM);
    printf("  -W <width>    Visible width (default: %d)\n", DEFAULT_VISIBLE_W);
    printf("  -H <height>   Visible height (default: %d)\n", DEFAULT_VISIBLE_H);
    printf("  -f <fps>      Frame rate for both cameras (default: %d)\n", DEFAULT_FPS);
    printf("  -c <file>     Fusion config (mode, alpha, canny thresholds)\n");
    printf("                mode: alpha_blend | edge_overlay, alpha: 0~1\n");
    printf("  -h            Show help\n");
    printf("\nAlpha 叠加示例: %s stream_RV1126.conf -c fusion_alpha.conf -v 23\n", prog);
    printf("拍照请使用:    fusion_snap stream_RV1126.conf -o ./snap\n", prog);
}

/* 从 fusion.conf 加载融合参数 */
static int load_fusion_params(const char* path, FusionParams_t* params)
{
    std::ifstream f(path);
    if (!f.good()) return -1;
    std::string str((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    f.close();

    cJSON* json = cJSON_ParseWithLength(str.c_str(), str.size());
    if (!json) return -1;

    cJSON* fusion = cJSON_GetObjectItem(json, "fusion");
    if (!fusion) {
        cJSON_Delete(json);
        return -1;
    }

    cJSON* mode = cJSON_GetObjectItem(fusion, "mode");
    if (mode && cJSON_IsString(mode)) {
        const char* s = cJSON_GetStringValue(mode);
        if (s && strcmp(s, "alpha_blend") == 0) {
            params->mode = FUSION_MODE_ALPHA_BLEND;
        } else {
            params->mode = FUSION_MODE_EDGE_OVERLAY;
        }
    }

    cJSON* alpha = cJSON_GetObjectItem(fusion, "alpha");
    if (alpha && cJSON_IsNumber(alpha)) {
        params->alpha = (float)cJSON_GetNumberValue(alpha);
        if (params->alpha < 0) params->alpha = 0;
        if (params->alpha > 1) params->alpha = 1;
    }

    cJSON* cl = cJSON_GetObjectItem(fusion, "canny_low");
    if (cl && cJSON_IsNumber(cl)) params->canny_low = (int)cJSON_GetNumberValue(cl);
    cJSON* ch = cJSON_GetObjectItem(fusion, "canny_high");
    if (ch && cJSON_IsNumber(ch)) params->canny_high = (int)cJSON_GetNumberValue(ch);

    cJSON_Delete(json);
    return 0;
}

int main(int argc, char* argv[])
{
    const char* thermal_config = NULL;
    const char* rtsp_port   = DEFAULT_RTSP_PORT;
    const char* url_path    = DEFAULT_URL_PATH;
    const char* fusion_conf = NULL;
    int visible_cam = DEFAULT_VISIBLE_CAM;
    int visible_w   = DEFAULT_VISIBLE_W;
    int visible_h   = DEFAULT_VISIBLE_H;
    int fps         = DEFAULT_FPS;
    int ret = 0;
    bool is_v4l2 = false;
    void* driver_handle = NULL;
    IrControlHandle_t* ir_control_handle = NULL;
    IrcmdHandle_t* ircmd_handle = NULL;
    FusionStreamerCtx_t fusion_ctx;
    memset(&fusion_ctx, 0, sizeof(fusion_ctx));

    if (argc < 2) {
        print_usage(argv[0]);
        return -1;
    }
    thermal_config = argv[1];

    int opt;
    optind = 2;
    while ((opt = getopt(argc, argv, "p:u:v:W:H:f:c:h")) != -1) {
        switch (opt) {
        case 'p': rtsp_port = optarg; break;
        case 'u': url_path  = optarg; break;
        case 'v': visible_cam = atoi(optarg); break;
        case 'W': visible_w = atoi(optarg); break;
        case 'H': visible_h = atoi(optarg); break;
        case 'f': fps = atoi(optarg); break;
        case 'c': fusion_conf = optarg; break;
        case 'h':
        default:
            print_usage(argv[0]);
            return 0;
        }
    }

    printf("========================================\n");
    printf("  Thermal+Visible Fusion RTSP Server\n");
    printf("  Thermal config: %s\n", thermal_config);
    printf("  Visible: /dev/video%d %dx%d @ %dfps\n", visible_cam, visible_w, visible_h, fps);
    printf("  Port: %s, URL: %s\n", rtsp_port, url_path);
    printf("========================================\n");

    signal(SIGINT,  signal_handler);
    signal(SIGTERM, signal_handler);

    /* === 1. 解析热成像配置 === */
    config config_obj;
    if (config_obj.parse_config(thermal_config) != 0) {
        printf("[Main] ERROR: parse thermal config failed\n");
        return -1;
    }

    single_config product_config;
    /* 使用第一个产品的配置，避免交互式选择 */
    std::string product_name = "WN2256";  /* 默认产品名，与 stream_RV1126.conf 一致，可通过 -P 覆盖 */
    if (!config_obj.get_config(product_config, product_name)) {
        printf("[Main] ERROR: get_config failed for product '%s'\n", product_name.c_str());
        return -1;
    }

    printf("[Main] IR SDK: ircmd=%s ircam=%s iruvc=%s\n",
           ircmd_version(), ircam_version(), iruvc_version());

    ircmd_log_register(IRCMD_LOG_ERROR, NULL, NULL);
    iruvc_log_register(IRUVC_LOG_ERROR, NULL, NULL);
    iruart_log_register(IRUART_LOG_ERROR, NULL, NULL);
    ircam_log_register(IRCAM_LOG_ERROR, NULL, NULL);

    /* === 2. 创建 IR 视频句柄 === */
    ir_video_handle_create(&ir_image_video_handle);
    is_v4l2 = (product_config.camera.image_channel_type == "dvp" ||
               product_config.camera.image_channel_type == "mipi");

    if (is_v4l2) {
        Irv4l2VideoHandle_t* v4l2_handle = irv4l2_handle_create(ir_image_video_handle);
        char* dev_node = (char*)product_config.camera.v4l2_config.image_stream.device_name.c_str();
        ret = irv4l2_camera_open(v4l2_handle, dev_node);
        if (ret != 0) {
            printf("[Main] ERROR: V4L2 camera open failed\n");
            goto cleanup;
        }
        CamDevParams_t cam_params;
        memset(&cam_params, 0, sizeof(CamDevParams_t));
        cam_params.width  = product_config.camera.v4l2_config.image_stream.dev_width;
        cam_params.height = product_config.camera.v4l2_config.image_stream.dev_height;
        cam_params.fps    = fps;   /* 可配置帧率 */
        cam_params.format = 0;
        ret = irv4l2_camera_init(v4l2_handle, &cam_params);
        if (ret != 0) {
            printf("[Main] ERROR: V4L2 init failed\n");
            goto cleanup;
        }
        driver_handle = v4l2_handle;
    } else {
        IruvcHandle_t* iruvc_handle = iruvc_camera_handle_create(ir_image_video_handle);
        IruvcDevParam_t dev_param;
        memset(&dev_param, 0, sizeof(IruvcDevParam_t));
        dev_param.pid = product_config.camera.uvc_stream_conf.dev_info.pid;
        dev_param.vid = product_config.camera.uvc_stream_conf.dev_info.vid;
        dev_param.same_idx = product_config.camera.uvc_stream_conf.dev_info.same_id;
        ret = ir_image_video_handle->ir_video_open(iruvc_handle, &dev_param);
        if (ret != 0) {
            printf("[Main] ERROR: UVC open failed\n");
            goto cleanup;
        }
        ret = ir_image_video_handle->ir_video_init(iruvc_handle, &dev_param);
        driver_handle = iruvc_handle;
    }

    /* === 3. 命令控制句柄（用于启动出图）=== */
    ir_control_handle_create(&ir_control_handle);
    if (product_config.control.is_i2c_control) {
        Iri2cHandle_t* i2c_handle = iri2c_handle_create(ir_control_handle);
        ret = iri2c_device_open(i2c_handle, product_config.control.i2c_param.dev_name.c_str());
        ircmd_handle = ircmd_create_handle(ir_control_handle);
    } else if (product_config.control.is_uart_control) {
        iruart_handle_create(ir_control_handle);
        ret = search_com(ir_control_handle, &ircmd_handle, product_config.control.uart_param.com_index);
        if (ret != 0) {
            printf("[Main] ERROR: search_com failed\n");
            goto cleanup;
        }
    } else if (product_config.control.is_usb_control || product_config.control.is_i2c_usb_control) {
        IruvcHandle_t* iruvc_handle = (IruvcHandle_t*)driver_handle;
        if (product_config.control.is_usb_control)
            iruvc_usb_handle_create_with_exist_instance(ir_control_handle, iruvc_handle);
        else
            iruvc_i2c_usb_handle_create_with_exist_instance(ir_control_handle, iruvc_handle);
        IruvcDevParam_t ctrl_dev_param;
        memset(&ctrl_dev_param, 0, sizeof(IruvcDevParam_t));
        ctrl_dev_param.pid = product_config.control.usb_param.pid;
        ctrl_dev_param.vid = product_config.control.usb_param.vid;
        ctrl_dev_param.same_idx = product_config.control.usb_param.same_id;
        ret = ir_control_handle->ir_control_open(iruvc_handle, &ctrl_dev_param);
        ircmd_handle = ircmd_create_handle(ir_control_handle);
    }

    if (!product_config.camera.is_auto_image && ircmd_handle) {
        basic_video_stream_continue(ircmd_handle);
    }

    /* === 4. 计算热成像帧参数 === */
    int img_w = product_config.camera.width;
    int img_h = product_config.camera.image_info_height;
    float img_ratio  = product_config.camera.image_info_ratio;
    float info_ratio = product_config.camera.info_line_ratio;
    float temp_ratio = product_config.camera.temp_line_ratio;
    float dummy_ratio = product_config.camera.dummy_info_ratio;
    int info_h = product_config.camera.info_line_height;
    int temp_h = product_config.camera.temp_info_height;
    int dummy_h = product_config.camera.dummy_info_height;

    uint32_t image_size = (uint32_t)(img_w * img_h * img_ratio);
    uint32_t info_size  = (uint32_t)(img_w * info_h * info_ratio);
    uint32_t temp_size  = (uint32_t)(img_w * temp_h * temp_ratio);
    uint32_t dummy_size = (uint32_t)(img_w * dummy_h * dummy_ratio);
    uint32_t raw_size   = image_size + info_size + temp_size + dummy_size;

    int stream_w, stream_h, stream_fps;
    float frame_ratio;
    if (is_v4l2) {
        stream_w   = product_config.camera.v4l2_config.image_stream.dev_width;
        stream_h   = product_config.camera.v4l2_config.image_stream.dev_height;
        stream_fps = product_config.camera.v4l2_config.image_stream.fps;
        frame_ratio = img_ratio;
    } else {
        stream_w   = product_config.camera.uvc_stream_conf.width;
        stream_h   = product_config.camera.uvc_stream_conf.height;
        stream_fps = product_config.camera.uvc_stream_conf.fps;
        frame_ratio = product_config.camera.uvc_stream_conf.frame_size_ratio;
    }
    IrFrameFmt_t frame_fmt = (IrFrameFmt_t)product_config.camera.format;

    printf("[Main] thermal: %dx%d, stream %dx%d@%d\n", img_w, img_h, stream_w, stream_h, stream_fps);

    /* === 5. 初始化融合推流 === */
    gst_init(&argc, &argv);
    g_loop = g_main_loop_new(NULL, FALSE);

    ret = fusion_streamer_init(&fusion_ctx,
                              ir_image_video_handle,
                              driver_handle,
                              frame_fmt,
                              img_w, img_h,
                              stream_w, stream_h,
                              raw_size, image_size,
                              frame_ratio,
                              visible_cam, visible_w, visible_h,
                              fps,
                              is_v4l2 ? TRUE : FALSE);
    if (ret != 0) {
        printf("[Main] ERROR: fusion_streamer_init failed\n");
        goto cleanup;
    }

    if (fusion_conf) {
        FusionParams_t fp;
        fp.mode = FUSION_MODE_EDGE_OVERLAY;
        fp.alpha = 0.5f;
        fp.canny_low = 50;
        fp.canny_high = 150;
        if (load_fusion_params(fusion_conf, &fp) == 0) {
            fusion_streamer_set_params(&fusion_ctx, &fp);
            printf("[Main] loaded fusion params from %s\n", fusion_conf);
        }
    }

    printf("[Main] starting fusion RTSP server...\n");
    fusion_streamer_run(&fusion_ctx, rtsp_port, url_path, g_loop);

cleanup:
    printf("[Main] cleaning up...\n");
    fusion_streamer_cleanup(&fusion_ctx);

    if (ircmd_handle) {
        ircmd_delete_handle(ircmd_handle);
        ircmd_handle = NULL;
    }
    if (ir_control_handle) {
        if (product_config.control.is_i2c_control)
            iri2c_handle_delete(ir_control_handle);
        else if (product_config.control.is_uart_control)
            iruart_handle_delete(ir_control_handle);
        ir_control_handle_delete(&ir_control_handle);
        ir_control_handle = NULL;
    }
    if (driver_handle && ir_image_video_handle) {
        ir_image_video_handle->ir_video_release(driver_handle, NULL);
        ir_image_video_handle->ir_video_close(driver_handle);
        if (is_v4l2)
            irv4l2_handle_delete((Irv4l2VideoHandle_t*)driver_handle);
        else
            iruvc_camera_handle_delete((IruvcHandle_t*)driver_handle);
        driver_handle = NULL;
    }
    ir_video_handle_delete(&ir_image_video_handle);

    printf("[Main] done.\n");
    return 0;
}
