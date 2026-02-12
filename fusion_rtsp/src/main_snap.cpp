/**
 * 热成像 + 可见光拍照工具
 *
 * 用法：./fusion_snap <thermal_config> [options]
 * 串行流程：可见光拍一张保存 -> 热成像拍一张保存
 */
#include "fusion_streamer.h"
#include "config.h"
#include "data.h"
#include "cmd.h"
#include "libircam.h"
#include "libiruvc.h"
#include "libirv4l2.h"
#include "libircmd.h"
#include "libiri2c.h"
#include "libiruart.h"
#include <getopt.h>
#include <unistd.h>
#include <cstdio>
#include <cstring>

IrVideoHandle_t* ir_image_video_handle = NULL;

#define DEFAULT_VISIBLE_CAM  23
#define DEFAULT_VISIBLE_W   1920
#define DEFAULT_VISIBLE_H   1080
#define DEFAULT_FPS         10

static void print_usage(const char* prog)
{
    printf("Usage: %s <thermal_config> [options]\n", prog);
    printf("  热成像与可见光各拍一张并保存（串行流程）\n\n");
    printf("Options:\n");
    printf("  -o <dir>      输出目录 (default: .)\n");
    printf("  -v <index>    可见光相机索引, 如 23 对应 /dev/video23 (default: %d)\n", DEFAULT_VISIBLE_CAM);
    printf("  -W <width>    可见光宽度 (default: %d)\n", DEFAULT_VISIBLE_W);
    printf("  -H <height>   可见光高度 (default: %d)\n", DEFAULT_VISIBLE_H);
    printf("  -h            帮助\n");
    printf("\nExample: %s stream_RV1126.conf -o ./snap\n", prog);
}

int main(int argc, char* argv[])
{
    const char* thermal_config = NULL;
    const char* out_dir = ".";
    int visible_cam = DEFAULT_VISIBLE_CAM;
    int visible_w   = DEFAULT_VISIBLE_W;
    int visible_h   = DEFAULT_VISIBLE_H;
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
    while ((opt = getopt(argc, argv, "o:v:W:H:h")) != -1) {
        switch (opt) {
        case 'o': out_dir = optarg; break;
        case 'v': visible_cam = atoi(optarg); break;
        case 'W': visible_w = atoi(optarg); break;
        case 'H': visible_h = atoi(optarg); break;
        case 'h':
        default:
            print_usage(argv[0]);
            return 0;
        }
    }

    printf("========================================\n");
    printf("  Fusion Snap (可见光 + 热成像拍照)\n");
    printf("  config: %s, output: %s\n", thermal_config, out_dir);
    printf("  visible: /dev/video%d %dx%d\n", visible_cam, visible_w, visible_h);
    printf("========================================\n");

    /* === 1. 解析热成像配置 === */
    config config_obj;
    if (config_obj.parse_config(thermal_config) != 0) {
        printf("[Snap] ERROR: parse thermal config failed\n");
        return -1;
    }

    single_config product_config;
    std::string product_name = "WN2256";
    if (!config_obj.get_config(product_config, product_name)) {
        printf("[Snap] ERROR: get_config failed for product '%s'\n", product_name.c_str());
        return -1;
    }

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
            printf("[Snap] ERROR: V4L2 camera open failed\n");
            goto cleanup;
        }
        CamDevParams_t cam_params;
        memset(&cam_params, 0, sizeof(CamDevParams_t));
        cam_params.width  = product_config.camera.v4l2_config.image_stream.dev_width;
        cam_params.height = product_config.camera.v4l2_config.image_stream.dev_height;
        cam_params.fps    = product_config.camera.v4l2_config.image_stream.fps;
        cam_params.format = 0;
        ret = irv4l2_camera_init(v4l2_handle, &cam_params);
        if (ret != 0) {
            printf("[Snap] ERROR: V4L2 init failed\n");
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
            printf("[Snap] ERROR: UVC open failed\n");
            goto cleanup;
        }
        ret = ir_image_video_handle->ir_video_init(iruvc_handle, &dev_param);
        driver_handle = iruvc_handle;
    }

    /* === 3. 命令控制句柄 === */
    ir_control_handle_create(&ir_control_handle);
    if (product_config.control.is_i2c_control) {
        Iri2cHandle_t* i2c_handle = iri2c_handle_create(ir_control_handle);
        ret = iri2c_device_open(i2c_handle, product_config.control.i2c_param.dev_name.c_str());
        ircmd_handle = ircmd_create_handle(ir_control_handle);
    } else if (product_config.control.is_uart_control) {
        iruart_handle_create(ir_control_handle);
        ret = search_com(ir_control_handle, &ircmd_handle, product_config.control.uart_param.com_index);
        if (ret != 0) {
            printf("[Snap] ERROR: search_com failed\n");
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

    /* === 5. 初始化并拍照 === */
    ret = fusion_streamer_init(&fusion_ctx,
                              ir_image_video_handle,
                              driver_handle,
                              frame_fmt,
                              img_w, img_h,
                              stream_w, stream_h,
                              raw_size, image_size,
                              frame_ratio,
                              visible_cam, visible_w, visible_h,
                              stream_fps < DEFAULT_FPS ? stream_fps : DEFAULT_FPS,
                              is_v4l2 ? TRUE : FALSE);
    if (ret != 0) {
        printf("[Snap] ERROR: fusion_streamer_init failed\n");
        goto cleanup;
    }

    char visible_path[256];
    char thermal_path[256];
    snprintf(visible_path, sizeof(visible_path), "%s/visible.jpg", out_dir);
    snprintf(thermal_path, sizeof(thermal_path), "%s/thermal.jpg", out_dir);

    printf("[Snap] 等待 10 秒后开始拍照...\n");
    sleep(10);

    ret = fusion_streamer_snap(&fusion_ctx, visible_path, thermal_path);
    printf("[Snap] %s\n", ret == 0 ? "ok" : "failed");

cleanup:
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

    return ret;
}
