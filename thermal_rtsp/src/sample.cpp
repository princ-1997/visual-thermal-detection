/**
 * 热红外 RTSP 实时视频流 — 主程序
 *
 * 功能：
 *   1. 解析配置文件，初始化 IR 热红外 UVC 相机
 *   2. 通过 GStreamer RTSP Server 对外提供 H.264 实时视频流
 *
 * 用法：
 *   ./thermal_rtsp <config_file> [-p port] [-u url_path]
 *   例: ./thermal_rtsp stream.conf -p 8554 -u /thermal
 *
 * 参考：
 *   - reference 项目的可见光 RTSP 方案 (appSource.cpp + main.cpp)
 *   - IR SDK 的 usb_stream_cmd 示例 (sample.cpp)
 */

#include "sample.h"
#include "libirv4l2.h"
#include <getopt.h>
#include <signal.h>
#include <atomic>

/* ======================== 全局变量 ======================== */
/* 这些全局变量由 data.h 声明为 extern，原本定义在 uvc_camera.cpp 中。
 * 由于我们不再使用 uvc_camera.cpp，需要在此定义。 */
std::atomic_bool isRUNNING(true);
int temp_measure_on = 0;
IrVideoHandle_t* ir_image_video_handle = NULL;
IrVideoHandle_t* ir_temp_video_handle  = NULL;

/* ======================== 默认参数 ======================== */
#define DEFAULT_RTSP_PORT    "8554"
#define DEFAULT_URL_PATH     "/thermal"

/* ======================== 信号处理 ======================== */
static GMainLoop* g_loop = NULL;

static void signal_handler(int sig)
{
    printf("\n[Main] caught signal %d, shutting down...\n", sig);
    isRUNNING = false;
    if (g_loop) {
        g_main_loop_quit(g_loop);
    }
}

/* ======================== 打印用法 ======================== */
static void print_usage(const char* prog)
{
    printf("Usage: %s <config_file> [options]\n", prog);
    printf("Options:\n");
    printf("  -p <port>      RTSP port (default: %s)\n", DEFAULT_RTSP_PORT);
    printf("  -u <url_path>  RTSP URL path (default: %s)\n", DEFAULT_URL_PATH);
    printf("  -h             Show this help\n");
}

/* ======================== 主函数 ======================== */
int main(int argc, char* argv[])
{
    const char* config_file = NULL;
    const char* rtsp_port   = DEFAULT_RTSP_PORT;
    const char* url_path    = DEFAULT_URL_PATH;
    int ret = 0;
    bool is_v4l2 = false;
    void* driver_handle = NULL;
    IrControlHandle_t* ir_control_handle = NULL;
    IrcmdHandle_t* ircmd_handle = NULL;
    RtspStreamerCtx_t rtsp_ctx;
    memset(&rtsp_ctx, 0, sizeof(rtsp_ctx));

    /* --- 解析命令行参数 --- */
    if (argc < 2) {
        print_usage(argv[0]);
        return -1;
    }
    config_file = argv[1];

    int opt;
    optind = 2;  /* 从第2个参数开始解析选项 */
    while ((opt = getopt(argc, argv, "p:u:h")) != -1) {
        switch (opt) {
        case 'p': rtsp_port = optarg; break;
        case 'u': url_path  = optarg; break;
        case 'h':
        default:
            print_usage(argv[0]);
            return 0;
        }
    }

    printf("========================================\n");
    printf("  Thermal IR RTSP Streaming Server\n");
    printf("  Config : %s\n", config_file);
    printf("  Port   : %s\n", rtsp_port);
    printf("  URL    : %s\n", url_path);
    printf("========================================\n");

    /* --- 注册信号处理 --- */
    signal(SIGINT,  signal_handler);
    signal(SIGTERM, signal_handler);

    /* === 1. 解析配置文件 === */
    config config_obj;
    if (config_obj.parse_config(config_file) != 0) {
        printf("[Main] ERROR: parse config failed\n");
        return -1;
    }

    single_config product_config;
    if (!config_obj.get_config(product_config)) {
        printf("[Main] ERROR: can't find config\n");
        return -1;
    }

    /* === 2. 打印 SDK 版本 === */
    printf("[Main] SDK versions:\n");
    printf("  ircmd : %s\n", ircmd_version());
    printf("  ircam : %s\n", ircam_version());
    printf("  iruvc : %s\n", iruvc_version());

    /* 设置日志级别 */
    ircmd_log_register(IRCMD_LOG_ERROR, NULL, NULL);
    iruvc_log_register(IRUVC_LOG_ERROR, NULL, NULL);
    iruart_log_register(IRUART_LOG_ERROR, NULL, NULL);
    ircam_log_register(IRCAM_LOG_ERROR, NULL, NULL);

    /* === 3. 创建 IR 视频句柄 === */
    ir_video_handle_create(&ir_image_video_handle);

    is_v4l2 = (product_config.camera.image_channel_type == "dvp" ||
               product_config.camera.image_channel_type == "mipi");

    if (is_v4l2) {
        /* === V4L2 路径 (DVP/MIPI) === */
        Irv4l2VideoHandle_t* v4l2_handle = irv4l2_handle_create(ir_image_video_handle);
        char* dev_node = (char*)product_config.camera.v4l2_config.image_stream.device_name.c_str();

        ret = irv4l2_camera_open(v4l2_handle, dev_node);
        printf("[Main] V4L2 open video (%s): %d\n", dev_node, ret);
        if (ret != 0) {
            printf("[Main] ERROR: V4L2 camera open failed\n");
            goto cleanup;
        }

        CamDevParams_t cam_params;
        memset(&cam_params, 0, sizeof(CamDevParams_t));
        cam_params.width  = product_config.camera.v4l2_config.image_stream.dev_width;
        cam_params.height = product_config.camera.v4l2_config.image_stream.dev_height;
        cam_params.fps    = product_config.camera.v4l2_config.image_stream.fps;
        /* format: 0=YUYV */
        cam_params.format = 0;

        ret = irv4l2_camera_init(v4l2_handle, &cam_params);
        printf("[Main] V4L2 init video: %d\n", ret);
        if (ret != 0) {
            printf("[Main] ERROR: V4L2 camera init failed\n");
            goto cleanup;
        }

        driver_handle = v4l2_handle;
    } else {
        /* === UVC 路径 (USB) === */
        IruvcHandle_t* iruvc_handle = iruvc_camera_handle_create(ir_image_video_handle);

        IruvcDevParam_t dev_param;
        memset(&dev_param, 0, sizeof(IruvcDevParam_t));
        dev_param.pid = product_config.camera.uvc_stream_conf.dev_info.pid;
        dev_param.vid = product_config.camera.uvc_stream_conf.dev_info.vid;
        dev_param.same_idx = product_config.camera.uvc_stream_conf.dev_info.same_id;

        ret = ir_image_video_handle->ir_video_open(iruvc_handle, &dev_param);
        printf("[Main] UVC open video: %d\n", ret);
        if (ret != 0) {
            printf("[Main] ERROR: UVC camera open failed\n");
            goto cleanup;
        }

        ret = ir_image_video_handle->ir_video_init(iruvc_handle, &dev_param);
        printf("[Main] UVC init video: %d\n", ret);

        driver_handle = iruvc_handle;
    }

    /* === 4. 创建命令控制句柄 === */
    ir_control_handle_create(&ir_control_handle);

    if (product_config.control.is_usb_control || product_config.control.is_i2c_usb_control) {
        /* USB 控制方式 */
        if (is_v4l2) {
            /* V4L2 驱动下的 USB 控制 */
            Irv4l2ControlHandle_t* v4l2_ctrl = NULL;
            if (product_config.control.is_usb_control) {
                v4l2_ctrl = irv4l2_usb_handle_create(ir_control_handle);
            } else {
                v4l2_ctrl = irv4l2_i2c_usb_handle_create(ir_control_handle);
            }
            /* V4L2 控制通道通过设备节点打开 */
            char* ctrl_node = (char*)product_config.camera.v4l2_config.image_stream.device_name.c_str();
            if (product_config.control.is_usb_control) {
                irv4l2_usb_device_open(v4l2_ctrl, ctrl_node);
            } else {
                irv4l2_i2c_usb_device_open(v4l2_ctrl, ctrl_node);
            }
        } else {
            /* UVC 驱动下的 USB 控制 */
            IruvcHandle_t* iruvc_handle = (IruvcHandle_t*)driver_handle;
            if (product_config.control.is_usb_control) {
                iruvc_usb_handle_create_with_exist_instance(ir_control_handle, iruvc_handle);
            } else {
                iruvc_i2c_usb_handle_create_with_exist_instance(ir_control_handle, iruvc_handle);
            }

            IruvcDevParam_t ctrl_dev_param;
            memset(&ctrl_dev_param, 0, sizeof(IruvcDevParam_t));
            ctrl_dev_param.pid = product_config.control.usb_param.pid;
            ctrl_dev_param.vid = product_config.control.usb_param.vid;
            ctrl_dev_param.same_idx = product_config.control.usb_param.same_id;
            ret = ir_control_handle->ir_control_open(iruvc_handle, &ctrl_dev_param);
            printf("[Main] open control node: %d\n", ret);
        }
        ircmd_handle = ircmd_create_handle(ir_control_handle);
    } else if (product_config.control.is_i2c_control) {
        /* I2C 控制方式 */
        Iri2cHandle_t* i2c_handle = iri2c_handle_create(ir_control_handle);
        ret = iri2c_device_open(i2c_handle,
                                product_config.control.i2c_param.dev_name.c_str());
        printf("[Main] I2C open control (%s): %d\n",
               product_config.control.i2c_param.dev_name.c_str(), ret);
        ircmd_handle = ircmd_create_handle(ir_control_handle);
    } else if (product_config.control.is_uart_control) {
        /* UART 控制方式 */
        iruart_handle_create(ir_control_handle);
        ret = search_com(ir_control_handle, &ircmd_handle,
                         product_config.control.uart_param.com_index);
        if (ret != 0) {
            printf("[Main] ERROR: failed to find available COM port\n");
            goto cleanup;
        }
    }

    /* === 5. 启动出图（非自动出图的设备需要命令触发） === */
    if (!product_config.camera.is_auto_image && ircmd_handle) {
        ret = basic_video_stream_continue(ircmd_handle);
        printf("[Main] basic_video_stream_continue: %d\n", ret);
    }

    /* === 6. 计算帧参数 === */
    /* 图像参数 */
    int img_w = product_config.camera.width;
    int img_h = product_config.camera.image_info_height;
    int info_h = product_config.camera.info_line_height;
    int temp_h = product_config.camera.temp_info_height;
    int dummy_h = product_config.camera.dummy_info_height;
    float img_ratio  = product_config.camera.image_info_ratio;
    float info_ratio = product_config.camera.info_line_ratio;
    float temp_ratio = product_config.camera.temp_line_ratio;
    float dummy_ratio = product_config.camera.dummy_info_ratio;

    uint32_t image_size = (uint32_t)(img_w * img_h * img_ratio);
    uint32_t info_size  = (uint32_t)(img_w * info_h * info_ratio);
    uint32_t temp_size  = (uint32_t)(img_w * temp_h * temp_ratio);
    uint32_t dummy_size = (uint32_t)(img_w * dummy_h * dummy_ratio);
    uint32_t raw_size   = image_size + info_size + temp_size + dummy_size;

    /* 流采集参数: V4L2 从 v4l2_config 获取, UVC 从 uvc_stream_conf 获取 */
    int stream_w, stream_h, stream_fps;
    float frame_ratio;
    if (is_v4l2) {
        stream_w   = product_config.camera.v4l2_config.image_stream.dev_width;
        stream_h   = product_config.camera.v4l2_config.image_stream.dev_height;
        stream_fps = product_config.camera.v4l2_config.image_stream.fps;
        frame_ratio = img_ratio;  /* V4L2 无独立 frame_size_ratio，使用 image_info_ratio */
    } else {
        stream_w   = product_config.camera.uvc_stream_conf.width;
        stream_h   = product_config.camera.uvc_stream_conf.height;
        stream_fps = product_config.camera.uvc_stream_conf.fps;
        frame_ratio = product_config.camera.uvc_stream_conf.frame_size_ratio;
    }
    IrFrameFmt_t frame_fmt = (IrFrameFmt_t)product_config.camera.format;

    printf("[Main] image: %dx%d, size=%u\n", img_w, img_h, image_size);
    printf("[Main] raw_size=%u (image=%u + info=%u + temp=%u + dummy=%u)\n",
           raw_size, image_size, info_size, temp_size, dummy_size);
    printf("[Main] stream: %dx%d, fps=%d, ratio=%.2f, mode=%s\n",
           stream_w, stream_h, stream_fps, frame_ratio, is_v4l2 ? "V4L2" : "UVC");

    /* === 7. 初始化 RTSP 推流模块 === */
    gst_init(&argc, &argv);
    g_loop = g_main_loop_new(NULL, FALSE);

    ret = rtsp_streamer_init(&rtsp_ctx,
                             ir_image_video_handle,
                             driver_handle,
                             frame_fmt,
                             img_w, img_h,
                             raw_size, image_size,
                             stream_w, stream_h, stream_fps,
                             frame_ratio, is_v4l2);
    if (ret != 0) {
        printf("[Main] ERROR: rtsp_streamer_init failed\n");
        goto cleanup;
    }

    /* === 8. 启动 RTSP 服务器（阻塞） === */
    printf("[Main] starting RTSP server...\n");
    rtsp_streamer_run(&rtsp_ctx, rtsp_port, url_path, g_loop);

    /* === 9. 清理资源 === */
cleanup:
    printf("[Main] cleaning up...\n");
    rtsp_streamer_cleanup(&rtsp_ctx);

    /* 释放命令句柄 */
    if (ircmd_handle) {
        ircmd_delete_handle(ircmd_handle);
        ircmd_handle = NULL;
    }

    /* 释放控制句柄 */
    if (ir_control_handle) {
        if (product_config.control.is_i2c_control) {
            iri2c_handle_delete(ir_control_handle);
        } else if (product_config.control.is_uart_control) {
            iruart_handle_delete(ir_control_handle);
        }
        ir_control_handle_delete(&ir_control_handle);
        ir_control_handle = NULL;
    }

    /* 释放视频句柄 */
    if (driver_handle && ir_image_video_handle) {
        ir_image_video_handle->ir_video_release(driver_handle, NULL);
        ir_image_video_handle->ir_video_close(driver_handle);
        if (is_v4l2) {
            irv4l2_handle_delete((Irv4l2VideoHandle_t*)driver_handle);
        } else {
            iruvc_camera_handle_delete((IruvcHandle_t*)driver_handle);
        }
        driver_handle = NULL;
    }
    ir_video_handle_delete(&ir_image_video_handle);

    printf("[Main] done.\n");
    return 0;
}
