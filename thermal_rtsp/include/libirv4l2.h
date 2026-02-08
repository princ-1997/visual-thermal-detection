#ifndef LIBV4L2_H
#define LIBV4L2_H

#include <linux/types.h>
#include <linux/videodev2.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <malloc.h>
#include <math.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <assert.h>
#include <time.h>
#include <poll.h>
#include <pthread.h>
#include "libircam.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define CMD_MAGIC 0xEF //定义幻数
#define CMD_MAX_NR 3 //定义命令的最大序数
#define CMD_GET _IOWR(CMD_MAGIC, 1,UvcCmdParam_t)
#define CMD_SET _IOW(CMD_MAGIC, 2,UvcCmdParam_t)
#define CMD_KBUF _IO(CMD_MAGIC, 3)


/// @brief Video Streaming Handle
typedef struct _Irv4l2VideoHandle_t Irv4l2VideoHandle_t;

/// @brief Control command handle
typedef struct _Irv4l2ControlHandle_t Irv4l2ControlHandle_t;

typedef struct
{
    int width;
    int height;
    int format;
    int fps;
    int reserved[48];
} CamDevParams_t;

typedef struct
{
    int width;
    int height;
    int reserved[56];
} CamStreamParams_t;

typedef struct
{
    int got_bytes;
    int reserved[28];
} FrameGetParams_t;

typedef struct
{
    int reserved[64];
} V4l2ConDevParams_t;

/**
 * @brief Log level definition in libircmd library
 */
typedef enum
{
    IRV4L2_LOG_DEBUG = 0,    ///< print debug and error infomation
    IRV4L2_LOG_ERROR = 1,    ///< only print error infomation
    IRV4L2_LOG_NO_PRINT = 2, ///< don't print debug and error infomation
} irv4l2_log_level_e;

/**
 * @brief register log level for libirv4l2 library
 *
 * @param[in] log_level log level
 * @param[in] callback the callback function for printing log messages
 * @param[in] priv_data the private parameter of callback function
 *
 */
void irv4l2_log_register(irv4l2_log_level_e log_level, void* (*callback)(void* callback_data, void* priv_data), void* priv_data);

/**
 * @brief Get libirv4l2 library's version number
 *
 * @return version number's string
 */
const char *libv4l2_version(void);

/**
 * @brief Create an irv4l2's video handle
 *
 * @param[in] ir_video_handle ir camera video handle
 *
 * @return Irv4l2VideoHandle_t struct pointer
 */
Irv4l2VideoHandle_t *irv4l2_handle_create(IrVideoHandle_t *ir_video_handle);

/**
 * @brief Delete and release irv4l2's video handle
 *
 * @param[in] v4l2_handle irv4l2's video handle
 *
 * @return see IrlibError_e
 */
IrlibError_e irv4l2_handle_delete(Irv4l2VideoHandle_t *v4l2_handle);

/**
 * @brief Open an video device with its device node
 *
 * @param[in] v4l2_handle irv4l2's video handle
 * @param[in] dev_node device's node
 *
 * @return see IrlibError_e
 */
IrlibError_e irv4l2_camera_open(Irv4l2VideoHandle_t *v4l2_handle, char *dev_node);

/**
 * @brief Initialize the device by the format of the video stream
 *
 * @param[in] v4l2_handle irv4l2's video handle
 * @param[in] params format of video streams
 *
 * @return see IrlibError_e
 */
IrlibError_e irv4l2_camera_init(Irv4l2VideoHandle_t *v4l2_handle, CamDevParams_t *params);

/**
 * @brief Start stream with specific one stream parameter of the camera
 *
 * @param[in] v4l2_handle irv4l2's video handle
 * @param[in] params camera's one selected stream parameter
 *
 * @return see IrlibError_e
 */
IrlibError_e irv4l2_camera_start_stream(Irv4l2VideoHandle_t *v4l2_handle, CamStreamParams_t *params);

/**
 * @brief Polling frame transfered from camera
 *
 * @param[in] v4l2_handle irv4l2's video handle
 * @param[in] params reserved params
 * @param[out] frame_data the data of one frame
 * @param[in] len the length of one frame
 *
 * @return see IrlibError_e
 */
IrlibError_e irv4l2_camera_frame_get(Irv4l2VideoHandle_t *v4l2_handle, FrameGetParams_t *params, uint8_t *frame_data, int len);

/**
 * @brief Stop the video stream
 *
 * @param[in] v4l2_handle irv4l2's video handle
 * @param[in] params camera's one selected stream parameter
 *
 * @return see IrlibError_e
 */
IrlibError_e irv4l2_camera_stop_stream(Irv4l2VideoHandle_t *v4l2_handle, CamStreamParams_t *params);

/**
 * @brief Release libirv4l2's service context
 *
 * @param[in] v4l2_handle irv4l2's video handle
 * @param[in] params format of video streams
 *
 * @return see IrlibError_e
 */
IrlibError_e irv4l2_camera_release(Irv4l2VideoHandle_t *v4l2_handle, CamDevParams_t *params);

/**
 * @brief Close the device
 *
 * @param[in] v4l2_handle irv4l2's video handle
 *
 * @return see IrlibError_e
 */
IrlibError_e irv4l2_camera_close(Irv4l2VideoHandle_t *v4l2_handle);



/**
 * @brief Create an irv4l2's control handle with the i2c_usb path
 *
 * @param[in] ir_control_handle ir camera control handle
 *
 * @return Irv4l2ControlHandle_t struct pointer
 */
Irv4l2ControlHandle_t* irv4l2_i2c_usb_handle_create(IrControlHandle_t *ir_control_handle);

/**
 * @brief Delete and release irv4l2's control handle with the i2c_usb path
 *
 * @param[in] irv4l2_handle irv4l2's control handle
 *
 * @return see IrlibError_e
 */
IrlibError_e irv4l2_i2c_usb_handle_delete(Irv4l2ControlHandle_t *irv4l2_handle);

/**
 * @brief Open an ir i2c_usb device with its device node
 *
 * @param[in] irv4l2_handle irv4l2's control handle
 * @param[in] dev_node device's node
 *
 * @return see IrlibError_e
 */
IrlibError_e irv4l2_i2c_usb_device_open(Irv4l2ControlHandle_t *irv4l2_handle,char *dev_node);

/**
 * @brief Initialize i2c_usb's configuration of the control node
 *
 * @param[in] irv4l2_handle irv4l2's control handle
 * @param[in] param the configuration of the control node
 *
 * @return see IrlibError_e
 */
IrlibError_e irv4l2_i2c_usb_device_init(Irv4l2ControlHandle_t *irv4l2_handle,V4l2ConDevParams_t *param);

/**
 * @brief read data from the ir i2c_usb device
 *
 * @param[in] irv4l2_handle irv4l2's control handle
 * @param[in] cmd_param read param
 * @param[out] data the pointer for data which prepare to read
 * @param[in] len the read length
 *
 * @return see IrlibError_e
 */
IrlibError_e irv4l2_i2c_usb_data_read(Irv4l2ControlHandle_t* irv4l2_handle, void* cmd_param,uint8_t* data, uint32_t len);

/**
 * @brief write data to the ir i2c_usb device
 *
 * @param[in] irv4l2_handle irv4l2's control handle
 * @param[in] cmd_param write param
 * @param[in] data the pointer for data which prepare to write
 * @param[in] len the write length
 *
 * @return see IrlibError_e
 */
IrlibError_e irv4l2_i2c_usb_data_write(Irv4l2ControlHandle_t * irv4l2_handle, void* cmd_param,uint8_t* data, uint32_t len);

/**
 * @brief Release i2c_usb's configuration of the control node
 *
 * @param[in] irv4l2_handle irv4l2's control handle
 * @param[in] param the configuration of the control node
 *
 * @return see IrlibError_e
 */
IrlibError_e irv4l2_i2c_usb_device_release(Irv4l2ControlHandle_t *irv4l2_handle,V4l2ConDevParams_t *param);

/**
 * @brief close an ir i2c_usb device
 *
 * @param[in] irv4l2_handle irv4l2's control handle
 *
 * @return see IrlibError_e
 */
IrlibError_e irv4l2_i2c_usb_device_close(Irv4l2ControlHandle_t *irv4l2_handle);




/**
 * @brief Create an irv4l2's control handle with the usb path
 *
 * @param[in] ir_control_handle ir camera control handle
 *
 * @return Irv4l2ControlHandle_t struct pointer
 */
Irv4l2ControlHandle_t* irv4l2_usb_handle_create(IrControlHandle_t *ir_control_handle);

/**
 * @brief Delete and release irv4l2's control handle with the usb path
 *
 * @param[in] handle irv4l2's control handle
 *
 * @return see IrlibError_e
 */
IrlibError_e irv4l2_usb_handle_delete(Irv4l2ControlHandle_t *irv4l2_handle);

/**
 * @brief Open an ir usb device with its device node
 *
 * @param[in] irv4l2_handle irv4l2's control handle
 * @param[in] dev_node device's node
 *
 * @return see IrlibError_e
 */
IrlibError_e irv4l2_usb_device_open(Irv4l2ControlHandle_t *irv4l2_handle,char *dev_node);

/**
 * @brief Initialize usb's configuration of the control node
 *
 * @param[in] irv4l2_handle irv4l2's control handle
 * @param[in] param the configuration of the control node
 *
 * @return see IrlibError_e
 */
IrlibError_e irv4l2_usb_device_init(Irv4l2ControlHandle_t *irv4l2_handle,V4l2ConDevParams_t *param);

/**
 * @brief Release usb's configuration of the control node
 *
 * @param[in] irv4l2_handle irv4l2's control handle
 * @param[in] param the configuration of the control node
 *
 * @return see IrlibError_e
 */
IrlibError_e irv4l2_usb_device_release(Irv4l2ControlHandle_t *irv4l2_handle,V4l2ConDevParams_t *param);

/**
 * @brief close an ir usb device
 *
 * @param[in] irv4l2_handle irv4l2's control handle
 *
 * @return see IrlibError_e
 */
IrlibError_e irv4l2_usb_device_close(Irv4l2ControlHandle_t *irv4l2_handle);

/**
 * @brief read data from the ir usb device
 *
 * @param[in] irv4l2_handle irv4l2's control handle
 * @param[in] cmd_param read param
 * @param[out] data the pointer for data which prepare to read
 * @param[in] len the read length
 *
 * @return see IrlibError_e
 */
IrlibError_e irv4l2_usb_data_read(Irv4l2ControlHandle_t* irv4l2_handle, void* cmd_param, uint8_t* data, uint32_t len);

/**
 * @brief write data to the ir usb device
 *
 * @param[in] irv4l2_handle irv4l2's control handle
 * @param[in] cmd_param write param
 * @param[in] data the pointer for data which prepare to write
 * @param[in] len the write length
 *
 * @return see IrlibError_e
 */
IrlibError_e irv4l2_usb_data_write(Irv4l2ControlHandle_t* irv4l2_handle, void* cmd_param, uint8_t* data, uint32_t len);


#ifdef __cplusplus
}
#endif

#endif