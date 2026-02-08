#ifndef _LIBSPI_H_
#define _LIBSPI_H_

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <stdarg.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <errno.h>
#include <linux/spi/spidev.h>


#include "libircam.h"
#ifdef __cplusplus
extern "C" {
#endif

#define BUFF_LEN 4096
#define DUMMY_LEN 512


/// @brief Video Streaming Handle
typedef struct _IrspiVideoHandle_t IrspiVideoHandle_t;

/**
* @brief Log level definition in libirspi library
*/
typedef enum
{
	/// print debug and error infomation
	IRSPI_LOG_DEBUG = 0,
	/// only print error infomation
	IRSPI_LOG_ERROR = 1,
	/// don't print debug and error infomation
	IRSPI_LOG_NO_PRINT = 2,
}IrspiLogLevels_e;

/**
* @brief frame types
*/
typedef enum
{
	IRSPI_IMAGE_FRAME = 0,
	IRSPI_TEMP_FRAME = 1,
}Irspiframetypes_e;

typedef struct
{
    uint32_t speed_hz;
    uint32_t delay_usecs;
    uint32_t bits_per_word;
	uint32_t mode; 				//polity and phase
	uint32_t buf_len;
	int width;
	int height;
} SpiParams_t;

/**
 * @brief register log level for libirspi library
 *
 * @param[in] log_level log level
 * @param[in] callback the callback function for printing log messages
 * @param[in] priv_data the private parameter of callback function
 *
 */
void irspi_log_register(IrspiLogLevels_e log_level, void* (*callback)(void* callback_data, void* priv_data), void* priv_data);

/**
 * @brief Get libirspi library's version number
 *
 * @return version number's string
 */
const char *irspi_version(void);

/**
 * @brief Create an irspi's video handle
 *
 * @param[in] ir_video_handle ir camera video handle
 *
 * @return IrspiVideoHandle_t struct pointer
 */
IrspiVideoHandle_t *irspi_handle_create(IrVideoHandle_t *ir_video_handle);

/**
 * @brief Delete and release irspi's video handle
 *
 * @param[in] irspi_handle irspi's video handle
 *
 * @return see IrlibError_e
 */
IrlibError_e irspi_handle_delete(IrspiVideoHandle_t *irspi_handle);

/**
 * @brief Open an video device with its device node
 *
 * @param[in] irspi_handle irspi's video handle
 * @param[in] dev_node device's node
 *
 * @return see IrlibError_e
 */
IrlibError_e irspi_camera_open(IrspiVideoHandle_t* irspi_handle, char* dev_node);

/**
 * @brief Initialize the device by the format of the video stream
 *
 * @param[in] irspi_handle irspi's video handle
 * @param[in] params irspi's parameters, see SpiParams_t
 *
 * @return see IrlibError_e
 */
IrlibError_e irspi_camera_init(IrspiVideoHandle_t* irspi_handle, SpiParams_t* params);

/**
 * @brief Start stream with specific one stream parameter of the camera
 *
 * @param[in] irspi_handle irspi's video handle
 *
 * @return see IrlibError_e
 */
IrlibError_e irspi_camera_start_stream(IrspiVideoHandle_t* irspi_handle);

/**
 * @brief Polling frame transfered from camera
 *
 * @param[in]   irspi_handle irspi's video handle
 * @param[in]	params not in use, fill with NULL
 * @param[out]  frame_data the data of one frame
 * @param[in]   len the length of one frame
 *
 * @return see IrlibError_e
 */
IrlibError_e irspi_camera_frame_get(IrspiVideoHandle_t* irspi_handle, void* params, uint8_t* frame_data, int len);

/**
 * @brief Stop the video stream
 *
 * @param[in] irspi_handle irspi's video handle
 *
 * @return see IrlibError_e
 */
IrlibError_e irspi_camera_stop_stream(IrspiVideoHandle_t* irspi_handle);

/**
 * @brief Release libirspi's service context
 *
 * @param[in] irspi_handle irspi's video handle
 * @param[in] params irspi's params, not in use, fill with NULL
 *
 * @return see IrlibError_e
 */
IrlibError_e irspi_camera_release(IrspiVideoHandle_t* irspi_handle, void* params);

/**
 * @brief Close the device
 *
 * @param[in] irspi_handle irspi's video handle
 *
 * @return see IrlibError_e
 */
IrlibError_e irspi_camera_close(IrspiVideoHandle_t* irspi_handle);


#ifdef __cplusplus
}
#endif
#endif
