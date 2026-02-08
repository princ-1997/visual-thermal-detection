#define _CRT_SECURE_NO_WARNINGS
#ifndef _LIBUART_H_
#define _LIBUART_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <errno.h>
#include <stdarg.h>

#if defined(_WIN32)
#define DLLEXPORT __declspec(dllexport)
#include "windows.h"
#elif defined(linux) || defined(unix)
#define DLLEXPORT
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/time.h>
#include <sys/select.h>
#endif

#include "libircam.h"

typedef struct _IruartHandle_t IruartHandle_t;


/**
* @brief Log level definition in libiruart library
*/
enum IruartLogLevels_e
{
    /// print debug and error information
    IRUART_LOG_DEBUG = 0,
    /// only print error information
    IRUART_LOG_ERROR = 1,
    /// don't print debug and error information
    IRUART_LOG_NO_PRINT = 2,
};


typedef struct
{
    int baudrate;
    int reserved[63];
} UartConDevParams_t;

/**
 * @brief register log level for libiruart library
 *
 * @param[in] log_level log level
 * @param[in] callback the callback function for printing log messages
 * @param[in] priv_data the private parameter of callback function
 *
 */
DLLEXPORT void iruart_log_register(enum IruartLogLevels_e log_level, void* (*callback)(void* callback_data, void* priv_data), void* priv_data);


/**
 * @brief Get libiruart library's version
 *
 * @return version number's string
 */
DLLEXPORT const char* iruart_version(void);


/**
 * @brief Get libiruart library's version number
 *
 * @return version number's string
 */
DLLEXPORT const char* iruart_version_number(void);


/**
 * @brief Create an iruart's handle
 *
 * @param[in] ir_control_handle ir camera control handle
 *
 * @return IruartHandle_t struct pointer
 */
DLLEXPORT IruartHandle_t* iruart_handle_create(IrControlHandle_t* ir_control_handle);


/**
 * @brief Delete and release iruart's handle
 *
 * @param[in] ir_control_handle ir camera control handle
 *
 * @return see IrlibError_e
 */
DLLEXPORT IrlibError_e iruart_handle_delete(IrControlHandle_t* ir_control_handle);


/**
 * @brief Open an ir uart device with its device node
 *
 * @param[in] uart_handle iruart's handle
 * @param[in] dev_node device's node
 *
 * @return see IrlibError_e
 */
DLLEXPORT IrlibError_e iruart_open(IruartHandle_t* uart_handle, const char* dev_node);


/**
 * @brief Initialize the configuration of the control node
 *
 * @param[in] uart_handle iruart's handle
 * @param[in] param the configuration of the control node
 *
 * @return see IrlibError_e
 */
DLLEXPORT IrlibError_e iruart_init(IruartHandle_t* uart_handle, UartConDevParams_t* param);


/**
 * @brief read data from the infiray uart device
 *
 * @param[in] uart_handle iruart's handle
 * @param[in] cmd_param read param
 * @param[out] data the pointer for data which prepare to read
 * @param[in] len the read length
 *
 * @return see IrlibError_e
 */
DLLEXPORT IrlibError_e iruart_data_read(IruartHandle_t* uart_handle, void* cmd_param, uint8_t* data, uint32_t len);


/**
 * @brief write data to the infiray uart device
 *
 * @param[in] uart_handle iruart's handle
 * @param[in] cmd_param write param
 * @param[in] data the pointer for data which prepare to write
 * @param[in] len the write length
 *
 * @return see IrlibError_e
 */
DLLEXPORT IrlibError_e iruart_data_write(IruartHandle_t* uart_handle, void* cmd_param, uint8_t* data, uint32_t len);


/**
 * @brief write data to the infiray uart device without reading return status
 *
 * @param[in] uart_handle iruart's handle
 * @param[in] cmd_param write param
 * @param[in] data the pointer for data which prepare to write
 * @param[in] len the write length
 *
 * @return see IrlibError_e
 */
DLLEXPORT IrlibError_e iruart_data_write_without_read_return_status(IruartHandle_t* uart_handle, void* cmd_param, \
    uint8_t* data, uint32_t len);


/**
 * @brief Release libiruart's configuration of the control node
 *
 * @param[in] uart_handle iruart's handle
 * @param[in] params the configuration of the control node
 *
 * @return see IrlibError_e
 */
DLLEXPORT IrlibError_e iruart_release(IruartHandle_t* uart_handle, UartConDevParams_t* param);


/**
 * @brief close an infiray uart device
 *
 * @param[in] uart_handle iruart's handle
 *
 * @return see IrlibError_e
 */
DLLEXPORT IrlibError_e iruart_close(IruartHandle_t* uart_handle);


/**
 * @brief download firmware with uart interface
 *
 * @param[in] uart_handle iruart's handle
 * @param[in] file_name the name of firmware file
 * @param[in] data data of firmware file
 * @param[in] file_len size of firmware file
 * @param[in] callback callback function of uart firmware download function
 * @param[in] priv_data the private parameter of firmware download callback function
 *
 * @return see IrlibError_e
 */
DLLEXPORT IrlibError_e iruart_firmware_download(IruartHandle_t* uart_handle, char* file_name, uint8_t* data, \
    uint32_t file_len, void* (*callback)(void* callback_data, void* priv_data), void* priv_data);


/**
 * @brief detect device status with uart interface
 *
 * @param[in] uart_handle iruart's handle
 * @param[in] params reserved parameter of status detection
 * @param[out] status status of devices,0:Unknow status,1:Rom code,2:Cache Code
 *
 * @return see IrlibError_e
 */
DLLEXPORT IrlibError_e iruart_detect_device_status(IruartHandle_t* uart_handle, void* params, int* status);


/**
 * @brief Get uart command channel type
 *
 * @param[in] uart_handle iruart's handle
 * @param[out] command_channel_type command channel of uart,see command_channel_e
 *
 * @return see IrlibError_e
 */
DLLEXPORT IrlibError_e iruart_command_channel_get(IruartHandle_t* uart_handle, int* command_channel_type);

#ifdef __cplusplus
}
#endif

#endif