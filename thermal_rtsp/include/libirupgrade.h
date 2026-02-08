#define _CRT_SECURE_NO_WARNINGS
#ifndef _LIBIRUPGRADE_H_
#define _LIBIRUPGRADE_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "libircmd.h"
#include "libircam.h"

#if defined(_WIN32)
#define DLLEXPORT __declspec(dllexport)
#include "windows.h"
#elif defined(linux) || defined(unix)
#define DLLEXPORT

#endif

/**
* @brief Log level definition in libirupgrade library
*/
enum IrupgradeLogLevels_e
{
    /// print debug and error infomation
    IRUPGRADE_LOG_DEBUG = 0,
    /// only print error infomation
    IRUPGRADE_LOG_ERROR = 1,
    /// don't print debug and error infomation
    IRUPGRADE_LOG_NO_PRINT = 2,
};

/**
 * @brief Get libirupgrade library's version
 *
 * @return version number's string
 */
DLLEXPORT const char* irupgrade_version(void);

/**
 * @brief Get libirupgrade library's version number
 *
 * @return version number's string
 */
DLLEXPORT const char* irupgrade_version_number(void);

/**
 * @brief register log level for libirupgrade library
 *
 * @param[in] log_level log level
 * @param[in] callback the callback function for printing log messages
 * @param[in] priv_data the private parameter of callback function
 *
 */
DLLEXPORT void irupgrade_log_register(enum IrupgradeLogLevels_e log_level, void* (*callback)(void* callback_data, void* priv_data), void* priv_data);

/**
 * @brief Data upgrade using upgrade package zip
 * Before calling this API, you need to make sure that ircmd_handle has been initialized
 *
 * @param[in] ircmd_handle ircmd's handle
 * @param[in] dev_info information of device node, dev_info and command_type together determine the open device node, The formats to be filled in are as follows
 *      UVC_USB_COMMAND_CHANNEL:        "UVC_USB:pid=0x0020,vid=0x3474,sameidx=0;"
 *      V4L2_USB_COMMAND_CHANNEL:       "V4L2_USB:/dev/video30;"
 *      UART_COMMAND_CHANNEL:           "UART:\\\\.\\COM10;"(windows) or "UART:/dev/ttyUSB1;"
 *      UVC_USB_I2C_COMMAND_CHANNEL:    "UVC_USB_I2C:pid=0x0020,vid=0x3474,sameidx=0;"
 *      V4L2_USB_I2C_COMMAND_CHANNEL:   "V4L2_USB_I2C:/dev/video40;"
 *      I2C_COMMAND_CHANNEL:            "ONLY_I2C:/dev/i2c-1;"
 * @param[in] upgrade_pack_path local path to find upgrade package zip
 * @param[in] param reserved parameters
 * @param[in] upgrade_process_callback Callback function for displaying progress information
 * @param[in] priv_data the private parameter of upgrade process callback function
 *
 * @return see IrlibError_e
 */
DLLEXPORT IrlibError_e upgrade_package_function(IrcmdHandle_t* ircmd_handle, char* dev_info, char* upgrade_pack_path, \
    void* param, void* (*upgrade_process_callback)(void* callback_data, void* priv_data), void* priv_data);

/**
 * @brief Single file download function
 * Before calling this API, you need to make sure that the device is connected and ircmd_handle has been initialized
 *
 * @param[in] ircmd_handle ircmd's handle
 * @param[in] target_file_path The target file name of the operation recognized by the firmware
 * @param[in] local_file_data File data intended to be downloaded to device
 * @param[in] local_file_length Length of file data(unit:Bytes)
 * @param[in] upgrade_process_callback Callback function for displaying progress information
 * @param[in] priv_data the private parameter of upgrade process callback function
 *
 * @return see IrlibError_e
 */
DLLEXPORT IrlibError_e single_file_write(IrcmdHandle_t* ircmd_handle, char* target_file_path, void* local_file_data, \
    int local_file_length, void* (*upgrade_process_callback)(void* callback_data, void* priv_data), void* priv_data);

/**
 * @brief Single file read function
 * Before calling this API, you need to make sure that the device is connected and ircmd_handle has been initialized
 *
 * @param[in] ircmd_handle ircmd's handle
 * @param[in] target_file_path The target file name of the operation recognized by the firmware
 * @param[out] file_data Used to store the read file data
 * @param[out] file_length Length of read file data(unit:Bytes)
 * @param[in] upgrade_process_callback Callback function for displaying progress information
 * @param[in] priv_data the private parameter of upgrade process callback function
 *
 * @return see IrlibError_e
 */
DLLEXPORT IrlibError_e single_file_read(IrcmdHandle_t* ircmd_handle, char* target_file_path, uint8_t* file_data, \
    int* file_length, void* (*upgrade_process_callback)(void* callback_data, void* priv_data), void* priv_data);

/**
 * @brief download firmware function, this function for download firmware.bin or firmware_with_bt.bin
 * Before calling this API, you need to make sure that the device is connected and ircmd_handle has been initialized
 *
 * @param[in] ircmd_handle ircmd's handle
 * @param[in] dev_param the device's param, from fuction parse_device_information
 * @param[in] target_file_path The target file name of the operation recognized by the firmware
 * @param[in] file_data data while been download into device
 * @param[in] file_length length of file_data
 * @param[in] upgrade_process_callback Callback function for displaying progress information
 * @param[in] priv_data the private parameter of upgrade process callback function
 *
 * @return see IrlibError_e
 */
DLLEXPORT IrlibError_e download_firmware(IrcmdHandle_t* ircmd_handle, void* dev_param,
    char* target_file_path, uint8_t* file_data, int file_length,
    void* (*upgrade_process_callback)(void* callback_data, void* priv_data), void* priv_data);

/**
 * @brief Parse device information to obtain valid device parameters
 *
 * @param[in] command_type type of command channel, see command_channel_e
 * @param[in] dev_info information of device node, dev_info and command_type together determine the open device node
 * @param[out] dev_param Used to store valid device parameters
 *
 * @return see IrlibError_e
 */
DLLEXPORT IrlibError_e parse_device_information(command_channel_e command_type, char* dev_info, void** dev_param);

#ifdef __cplusplus
}
#endif

#endif