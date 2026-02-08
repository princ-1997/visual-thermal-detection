#ifndef LIBIRDFU_H
#define LIBIRDFU_H
#define DFU_FUNCTION_IN_DEVELOPING
#ifdef __cplusplus
extern "C" {
#endif

#if defined(_WIN32)
#define DLLEXPORT __declspec(dllexport)
#elif defined(linux) || defined(unix)
#define DLLEXPORT
#endif

#include "libircam.h"

/**
* @brief Log level definition in libirdfu library
*/
typedef enum
{
	/// print debug and error infomation
	IRDFU_LOG_DEBUG = 0,
	/// only print error infomation
	IRDFU_LOG_ERROR = 1,
	/// don't print debug and error infomation
	IRDFU_LOG_NO_PRINT = 2
} IrdfuLogLevels_e;

DLLEXPORT typedef void (*irdfu_callback)(void* callback_data, void* priv_data);

/**
 * @brief Get libirdfu library's version
 *
 * @return version's string
 */
DLLEXPORT const char* irdfu_version(void);

/**
 * @brief Get libirdfu library's version number
 *
 * @return version number's string
 */
DLLEXPORT const char* irdfu_version_number(void);

/**
 * @brief Register log print via log_level
 *
 * @param[in] log_level debug information level
 * @param[in] callback callback function for printing log messages
 * @param[in] priv_data the private parameter of callback function
 */
DLLEXPORT void irdfu_log_register(IrdfuLogLevels_e log_level, void* (*callback)(void* callback_data, void* priv_data), void* priv_data);

/**
 * @brief Assigning values to function pointers
 *
 * @param[in] ir_control_handle ir camera control handle
 *
 * @return see IrlibError_e
 */
DLLEXPORT IrlibError_e irdfu_handle_create(IrControlHandle_t* ir_control_handle);

/**
 * @brief Emptying function pointer data
 *
 * @param[in] ir_control_handle ir camera control handle
 *
 * @return see IrlibError_e
 */
DLLEXPORT IrlibError_e irdfu_handle_delete(IrControlHandle_t* ir_control_handle);

/**
 * @brief detect device status with usb interface
 *
 * @param[in] dfu_handle irdfu's handle
 * @param[in] params reserved parameter of status detection
 * @param[out] status status of devices,0:Unknow status,1:Rom code,2:Cache Code
 *
 * @return see IrlibError_e
 */
DLLEXPORT IrlibError_e irdfu_detect_device_status(void* dfu_handle, void* params, int* status);

/**
 * @brief download firmware with usb interface
 *
 * @param[in] dfu_handle irdfu's handle
 * @param[in] file_name the name of firmware file
 * @param[in] data data of firmware file
 * @param[in] file_len size of firmware file
 * @param[in] callback callback function of usb firmware download function
 * @param[in] priv_data the private parameter of callback function
 *
 * @return see IrlibError_e
 */
DLLEXPORT IrlibError_e irdfu_firmware_download(void* dfu_handle, char* file_name, uint8_t* data, \
	uint32_t file_len, void* (*callback)(void* callback_data, void* priv_data), void* priv_data);

/**
 * @brief Original data download interface
 *
 * @param[in] filename the name of firmware file
 * @param[in] usr_cb callback function of usb download function
 * @param[in] priv_data the private parameter of callback function
 *
 * @return see IrlibError_e
 */
#ifndef DFU_FUNCTION_IN_DEVELOPING
DLLEXPORT IrlibError_e irdfu_download(const char* filename, irdfu_callback usr_cb, void* priv_data);
#endif

#ifdef __cplusplus
}
#endif
#endif /* LIBIRDFU_H */