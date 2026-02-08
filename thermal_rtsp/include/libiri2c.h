#ifndef _LIBI2C_H_
#define _LIBI2C_H_
#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <string.h>
#include <stdint.h>

#include "libircam.h"


/**
* @brief Error type in libiri2c library
*/


extern void (*iri2c_debug_print)(const char* fmt, ...);
extern void (*iri2c_error_print)(const char* fmt, ...);

/**
* @brief Log level definition in libiri2c library
*/
enum Iri2cLogLevels_e
{
    /// print debug and error infomation
    IRI2C_LOG_DEBUG = 0,
    /// only print error infomation
    IRI2C_LOG_ERROR = 1,
    /// don't print debug and error infomation
    IRI2C_LOG_NO_PRINT = 2,
};

typedef struct _Iri2cHandle_t Iri2cHandle_t;

typedef struct{
    uint16_t    polling_time;
    uint16_t    reg_addr;
    uint16_t    reserved[60];
}I2cParam_t;

typedef struct
{
    int fd;
    int reserved[63];
} I2cConDevParams_t;

/**
 * @brief register log level for libiri2c library
 *
 * @param[in] log_level log level
 * @param[in] callback the callback function for printing log messages
 * @param[in] priv_data the private parameter of callback function
 *
 */
void iri2c_log_register(enum Iri2cLogLevels_e log_level, void* (*callback)(void* callback_data, void* priv_data), void* priv_data);

/**
 * @brief Get libiri2c library's version
 *
 * @return version's string
 */
const char* iri2c_version(void);

/**
 * @brief Get libiri2c library's version number
 *
 * @return version number's string
 */
const char* iri2c_version_number(void);

/**
 * @brief Create an iri2c's handle
 *
 * @param[in] ir_control_handle ir camera control handle
 *
 * @return Iri2cHandle_t struct pointer
 */
Iri2cHandle_t* iri2c_handle_create(IrControlHandle_t* ir_control_handle);

/**
 * @brief Delete and release iri2c's handle
 *
 * @param[in] ir_control_handle ir camera control handle
 *
 * @return see IrlibError_e
 */
IrlibError_e iri2c_handle_delete(IrControlHandle_t* ir_control_handle);

/**
 * @brief Open an ir i2c device with its device node
 *
 * @param[in] i2c_handle iri2c's handle
 * @param[in] dev_node device's node
 *
 * @return see IrlibError_e
 */
IrlibError_e iri2c_device_open(Iri2cHandle_t* i2c_handle, const char* dev_node);

/**
 * @brief Initialize the configuration of the control node
 *
 * @param[in] i2c_handle iri2c's handle
 * @param[in] param the configuration of the control node
 *
 * @return see IrlibError_e
 */
IrlibError_e iri2c_device_init(Iri2cHandle_t* i2c_handle, I2cConDevParams_t* param);

/**
 * @brief read data from the i2c device
 *
 * @param[in] i2c_handle iri2c's handle
 * @param[in] cmd_param read param
 * @param[out] data the pointer for data which prepare to read
 * @param[in] len the read length
 *
 * @return see IrlibError_e
 */
IrlibError_e iri2c_data_read(Iri2cHandle_t* i2c_handle, void* cmd_param, uint8_t* data, int len);

/**
 * @brief write data to the i2c device
 *
 * @param[in] i2c_handle iri2c's handle
 * @param[in] cmd_param write param
 * @param[in] data the pointer for data which prepare to write
 * @param[in] len the write length
 *
 * @return see IrlibError_e
 */
IrlibError_e iri2c_data_write(Iri2cHandle_t* i2c_handle, void* cmd_param, uint8_t* data, int len);

/**
 * @brief read the device's register data
 *
 * @param[in] i2c_handle iri2c's handle
 * @param[in] reg_addr the register address
 * @param[out] data the pointer for data which prepare to read
 * @param[in] len the read length
 *
 * @return see IrlibError_e
 */
IrlibError_e iri2c_register_data_read(Iri2cHandle_t* i2c_handle, uint32_t reg_addr, uint8_t* data, int len);

/**
 * @brief Release ir2c's configuration of the control node
 *
 * @param[in] i2c_handle iri2c's handle
 * @param[in] params the configuration of the control node
 *
 * @return see IrlibError_e
 */
IrlibError_e iri2c_device_release(Iri2cHandle_t* i2c_handle, I2cConDevParams_t* param);

/**
 * @brief close an i2c device
 *
 * @param[in] i2c_handle iri2c's handle
 *
 * @return see IrlibError_e
 */
IrlibError_e iri2c_device_close(Iri2cHandle_t* i2c_handle);

/**
 * @brief download firmware with i2c interface
 *
 * @param[in] i2c_handle iri2c's handle
 * @param[in] file_name the name of firmware file
 * @param[in] data data of firmware file
 * @param[in] file_len size of firmware file
 * @param[in] callback callback function of i2c firmware download function
 * @param[in] priv_data the private parameter of firmware download callback function
 *
 * @return see IrlibError_e
 */
IrlibError_e iri2c_firmware_download(Iri2cHandle_t* i2c_handle, char* file_name, uint8_t* data, \
    uint32_t file_len, void* (*callback)(void* callback_data, void* priv_data), void* priv_data);

/**
 * @brief detect device status with i2c interface
 *
 * @param[in] i2c_handle iri2c's handle
 * @param[in] params device parameter
 * @param[out] status device status
 *
 * @return see IrlibError_e
 */
IrlibError_e iri2c_detect_device_status(Iri2cHandle_t* i2c_handle,void* params, int* status);


/**
 * @brief Get i2c command channel type
 *
 * @param[in] i2c_handle iri2c's handle
 * @param[out] command_channel_type command channel of i2c,see command_channel_e
 *
 * @return see IrlibError_e
 */
IrlibError_e iri2c_command_channel_get(Iri2cHandle_t* i2c_handle, int* command_channel_type);


/**
 * @brief download bootloader with i2c interface
 *
 * @param[in] i2c_handle iri2c's handle
 * @param[in] file_name the name of bootloader file
 * @param[in] data data of bootloader file
 * @param[in] file_len size of bootloader file
 * @param[in] callback callback function of i2c bootloader download function
 * @param[in] priv_data the private parameter of bootloader download callback function
 *
 * @return see IrlibError_e
 */
IrlibError_e iri2c_bootloader_download(Iri2cHandle_t* i2c_handle, char* file_name, uint8_t* data, \
    uint32_t file_len, void* (*callback)(void* callback_data, void* priv_data), void* priv_data);

#ifdef __cplusplus
}
#endif
#endif
