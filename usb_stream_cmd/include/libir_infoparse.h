#ifndef _INFO_PARSING_
#define _INFO_PARSING_
#define _CRT_SECURE_NO_WARNINGS
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "libircam.h"

#if defined(_WIN32)
#define DLLEXPORT __declspec(dllexport)
#elif defined(linux) || defined(unix)
#define DLLEXPORT
#endif

#pragma pack (1)

    /**
    * @brief Log level definition in libir_infoparse library
    */
    enum IrinfoparseLogLevels_e
    {
        /// print debug and error information
        IRINFOPARSE_LOG_DEBUG = 0,
        /// only print error information
        IRINFOPARSE_LOG_ERROR = 1,
        /// don't print debug and error information
        IRINFOPARSE_LOG_NO_PRINT = 2,
    };

enum IrinfoFrameFormat_e
{
    YUV422_FORMAT = 0,
    Y14_FORMAT,
    NV12_FORMAT,
};

enum IrinfoFrameMode_e
{
    ONLY_IMAGE_MODE = 0,
    ONLY_TEMP_MODE,
    IMAGE_AND_TEMP_MODE,
};

enum IrinfoFrameType_e
{
    REAL_TIME_IMAGE = 0,
    FREEZE_IMAGE,
};

enum IrinfoTempType_e
{
    DEGREES_CELSIUS_TEMP = 0,
    FAHRENHEIT_TEMP,
    KELVIN_TEMP
};

typedef struct {
    ///product name
    uint8_t dev_pn[4];
    ///Image output width
    uint16_t width;
    ///Image output height
    uint16_t height;
    /// Physical focal length
    uint8_t focal_length;
    /// Aperture size
    uint8_t Fnumber;
    /// Temperature measurement range
    uint8_t tpd_meter_range;
    /// fireware version
    uint8_t fireware_version[4];
}IrinfoDeviceInfo_t;

typedef struct {
    /// frame fps
    uint8_t frame_fps;
    /// Image data format(0:YUV422    1:Y14   2:NV12)
    uint8_t frame_format;
    /// Output video stream splicing mode(0:image    1:temp   2:image+temp)
    uint8_t frame_mode;
    /// Image type(0:Real-time Map    1:Freeze Map)
    uint8_t frame_type;
    /// ISP KBC module frame count
    uint16_t frame_count;
    /// Video module frame count
    uint16_t frame_cnt;
    /// Temp module frame count
    uint16_t temp_frame_cnt;
}IrinfoFrameSta_t;

typedef struct {
    uint8_t palette_idx;
    uint8_t palette_mode;
    uint8_t width_zoom_factor;
    uint8_t height_zoom_factor;
    uint8_t mirror_status;
    uint8_t flip_status;
}IrinfoImageSta_t;

typedef struct {
    /// Real-time gain status(0:low gain    1:high gain)
    uint8_t gain_mode;
    /// Isotherm switch status(0:off    1:on)
    uint8_t isotherm_switch;
    /// Output temp data type(0:degrees celsius temp    1:fahrenheit temp  2:kelvin)
    uint8_t temp_type;
    /// Temp area(0:high_temp_area  1:normal_temp_area   2:low_temp_area   3:fix_temp_area)
    uint8_t temp_area;
}IrinfoTpdSta_t;

typedef struct {
    /// Update the shutter countdown
    uint8_t ffc_countdown;
    /// shutter status(The current value is 0xFF)
    uint8_t shutter_status;
    /// auto shutter status(0:off     1:on)
    uint8_t auto_shutter_status;
    /// Shutter temperature information(celsius)
    float shutter_tpd_value;
    /// Time since the last FFC
    uint16_t after_ffc_cutdown;
    /// The shutter closing time for overexposure protection
    uint16_t shutter_close_time;
}IrinfoShutterSta_t;

typedef struct {
    /// Rmcover strength
    uint16_t rmv_strength;
    /// Device testing movement status
    uint8_t move_status;
    /// Current sl noise judgment level
    uint16_t noise_value;
    /// Current ide noise judgment level
    uint16_t noise_self;
    /// Motion Alignment Status
    uint32_t regis_status;
}IrinfoRegSta_t;

typedef struct {
    /// sensor temperature
    uint16_t sensor_temp;
    /// UTC (utc[0]=year) year/month/day/hour/minute/second
    uint8_t utc[7];
    /// Alarm bit(1:Ultra-high temperature operation   2:Ultra-low temperature operation)
    uint8_t alarm_bit;
    /// error bit(0:No abnormal)
    uint8_t error_bit;
    /// sensor temperature
    uint16_t vtemp;
    /// Sun detection(0:No-Sun-Detected    1:Sun-Detected)
    uint8_t sun_protect_flag;
    /// Lens temperature information(celsius)
    uint8_t len_tpd_value;
}IrinfoDeviceSta_t;

typedef struct {
		/// x position
		uint16_t x;
		/// y position
		uint16_t y;
}IrinfoPoint_t;

typedef struct {
    IrinfoPoint_t irsc_pixel;
    uint16_t irsc_pixel_value;
}IrinfoKBCInfo_t;

typedef struct {
    /// Enabling status(0:off    1 : on)
    uint8_t frame_info_en;
    IrinfoPoint_t max_pixel;
    float max_pixel_value;
    IrinfoPoint_t min_pixel;
    float min_pixel_value;
}IrinfoFrameTpd_t;

typedef struct {
    /// Enabling status(0:off    1 : on)
    uint8_t pixel_info_en;
    IrinfoPoint_t pixel;
    float pixel_value;
}IrinfoPixelTpd_t;

typedef struct {
    /// Enabling status(0:off    1 : on)
    uint8_t line_rect_info_en;
    IrinfoPoint_t start_pixel;
    IrinfoPoint_t end_pixel;
    IrinfoPoint_t max_pixel;
    IrinfoPoint_t min_pixel;
    float max_pixel_value;
    float min_pixel_value;
    float ave_pixel_value;
}IrinfoLineRectTpd_t;

/// New temperature measurement points at the software assignable information
typedef struct {
    /// Enabling status(0:off    1 : on)
    uint8_t pixel_info_en;
    float pixel_value;
}IrinfoNewPixelTpd_t;

/// New temperature measurement rectangle at the software assignable information
typedef struct {
    /// Enabling status(0:off    1 : on)
    uint8_t line_rect_info_en;
    IrinfoPoint_t max_pixel;
    IrinfoPoint_t min_pixel;
    float max_pixel_value;
    float min_pixel_value;
    float ave_pixel_value;
}IrinfoNewLineRectTpd_t;

typedef struct {
    /// Enabling status(0:off    1 : on)
    uint8_t shape_info_en;
    IrinfoPoint_t max_pixel;
    IrinfoPoint_t min_pixel;
    float max_pixel_value;
    float min_pixel_value;
    float ave_pixel_value;
}IrinfoShapeTpd_t;

/**
 * @brief KB correction module
 */
typedef struct {
    /// K environment variable correction parameter
    uint32_t k_e;
    /// B environment variable correction parameter
    uint32_t b_e;
    /// K distance correction parameter
    uint32_t k_d;
    /// B distance correction parameter
    uint32_t b_d;
    /// K non-stationary correction parameter
    uint32_t k_us;
    /// B non-stationary correction parameter
    uint32_t b_us;
    uint8_t dummy0;
}IrinfoKBInfo_t;

/**
 * @brief Autofocus Module
 */
typedef struct {
    /// Autofocus INFO enable(0:off    1:on)
    uint8_t af_info_en;
    /// Autofocus detail evaluation value
    uint8_t af_sum[5];//TODO
}IrinfoAFInfo_t;

/**
 * @brief SNR Module
 */
typedef struct {
    /// Maximum value of pixel points in the SNR module frame
    uint16_t frame_max_value;
    IrinfoPoint_t frame_max_pixel;
    /// Minimum value of pixel points in the SNR module frame
    uint16_t frame_min_value;
    IrinfoPoint_t frame_min_pixel;
    IrinfoPoint_t pixel;
    /// Value of the specified point in the SNR module frame
    uint16_t pixel_value;
}IrinfoSNRInfo_t;

/**
 * @brief AGC Module
 */
typedef struct {
    /// Histogram Statistics High Throw Point
    uint16_t histogram_max_value;
    /// Histogram Statistics Low Throw Point
    uint16_t histogram_min_value;
    /// AGC input frame average
    uint16_t frame_ave_value;
    /// Output the coordinates of the point specified by AGC
    IrinfoPoint_t pixel;
    /// Output the value of the point specified by AGC
    uint16_t pixel_value;
}IrinfoAGCInfo_t;

typedef struct {
    IrinfoFrameTpd_t frame_tpd;
    IrinfoPixelTpd_t pixel_tpd_0;
    IrinfoPixelTpd_t pixel_tpd_1;
    IrinfoPixelTpd_t pixel_tpd_2;
    IrinfoPixelTpd_t pixel_tpd_3;
    IrinfoNewPixelTpd_t pixel_tpd_4;
    IrinfoNewPixelTpd_t pixel_tpd_5;
    IrinfoNewPixelTpd_t pixel_tpd_6;
    IrinfoNewPixelTpd_t pixel_tpd_7;
    IrinfoNewPixelTpd_t pixel_tpd_8;
    IrinfoLineRectTpd_t line_tpd_0;
    IrinfoLineRectTpd_t line_tpd_1;
    IrinfoLineRectTpd_t rect_tpd_0;
    IrinfoLineRectTpd_t rect_tpd_1;
    IrinfoNewLineRectTpd_t rect_tpd_2;
    IrinfoShapeTpd_t shape_tpd_0;
    IrinfoShapeTpd_t shape_tpd_1;
    IrinfoKBInfo_t kb_info;
}IrinfoTpdInfo_t;

typedef struct {
    /// info line data version
    uint8_t info_data_version;
    IrinfoRegSta_t reg_status; //TODO
    IrinfoKBCInfo_t kbc_info;
    IrinfoAFInfo_t af_info;
    IrinfoSNRInfo_t snr_info;
    IrinfoAGCInfo_t agc_info;
}IrinfoTestInfo_t;

typedef struct {
    /// info line data version
    uint8_t info_data_version;
    IrinfoFrameSta_t frame_status;
    IrinfoImageSta_t image_status;
    IrinfoTpdSta_t tpd_status;
    IrinfoShutterSta_t shutter_status;
    IrinfoDeviceSta_t device_status;
}IrinfoStatusInfo_t;

typedef struct {
    /// info line data version
    uint8_t info_data_version;
    IrinfoDeviceInfo_t device_info;
    IrinfoTpdInfo_t tpd_info;
}IrinfoFuncInfo_t;

#pragma pack ()

/**
 * @brief Get libir_infoparse library's version number
 *
 * @return version number's string
 */
DLLEXPORT const char* irinfoparse_version(void);

/**
 * @brief register log level for libir_infoparse library
 *
 * @param[in] log_level log level
 * @param[in] callback the callback function for printing log messages
 * @param[in] priv_data the private parameter of callback function
 *
 */
DLLEXPORT void irinfoparse_log_register(enum IrinfoparseLogLevels_e log_level,\
    void* (*callback)(void* callback_data, void* priv_data), void* priv_data);

/**
 * @brief Get status info from infomation line
 *
 * @param[in] info_data infomation line
 * @param[in] status_info
 *
 */
DLLEXPORT IrlibError_e irinfoparse_get_irinfo_status_info(uint8_t* info_data, IrinfoStatusInfo_t* status_info);

/**
 * @brief Get test info from infomation line
 *
 * @param[in] info_data infomation line
 * @param[in] test_info
 *
 */
DLLEXPORT IrlibError_e irinfoparse_get_irinfo_test_info(uint8_t* info_data, IrinfoTestInfo_t* test_info);

/**
 * @brief Get function info from infomation line
 *
 * @param[in] info_data infomation line
 * @param[in] function_info
 *
 */
DLLEXPORT IrlibError_e irinfoparse_get_irinfo_function_info(uint8_t* info_data, IrinfoFuncInfo_t* function_info);


#ifdef __cplusplus
}
#endif

#endif