#pragma once
#define HAVE_STRUCT_TIMESPEC

#include <stdio.h>
#include <stdint.h>
#include <malloc.h>
#include "libircmd.h"
#include "libircmd_temp.h"
#include "libircam.h"
#include "config.h"
#include <atomic>

#if defined(_WIN32)
#include <pthread.h>
#include <Windows.h>
#include "io.h"
#elif defined(linux) || defined(unix)
#include <unistd.h>
#include <semaphore.h>
#endif
extern std::atomic_bool isRUNNING;
extern int temp_measure_on;
extern IrVideoHandle_t* ir_image_video_handle;
extern IrVideoHandle_t* ir_temp_video_handle;

#define VERSION_NAME "libir_sample"
#define VERSION_NUMBER "2.4.5"
#define MAX_MALLOC_DATA_LENGTH  10000000

//#define INFO_LINE

//thread's semaphore
#if defined(_WIN32)
extern HANDLE image_sem, image_done_sem;
extern HANDLE i_temp_sem;
extern HANDLE i_temp_done;
extern HANDLE cap_sem, cap_done_sem;
extern HANDLE info_sem, info_done_sem;
extern HANDLE temp_sem, temp_done_sem;
extern HANDLE cmd_sem;
#elif defined(linux) || defined(unix)
extern sem_t image_sem, image_done_sem;
extern sem_t i_temp_sem;
extern sem_t i_temp_done;
extern sem_t cap_sem, cap_done_sem;
extern sem_t info_sem, info_done_sem;
extern sem_t temp_sem, temp_done_sem;
extern sem_t cmd_sem;
#endif

typedef enum
{
    YUYV_IMAGE = 0,
    NV12_IMAGE,
    NV12_AND_TEMP,
    YUYV_AND_TEMP,
    UYVY_IMAGE,
}FrameOutputFmt_t;

/*struct cameraArg
{
    char *name;
    int width;
    int height;
    uint8_t *dst;
    int length;
    int fmt;
};*/

typedef enum
{
    INPUT_FMT_Y14 = 0,
    INPUT_FMT_Y16,
    INPUT_FMT_YUV422,
    INPUT_FMT_YUV444,
    INPUT_FMT_RGB888,
    INPUT_FMT_INFO_LINE,
}InputFormat_t;

typedef enum
{
    OUTPUT_FMT_Y14 = 0,
    OUTPUT_FMT_YUV422,
    OUTPUT_FMT_YUV444,
    OUTPUT_FMT_RGB888,
    OUTPUT_FMT_BGR888,
    OUTPUT_FMT_INFO_LINE,
}OutputFormat_t;

typedef struct {
    uint32_t width;
    uint32_t height;
    uint8_t* data;
    uint32_t byte_size;
    InputFormat_t  input_format;
    OutputFormat_t  output_format;
}FrameInfo_t;

typedef struct {
    char* image_name;
    void* image_dev_params;
    void* image_driver_handle;

    char* temp_name;
    void* temp_dev_params;
    void* temp_driver_handle;

    void* stream_config;

    IrcmdHandle_t* ircmd_handle;
    uint32_t width;
    uint32_t height;
    uint8_t* raw_frame;
    uint32_t raw_byte_size;
    uint8_t* raw_temp_frame;
    uint32_t raw_temp_byte_size;
    FrameOutputFmt_t frame_output_format;

    FrameInfo_t image_info;
    FrameInfo_t information_line;
    FrameInfo_t temp_info;
    FrameInfo_t dummy_info;//temp_information_line when mipi 2vc
    single_config product_config;
}StreamFrameInfo_t;

//initial the pthread's cond and mutex
int init_pthread_sem();

//release the pthread's cond and mutex
int destroy_pthread_sem();

//create space for getting frames
int create_data_demo(StreamFrameInfo_t* stream_frame_info);

//destroy the space
int destroy_data_demo(StreamFrameInfo_t* stream_frame_info);

void load_stream_frame_info(StreamFrameInfo_t* stream_info, bool is_v4l2_driver, bool use_single_channel);


