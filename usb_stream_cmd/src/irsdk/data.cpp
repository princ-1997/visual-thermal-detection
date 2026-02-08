#include "data.h"

//thread's semaphore
#if defined(_WIN32)
HANDLE image_sem, image_done_sem;
HANDLE i_temp_sem;
HANDLE i_temp_done;
HANDLE cap_sem, cap_done_sem;
HANDLE info_sem, info_done_sem;
HANDLE temp_sem, temp_done_sem;
HANDLE cmd_sem;
#elif defined(linux) || defined(unix)
sem_t image_sem, image_done_sem;
sem_t i_temp_sem;
sem_t i_temp_done;
sem_t cap_sem, cap_done_sem;
sem_t info_sem, info_done_sem;
sem_t temp_sem, temp_done_sem;
sem_t cmd_sem;
#endif

//init the semaphore
int init_pthread_sem()
{
#if defined(_WIN32)
	image_sem = CreateSemaphore(NULL, 0, 1, NULL);
	image_done_sem = CreateSemaphore(NULL, 1, 1, NULL);
    i_temp_sem = CreateSemaphore(NULL, 0, 1, NULL);
    i_temp_done = CreateSemaphore(NULL, 0, 1, NULL);
	cap_sem = CreateSemaphore(NULL, 1, 1, NULL);
	cap_done_sem = CreateSemaphore(NULL, 0, 1, NULL);
    info_sem = CreateSemaphore(NULL, 0, 1, NULL);
	info_done_sem = CreateSemaphore(NULL, 1, 1, NULL);
    temp_sem = CreateSemaphore(NULL, 0, 1, NULL);
    temp_done_sem = CreateSemaphore(NULL, 1, 1, NULL);
    cmd_sem = CreateSemaphore(NULL, 0, 1, NULL);
#elif defined(linux) || defined(unix)
	sem_init(&image_sem, 0, 0);
	sem_init(&image_done_sem, 0, 1);
    sem_init(&i_temp_sem, 0, 0);
    sem_init(&i_temp_done, 0, 0);
	sem_init(&cap_sem, 0, 1);
	sem_init(&cap_done_sem, 0, 0);
    sem_init(&info_sem, 0, 0);
	sem_init(&info_done_sem, 0, 1);
    sem_init(&temp_sem, 0, 0);
    sem_init(&temp_done_sem, 0, 1);
    sem_init(&cmd_sem,0,0);
#endif
	return 0;
}

//recycle the semaphore
int destroy_pthread_sem()
{
#if defined(_WIN32)
	CloseHandle(image_sem);
	CloseHandle(image_done_sem);
    CloseHandle(i_temp_sem);
	CloseHandle(cap_sem);
	CloseHandle(cap_done_sem);
    CloseHandle(info_sem);
	CloseHandle(info_done_sem);
    CloseHandle(temp_sem);
    CloseHandle(temp_done_sem);
    CloseHandle(cmd_sem);
#elif defined(linux) || defined(unix)
	sem_destroy(&image_sem);
	sem_destroy(&image_done_sem);
    sem_destroy(&i_temp_sem);
	sem_destroy(&cap_sem);
	sem_destroy(&cap_done_sem);
    sem_destroy(&info_sem);
	sem_destroy(&info_done_sem);
    sem_destroy(&temp_sem);
    sem_destroy(&temp_done_sem);
    sem_destroy(&cmd_sem);
#endif
	return 0;
}


//create the raw frame/image frame/temperature frame's buffer
int create_data_demo(StreamFrameInfo_t* stream_frame_info)
{
    if (stream_frame_info == NULL)
    {
        printf("stream_frame_info is NULL\n");
        return -1;
    }

    if (stream_frame_info->raw_frame == NULL && stream_frame_info->image_info.data == NULL && \
        stream_frame_info->information_line.data == NULL && stream_frame_info->temp_info.data == NULL)
    {
        stream_frame_info->raw_frame = (uint8_t*)malloc(stream_frame_info->raw_byte_size);
        stream_frame_info->raw_temp_frame = (uint8_t*)malloc(stream_frame_info->raw_temp_byte_size);
        stream_frame_info->image_info.data = (uint8_t*)malloc(stream_frame_info->image_info.byte_size);
        stream_frame_info->information_line.data = (uint8_t*)malloc(stream_frame_info->information_line.byte_size);
        stream_frame_info->temp_info.data = (uint8_t*)malloc(stream_frame_info->temp_info.byte_size);
        stream_frame_info->dummy_info.data = (uint8_t*)malloc(stream_frame_info->dummy_info.byte_size);
    }
    else
    {
        printf("create_data_demo failed!\n");
        return -1;
    }

    if (stream_frame_info->raw_byte_size == 0)
    {
        stream_frame_info->raw_frame = NULL;
    }
    if (stream_frame_info->image_info.byte_size == 0)
    {
        stream_frame_info->image_info.data = NULL;
    }
    if (stream_frame_info->information_line.byte_size == 0)
    {
        stream_frame_info->information_line.data = NULL;
    }
    if (stream_frame_info->temp_info.byte_size == 0)
    {
        stream_frame_info->temp_info.data = NULL;
    }
    if (stream_frame_info->dummy_info.data == 0)
    {
        stream_frame_info->dummy_info.data = NULL;
    }
	return 0;
}

//recycle the raw frame/image frame/temperature frame's buffer
int destroy_data_demo(StreamFrameInfo_t* stream_frame_info)
{
    if (stream_frame_info == NULL)
    {
        printf("stream_frame_info is NULL\n");
        return -1;
    }

    if (stream_frame_info->raw_frame != NULL)
    {
        free(stream_frame_info->raw_frame);
        stream_frame_info->raw_frame = NULL;
    }

    if (stream_frame_info->raw_temp_frame != NULL)
    {
        free(stream_frame_info->raw_temp_frame);
        stream_frame_info->raw_temp_frame = NULL;
    }

    if (stream_frame_info->image_info.data != NULL)
    {
        free(stream_frame_info->image_info.data);
        stream_frame_info->image_info.data = NULL;
    }

    if (stream_frame_info->information_line.data != NULL)
    {
        free(stream_frame_info->information_line.data);
        stream_frame_info->information_line.data = NULL;
    }

    if (stream_frame_info->temp_info.data != NULL)
    {
        free(stream_frame_info->temp_info.data);
        stream_frame_info->temp_info.data = NULL;
    }

    if (stream_frame_info->dummy_info.data != NULL)
    {
        free(stream_frame_info->dummy_info.data);
        stream_frame_info->dummy_info.data = NULL;
    }

	return 0;
}

void load_stream_frame_info(StreamFrameInfo_t* stream_info, bool is_v4l2_driver, bool use_single_channel)
{
    //select match format
    stream_info->frame_output_format = (FrameOutputFmt_t)stream_info->product_config.camera.format;
    stream_info->width = stream_info->product_config.camera.width;
    stream_info->height = stream_info->product_config.camera.height;

    stream_info->image_info.width = stream_info->product_config.camera.width;
    stream_info->information_line.width = stream_info->product_config.camera.width;
    stream_info->temp_info.width = stream_info->product_config.camera.width;
    stream_info->dummy_info.width = stream_info->product_config.camera.width;

    stream_info->image_info.height = stream_info->product_config.camera.image_info_height;
    stream_info->information_line.height = stream_info->product_config.camera.info_line_height;
    stream_info->temp_info.height = stream_info->product_config.camera.temp_info_height;
    stream_info->dummy_info.height = stream_info->product_config.camera.dummy_info_height;

    stream_info->image_info.byte_size = stream_info->image_info.width * stream_info->image_info.height * stream_info->product_config.camera.image_info_ratio;
    stream_info->information_line.byte_size = stream_info->information_line.width * stream_info->information_line.height * stream_info->product_config.camera.info_line_ratio;
    stream_info->temp_info.byte_size = stream_info->temp_info.width * stream_info->temp_info.height * stream_info->product_config.camera.temp_line_ratio;
    stream_info->dummy_info.byte_size = stream_info->dummy_info.width * stream_info->dummy_info.height * stream_info->product_config.camera.dummy_info_ratio;

    if (!is_v4l2_driver || !stream_info->product_config.camera.v4l2_config.has_temp || use_single_channel)
    {
        stream_info->raw_byte_size = stream_info->image_info.byte_size + stream_info->information_line.byte_size + \
            stream_info->temp_info.byte_size + stream_info->dummy_info.byte_size;
    }
    else
    {
        stream_info->raw_byte_size = stream_info->image_info.byte_size + stream_info->information_line.byte_size;
        stream_info->raw_temp_byte_size = stream_info->temp_info.byte_size + stream_info->dummy_info.byte_size;
    }

    printf("stream_info image:%dx%d,information_line:%dx%d,temp:%dx%d,dummy:%dx%d\n", \
        stream_info->image_info.width, stream_info->image_info.height, \
        stream_info->information_line.width, stream_info->information_line.height, \
        stream_info->temp_info.width, stream_info->temp_info.height, \
        stream_info->dummy_info.width, stream_info->dummy_info.height);
    printf("stream_info->raw_byte_size = 0x%x\n", stream_info->raw_byte_size);
    printf("stream_info->raw_temp_byte_size = 0x%x\n", stream_info->raw_temp_byte_size);
    printf("stream_info->image_info.byte_size = 0x%x\n", stream_info->image_info.byte_size);
    printf("stream_info->information_line.byte_size = 0x%x\n", stream_info->information_line.byte_size);
    printf("stream_info->temp_info.byte_size = 0x%x\n", stream_info->temp_info.byte_size);
    printf("stream_info->dummy_info.byte_size = 0x%x\n", stream_info->dummy_info.byte_size);

    create_data_demo(stream_info);
}