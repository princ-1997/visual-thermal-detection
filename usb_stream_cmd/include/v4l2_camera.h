#ifndef V4L2_CAMERA_H_
#define V4L2_CAMERA_H_

#include "data.h"
#include "libirparse.h"
#include "libirv4l2.h"

//stream thread, use v4l2 framework to get the raw frame, cut to temperature and image, and then send to other thread
void *v4l2_stream_function(void *threadarg);

//stream thread, mipi 2vc/usb 2interface use v4l2 framework to get the raw frame, cut to temperature and image, and then send to other thread
void* v4l2_double_channel_stream_function(void* threadarg);

// stream thread, mipi 2vc/usb 2interface use v4l2 framework to get the raw frame
// v4l2_temp_channel_stream_function for get temp raw data
// v4l2_image_channel_stream_function for get image raw data, include infoline
void* v4l2_temp_channel_stream_function(void* threadarg);
void* v4l2_image_channel_stream_function(void* threadarg);

void stop_stream();

#endif