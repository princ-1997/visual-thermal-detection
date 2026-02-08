#ifndef UVC_CAMERA_H_
#define UVC_CAMERA_H_

#include "data.h"
#include "libiruvc.h"
#include "libirparse.h"

//stream thread,use UVC framework to get the raw frame, cut to temperature and image, and then send to other thread
void* uvc_stream_function(void* threadarg);

void stop_stream();

#endif