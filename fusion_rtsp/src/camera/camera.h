/**
 * Copyright 2021 by Guangzhou Easy EAI Technologny Co.,Ltd.
 * camera.h - MIPI/USB camera interface
 */
#ifndef CAMERA_H
#define CAMERA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <linux/videodev2.h>
#include <rga/RgaApi.h>

/* mipi camera */
int mipicamera_init(int camIndex, int width, int height, int rot);
void mipicamera_exit(int camIndex);
int mipicamera_getframe(int camIndex, char *pbuf);
void mipicamera_set_format(int camIndex, int format);

#ifdef __cplusplus
}
#endif
#endif
