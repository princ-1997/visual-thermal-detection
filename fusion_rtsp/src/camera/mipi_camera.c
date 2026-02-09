/**
 * Copyright 2021 by Guangzhou Easy EAI Technologny Co.,Ltd.
 * mipi_camera.c - MIPI/V4L2 camera implementation
 */
#include <assert.h>
#include <fcntl.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>
#include <stdint.h>
#include <errno.h>

#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>

#include <rga/RgaApi.h>

#include "camera.h"

#define PTRINT uint64_t
#define CAM_MAX_NUM     4
#define BUF_COUNT       3

typedef struct {
    int cam_chn_num;
    int video_num;
    int fd;
    enum v4l2_buf_type buff_Type;
    uint8_t *mptr[BUF_COUNT];
    uint32_t size[BUF_COUNT];
    int in_width;
    int in_height;
    uint32_t in_format;
    int rotation;
    int out_width;
    int out_height;
    int out_format;
} mipi_camera_t;

static mipi_camera_t mipiCam[CAM_MAX_NUM] = {0};

static RgaSURF_FORMAT rga_fmt(uint32_t v4l2_fmt)
{
    RgaSURF_FORMAT retFmt = RK_FORMAT_UNKNOWN;
    switch(v4l2_fmt) {
        case V4L2_PIX_FMT_YUYV: retFmt = RK_FORMAT_YUYV_422; break;
        case V4L2_PIX_FMT_YVYU: retFmt = RK_FORMAT_YVYU_422; break;
        case V4L2_PIX_FMT_NV12: retFmt = RK_FORMAT_YCbCr_420_SP; break;
        case V4L2_PIX_FMT_NV21: retFmt = RK_FORMAT_YCrCb_420_SP; break;
        case V4L2_PIX_FMT_BGR24: retFmt = RK_FORMAT_BGR_888; break;
        case V4L2_PIX_FMT_RGB24: retFmt = RK_FORMAT_RGB_888; break;
        default: retFmt = RK_FORMAT_UNKNOWN; break;
    }
    return retFmt;
}

#define FMT_NUM_PLANES 1
static int v4l2_camera_init(mipi_camera_t *cam)
{
    int ret = -1;
    char device[32] = {0};
    sprintf(device, "/dev/video%d", cam->video_num);
    cam->buff_Type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    int fd = open(device, O_RDWR | O_CLOEXEC);
    if (fd == -1) {
        printf("Error opening camera device[%s]\n", device);
        return -1;
    }

    struct v4l2_capability cap;
    memset(&cap, 0, sizeof(cap));
    ret = ioctl(fd, VIDIOC_QUERYCAP, &cap);
    if (ret < 0) {
        perror("VIDIOC_QUERYCAP fail");
        close(fd);
        return -1;
    }
    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) && !(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE)) {
        printf("Not a video capture device\n");
        close(fd);
        return -1;
    }
    if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
        printf("No streaming support\n");
        close(fd);
        return -1;
    }

    if (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)
        cam->buff_Type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    else
        cam->buff_Type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;

    struct v4l2_format vfmt;
    memset(&vfmt, 0, sizeof(vfmt));
    vfmt.type = cam->buff_Type;
    if (ioctl(fd, VIDIOC_G_FMT, &vfmt) < 0) {
        perror("VIDIOC_G_FMT fail");
        close(fd);
        return -1;
    }
    cam->in_width = vfmt.fmt.pix.width;
    cam->in_height = vfmt.fmt.pix.height;
    cam->in_format = vfmt.fmt.pix.pixelformat;
    vfmt.fmt.pix.field = V4L2_FIELD_INTERLACED;
    vfmt.fmt.pix_mp.quantization = V4L2_QUANTIZATION_FULL_RANGE;
    ret = ioctl(fd, VIDIOC_S_FMT, &vfmt);
    if (ret < 0) {
        perror("VIDIOC_S_FMT fail");
        close(fd);
        return -1;
    }

    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = BUF_COUNT;
    req.type = cam->buff_Type;
    req.memory = V4L2_MEMORY_MMAP;
    if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
        perror("VIDIOC_REQBUFS fail");
        close(fd);
        return -1;
    }

    for (unsigned int n = 0; n < req.count; n++) {
        struct v4l2_buffer mapbuffer;
        struct v4l2_plane planes[FMT_NUM_PLANES] = {0};
        memset(&mapbuffer, 0, sizeof(mapbuffer));
        mapbuffer.index = n;
        mapbuffer.type = cam->buff_Type;
        mapbuffer.memory = V4L2_MEMORY_MMAP;
        if (V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE == mapbuffer.type) {
            mapbuffer.m.planes = planes;
            mapbuffer.length = FMT_NUM_PLANES;
        }
        if (ioctl(fd, VIDIOC_QUERYBUF, &mapbuffer) < 0) {
            close(fd);
            return -1;
        }
        if (V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE == mapbuffer.type) {
            cam->size[n] = mapbuffer.m.planes[0].length;
            cam->mptr[n] = (uint8_t *)mmap(NULL, mapbuffer.m.planes[0].length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, mapbuffer.m.planes[0].m.mem_offset);
        } else {
            cam->size[n] = mapbuffer.length;
            cam->mptr[n] = (uint8_t *)mmap(NULL, mapbuffer.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, mapbuffer.m.offset);
        }
        if (MAP_FAILED == cam->mptr[n]) {
            for (int i = (int)n - 1; i >= 0; i--) munmap(cam->mptr[i], cam->size[i]);
            close(fd);
            return -1;
        }
        if (ioctl(fd, VIDIOC_QBUF, &mapbuffer) < 0) {
            for (unsigned int i = 0; i <= n; i++) munmap(cam->mptr[i], cam->size[i]);
            close(fd);
            return -1;
        }
    }

    int type = cam->buff_Type;
    if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) {
        for (int i = 0; i < BUF_COUNT; i++) munmap(cam->mptr[i], cam->size[i]);
        close(fd);
        return -1;
    }
    return fd;
}

static void v4l2_camera_exit(mipi_camera_t *cam)
{
    if (cam->fd < 0) return;
    int type = (int)cam->buff_Type;
    ioctl(cam->fd, VIDIOC_STREAMOFF, &type);
    for (int i = 0; i < BUF_COUNT; i++) munmap(cam->mptr[i], cam->size[i]);
    close(cam->fd);
    cam->fd = 0;
    usleep(200000);
}

static int v4l2_camera_getframe(mipi_camera_t *cam, char *pbuf)
{
    if (cam->fd < 0) return -1;

    struct v4l2_buffer readbuffer;
    memset(&readbuffer, 0, sizeof(readbuffer));
    readbuffer.type = cam->buff_Type;
    readbuffer.memory = V4L2_MEMORY_MMAP;
    if (V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE == readbuffer.type) {
        struct v4l2_plane planes[FMT_NUM_PLANES];
        memset(planes, 0, sizeof(planes));
        readbuffer.m.planes = planes;
        readbuffer.length = FMT_NUM_PLANES;
    }

    if (ioctl(cam->fd, VIDIOC_DQBUF, &readbuffer) < 0)
        return -1;

    rga_info_t src, dst;
    memset(&src, 0, sizeof(rga_info_t));
    memset(&dst, 0, sizeof(rga_info_t));
    src.fd = -1;
    src.virAddr = cam->mptr[readbuffer.index];
    src.mmuFlag = 1;
    src.rotation = cam->rotation;
    rga_set_rect(&src.rect, 0, 0, cam->in_width, cam->in_height, cam->in_width, cam->in_height, rga_fmt(cam->in_format));
    dst.fd = -1;
    dst.virAddr = pbuf;
    dst.mmuFlag = 1;
    rga_set_rect(&dst.rect, 0, 0, cam->out_width, cam->out_height, cam->out_width, cam->out_height, cam->out_format);
    c_RkRgaBlit(&src, &dst, NULL);

    ioctl(cam->fd, VIDIOC_QBUF, &readbuffer);
    return 0;
}

static mipi_camera_t *ptrCam(int camIndex)
{
    for (int i = 0; i < CAM_MAX_NUM; i++) {
        if (mipiCam[i].fd <= 0) continue;
        if (mipiCam[i].video_num == camIndex) return &mipiCam[i];
    }
    return NULL;
}

void mipicamera_set_format(int camIndex, int format)
{
    mipi_camera_t *p = ptrCam(camIndex);
    if (p) p->out_format = format;
}

int mipicamera_init(int camIndex, int outWidth, int outHeight, int rot)
{
    int i;
    for (i = 0; i < CAM_MAX_NUM; i++) {
        if (mipiCam[i].fd <= 0) break;
    }
    if (i >= CAM_MAX_NUM) return -1;

    memset(&mipiCam[i], 0, sizeof(mipi_camera_t));
    mipiCam[i].out_width = outWidth;
    mipiCam[i].out_height = outHeight;
    mipiCam[i].out_format = RK_FORMAT_BGR_888;
    mipiCam[i].rotation = (rot == 90) ? HAL_TRANSFORM_ROT_90 : (rot == 180) ? HAL_TRANSFORM_ROT_180 : (rot == 270) ? HAL_TRANSFORM_ROT_270 : 0;

    if (c_RkRgaInit()) return -1;
    mipiCam[i].video_num = camIndex;
    mipiCam[i].fd = v4l2_camera_init(&mipiCam[i]);
    if (mipiCam[i].fd < 0) return -1;
    return 0;
}

void mipicamera_exit(int camIndex)
{
    mipi_camera_t *p = ptrCam(camIndex);
    if (p) v4l2_camera_exit(p);
}

int mipicamera_getframe(int camIndex, char *pbuf)
{
    mipi_camera_t *p = ptrCam(camIndex);
    return p ? v4l2_camera_getframe(p, pbuf) : -1;
}
