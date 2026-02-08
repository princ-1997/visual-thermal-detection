//=====================  C++  =====================
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <stdint.h>
#include <string.h>
//=====================   C   =====================
#include "system.h"
#include "config.h"
//=====================  SDK  =====================
#include "system_opt.h"
//=====================  PRJ  =====================
#include "appSource.h"
#include "camera/camera.h"
#include "person_detector.h"
//===================== OpenCV =====================
#include <opencv2/opencv.hpp>

// 人体检测是否已初始化标志
static gboolean g_detectorInited = FALSE;
static uint32_t g_lastResultSeq = 0;
static DetectResult_t g_lastResult = {0};
static int g_lastResultCount = 0;

// 预定义颜色表（BGR格式，用于绘制不同检测框）
static const cv::Scalar g_boxColors[] = {
    cv::Scalar(0, 0, 255),     // 红色
    cv::Scalar(255, 0, 0),     // 蓝色
    cv::Scalar(0, 255, 0),     // 绿色
    cv::Scalar(0, 255, 255),   // 黄色
    cv::Scalar(255, 0, 255),   // 紫色
    cv::Scalar(255, 255, 0),   // 青色
    cv::Scalar(0, 128, 255),   // 橙色
    cv::Scalar(255, 128, 0),   // 浅蓝
};
static const int g_colorCount = sizeof(g_boxColors) / sizeof(g_boxColors[0]);

// 在NV12帧上绘制检测框和置信度
static void drawDetectionBoxes(uint8_t* nv12Data, int width, int height, 
                               const DetectResult_t* result)
{
    if (!nv12Data || !result || result->count == 0) {
        return;
    }
    
    // 将NV12转换为BGR进行绘制
    cv::Mat nv12Mat(height * 3 / 2, width, CV_8UC1, nv12Data);
    cv::Mat bgrMat;
    cv::cvtColor(nv12Mat, bgrMat, cv::COLOR_YUV2BGR_NV12);
    
    // 绘制每个检测框
    for (int i = 0; i < result->count; i++) {
        const DetectBox_t* box = &result->boxes[i];
        
        // 选择颜色（循环使用颜色表）
        cv::Scalar color = g_boxColors[i % g_colorCount];
        
        // 绘制矩形框
        cv::rectangle(bgrMat, 
                      cv::Point(box->left, box->top), 
                      cv::Point(box->right, box->bottom), 
                      color, 2);
        
        // 准备标签文本: "person XX.X%"
        char label[32];
        snprintf(label, sizeof(label), "person %.1f%%", box->confidence * 100);
        
        // 计算文本大小
        int baseline = 0;
        double fontScale = 0.6;
        int thickness = 2;
        cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 
                                            fontScale, thickness, &baseline);
        
        // 绘制文本背景（半透明效果）
        int textY = box->top - 5;
        if (textY < textSize.height) {
            textY = box->top + textSize.height + 5;  // 如果顶部空间不足，放到框内
        }
        
        // 绘制文本
        cv::putText(bgrMat, label, 
                    cv::Point(box->left, textY),
                    cv::FONT_HERSHEY_SIMPLEX, fontScale, color, thickness);
    }
    
    // 将BGR转换回NV12
    cv::Mat nv12Out;
    cv::cvtColor(bgrMat, nv12Out, cv::COLOR_BGR2YUV_I420);
    
    // I420和NV12的Y平面相同，只需要重排UV平面
    // I420: YYYYYYYY UU VV
    // NV12: YYYYYYYY UVUV
    int ySize = width * height;
    int uvSize = ySize / 4;
    
    // 复制Y平面
    memcpy(nv12Data, nv12Out.data, ySize);
    
    // 交错U和V平面转换为NV12格式
    uint8_t* uPlane = nv12Out.data + ySize;
    uint8_t* vPlane = uPlane + uvSize;
    uint8_t* uvDst = nv12Data + ySize;
    
    for (int i = 0; i < uvSize; i++) {
        uvDst[i * 2] = uPlane[i];
        uvDst[i * 2 + 1] = vPlane[i];
    }
}

static void appSrc_Init(gpointer data)
{
    SrcCfg_t *pSourceCfg = (SrcCfg_t *)data;
    if(NULL == pSourceCfg){
        return ;
    }
    if(pSourceCfg->videoDesc.bInited){
        return ;
    }
    
    const char *str = pSourceCfg->loaction;
    int camIndex = atoi(str+strlen(str)-2);
    if(0 == mipicamera_init(camIndex, pSourceCfg->videoDesc.width, pSourceCfg->videoDesc.height, 0)){
        mipicamera_set_format(camIndex, RK_FORMAT_YCbCr_420_SP);
        pSourceCfg->videoDesc.timestamp = 0;

        pSourceCfg->videoDesc.bInited = TRUE;
    }
}

static void appSrc_unInit(gpointer data)
{
    SrcCfg_t *pSourceCfg = (SrcCfg_t *)data;
    if(NULL == pSourceCfg){
        return ;
    }
    
    const char *str = pSourceCfg->loaction;
    int camIndex = atoi(str+strlen(str)-2);
    mipicamera_exit(camIndex);
    
    pSourceCfg->videoDesc.bInited = FALSE;
}

// 人体检测结果回调函数
static void onPersonDetected(int personCount, void* userData)
{
    // 这里可以添加检测结果的处理逻辑
    // 例如：触发告警、记录日志、发送通知等
    if (personCount > 0) {
        printf("[App] Person detected callback: %d person(s)\n", personCount);
    }
}

// 初始化人体检测器
int initPersonDetector(const char* modelPath, int detectFps, int width, int height)
{
    if (g_detectorInited) {
        return 0;
    }
    
    DetectorConfig_t config;
    config.modelPath = modelPath;
    config.detectFps = detectFps;
    config.imageWidth = width;
    config.imageHeight = height;
    config.confidenceThreshold = 0.4f;
    config.callback = onPersonDetected;
    config.callbackUserData = NULL;
    
    int ret = personDetector_init(&config);
    if (ret == 0) {
        g_detectorInited = TRUE;
        printf("[App] Person detector initialized, fps=%d\n", detectFps);
    }
    return ret;
}

// 释放人体检测器
void releasePersonDetector(void)
{
    if (g_detectorInited) {
        personDetector_release();
        g_detectorInited = FALSE;
    }
}

// 设置检测帧率
void setPersonDetectFps(int fps)
{
    if (g_detectorInited) {
        personDetector_setFps(fps);
    }
}

// 获取最近检测到的人数
int getLastPersonCount(void)
{
    if (g_detectorInited) {
        return personDetector_getLastPersonCount();
    }
    return 0;
}

static void need_data(GstElement *appsrc, guint unused, gpointer user_data)
{
    SrcCfg_t *pSourceCfg = (SrcCfg_t *)user_data;
    if(NULL == pSourceCfg){
        return ;
    }
    if(pSourceCfg->videoDesc.bInited){
        GstFlowReturn ret;
        
        guint size = 1.5 * pSourceCfg->videoDesc.width * pSourceCfg->videoDesc.height;    
        GstBuffer *buffer = gst_buffer_new_allocate (NULL, size, NULL);
        
        const char *str = pSourceCfg->loaction;
        int camIndex = atoi(str+strlen(str)-2);
        GstMapInfo map;
        if (gst_buffer_map(buffer, &map, GST_MAP_WRITE)){
            mipicamera_getframe(camIndex, (char *)map.data);
            
            // 异步提交帧数据进行人体检测（不阻塞视频流）
            if (g_detectorInited) {
                personDetector_submitFrame((const uint8_t*)map.data, 
                                           pSourceCfg->videoDesc.width, 
                                           pSourceCfg->videoDesc.height);
                
                // 获取最新检测结果并按更新序号缓存
                DetectResult_t detectResult;
                uint32_t seq = 0;
                int count = personDetector_getLastResultWithSeq(&detectResult, &seq);
                if (seq != g_lastResultSeq) {
                    g_lastResultSeq = seq;
                    g_lastResultCount = count;
                    if (count > 0) {
                        memcpy(&g_lastResult, &detectResult, sizeof(DetectResult_t));
                    }
                }

                // 若最近一次识别有人，保持识别框到下一个识别帧
                if (g_lastResultCount > 0) {
                    drawDetectionBoxes((uint8_t*)map.data, 
                                       pSourceCfg->videoDesc.width, 
                                       pSourceCfg->videoDesc.height, 
                                       &g_lastResult);
                }
            }
            
            gst_buffer_unmap(buffer, &map);	//解除映射
        }
        
        /* increment the timestamp every 1/framerate second */
        GST_BUFFER_PTS(buffer) = pSourceCfg->videoDesc.timestamp;
        GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale_int(1, GST_SECOND, pSourceCfg->videoDesc.framerate);
        pSourceCfg->videoDesc.timestamp += GST_BUFFER_DURATION(buffer);
        
        g_signal_emit_by_name(appsrc, "push-buffer", buffer, &ret);
        gst_buffer_unref(buffer);
    }
}


/* called when a new media pipeline is constructed. We can query the
 * pipeline and configure our appsrc */
void media_configure(GstRTSPMediaFactory * factory, GstRTSPMedia *media, gpointer user_data)
{
    SrcCfg_t *pSourceCfg = (SrcCfg_t *)user_data;
    if(NULL == pSourceCfg){
        return ;
    }
    
    /* get the element used for providing the streams of the media */
    GstElement *element = gst_rtsp_media_get_element(media);
    /* get our appsrc, we named it 'mysrc' with the name property */
    GstElement *appsrc = gst_bin_get_by_name_recurse_up(GST_BIN(element), "videosrc");

    /* this instructs appsrc that we will be dealing with timed buffer */
    gst_util_set_object_arg(G_OBJECT(appsrc), "format", "time");
    /* configure the caps of the video */
    GstCaps *caps = gst_caps_new_simple ("video/x-raw",
          "format",    G_TYPE_STRING, pSourceCfg->videoDesc.videoSrcType,
          "width",     G_TYPE_INT,    pSourceCfg->videoDesc.width,
          "height",    G_TYPE_INT,    pSourceCfg->videoDesc.height,
          "framerate", GST_TYPE_FRACTION, pSourceCfg->videoDesc.framerate, 1, NULL);
    if(caps){
        g_object_set(G_OBJECT(appsrc), "caps", caps, NULL);
        gst_caps_unref(caps);
    }
    g_object_set(G_OBJECT(appsrc), "is-live", TRUE, "block", TRUE, NULL);
    
    appSrc_Init(pSourceCfg);

    /* make sure ther datais freed when the media is gone */
    g_object_set_data_full(G_OBJECT(media), "my-extra-data", user_data, appSrc_unInit);
    
    /* install the callback that will be called when a buffer is needed */
    g_signal_connect(appsrc, "need-data", (GCallback)need_data, user_data);
    
    gst_object_unref(appsrc);
    gst_object_unref(element);
}


