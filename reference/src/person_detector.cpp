#include "person_detector.h"
#include "person_detect.h"
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>

// 检测器状态
typedef struct {
    bool initialized;
    rknn_context ctx;
    
    // 配置
    int detectFps;
    int imageWidth;
    int imageHeight;
    float confidenceThreshold;
    DetectCallback callback;
    void* callbackUserData;
    
    // 检测线程
    pthread_t detectThread;
    bool threadRunning;
    
    // 帧数据缓冲（双缓冲）
    uint8_t* frameBuffer;
    int frameBufferSize;
    bool frameReady;
    pthread_mutex_t frameMutex;
    pthread_cond_t frameCond;
    
    // 检测结果
    int lastPersonCount;
    DetectResult_t lastResult;      // 最近一次检测结果（包含检测框）
    pthread_mutex_t resultMutex;    // 结果访问互斥锁
    uint32_t resultSeq;             // 结果更新序号
    
    // 帧率控制
    int frameCounter;
    int frameSkip;  // 跳帧数，根据源帧率和检测帧率计算
} PersonDetector_t;

static PersonDetector_t g_detector = {0};

// 获取当前时间（毫秒）
static long long getCurrentTimeMs(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (long long)tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

// 检测线程函数
static void* detectThreadFunc(void* arg)
{
    PersonDetector_t* detector = (PersonDetector_t*)arg;
    detect_result_group_t detectResult;
    
    printf("[PersonDetector] Detection thread started\n");
    
    while (detector->threadRunning) {
        // 等待新帧
        pthread_mutex_lock(&detector->frameMutex);
        while (!detector->frameReady && detector->threadRunning) {
            pthread_cond_wait(&detector->frameCond, &detector->frameMutex);
        }
        
        if (!detector->threadRunning) {
            pthread_mutex_unlock(&detector->frameMutex);
            break;
        }
        
        // 复制帧数据到临时缓冲区进行处理
        int width = detector->imageWidth;
        int height = detector->imageHeight;
        int bufferSize = detector->frameBufferSize;
        
        uint8_t* tempBuffer = (uint8_t*)malloc(bufferSize);
        if (tempBuffer) {
            memcpy(tempBuffer, detector->frameBuffer, bufferSize);
        }
        detector->frameReady = false;
        pthread_mutex_unlock(&detector->frameMutex);
        
        if (!tempBuffer) {
            continue;
        }
        
        // 将NV12转换为BGR供检测使用
        cv::Mat nv12Mat(height * 3 / 2, width, CV_8UC1, tempBuffer);
        cv::Mat bgrMat;
        cv::cvtColor(nv12Mat, bgrMat, cv::COLOR_YUV2BGR_NV12);
        
        // 执行检测
        long long startTime = getCurrentTimeMs();
        memset(&detectResult, 0, sizeof(detectResult));
        int ret = person_detect_run(detector->ctx, bgrMat, &detectResult);
        long long endTime = getCurrentTimeMs();
        
        if (ret == 0) {
            // 统计检测到的人数（过滤低置信度）并保存检测框
            int personCount = 0;
            DetectResult_t tempResult;
            memset(&tempResult, 0, sizeof(tempResult));
            
            for (int i = 0; i < detectResult.count && personCount < MAX_DETECT_BOXES; i++) {
                if (detectResult.results[i].prop >= detector->confidenceThreshold) {
                    tempResult.boxes[personCount].left = detectResult.results[i].box.left;
                    tempResult.boxes[personCount].top = detectResult.results[i].box.top;
                    tempResult.boxes[personCount].right = detectResult.results[i].box.right;
                    tempResult.boxes[personCount].bottom = detectResult.results[i].box.bottom;
                    tempResult.boxes[personCount].confidence = detectResult.results[i].prop;
                    
                    printf("[PersonDetector] Person %d: (%d,%d)-(%d,%d) conf=%.2f%%\n",
                           personCount + 1,
                           tempResult.boxes[personCount].left,
                           tempResult.boxes[personCount].top,
                           tempResult.boxes[personCount].right,
                           tempResult.boxes[personCount].bottom,
                           tempResult.boxes[personCount].confidence * 100);
                    personCount++;
                }
            }
            tempResult.count = personCount;
            
            // 更新检测结果（加锁保护）
            pthread_mutex_lock(&detector->resultMutex);
            detector->lastPersonCount = personCount;
            memcpy(&detector->lastResult, &tempResult, sizeof(DetectResult_t));
            detector->resultSeq++;
            pthread_mutex_unlock(&detector->resultMutex);
            
            printf("[PersonDetector] Detected %d person(s), time: %lldms\n", 
                   personCount, endTime - startTime);
            
            // 调用回调
            if (detector->callback) {
                detector->callback(personCount, detector->callbackUserData);
            }
        }
        
        free(tempBuffer);
    }
    
    printf("[PersonDetector] Detection thread stopped\n");
    return NULL;
}

int personDetector_init(const DetectorConfig_t* config)
{
    if (!config || !config->modelPath) {
        printf("[PersonDetector] Invalid config\n");
        return -1;
    }
    
    if (g_detector.initialized) {
        printf("[PersonDetector] Already initialized\n");
        return 0;
    }
    
    memset(&g_detector, 0, sizeof(g_detector));
    
    // 初始化RKNN模型
    int ret = person_detect_init(&g_detector.ctx, config->modelPath);
    if (ret != 0) {
        printf("[PersonDetector] Failed to init person_detect model: %s\n", config->modelPath);
        return -1;
    }
    printf("[PersonDetector] Model loaded: %s\n", config->modelPath);
    
    // 保存配置
    g_detector.detectFps = config->detectFps > 0 ? config->detectFps : 2;
    g_detector.imageWidth = config->imageWidth;
    g_detector.imageHeight = config->imageHeight;
    g_detector.confidenceThreshold = config->confidenceThreshold > 0 ? config->confidenceThreshold : 0.4f;
    g_detector.callback = config->callback;
    g_detector.callbackUserData = config->callbackUserData;
    
    // 计算跳帧数（假设源视频30fps）
    g_detector.frameSkip = 30 / g_detector.detectFps;
    g_detector.frameCounter = 0;
    
    // 分配帧缓冲区（NV12格式：1.5倍像素数）
    g_detector.frameBufferSize = g_detector.imageWidth * g_detector.imageHeight * 3 / 2;
    g_detector.frameBuffer = (uint8_t*)malloc(g_detector.frameBufferSize);
    if (!g_detector.frameBuffer) {
        printf("[PersonDetector] Failed to allocate frame buffer\n");
        person_detect_release(g_detector.ctx);
        return -1;
    }
    
    // 初始化同步原语
    pthread_mutex_init(&g_detector.frameMutex, NULL);
    pthread_cond_init(&g_detector.frameCond, NULL);
    pthread_mutex_init(&g_detector.resultMutex, NULL);
    
    // 启动检测线程
    g_detector.threadRunning = true;
    ret = pthread_create(&g_detector.detectThread, NULL, detectThreadFunc, &g_detector);
    if (ret != 0) {
        printf("[PersonDetector] Failed to create detection thread\n");
        free(g_detector.frameBuffer);
        person_detect_release(g_detector.ctx);
        pthread_mutex_destroy(&g_detector.frameMutex);
        pthread_cond_destroy(&g_detector.frameCond);
        return -1;
    }
    
    g_detector.initialized = true;
    printf("[PersonDetector] Initialized, detectFps=%d, frameSkip=%d\n", 
           g_detector.detectFps, g_detector.frameSkip);
    
    return 0;
}

int personDetector_submitFrame(const uint8_t* frameData, int width, int height)
{
    if (!g_detector.initialized || !frameData) {
        return -1;
    }
    
    // 帧率控制：跳帧处理
    g_detector.frameCounter++;
    if (g_detector.frameCounter < g_detector.frameSkip) {
        return 0;  // 跳过此帧
    }
    g_detector.frameCounter = 0;
    
    // 尝试获取锁，如果获取不到说明检测线程正在处理，跳过此帧
    if (pthread_mutex_trylock(&g_detector.frameMutex) != 0) {
        return 0;  // 检测线程忙，跳过此帧
    }
    
    // 如果上一帧还没处理完，也跳过
    if (g_detector.frameReady) {
        pthread_mutex_unlock(&g_detector.frameMutex);
        return 0;
    }
    
    // 复制帧数据
    int size = width * height * 3 / 2;
    if (size <= g_detector.frameBufferSize) {
        memcpy(g_detector.frameBuffer, frameData, size);
        g_detector.frameReady = true;
        pthread_cond_signal(&g_detector.frameCond);
    }
    
    pthread_mutex_unlock(&g_detector.frameMutex);
    return 0;
}

void personDetector_setFps(int fps)
{
    if (fps > 0 && fps <= 30) {
        g_detector.detectFps = fps;
        g_detector.frameSkip = 30 / fps;
        g_detector.frameCounter = 0;
        printf("[PersonDetector] FPS set to %d, frameSkip=%d\n", fps, g_detector.frameSkip);
    }
}

int personDetector_getFps(void)
{
    return g_detector.detectFps;
}

int personDetector_getLastPersonCount(void)
{
    return g_detector.lastPersonCount;
}

int personDetector_getLastResult(DetectResult_t* result)
{
    if (!result || !g_detector.initialized) {
        return 0;
    }
    
    pthread_mutex_lock(&g_detector.resultMutex);
    memcpy(result, &g_detector.lastResult, sizeof(DetectResult_t));
    pthread_mutex_unlock(&g_detector.resultMutex);
    
    return result->count;
}

int personDetector_getLastResultWithSeq(DetectResult_t* result, uint32_t* seq)
{
    if (!result || !seq || !g_detector.initialized) {
        return 0;
    }
    
    pthread_mutex_lock(&g_detector.resultMutex);
    memcpy(result, &g_detector.lastResult, sizeof(DetectResult_t));
    *seq = g_detector.resultSeq;
    pthread_mutex_unlock(&g_detector.resultMutex);
    
    return result->count;
}

void personDetector_release(void)
{
    if (!g_detector.initialized) {
        return;
    }
    
    printf("[PersonDetector] Releasing...\n");
    
    // 停止检测线程
    pthread_mutex_lock(&g_detector.frameMutex);
    g_detector.threadRunning = false;
    pthread_cond_signal(&g_detector.frameCond);
    pthread_mutex_unlock(&g_detector.frameMutex);
    
    pthread_join(g_detector.detectThread, NULL);
    
    // 释放资源
    pthread_mutex_destroy(&g_detector.frameMutex);
    pthread_cond_destroy(&g_detector.frameCond);
    pthread_mutex_destroy(&g_detector.resultMutex);
    
    if (g_detector.frameBuffer) {
        free(g_detector.frameBuffer);
        g_detector.frameBuffer = NULL;
    }
    
    person_detect_release(g_detector.ctx);
    
    g_detector.initialized = false;
    printf("[PersonDetector] Released\n");
}
