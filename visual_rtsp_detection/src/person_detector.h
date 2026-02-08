#ifndef __PERSON_DETECTOR_H__
#define __PERSON_DETECTOR_H__

#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 最大检测框数量
#define MAX_DETECT_BOXES 16

// 检测框结构
typedef struct {
    int left;
    int top;
    int right;
    int bottom;
    float confidence;   // 置信度 (0.0 - 1.0)
} DetectBox_t;

// 检测结果结构
typedef struct {
    int count;                          // 检测到的目标数量
    DetectBox_t boxes[MAX_DETECT_BOXES]; // 检测框数组
} DetectResult_t;

// 检测结果回调函数类型
typedef void (*DetectCallback)(int personCount, void* userData);

// 检测器配置
typedef struct {
    const char* modelPath;      // 模型文件路径
    int detectFps;              // 检测帧率（如2表示每秒检测2帧）
    int imageWidth;             // 图像宽度
    int imageHeight;            // 图像高度
    float confidenceThreshold;  // 置信度阈值
    DetectCallback callback;    // 检测结果回调
    void* callbackUserData;     // 回调用户数据
} DetectorConfig_t;

// 初始化人体检测器
// 返回: 0成功, -1失败
int personDetector_init(const DetectorConfig_t* config);

// 提交帧数据进行检测（异步，不阻塞）
// frameData: NV12格式的帧数据
// 返回: 0成功提交, -1检测器未初始化或队列满
int personDetector_submitFrame(const uint8_t* frameData, int width, int height);

// 设置检测帧率
void personDetector_setFps(int fps);

// 获取当前检测帧率
int personDetector_getFps(void);

// 获取最近一次检测到的人数
int personDetector_getLastPersonCount(void);

// 获取最近一次检测结果（包含检测框信息）
// result: 输出参数，存放检测结果
// 返回: 检测到的目标数量
int personDetector_getLastResult(DetectResult_t* result);

// 获取最近一次检测结果及序号（用于判断结果是否更新）
// seq: 输出参数，结果更新序号
// 返回: 检测到的目标数量
int personDetector_getLastResultWithSeq(DetectResult_t* result, uint32_t* seq);

// 释放人体检测器资源
void personDetector_release(void);

#ifdef __cplusplus
}
#endif

#endif // __PERSON_DETECTOR_H__
