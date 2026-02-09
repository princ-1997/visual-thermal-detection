/**
 * 热成像与可见光融合配置
 */
#ifndef __FUSION_CONFIG_H__
#define __FUSION_CONFIG_H__

#include <glib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 融合模式 */
typedef enum {
    FUSION_MODE_ALPHA_BLEND = 0,   /* Alpha 叠加 */
    FUSION_MODE_EDGE_OVERLAY = 1   /* 边缘叠加（默认）*/
} FusionMode_t;

/* 融合参数 */
typedef struct {
    FusionMode_t mode;     /* 融合模式 */
    float alpha;           /* Alpha 叠加时的透明度 (0~1) */
    int canny_low;         /* Canny 边缘检测低阈值 */
    int canny_high;        /* Canny 边缘检测高阈值 */
} FusionParams_t;

/* 人体检测模型默认路径（build.sh 会将 fusion_rtsp/person_detect.model 复制到 Release）*/
#define DEFAULT_DETECT_MODEL "./person_detect.model"

/* 人体检测配置 */
typedef struct {
    gboolean enabled;           /* 是否启用检测 */
    const char* model_path;     /* 模型文件路径 */
    int detect_fps;            /* 检测帧率 (1-30) */
    float confidence_threshold;/* 置信度阈值 (0~1) */
} DetectConfig_t;

#ifdef __cplusplus
}
#endif

#endif /* __FUSION_CONFIG_H__ */
