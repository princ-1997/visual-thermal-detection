/**
 * 热成像与可见光融合配置
 */
#ifndef __FUSION_CONFIG_H__
#define __FUSION_CONFIG_H__

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

#ifdef __cplusplus
}
#endif

#endif /* __FUSION_CONFIG_H__ */
