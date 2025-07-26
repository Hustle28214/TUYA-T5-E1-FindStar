#ifndef APP_COMPASS_H
#define APP_COMPASS_H

#include "tuya_cloud_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// 磁力计校准数据结构
typedef struct {
    float x_min;
    float x_max;
    float y_min;
    float y_max;
} MAG_CALIBRATION_T;

/**
 * @brief 初始化指南针模块
 */
OPERATE_RET app_compass_init(void);  // 改为小写 void

/**
 * @brief 获取当前方向角度（0-360度）
 * @param angle 返回的角度值
 * @return 操作结果
 */
OPERATE_RET app_compass_get_angle(float *angle);

/**
 * @brief 执行磁力计校准
 * @param duration 校准持续时间（毫秒）
 */
void app_compass_calibrate(uint32_t duration);  // 改为小写 void

#ifdef __cplusplus
}
#endif

#endif /* APP_COMPASS_H */