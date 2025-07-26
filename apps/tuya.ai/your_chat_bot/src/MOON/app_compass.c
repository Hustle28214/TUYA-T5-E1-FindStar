#include "app_compass.h"
#include "tal_log.h"
#include "tal_system.h"
#include "tal_thread.h"
#include "qmc5883l.h"  // 添加磁力计驱动头文件

// 校准数据结构
static MAG_CALIBRATION_T sg_cal_data = {0};
static BOOL_T sg_calibrating = FALSE;

OPERATE_RET app_compass_init(void)
{
    // 初始化磁力计
    qmc5883l_init();
    
    // 初始化校准数据
    sg_cal_data.x_min = 10000.0f;
    sg_cal_data.x_max = -10000.0f;
    sg_cal_data.y_min = 10000.0f;
    sg_cal_data.y_max = -10000.0f;
    
    PR_DEBUG("Compass initialized");
    return OPRT_OK;
}

OPERATE_RET app_compass_get_angle(float *angle)
{
    int16_t x, y;
    float x_cal, y_cal;
    
    // 读取原始数据
    x = getX();
    y = getY();
    
    // 应用校准
    x_cal = (x - sg_cal_data.x_min) / (sg_cal_data.x_max - sg_cal_data.x_min) * 2 - 1;
    y_cal = (y - sg_cal_data.y_min) / (sg_cal_data.y_max - sg_cal_data.y_min) * 2 - 1;
    
    // 计算角度（0-360度）
    float heading = atan2f(y_cal, x_cal) * 180.0f / 3.14159265f;
    if (heading < 0) {
        heading += 360.0f;
    }
    
    // 调整角度使0度为北方
    heading = fmodf(heading + 360.0f, 360.0f);
    
    *angle = heading;
    return OPRT_OK;
}

// 校准线程函数
static void __calibration_thread(void *arg)
{
    uint32_t duration = *(uint32_t *)arg;
    uint32_t start_time = tal_system_get_millisecond();
    
    PR_NOTICE("Starting compass calibration. Rotate device slowly for %d ms", duration);
    
    while ((tal_system_get_millisecond() - start_time) < duration) {
        int16_t x = getX();
        int16_t y = getY();
        
        // 更新校准数据
        if (x < sg_cal_data.x_min) sg_cal_data.x_min = x;
        if (x > sg_cal_data.x_max) sg_cal_data.x_max = x;
        if (y < sg_cal_data.y_min) sg_cal_data.y_min = y;
        if (y > sg_cal_data.y_max) sg_cal_data.y_max = y;
        
        tal_system_sleep(50);
    }
    
    PR_NOTICE("Calibration complete. X: [%d, %d] Y: [%d, %d]", 
              (int)sg_cal_data.x_min, (int)sg_cal_data.x_max,
              (int)sg_cal_data.y_min, (int)sg_cal_data.y_max);
    
    sg_calibrating = FALSE;
    tkl_system_free(arg);
}

void app_compass_calibrate(uint32_t duration)
{
    if (sg_calibrating) {
        PR_WARN("Calibration already in progress");
        return;
    }
    
    sg_calibrating = TRUE;
    
    // 重置校准数据
    sg_cal_data.x_min = 10000.0f;
    sg_cal_data.x_max = -10000.0f;
    sg_cal_data.y_min = 10000.0f;
    sg_cal_data.y_max = -10000.0f;
    
    // 创建校准线程
    uint32_t *duration_ptr = tkl_system_malloc(sizeof(uint32_t));
    if (duration_ptr == NULL) {
        PR_ERR("Failed to allocate memory for calibration duration");
        sg_calibrating = FALSE;
        return;
    }
    *duration_ptr = duration;
    
    THREAD_CFG_T cfg = {
        .thrdname = "compass_cal",
        .priority = THREAD_PRIO_4,
        .stackDepth = 1024 * 2,
    };
    
    THREAD_HANDLE cal_thread;
    if (tal_thread_create_and_start(&cal_thread, NULL, NULL, __calibration_thread, duration_ptr, &cfg) != OPRT_OK) {
        PR_ERR("Failed to create calibration thread");
        tkl_system_free(duration_ptr);
        sg_calibrating = FALSE;
    }
}