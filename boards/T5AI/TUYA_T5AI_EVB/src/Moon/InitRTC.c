#include "tkl_rtc.h"
#include "tuya_cloud_types.h"
#include "tuya_error_code.h"
OPERATE_RET InitRTC(void){
    OPERATE_RET ret = tkl_rtc_init();
    if(ret!=OPRT_OK)
    {   //TODO:串口打印失败日志
        printf("RTC init failed! Error: %d\n", ret);
        return ret;
    }

    TIME_T current_time;
    ret = tkl_rtc_time_get(&current_time);
    if (OPRT_OK == ret) {
        printf("RTC initialized. Current time: 0x%lX\n", current_time);
    } else {
        printf("Warning: RTC time get failed after init\n");
    }
    
    return OPRT_OK;
}

OPERATE_RET GetRTCTime(TIME_T *time_out) {
    if (NULL == time_out) {
        return OPRT_INVALID_PARM; // (-0x0002) //-2, Invalid parameter
    }
    return tkl_rtc_time_get(time_out);
}

TIME_T GetRTCTimeValue(void) {
    TIME_T time_val = 0;
    OPERATE_RET ret = tkl_rtc_time_get(&time_val);
    return (OPRT_OK == ret) ? time_val : 0; // 错误时返回0
}

void DeinitRTC(void){tkl_rtc_deinit();}