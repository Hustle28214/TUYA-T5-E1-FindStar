#ifndef __INITRTC_H__
#define __INITRTC_H__

#include "tuya_cloud_types.h"
#include "tkl_rtc.h"

void InitRTC(void);
void DeinitRTC(void);
TIME_T GetRTCTimeValue(void);
OPERATE_RET GetRTCTime(TIME_T *time_out);


#endif