#include "math.h"
#include "InitRTC.h"
#include "tuya_cloud_types.h"
#include <time.h>
#include <stdio.h>
#define PI 3.141592653589793
#define DEG_TO_RAD (PI / 180.0)
#define RAD_TO_DEG (180.0 / PI)
static const double latitude = 30.293316;// 目前GNSS效果太差。写死。考虑附录二的那个城市经纬度获取方法
static const double longitude = 120.00799;
static const double align_threshold = 5;
volatile int moon_aligned_flag = 0;
// 获取时间(TIME_T 格式，十六进制)
TIME_T time_used = GetRTCTime();
// 1. moon 计算月亮的位置
// 简化版月亮位置计算
void calculate_moon_position(time_t timestamp, double *azimuth, double *elevation) {
    // 将时间戳转换为UTC时间
    struct tm *timeinfo = gmtime(&timestamp);
    // 计算儒略日
    int year = timeinfo->tm_year + 1900;
    int month = timeinfo->tm_mon + 1;
    int day = timeinfo->tm_mday;
    double hour = timeinfo->tm_hour + timeinfo->tm_min / 60.0 + timeinfo->tm_sec / 3600.0;
    if (month <= 2) {
        year--;
        month += 12;
    }
    
    int a = year / 100;
    int b = 2 - a + a / 4;
    double jd = (int)(365.25 * (year + 4716)) + (int)(30.6001 * (month + 1)) + day + b - 1524.5;
    jd += hour / 24.0;
    
    // 计算自J2000.0起的天数
    double t = (jd - 2451545.0) / 36525.0;
    
    // 计算月亮平黄经（简化）
    double l = 218.316 + 481267.8813 * t;
    
    // 计算月亮黄纬（简化）
    double b_moon = 5.128 * sin(DEG_TO_RAD * (93.272 + 483202.019 * t));
    
    // 计算月亮黄经（简化）
    double l_prime = 3.8104 + 8399.7091 * t;
    double d = 297.850 + 445267.1115 * t;
    double lambda = l + 6.289 * sin(DEG_TO_RAD * l_prime);
    
    // 转换为赤道坐标
    double epsilon = 23.439 - 0.0000004 * t;
    double alpha = atan2(sin(DEG_TO_RAD * lambda) * cos(DEG_TO_RAD * epsilon) - 
                  tan(DEG_TO_RAD * b_moon) * sin(DEG_TO_RAD * epsilon),
                  cos(DEG_TO_RAD * lambda));
    double delta = asin(sin(DEG_TO_RAD * b_moon) * cos(DEG_TO_RAD * epsilon) + 
                     cos(DEG_TO_RAD * b_moon) * sin(DEG_TO_RAD * epsilon) * sin(DEG_TO_RAD * lambda));
    
    // 计算本地恒星时
    double gmst = 280.46061837 + 360.98564736629 * (jd - 2451545.0);
    gmst = fmod(gmst, 360.0);
    if (gmst < 0) gmst += 360.0;
    
    double lst = gmst + LONGITUDE;
    lst = fmod(lst, 360.0);
    if (lst < 0) lst += 360.0;
    
    // 计算时角
    double ha = lst - alpha * RAD_TO_DEG;
    ha = fmod(ha, 360.0);
    if (ha < 0) ha += 360.0;
    
    // 计算方位角和高度角
    double lat_rad = LATITUDE * DEG_TO_RAD;
    double ha_rad = ha * DEG_TO_RAD;
    double delta_rad = delta;
    
    double sin_alt = sin(lat_rad) * sin(delta_rad) + 
                    cos(lat_rad) * cos(delta_rad) * cos(ha_rad);
    double alt = asin(sin_alt);
    
    double cos_az = (sin(delta_rad) - sin(lat_rad) * sin(alt)) / 
                   (cos(lat_rad) * cos(alt));
    double az = acos(cos_az);
    
    if (sin(ha_rad) > 0) {
        az = 2 * PI - az;
    }
    
    *azimuth = fmod(az * RAD_TO_DEG + 180.0, 360.0); // 转换为0-360度
    *elevation = alt * RAD_TO_DEG;
}
// 从磁力计获取方位角逻辑示例
// double get_magnetometer_azimuth() {
//     // 读取磁力计原始数据
//     MagnetometerRaw raw = QMC5883L_ReadRaw();
    
//     // 计算方位角（0-360度，0=北，90=东）
//     double azimuth = atan2(raw.Y, raw.X) * RAD_TO_DEG;
    
//     // 转换为0-360度范围
//     if (azimuth < 0) {
//         azimuth += 360.0;
//     }
    
//     return azimuth;
// }

// 检查月亮方位和磁力计方向是否对齐
void check_alignment() {
    // 使用封装函数获取RTC时间
    TIME_T rtc_time = GetRTCTimeValue();
    
    // 检查时间是否有效
    if (rtc_time == 0) {
        printf("Error: Invalid RTC time\n");
        moon_aligned_flag = 0;
        return;
    }
    
    // 转换为time_t类型
    time_t timestamp = (time_t)rtc_time;
    
    // 计算月亮方位角
    double moon_azimuth, moon_elevation;
    calculate_moon_position(timestamp, &moon_azimuth, &moon_elevation);
    
    // 获取磁力计方位角
    double mag_azimuth = get_magnetometer_azimuth();
    
    // 计算角度差（考虑圆周）
    double diff = fabs(moon_azimuth - mag_azimuth);
    if (diff > 180.0) {
        diff = 360.0 - diff;
    }
    
    // 检查是否在误差范围内对齐
    moon_aligned_flag = (diff <= ALIGNMENT_THRESHOLD) ? 1 : 0;
    
    // 调试输出
    printf("Time: %ld, Moon Az: %.2f°, Mag Az: %.2f°, Diff: %.2f°, Aligned: %d\n",
           timestamp, moon_azimuth, mag_azimuth, diff, moon_aligned_flag);
}