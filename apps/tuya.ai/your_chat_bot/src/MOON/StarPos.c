#include <math.h>
#include "qmc5883l.h"
#include "InitRTC.h"
#include "tuya_cloud_types.h"
#include "tal_time_service.h"

#define PI 3.14159265358979323846
#define DEG_TO_RAD (PI / 180.0)
#define RAD_TO_DEG (180.0 / PI)

// 配置参数
static const double latitude = 30.293316; // 纬度 (度)
static const double longitude = 120.00799; // 经度 (度)
static const double align_threshold = 5; // 对准阈值 (度)
static const double mag_declination = -5.0; // 磁偏角 (度) - 根据位置调整
volatile int moon_aligned_flag = 0; // 月亮对准标志

// 获取当前设备指向的方位角（磁北）
float get_current_azimuth(void) {
    int16_t x = getX();
    int16_t y = getY();
    
    // 应用校准（如果需要）
    // qmc5883l_apply_calibration(&x, &y, NULL);
    
    // 计算方位角（弧度）
    float azimuth = atan2((float)y, (float)x);
    
    // 转换为度
    azimuth *= RAD_TO_DEG;
    
    // 规范化到0-360度
    if (azimuth < 0) {
        azimuth += 360.0;
    }
    
    // 应用磁偏角校正（从磁北转换为真北）
    azimuth += mag_declination;
    
    // 再次规范化
    azimuth = fmod(azimuth, 360.0);
    if (azimuth < 0) azimuth += 360.0;
    
    return azimuth;
}

// 检查是否对准月亮
int check_moon_alignment(void) {
    float moon_azimuth = calculate_moon_direction();
    float current_azimuth = get_current_azimuth();
    
    // 计算角度差（考虑圆周性质）
    float angle_diff = fabs(moon_azimuth - current_azimuth);
    if (angle_diff > 180.0) {
        angle_diff = 360.0 - angle_diff;
    }
    
    moon_aligned_flag = (angle_diff <= align_threshold) ? 1 : 0;
    return moon_aligned_flag;
}

// 月亮位置计算相关函数
double utc_to_julian_day(TIME_T utc_time) {
    return (utc_time / 86400.0) + 2440587.5;
}

void calculate_moon_position(double jd, double *moon_lon, double *moon_lat) {
    // 计算从J2000.0起算的儒略世纪数
    double T = (jd - 2451545.0) / 36525.0;
    
    // 计算月亮平黄经
    double Lp = fmod(218.3164477 + 481267.88123421 * T, 360.0);
    
    // 计算月亮平近点角
    double D = fmod(297.8501921 + 445267.1114034 * T, 360.0);
    
    // 计算太阳平近点角
    double M = fmod(357.5291092 + 35999.0502909 * T, 360.0);
    
    // 计算月亮平近点角
    double Mp = fmod(134.9633964 + 477198.8675055 * T, 360.0);
    
    // 计算月亮升交点平黄经
    double F = fmod(93.2720950 + 483202.0175233 * T, 360.0);
    
    // 转换为弧度
    Lp *= DEG_TO_RAD;
    D *= DEG_TO_RAD;
    M *= DEG_TO_RAD;
    Mp *= DEG_TO_RAD;
    F *= DEG_TO_RAD;
    
    // 主要周期项
    double sum_lon = 6.288774 * sin(Mp) 
                   + 1.274018 * sin(2*D - Mp)
                   + 0.658314 * sin(2*D)
                   + 0.213618 * sin(2*Mp)
                   - 0.185116 * sin(M)
                   - 0.114332 * sin(2*F);
    
    double sum_lat = 5.128189 * sin(F)
                   + 0.280606 * sin(Mp + F)
                   + 0.277693 * sin(Mp - F)
                   + 0.173238 * sin(2*D - F);
    
    // 黄经
    double lambda = Lp + sum_lon * DEG_TO_RAD;
    
    // 黄纬
    double beta = sum_lat * DEG_TO_RAD;
    
    *moon_lon = lambda;
    *moon_lat = beta;
}

void ecliptic_to_equatorial(double lon_ecl, double lat_ecl, double jd, 
                           double *ra, double *dec) {
    // 计算黄赤交角
    double T = (jd - 2451545.0) / 36525.0;
    double epsilon = (23.4392911 - 0.013004167 * T) * DEG_TO_RAD;
    
    // 转换为赤道坐标
    double sin_lon = sin(lon_ecl);
    double cos_lon = cos(lon_ecl);
    double sin_lat = sin(lat_ecl);
    double cos_lat = cos(lat_ecl);
    double sin_eps = sin(epsilon);
    double cos_eps = cos(epsilon);
    
    *ra = atan2(sin_lon * cos_eps - tan(lat_ecl) * sin_eps, cos_lon);
    *dec = asin(sin_lat * cos_eps + cos_lat * sin_eps * sin_lon);
}

double calculate_local_sidereal_time(double jd, double longitude) {
    // 计算格林尼治恒星时
    double T = (jd - 2451545.0) / 36525.0;
    double GMST = 280.46061837 + 360.98564736629 * (jd - 2451545.0)
                + 0.000387933 * T * T - T * T * T / 38710000.0;
    
    // 规范化到0-360度
    GMST = fmod(GMST, 360.0);
    if (GMST < 0) GMST += 360.0;
    
    // 转换为本地恒星时
    double LST = GMST + longitude;
    LST = fmod(LST, 360.0);
    if (LST < 0) LST += 360.0;
    
    return LST * DEG_TO_RAD; // 返回弧度
}

void calculate_alt_az(double ra, double dec, double lst, double lat, 
                      double *alt, double *az) {
    double lat_rad = lat * DEG_TO_RAD;
    
    // 计算时角
    double HA = lst - ra;
    
    // 计算高度角
    double sin_alt = sin(dec) * sin(lat_rad) 
                   + cos(dec) * cos(lat_rad) * cos(HA);
    *alt = asin(sin_alt);
    
    // 计算方位角
    double cos_az = (sin(dec) - sin(lat_rad) * sin_alt) 
                  / (cos(lat_rad) * cos(*alt));
    
    // 确保在有效范围内
    if (cos_az > 1.0) cos_az = 1.0;
    if (cos_az < -1.0) cos_az = -1.0;
    
    *az = acos(cos_az);
    
    // 根据时角确定方位角象限
    if (sin(HA) > 0) {
        *az = 2 * PI - *az;
    }
}

// 计算月亮方位角（角度制）
float calculate_moon_direction(void) {
    // 1. 获取当前UTC时间
    TIME_T utc_time = tal_time_get_posix();
    
    // 2. 计算儒略日
    double jd = utc_to_julian_day(utc_time);
    
    // 3. 计算月亮黄道坐标
    double moon_lon, moon_lat;
    calculate_moon_position(jd, &moon_lon, &moon_lat);
    
    // 4. 转换为赤道坐标
    double ra, dec;
    ecliptic_to_equatorial(moon_lon, moon_lat, jd, &ra, &dec);
    
    // 5. 计算本地恒星时
    double lst = calculate_local_sidereal_time(jd, longitude);
    
    // 6. 计算高度角和方位角
    double alt, az;
    calculate_alt_az(ra, dec, lst, latitude, &alt, &az);
    
    // 7. 转换为度并规范化
    double azimuth = az * RAD_TO_DEG;
    azimuth = fmod(azimuth + 360.0, 360.0);
    
    return (float)azimuth;
}

// 主应用逻辑
void moon_tracker_init(void) {
    // 初始化磁力计
    qmc5883l_init();
    
    // 可选：执行磁力计校准
    // calibrate_magnetometer();
    
    // 初始化RTC
    init_rtc();
}

void moon_tracking_loop(void) {
    // 检查是否对准月亮
    if (check_moon_alignment()) {
        // 已对准月亮
        printf("已对准月亮！\n");
        // 执行相关操作，如点亮LED或发送信号
    } else {
        // 获取月亮方位角
        float moon_az = calculate_moon_direction();
        float current_az = get_current_azimuth();
        
        // 计算偏差
        float angle_diff = moon_az - current_az;
        if (angle_diff > 180.0) angle_diff -= 360.0;
        if (angle_diff < -180.0) angle_diff += 360.0;
        
        printf("当前方位: %.2f°, 月亮方位: %.2f°, 偏差: %.2f°\n", 
               current_az, moon_az, angle_diff);
        
        // 根据偏差控制设备转动
        // control_motor(angle_diff);
    }
    
    // 等待一段时间再更新（月亮移动较慢）
    tkl_system_sleep(10000); // 10秒
}

// 磁力计校准函数
void calibrate_magnetometer(void) {
    printf("开始磁力计校准...\n");
    printf("请将设备在各个方向缓慢旋转2-3圈\n");
    
    // 重置校准数据
    qmc5883l_reset_calibration();
    
    TIME_T start_time = tal_time_get_posix();
    TIME_T end_time = start_time + 30; // 校准30秒
    
    while (tal_time_get_posix() < end_time) {
        if (qmc5883l_ready()) {
            int16_t x = getX();
            int16_t y = getY();
            int16_t z = getZ();
            
            qmc5883l_update_calibration(x, y, z);
        }
        tkl_system_sleep(100); // 100ms
    }
    
    printf("校准完成！\n");
    printf("X: min=%.2f, max=%.2f\n", mag_calibration.x_min, mag_calibration.x_max);
    printf("Y: min=%.2f, max=%.2f\n", mag_calibration.y_min, mag_calibration.y_max);
    printf("Z: min=%.2f, max=%.2f\n", mag_calibration.z_min, mag_calibration.z_max);
}