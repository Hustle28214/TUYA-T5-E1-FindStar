
/**
 * @file app_display.c
 * @brief Handle display initialization and message processing
 */

 #include "tuya_cloud_types.h"
 #include "app_display.h"
 #include "tuya_lvgl.h"
 #include <math.h>      // 添加数学函数支持 (atan2f, fmodf)
 #include "tal_log.h"
 #include "tal_queue.h"
 #include "tal_thread.h"
 #include "tal_system.h"
 #include "tkl_memory.h"
 #include "tkl_i2c.h"
 #include "tkl_pinmux.h"
 #include "tkl_output.h"
 #include "tkl_gpio.h"
 #include "lvgl.h"
 #include "tal_time_service.h" // 添加时间服务头文件
 // 直接包含背景图实现文件
 #include "Compass/background.c"
 // MPU6050寄存器定义
 #define MPU6050_ADDR             0x68
 #define MPU6050_REG_PWR_MGMT_1   0x6B
 #define MPU6050_REG_ACCEL_XOUT_H 0x3B
 #define MPU6050_REG_ACCEL_CONFIG 0x1C
 // 加速度量程设置
 enum ACCEL_FS {
     AFS_2G = 0,
     AFS_4G,
     AFS_8G,
     AFS_16G
 };
 // 月亮位置计算参数
 #define PI 3.14159265358979323846
 #define DEG_TO_RAD (PI / 180.0)
 #define RAD_TO_DEG (180.0 / PI)
 // 当前位置（设备所在地）
 static const double current_latitude = 30.293316;
 static const double current_longitude = 120.00799;
 // 家乡位置预设
 static const double home_latitude = 22.279142; 
 static const double home_longitude = 113.529025; 
 // 全局变量
 static float sg_accel_scale = 1.0f;
 static float sg_current_angle = 0.0f;
 static uint32_t sg_last_update = 0;
 // LVGL对象
 static lv_obj_t *sg_background_img = NULL;
 static lv_obj_t *sg_moon_label = NULL;
 static lv_obj_t *sg_home_label = NULL; 
 static lv_obj_t *sg_azimuth_label = NULL;
 // TODO: 后续添加QMC5883L的使用，现场坏掉的磁力计也换一下
 /***********************************************************
 ***********************typedef define***********************
 ***********************************************************/
 // 使用联合体处理不同类型的消息
 typedef struct {
     TY_DISPLAY_TYPE_E type;
     union {
         struct {
             int len;
             char *data;
         } dynamic_data;
         struct {
             int16_t rotation;
         } rotation_data;
     } payload;
 } DISPLAY_MSG_T;
 
 typedef struct {
     QUEUE_HANDLE queue_hdl;
     THREAD_HANDLE thrd_hdl;
 } TUYA_DISPLAY_T;
 
 /***********************************************************
 ***********************variable define**********************
 ***********************************************************/
 static TUYA_DISPLAY_T sg_display = {0};
 static lv_timer_t *sg_compass_timer = NULL;  // 指南针定时器
 
 /***********************************************************
 ***********************function define**********************
 ***********************************************************/
 
 // MPU6050 I2C读写函数
 static OPERATE_RET mpu6050_write_reg(uint8_t reg, uint8_t value)
 {
     uint8_t data[2] = {reg, value};
     return tkl_i2c_master_send(0, MPU6050_ADDR, data, 2, TRUE);
 }
 
 static OPERATE_RET mpu6050_read_reg(uint8_t reg, uint8_t *value, uint16_t len)
 {
     OPERATE_RET ret = tkl_i2c_master_send(0, MPU6050_ADDR, &reg, 1, FALSE);
     if (ret != OPRT_OK) {
         return ret;
     }
     return tkl_i2c_master_receive(0, MPU6050_ADDR, value, len, TRUE);
 }
 
 // MPU6050初始化
 OPERATE_RET app_mpu6050_init(void)
 {
     OPERATE_RET ret = OPRT_OK;
     tkl_io_pinmux_config(I2C_SCL_PIN, TUYA_IIC0_SCL);
     tkl_io_pinmux_config(I2C_SDA_PIN, TUYA_IIC0_SDA);
     TUYA_IIC_BASE_CFG_T i2c_cfg = {
         .role = TUYA_IIC_MODE_MASTER,
         .speed = I2C_SPEED, // 400kHz
         .addr_width = QMC_ADDR_MODE,
     };
     tkl_i2c_init(0, &i2c_cfg);
     ret = mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1, 0x00);
     if (ret != OPRT_OK) {
         PR_ERR("Failed to wake up MPU6050: %d", ret);
         return ret;
     }
     // 设置加速度计量程为±2g
     ret = mpu6050_write_reg(MPU6050_REG_ACCEL_CONFIG, AFS_2G << 3);
     if (ret != OPRT_OK) {
         PR_ERR("Failed to set accelerometer range: %d", ret);
         return ret;
     }
     sg_accel_scale = 16384.0f; // 2g量程的比例因子
     PR_DEBUG("MPU6050 initialized");
     sg_last_update = tal_system_get_millisecond();
     return OPRT_OK;
 }
 
 // 从MPU6050读取原始加速度数据
 static OPERATE_RET mpu6050_read_accel(int16_t *accel_x, int16_t *accel_y, int16_t *accel_z)
 {
     uint8_t buffer[6];
     OPERATE_RET ret = mpu6050_read_reg(MPU6050_REG_ACCEL_XOUT_H, buffer, 6);
     if (ret != OPRT_OK) {
         return ret;
     }
     *accel_x = (int16_t)((buffer[0] << 8) | buffer[1]);
     *accel_y = (int16_t)((buffer[2] << 8) | buffer[3]);
     *accel_z = (int16_t)((buffer[4] << 8) | buffer[5]);
     
     return OPRT_OK;
 }
 
 // 获取指南针角度
 OPERATE_RET app_compass_get_angle(float *angle)
 {
     int16_t accel_x, accel_y, accel_z;
     if (mpu6050_read_accel(&accel_x, &accel_y, &accel_z) != OPRT_OK) {
         PR_ERR("Failed to read accelerometer data");
         return OPRT_COM_ERROR;
     }
     float ax = accel_x / sg_accel_scale;
     float ay = accel_y / sg_accel_scale;
     *angle = atan2f(ay, ax) * 180.0f / M_PI;
     if (*angle < 0) {
         *angle += 360.0f;
     }
     uint32_t current_time = tal_system_get_millisecond();
     sg_last_update = current_time;
     float alpha = 0.2f;
     sg_current_angle = alpha * (*angle) + (1 - alpha) * sg_current_angle;
     *angle = sg_current_angle;
     
     return OPRT_OK;
 }
 
 // 月亮位置计算相关函数
 double utc_to_julian_day(TIME_T utc_time) {
     return (utc_time / 86400.0) + 2440587.5;
 }
 
 void calculate_moon_position(double jd, double *moon_lon, double *moon_lat) {
     //从J2000.0起算的儒略世纪数
     double T = (jd - 2451545.0) / 36525.0;
     //平黄经
     double Lp = fmod(218.3164477+481267.88123421*T, 360.0);
     //平近点角
     double D = fmod(297.8501921 + 445267.1114034 * T, 360.0);
     //太阳平近点角
     double M = fmod(357.5291092 + 35999.0502909 * T, 360.0);
     //月亮平近点角
     double Mp = fmod(134.9633964 + 477198.8675055 * T, 360.0);
     //月亮升交点平黄经
     double F = fmod(93.2720950 + 483202.0175233 * T, 360.0);
     //DEG_TO_RAD:角度转RAD
     Lp *= DEG_TO_RAD;
     D *= DEG_TO_RAD;
     M *= DEG_TO_RAD;
     Mp *= DEG_TO_RAD;
     F *= DEG_TO_RAD;
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
     double lambda = Lp + sum_lon * DEG_TO_RAD;
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
     double T = (jd - 2451545.0) / 36525.0;
     double GMST = 280.46061837 + 360.98564736629 * (jd - 2451545.0)
                 + 0.000387933 * T * T - T * T * T / 38710000.0;
     GMST = fmod(GMST, 360.0);
     if (GMST < 0) GMST += 360.0;
     double LST = GMST + longitude;
     LST = fmod(LST, 360.0);
     if (LST < 0) LST += 360.0;
     return LST * DEG_TO_RAD;
 }
 
 void calculate_alt_az(double ra, double dec, double lst, double lat, 
                       double *alt, double *az) {
     double lat_rad = lat * DEG_TO_RAD;
     double HA = lst - ra;
     double sin_alt = sin(dec) * sin(lat_rad) 
                    + cos(dec) * cos(lat_rad) * cos(HA);
     *alt = asin(sin_alt);
     double cos_az = (sin(dec) - sin(lat_rad) * sin_alt) 
                   / (cos(lat_rad) * cos(*alt));
     if (cos_az > 1.0) cos_az = 1.0;
     if (cos_az < -1.0) cos_az = -1.0;
     *az = acos(cos_az);
     if (sin(HA) > 0) {
         *az = 2 * PI - *az;
     }
 }
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
     double lst = calculate_local_sidereal_time(jd, current_longitude);
     // 6. 计算高度角和方位角
     double alt, az;
     calculate_alt_az(ra, dec, lst, current_latitude, &alt, &az);
     // 7. 转换为度并规范化
     double azimuth = az * RAD_TO_DEG;
     azimuth = fmod(azimuth + 360.0, 360.0);
     return (float)azimuth;
 }
 float calculate_home_direction(void) {
     double lat1 = current_latitude * DEG_TO_RAD;
     double lon1 = current_longitude * DEG_TO_RAD;
     double lat2 = home_latitude * DEG_TO_RAD;
     double lon2 = home_longitude * DEG_TO_RAD;
     double dLon = lon2 - lon1;//经度差
     double y = sin(dLon) * cos(lat2);
     double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
     double azimuth = atan2(y, x);
     azimuth = azimuth * RAD_TO_DEG;
     if (azimuth < 0) {
         azimuth += 360.0;
     }
     
     return (float)azimuth;
 }
 
 //显示标签
 static void init_display(void) {
    //  sg_moon_label = lv_label_create(lv_scr_act());
    //  if (sg_moon_label) {
        
    //      lv_obj_set_style_text_font(sg_moon_label, &lv_font_montserrat_24, 0);
    //      lv_obj_set_style_text_color(sg_moon_label, lv_color_hex(0xFFD700), 0);
    //      lv_label_set_text(sg_moon_label, "MOON");
    //      lv_obj_align(sg_moon_label, LV_ALIGN_CENTER, 0, -40);
    //      lv_obj_clear_flag(sg_moon_label, LV_OBJ_FLAG_HIDDEN); // 直接显示
    //  }
     
    //  // 创建"you miss home?"标签 - 直接显示
    //  sg_home_label = lv_label_create(lv_scr_act());
    //  if (sg_home_label) {
    //      lv_obj_set_style_text_font(sg_home_label, &lv_font_montserrat_24, 0);
    //      lv_obj_set_style_text_color(sg_home_label, lv_color_hex(0x00FF00), 0);
    //      lv_label_set_text(sg_home_label, "you miss home?");
    //      lv_obj_align(sg_home_label, LV_ALIGN_CENTER, 0, -80);
    //      lv_obj_clear_flag(sg_home_label, LV_OBJ_FLAG_HIDDEN); // 直接显示
    //  }
     
     // 创建方位角信息标签 - 调试用
    //  sg_azimuth_label = lv_label_create(lv_scr_act());
    //  if (sg_azimuth_label) {
    //      lv_obj_set_style_text_font(sg_azimuth_label, &lv_font_montserrat_14, 0);
    //      lv_obj_set_style_text_color(sg_azimuth_label, lv_color_hex(0xFFFFFF), 0);
    //      lv_label_set_text(sg_azimuth_label, "Initializing...");
    //      lv_obj_align(sg_azimuth_label, LV_ALIGN_BOTTOM_MID, 0, -20);
    //  }
 }
 
 // 更新显示内容
 static void update_display(float current_az, float moon_az, float home_az) {
     char buf[64];
     // 更新方位角信息
     snprintf(buf, sizeof(buf), "D:%.0f° M:%.0f° H:%.0f°", current_az, moon_az, home_az);
     if (sg_azimuth_label) {
         lv_label_set_text(sg_azimuth_label, buf);
     }
 }
 
 // 月亮和家乡追踪任务 - 简化逻辑
 static void tracker_task(void *arg) {
     (void)arg;
     PR_DEBUG("Tracker task started");
     // 计算家乡方位
     float home_azimuth = calculate_home_direction();
     while (1) {
         // 获取当前方位
         float current_azimuth;
         if (app_compass_get_angle(&current_azimuth) != OPRT_OK) {
             tal_system_sleep(1000);
             continue;
         }
         float moon_azimuth = calculate_moon_direction();
         update_display(current_azimuth, moon_azimuth, home_azimuth);
         PR_DEBUG("POS: %.1f°, MOON: %.1f°, HOME: %.1f°", 
                  current_azimuth, moon_azimuth, home_azimuth);
         
         tal_system_sleep(1000); // 1秒
     }
 }
 
 static void __app_display_msg_handle(DISPLAY_MSG_T *msg_data)
 {
     if (msg_data == NULL) {
         return;
     }
     if (msg_data->type != DISPLAY_MSG_SET_BACKGROUND_ROTATION && 
         msg_data->payload.dynamic_data.data != NULL) {
         tkl_system_psram_free(msg_data->payload.dynamic_data.data);
         msg_data->payload.dynamic_data.data = NULL;
     }
 }
 
 static void __compass_timer_cb(lv_timer_t *timer)
 {
     (void)timer;
     
     float angle_deg;
     if (app_compass_get_angle(&angle_deg) == OPRT_OK) {
         DISPLAY_MSG_T msg;
         msg.type = DISPLAY_MSG_SET_BACKGROUND_ROTATION;
         msg.payload.rotation_data.rotation = (int16_t)(angle_deg * 10);
         tal_queue_post(sg_display.queue_hdl, &msg, 0);
     }
 }
 
 static void __display_task(void *args)
 {
     (void)args;
 
     tuya_lvgl_mutex_lock();
     sg_background_img = lv_image_create(lv_scr_act());
     if (!sg_background_img) {
         PR_ERR("Failed to create background image object");
     } else {
         lv_image_set_src(sg_background_img, &background);
         lv_obj_align(sg_background_img, LV_ALIGN_CENTER, 0, 0);
         lv_img_set_zoom(sg_background_img, 256); // 256 = 100% 缩放
         lv_image_set_rotation(sg_background_img, 0);
     }

     init_display();
     
 #if defined(BOARD_CHOICE_WAVESHARE_ESP32_S3_TOUCH_AMOLED_1_8)
     extern void lcd_sh8601_set_backlight(uint8_t brightness);
     lcd_sh8601_set_backlight(80); // set backlight to 80%
 #endif
     
     tuya_lvgl_mutex_unlock();
     PR_DEBUG("Display init success");
     if (app_mpu6050_init() != OPRT_OK) {
         PR_ERR("MPU6050 initialization failed");
     }
     //指南针定时器
     sg_compass_timer = lv_timer_create(__compass_timer_cb, 200, NULL);
     if (sg_compass_timer) {
         lv_timer_set_repeat_count(sg_compass_timer, -1);  // 无限重复
     } else {
         PR_ERR("Failed to create compass timer");
     }
     THREAD_CFG_T tracker_cfg = {
         .thrdname = "tracker_task",
         .priority = THREAD_PRIO_3,
         .stackDepth = 2048,
     };
     tal_thread_create_and_start(NULL, NULL, NULL, tracker_task, NULL, &tracker_cfg);
 
     for (;;) {
         DISPLAY_MSG_T msg_data;
         if (tal_queue_fetch(sg_display.queue_hdl, &msg_data, 0xFFFFFFFF) != OPRT_OK) {
             continue;
         }
 
         switch (msg_data.type) {
             case DISPLAY_MSG_SET_BACKGROUND_ROTATION:
                 tuya_lvgl_mutex_lock();
                 if (sg_background_img) {
                     lv_image_set_rotation(sg_background_img, msg_data.payload.rotation_data.rotation);
                 }
                 tuya_lvgl_mutex_unlock();
                 break;
                 
             default:
                 __app_display_msg_handle(&msg_data);
                 break;
         }
     }
 }
 
 OPERATE_RET app_display_init(void)
 {
     OPERATE_RET rt = OPRT_OK;
 
     memset(&sg_display, 0, sizeof(TUYA_DISPLAY_T));
 
     // lvgl initialization
     TUYA_CALL_ERR_RETURN(tuya_lvgl_init());
     PR_DEBUG("lvgl init success");
 
     TUYA_CALL_ERR_RETURN(tal_queue_create_init(&sg_display.queue_hdl, sizeof(DISPLAY_MSG_T), 8));
     THREAD_CFG_T cfg = {
         .thrdname = "display_task",
         .priority = THREAD_PRIO_2,
         .stackDepth = 1024 * 4,
     };
     TUYA_CALL_ERR_RETURN(tal_thread_create_and_start(&sg_display.thrd_hdl, NULL, NULL, __display_task, NULL, &cfg));
     PR_DEBUG("Display task created");
 
     return rt;
 }
 
 OPERATE_RET app_display_send_msg(TY_DISPLAY_TYPE_E tp, uint8_t *data, int len)
 {
     DISPLAY_MSG_T msg_data;
     
     msg_data.type = tp;
     if (len > 0 && data != NULL) {
         msg_data.payload.dynamic_data.data = (char *)tkl_system_psram_malloc(len + 1);
         if (NULL == msg_data.payload.dynamic_data.data) {
             return OPRT_MALLOC_FAILED;
         }
         memcpy(msg_data.payload.dynamic_data.data, data, len);
         msg_data.payload.dynamic_data.data[len] = '\0';
         msg_data.payload.dynamic_data.len = len;
     } else {
         msg_data.payload.dynamic_data.data = NULL;
         msg_data.payload.dynamic_data.len = 0;
     }
 
     return tal_queue_post(sg_display.queue_hdl, &msg_data, 0xFFFFFFFF);
 }
 
 OPERATE_RET app_display_set_rotation(int16_t rotation)
 {
     DISPLAY_MSG_T msg;
     msg.type = DISPLAY_MSG_SET_BACKGROUND_ROTATION;
     msg.payload.rotation_data.rotation = rotation;
     return tal_queue_post(sg_display.queue_hdl, &msg, 0);
 }