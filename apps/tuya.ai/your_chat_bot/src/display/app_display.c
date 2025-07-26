// /**
//  * @file app_display.c
//  * @brief Handle display initialization and message processing
//  */

// #include "tuya_cloud_types.h"
// #include "app_display.h"
// #include "tuya_lvgl.h"
// #include "app_compass.h"  // 添加指南针处理头文件
// // #include "qmc5883l.h"  // 确保包含磁力计驱动头文件
// #include <math.h>      // 添加数学函数支持 (atan2f, fmodf)
// #include "tal_log.h"
// #include "tal_queue.h"
// #include "tal_thread.h"
// #include "tal_system.h"
// #include "tkl_memory.h"

// #include "tkl_i2c.h"
// #include "tkl_pinmux.h"
// #include "tkl_output.h"
// #include "tkl_gpio.h"
// #include "lvgl.h"
//  // 校准数据结构
// static MAG_CALIBRATION_T sg_cal_data = {0};
// static BOOL_T sg_calibrating = FALSE;
//  // 直接包含背景图实现文件
//  #include "Compass/background.c"
 

//  int odr=QMC5883L_CONFIG_OS512;
//  int _range =QMC5883L_CONFIG_2GAUSS;
//  int sample_rate=QMC5883L_CONFIG_50HZ;
//  int mode = QMC5883L_CONFIG_CONT;//09H CONFIG 控制位需要的数值
//   // 用静态标识定义设备底层读写指令
//   static OPERATE_RET qmc5883l_read_reg(uint8_t addr, uint8_t reg, uint8_t *value) {
//     OPERATE_RET ret = tkl_i2c_master_send(I2C_NUMBER, addr, &reg, 1, FALSE);
//     if (ret != OPRT_OK) {
//         PR_ERR("I2C send error: %d", ret);
//         return ret;
//     }
//     ret = tkl_i2c_master_receive(I2C_NUMBER, addr, value, 1, FALSE);
//     if (ret != OPRT_OK) {
//         PR_ERR("I2C receive error: %d", ret);
//         return 0;
//     }
//     return ret;
// }
// static OPERATE_RET qmc5883l_write_reg(uint8_t addr, uint8_t reg, uint8_t value){
//     // 搞清楚i2c的主设备send函数需要怎么封装数据？
    
//     uint8_t data[2];
//     data[0] = reg;
//     data[1] = value;
    
//     OPERATE_RET ret = tkl_i2c_master_send(I2C_NUMBER, QMC5883L_ADDRESS, data, 2, FALSE);
//     if (ret != OPRT_OK) {
//         PR_ERR("write_reg: I2C SEND error: %d", ret);
//     }
//     return ret;

// }
//  void qmc5883l_i2c_init(void){
//      TUYA_IIC_BASE_CFG_T qmc5883_cfg = {
//          .role = TUYA_IIC_MODE_MASTER,
//          .speed = I2C_SPEED,
//          .addr_width = QMC_ADDR_MODE
//      };
//      OPERATE_RET op_ret = OPRT_OK;
//      op_ret = tkl_i2c_init(I2C_NUMBER,&qmc5883_cfg);// I2C0, 400KHz高速传输
//      if (OPRT_OK != op_ret) {
//         PR_ERR("i2c init fail, err<%d>!", op_ret);
//     }
//      qmc5883l_write_reg(QMC5883L_ADDRESS,QMC5883L_REG_CONTROL,QMC5883L_CONFIG_OS512|QMC5883L_CONFIG_2GAUSS|QMC5883L_CONFIG_50HZ|QMC5883L_CONFIG_CONT);
//      // 或者换一种方式，不使用寄存器写指令直接指定？这样也许更安全，可以当备份写法
//  }
//  void qmc5883l_device_init(void){
//      qmc5883l_write_reg(QMC5883L_ADDRESS,QMC5883L_REG_CONTROL,odr|_range|sample_rate|mode);// 详见寄存器地址0x09的存储四个数值介绍
//  }
 
//  void qmc5883l_init(void)
//  {   
//      tkl_io_pinmux_config(I2C_SCL_PIN, TUYA_IIC0_SCL);
     
//      tkl_io_pinmux_config(I2C_SDA_PIN, TUYA_IIC0_SDA);
//      TUYA_IIC_BASE_CFG_T i2c_cfg = {
//          .role = TUYA_IIC_MODE_MASTER,
//          .speed = I2C_SPEED,
//          .addr_width = QMC_ADDR_MODE,
//      };
//      tkl_i2c_init(I2C_NUMBER,&i2c_cfg);// I2C0, 400KHz高速传输，看情况是否要单独弄一个config函数
//      qmc5883l_reset();
//      qmc5883l_device_init();
//      //tkl_i2c_master_send(I2C_NUMBER,QMC5883L_ADDRESS,QMC5883L_REG_CONTROL,0x00,FALSE); // 不发送停止位，TODO: 核实size该填什么
//      uint8_t reg = QMC5883L_REG_CONTROL;
//      tkl_i2c_master_send(I2C_NUMBER, QMC5883L_ADDRESS, &reg, 1, FALSE);
//  }

//  // 调用指令层的函数其实也可以不用写addr，上面直接写死。不过为了作为模板独立开来使用还是每次指定一下addr
//  int16_t qmc5883l_read_16bit_data(uint8_t addr, uint8_t lsb_reg, uint8_t msb_reg){
//      //uint8_t data[6];
//      uint8_t lsb,msb;
//      if (qmc5883l_read_reg(addr,lsb_reg,&lsb)!= OPRT_OK) return 0;
//      if (qmc5883l_read_reg(addr,msb_reg,&msb)!= OPRT_OK) return 0;
//      return (int16_t)((msb << 8) | lsb);
//  }// 大小端组合的封装函数
 
//  // void qmc5883l_write_data(uint8_t addr, uint8_t reg, uint8_t value){
//  //     //uint8_t data[6];
//  //     // TODO: 还没写好
//  //     qmc5883l_write_reg(addr,reg,value);
//  // }
 
//  // 获取X轴数据
//  int16_t getX(void){
//      // Reg: 0x00, 0x01
//      // 相当于低位和高位组在一起发送
//      // uint8_t data[2];
//      // data[0] = qmc5883l_read_data(QMC5883L_ADDRESS,QMC5883L_REG_X_LSB);
//      // data[1] = qmc5883l_read_data(QMC5883L_ADDRESS,QMC5883L_REG_X_MSB);
//      // // xfer_pending: xfer_pending: TRUE : not send stop condition, FALSE : send stop condition.
//      // return data;
//      return qmc5883l_read_16bit_data(QMC5883L_ADDRESS,QMC5883L_REG_X_LSB, QMC5883L_REG_X_MSB);
//  }
 
//  // 获取Y轴数据
//  int16_t getY(void){
//      // // Reg: 0x02, 0x03
//      // uint8_t data[2];
//      // data[0] = qmc5883l_read_data(QMC5883L_ADDRESS,QMC5883L_REG_Y_LSB);
//      // data[1] = qmc5883l_read_data(QMC5883L_ADDRESS,QMC5883L_REG_Y_MSB);
//      // return data;
//      return qmc5883l_read_16bit_data(QMC5883L_ADDRESS,QMC5883L_REG_Y_LSB, QMC5883L_REG_Y_MSB);
//  }
//  // 获取Z轴数据
//  int16_t getZ(void){
//      // // Reg: 0x02, 0x03
//      // uint8_t data[2];
//      // data[0] = qmc5883l_read_data(QMC5883L_ADDRESS,QMC5883L_REG_Z_LSB);
//      // data[1] = qmc5883l_read_data(QMC5883L_ADDRESS,QMC5883L_REG_Z_MSB);
//      // return data;
//      return qmc5883l_read_16bit_data(QMC5883L_ADDRESS,QMC5883L_REG_Z_LSB, QMC5883L_REG_Z_MSB);
//  }
 
//  //TODO: 或许可以添加一个平滑函数，用来处理磁力计数据的抖动，但暂时raw也够用
//  void Reset_Range(int range_val){
//      switch(range_val) {
//          case 2:
//            _range = QMC5883L_CONFIG_2GAUSS;
//            break;
//          case 8:
//            _range = QMC5883L_CONFIG_8GAUSS;
//            break;
//        }
//        qmc5883l_device_init();
//  }
//  void Reset_qmc5883l_odr(int odr){
//      switch(odr){
//          case 64:
//              odr = QMC5883L_CONFIG_OS64;
//              break;
//          case 128:
//              odr = QMC5883L_CONFIG_OS128;
//              break;
//          case 256:
//              odr = QMC5883L_CONFIG_OS256;
//              break;
//          case 512:
//              odr = QMC5883L_CONFIG_OS512;
//              break;
//      }
//      qmc5883l_device_init();
//  }
//  void Reset_qmc5883l_samplingrate(int sample_rate){
//      // 采样速率选择，只能选择四个速率当中的一个
//      switch (sample_rate)
//      {
//      case 10:
//          sample_rate=QMC5883L_CONFIG_10HZ;
//          break;
//      case 50:
//          sample_rate=QMC5883L_CONFIG_50HZ;
//          break;
//      case 100:
//          sample_rate=QMC5883L_CONFIG_100HZ;
//          break;
//      case 200:
//          sample_rate=QMC5883L_CONFIG_200HZ;
//          break;
//      }
//      qmc5883l_device_init();
//  }
//  void Reset_qmc5883l_mode(int mode){
//      switch (mode)
//      {
//      case 0:
//          mode = QMC5883L_CONFIG_STANDBY;
//          break;
//      case 1:
//          mode = QMC5883L_CONFIG_CONT;
//          break;
//      }
//      qmc5883l_device_init();
//  }
//  void Reset_QMC5883L(void) {
//     uint8_t data[2] = {QMC5883L_REG_RESET, 0x01}; // 寄存器地址 + 数据
//     tkl_i2c_master_send(I2C_NUMBER, QMC5883L_ADDRESS, data, sizeof(data), FALSE);
// }
 
//  // void qmc5883l_Ready(void){
//  //     if(!qmc5883l_read_data(QMC5883L_ADDRESS,QMC5883L_REG_STATUS))
//  //     {
//  //         return 0;
//  //     }
//  //     // TODO: 找到status状态位
//  // }
 
//  bool qmc5883l_ready(void) {
//      uint8_t status;
//      if (qmc5883l_read_reg(QMC5883L_ADDRESS, QMC5883L_REG_STATUS, &status) != OPRT_OK) {
//          return false;
//      }
//      return (status & 0x01) != 0; // 根据数据手册调整
//  }
 
 
//  void Reset_I2C(void){
//      // I2C的析构函数
//      tkl_i2c_reset(I2C_NUMBER);
//  }
//  void qmc5883l_reset(void) {
//      qmc5883l_write_reg(QMC5883L_ADDRESS, QMC5883L_REG_RESET, 0x01);
//  }
//  /* 校准磁力计部分 */
 
//  typedef struct {
//      float x_min;
//      float x_max;
//      float y_min;
//      float y_max;
//      float z_min;
//      float z_max;
//  } MagCalibration;
 
//  MagCalibration mag_calibration = {0};
 
//  void qmc5883l_reset_calibration(void){
//      mag_calibration.x_min = -32768.0f;
//      mag_calibration.x_max = 32767.0f;
//      mag_calibration.y_min = -32768.0f;
//      mag_calibration.y_max = 32767.0f;
//      mag_calibration.z_min = -32768.0f;
//      mag_calibration.z_max = 32767.0f;
//  }
 
//  void qmc5883l_update_calibration(float x, float y, float z){
//      float current_x = x;
//      float current_y = y;
//      float current_z = z;
//      if(x < mag_calibration.x_min){
//          mag_calibration.x_min = current_x;
//      }
//      if(x > mag_calibration.x_max){
//          mag_calibration.x_max = current_x;
//      }
//      if(y < mag_calibration.y_min){
//          mag_calibration.y_min = current_y;
//      }
//      if(y > mag_calibration.y_max){
//          mag_calibration.y_max = current_y;
//      }
//      if(z < mag_calibration.z_min){
//          mag_calibration.z_min = current_z;
//      }
//      if(z > mag_calibration.z_max){
//          mag_calibration.z_max = current_z;
//      }
//  }
 
//  void qmc5883l_calibration(float* x, float* y, float* z){
//      qmc5883l_reset_calibration();// 先置0
//      // 校准坐标和范围
//      float offset_x = (mag_calibration.x_min + mag_calibration.x_max) / 2.0f;
//      float offset_y = (mag_calibration.y_min + mag_calibration.y_max) / 2.0f;
//      float offset_z = (mag_calibration.z_min + mag_calibration.z_max) / 2.0f;
//      float range_x = mag_calibration.x_max - mag_calibration.x_min;
//      float range_y = mag_calibration.y_max - mag_calibration.y_min;
//      //float range_z = mag_calibration.z_max - mag_calibration.z_min;
//      float avg_range = (range_x + range_y) / 2.0f;// 不计算z轴
//      float scale_x = avg_range / range_x;
//      float scale_y = avg_range / range_y;
//      float scale_z = 1.0f; // Z轴不需要缩放
     
//      // 应用校准
//      *x = (float)((*x - offset_x) * scale_x);
//      *y = (float)((*y - offset_y) * scale_y);
//      *z = (float)((*z - offset_z) * scale_z);
//  }
 
 
//  // 示例校准封装函数
 
//  // void qmc5883l_calibration_example(void){
//  //     qmc5883l_reset_calibration();
 
//  //     current_x = getX();
//  //     current_y = getY();
//  //     current_z = getZ();
//  // }
 
//  // 为了追求高精度可以结合MPU6050进行校准
 
//  // void Reset_qmc5883l_Addr(uint16_t new_addr){
//  //     // 重新指定QMC5883L的设备地址（应该用不上）
//  //     QMC5883L_ADDRESS = new_addr;
//  // }
 
 


//  /***********************************************************
//  ***********************typedef define***********************
//  ***********************************************************/
//  // 使用联合体处理不同类型的消息
//  typedef struct {
//      TY_DISPLAY_TYPE_E type;
//      union {
//          struct {
//              int len;
//              char *data;
//          } dynamic_data;
//          struct {
//              int16_t rotation;  // 旋转角度 (0-3600, 单位0.1度)
//          } rotation_data;
//      } payload;
//  } DISPLAY_MSG_T;
 
//  typedef struct {
//      QUEUE_HANDLE queue_hdl;
//      THREAD_HANDLE thrd_hdl;
//  } TUYA_DISPLAY_T;
 
//  /***********************************************************
//  ***********************variable define**********************
//  ***********************************************************/
//  static TUYA_DISPLAY_T sg_display = {0};
//  static lv_obj_t *sg_background_img = NULL;
//  static lv_timer_t *sg_compass_timer = NULL;  // 指南针定时器
 
//  /***********************************************************
//  ***********************function define**********************
//  ***********************************************************/
 
//  static void __app_display_msg_handle(DISPLAY_MSG_T *msg_data)
//  {
//      if (msg_data == NULL) {
//          return;
//      }
     
//      // 只释放动态分配的数据
//      if (msg_data->type != DISPLAY_MSG_SET_BACKGROUND_ROTATION && 
//          msg_data->payload.dynamic_data.data != NULL) {
//          tkl_system_psram_free(msg_data->payload.dynamic_data.data);
//          msg_data->payload.dynamic_data.data = NULL;
//      }
//  }
 
//  // 指南针定时器回调
//  static void __compass_timer_cb(lv_timer_t *timer)
//  {
//      (void)timer;
     
//      float angle_deg;
//      if (app_compass_get_angle(&angle_deg) == OPRT_OK) {
//          // 转换为指南针背景图需要旋转的角度（使指南针指向南方）
//          float rotation_deg = angle_deg + 180.0f;
//          if (rotation_deg >= 360.0f) {
//              rotation_deg -= 360.0f;
//          }
         
//          // 转换为0.1度单位（LVGL要求）
//          int16_t rotation = (int16_t)(rotation_deg * 10);
         
//          // 创建旋转消息
//          DISPLAY_MSG_T msg;
//          msg.type = DISPLAY_MSG_SET_BACKGROUND_ROTATION;
//          msg.payload.rotation_data.rotation = rotation;
         
//          // 发送消息到显示队列
//          tal_queue_post(sg_display.queue_hdl, &msg, 0);
//      }
//  }
 
//  static void __display_task(void *args)
//  {
//      (void)args;
 
//      tuya_lvgl_mutex_lock();
     
//      // 创建背景图对象
//      sg_background_img = lv_image_create(lv_scr_act());
//      if (!sg_background_img) {
//          PR_ERR("Failed to create background image object");
//      } else {
//          // 设置背景图并居中
//          lv_image_set_src(sg_background_img, &background);
//          lv_obj_align(sg_background_img, LV_ALIGN_CENTER, 0, 0);
         
//          // 设置图片缩放模式
//          lv_img_set_zoom(sg_background_img, 256); // 256 = 100% 缩放
//          lv_image_set_rotation(sg_background_img, 0);
//      }
     
//  #if defined(BOARD_CHOICE_WAVESHARE_ESP32_S3_TOUCH_AMOLED_1_8)
//      extern void lcd_sh8601_set_backlight(uint8_t brightness);
//      lcd_sh8601_set_backlight(80); // set backlight to 80%
//  #endif
     
//      tuya_lvgl_mutex_unlock();
//      PR_DEBUG("Display init success");
 
//      // 初始化指南针模块
//      if (app_compass_init() != OPRT_OK) {
//          PR_ERR("Compass initialization failed");
//      }
     
//      // 创建指南针定时器
//      sg_compass_timer = lv_timer_create(__compass_timer_cb, 200, NULL);
//      if (sg_compass_timer) {
//          lv_timer_set_repeat_count(sg_compass_timer, -1);  // 无限重复
//      } else {
//          PR_ERR("Failed to create compass timer");
//      }
 
//      for (;;) {
//          DISPLAY_MSG_T msg_data;
//          if (tal_queue_fetch(sg_display.queue_hdl, &msg_data, 0xFFFFFFFF) != OPRT_OK) {
//              continue;
//          }
 
//          switch (msg_data.type) {
//              case DISPLAY_MSG_SET_BACKGROUND_ROTATION:
//                  tuya_lvgl_mutex_lock();
//                  if (sg_background_img) {
//                      lv_image_set_rotation(sg_background_img, msg_data.payload.rotation_data.rotation);
//                  }
//                  tuya_lvgl_mutex_unlock();
//                  break;
                 
//              default:
//                  __app_display_msg_handle(&msg_data);
//                  break;
//          }
//      }
//  }
 
//  OPERATE_RET app_display_init(void)
//  {
//      OPERATE_RET rt = OPRT_OK;
 
//      memset(&sg_display, 0, sizeof(TUYA_DISPLAY_T));
 
//      // lvgl initialization
//      TUYA_CALL_ERR_RETURN(tuya_lvgl_init());
//      PR_DEBUG("lvgl init success");
 
//      TUYA_CALL_ERR_RETURN(tal_queue_create_init(&sg_display.queue_hdl, sizeof(DISPLAY_MSG_T), 8));
//      THREAD_CFG_T cfg = {
//          .thrdname = "display_task",
//          .priority = THREAD_PRIO_2,
//          .stackDepth = 1024 * 4,
//      };
//      TUYA_CALL_ERR_RETURN(tal_thread_create_and_start(&sg_display.thrd_hdl, NULL, NULL, __display_task, NULL, &cfg));
//      PR_DEBUG("Display task created");
 
//      return rt;
//  }
 
//  OPERATE_RET app_display_send_msg(TY_DISPLAY_TYPE_E tp, uint8_t *data, int len)
//  {
//      DISPLAY_MSG_T msg_data;
     
//      msg_data.type = tp;
//      if (len > 0 && data != NULL) {
//          msg_data.payload.dynamic_data.data = (char *)tkl_system_psram_malloc(len + 1);
//          if (NULL == msg_data.payload.dynamic_data.data) {
//              return OPRT_MALLOC_FAILED;
//          }
//          memcpy(msg_data.payload.dynamic_data.data, data, len);
//          msg_data.payload.dynamic_data.data[len] = '\0';
//          msg_data.payload.dynamic_data.len = len;
//      } else {
//          msg_data.payload.dynamic_data.data = NULL;
//          msg_data.payload.dynamic_data.len = 0;
//      }
 
//      return tal_queue_post(sg_display.queue_hdl, &msg_data, 0xFFFFFFFF);
//  }
 
//  OPERATE_RET app_display_set_rotation(int16_t rotation)
//  {
//      DISPLAY_MSG_T msg;
//      msg.type = DISPLAY_MSG_SET_BACKGROUND_ROTATION;
//      msg.payload.rotation_data.rotation = rotation;
//      return tal_queue_post(sg_display.queue_hdl, &msg, 0);
//  }

// //  // 校准数据结构
// // static MAG_CALIBRATION_T sg_cal_data = {0};
// // static BOOL_T sg_calibrating = FALSE;

// OPERATE_RET app_compass_init(void)
// {
//     // 初始化磁力计
//     qmc5883l_init();
    
//     // 初始化校准数据
//     sg_cal_data.x_min = 10000.0f;
//     sg_cal_data.x_max = -10000.0f;
//     sg_cal_data.y_min = 10000.0f;
//     sg_cal_data.y_max = -10000.0f;
    
//     PR_DEBUG("Compass initialized");
//     return OPRT_OK;
// }

// OPERATE_RET app_compass_get_angle(float *angle)
// {
//     int16_t x, y;
//     float x_cal, y_cal;
    
//     // 读取原始数据
//     x = getX();


//     y = getY();
    
//     // 应用校准
//     x_cal = (x - sg_cal_data.x_min) / (sg_cal_data.x_max - sg_cal_data.x_min) * 2 - 1;
//     y_cal = (y - sg_cal_data.y_min) / (sg_cal_data.y_max - sg_cal_data.y_min) * 2 - 1;
    
//     // 计算角度（0-360度）
//     float heading = atan2f(y_cal, x_cal) * 180.0f / 3.14159265f;
//     if (heading < 0) {
//         heading += 360.0f;
//     }
    
//     // 调整角度使0度为北方
//     heading = fmodf(heading + 360.0f, 360.0f);
    
//     *angle = heading;
//     return OPRT_OK;
// }

// // 校准线程函数
// static void __calibration_thread(void *arg)
// {
//     uint32_t duration = *(uint32_t *)arg;
//     uint32_t start_time = tal_system_get_millisecond();
    
//     PR_NOTICE("Starting compass calibration. Rotate device slowly for %d ms", duration);
    
//     while ((tal_system_get_millisecond() - start_time) < duration) {
//         int16_t x = getX();
//         int16_t y = getY();
        
//         // 更新校准数据
//         if (x < sg_cal_data.x_min) sg_cal_data.x_min = x;
//         if (x > sg_cal_data.x_max) sg_cal_data.x_max = x;
//         if (y < sg_cal_data.y_min) sg_cal_data.y_min = y;
//         if (y > sg_cal_data.y_max) sg_cal_data.y_max = y;
        
//         tal_system_sleep(50);
//     }
    
//     PR_NOTICE("Calibration complete. X: [%d, %d] Y: [%d, %d]", 
//               (int)sg_cal_data.x_min, (int)sg_cal_data.x_max,
//               (int)sg_cal_data.y_min, (int)sg_cal_data.y_max);
    
//     sg_calibrating = FALSE;
//     tkl_system_free(arg);
// }

// void app_compass_calibrate(uint32_t duration)
// {
//     if (sg_calibrating) {
//         PR_WARN("Calibration already in progress");
//         return;
//     }
    
//     sg_calibrating = TRUE;
    
//     // 重置校准数据
//     sg_cal_data.x_min = 10000.0f;
//     sg_cal_data.x_max = -10000.0f;
//     sg_cal_data.y_min = 10000.0f;
//     sg_cal_data.y_max = -10000.0f;
    
//     // 创建校准线程
//     uint32_t *duration_ptr = tkl_system_malloc(sizeof(uint32_t));
//     if (duration_ptr == NULL) {
//         PR_ERR("Failed to allocate memory for calibration duration");
//         sg_calibrating = FALSE;
//         return;
//     }
//     *duration_ptr = duration;
    
//     THREAD_CFG_T cfg = {
//         .thrdname = "compass_cal",
//         .priority = THREAD_PRIO_4,
//         .stackDepth = 1024 * 2,
//     };
    
//     THREAD_HANDLE cal_thread;
//     if (tal_thread_create_and_start(&cal_thread, NULL, NULL, __calibration_thread, duration_ptr, &cfg) != OPRT_OK) {
//         PR_ERR("Failed to create calibration thread");
//         tkl_system_free(duration_ptr);
//         sg_calibrating = FALSE;
//     }
// }
// void test_compass(void)
// {
//     PR_NOTICE("Compass Test - Rotate device slowly");
    
//     for (int i = 0; i < 20; i++) {
//         int16_t x = getX();
//         int16_t y = getY();
//         PR_NOTICE("X: %6d, Y: %6d", x, y);
//         tal_system_sleep(500);
//     }
// }

// /**
//  * @file app_display.c
//  * @brief Handle display initialization and message processing
//  */

//  #include "tuya_cloud_types.h"
//  #include "app_display.h"
//  #include "tuya_lvgl.h"
//  #include "app_mpu6050.h"  // 使用MPU6050替代磁力计
//  #include <math.h>      // 添加数学函数支持 (atan2f, fmodf)
//  #include "tal_log.h"
//  #include "tal_queue.h"
//  #include "tal_thread.h"
//  #include "tal_system.h"
//  #include "tkl_memory.h"
//  #include "tkl_i2c.h"
//  #include "tkl_pinmux.h"
//  #include "tkl_output.h"
//  #include "tkl_gpio.h"
//  #include "lvgl.h"
 
//  // 直接包含背景图实现文件
//  #include "Compass/background.c"
 
//  /***********************************************************
//  ***********************typedef define***********************
//  ***********************************************************/
//  // 使用联合体处理不同类型的消息
//  typedef struct {
//      TY_DISPLAY_TYPE_E type;
//      union {
//          struct {
//              int len;
//              char *data;
//          } dynamic_data;
//          struct {
//              int16_t rotation;  // 旋转角度 (0-3600, 单位0.1度)
//          } rotation_data;
//      } payload;
//  } DISPLAY_MSG_T;
 
//  typedef struct {
//      QUEUE_HANDLE queue_hdl;
//      THREAD_HANDLE thrd_hdl;
//  } TUYA_DISPLAY_T;
 
//  /***********************************************************
//  ***********************variable define**********************
//  ***********************************************************/
//  static TUYA_DISPLAY_T sg_display = {0};
//  static lv_obj_t *sg_background_img = NULL;
//  static lv_timer_t *sg_compass_timer = NULL;  // 指南针定时器
 
//  /***********************************************************
//  ***********************function define**********************
//  ***********************************************************/
 
//  static void __app_display_msg_handle(DISPLAY_MSG_T *msg_data)
//  {
//      if (msg_data == NULL) {
//          return;
//      }
     
//      // 只释放动态分配的数据
//      if (msg_data->type != DISPLAY_MSG_SET_BACKGROUND_ROTATION && 
//          msg_data->payload.dynamic_data.data != NULL) {
//          tkl_system_psram_free(msg_data->payload.dynamic_data.data);
//          msg_data->payload.dynamic_data.data = NULL;
//      }
//  }
 
//  // 指南针定时器回调
//  static void __compass_timer_cb(lv_timer_t *timer)
//  {
//      (void)timer;
     
//      float angle_deg;
//      if (app_compass_get_angle(&angle_deg) == OPRT_OK) {
//          // 创建旋转消息
//          DISPLAY_MSG_T msg;
//          msg.type = DISPLAY_MSG_SET_BACKGROUND_ROTATION;
//          msg.payload.rotation_data.rotation = (int16_t)(angle_deg * 10);
         
//          // 发送消息到显示队列
//          tal_queue_post(sg_display.queue_hdl, &msg, 0);
//      }
//  }
 
//  static void __display_task(void *args)
//  {
//      (void)args;
 
//      tuya_lvgl_mutex_lock();
     
//      // 创建背景图对象
//      sg_background_img = lv_image_create(lv_scr_act());
//      if (!sg_background_img) {
//          PR_ERR("Failed to create background image object");
//      } else {
//          // 设置背景图并居中
//          lv_image_set_src(sg_background_img, &background);
//          lv_obj_align(sg_background_img, LV_ALIGN_CENTER, 0, 0);
         
//          // 设置图片缩放模式
//          lv_img_set_zoom(sg_background_img, 256); // 256 = 100% 缩放
//          lv_image_set_rotation(sg_background_img, 0);
//      }
     
//  #if defined(BOARD_CHOICE_WAVESHARE_ESP32_S3_TOUCH_AMOLED_1_8)
//      extern void lcd_sh8601_set_backlight(uint8_t brightness);
//      lcd_sh8601_set_backlight(80); // set backlight to 80%
//  #endif
     
//      tuya_lvgl_mutex_unlock();
//      PR_DEBUG("Display init success");
 
//      // 初始化MPU6050模块
//      if (app_mpu6050_init() != OPRT_OK) {
//          PR_ERR("MPU6050 initialization failed");
//      }
     
//      // 创建指南针定时器
//      sg_compass_timer = lv_timer_create(__compass_timer_cb, 200, NULL);
//      if (sg_compass_timer) {
//          lv_timer_set_repeat_count(sg_compass_timer, -1);  // 无限重复
//      } else {
//          PR_ERR("Failed to create compass timer");
//      }
 
//      for (;;) {
//          DISPLAY_MSG_T msg_data;
//          if (tal_queue_fetch(sg_display.queue_hdl, &msg_data, 0xFFFFFFFF) != OPRT_OK) {
//              continue;
//          }
 
//          switch (msg_data.type) {
//              case DISPLAY_MSG_SET_BACKGROUND_ROTATION:
//                  tuya_lvgl_mutex_lock();
//                  if (sg_background_img) {
//                      lv_image_set_rotation(sg_background_img, msg_data.payload.rotation_data.rotation);
//                  }
//                  tuya_lvgl_mutex_unlock();
//                  break;
                 
//              default:
//                  __app_display_msg_handle(&msg_data);
//                  break;
//          }
//      }
//  }
 
//  OPERATE_RET app_display_init(void)
//  {
//      OPERATE_RET rt = OPRT_OK;
 
//      memset(&sg_display, 0, sizeof(TUYA_DISPLAY_T));
 
//      // lvgl initialization
//      TUYA_CALL_ERR_RETURN(tuya_lvgl_init());
//      PR_DEBUG("lvgl init success");
 
//      TUYA_CALL_ERR_RETURN(tal_queue_create_init(&sg_display.queue_hdl, sizeof(DISPLAY_MSG_T), 8));
//      THREAD_CFG_T cfg = {
//          .thrdname = "display_task",
//          .priority = THREAD_PRIO_2,
//          .stackDepth = 1024 * 4,
//      };
//      TUYA_CALL_ERR_RETURN(tal_thread_create_and_start(&sg_display.thrd_hdl, NULL, NULL, __display_task, NULL, &cfg));
//      PR_DEBUG("Display task created");
 
//      return rt;
//  }
 
//  OPERATE_RET app_display_send_msg(TY_DISPLAY_TYPE_E tp, uint8_t *data, int len)
//  {
//      DISPLAY_MSG_T msg_data;
     
//      msg_data.type = tp;
//      if (len > 0 && data != NULL) {
//          msg_data.payload.dynamic_data.data = (char *)tkl_system_psram_malloc(len + 1);
//          if (NULL == msg_data.payload.dynamic_data.data) {
//              return OPRT_MALLOC_FAILED;
//          }
//          memcpy(msg_data.payload.dynamic_data.data, data, len);
//          msg_data.payload.dynamic_data.data[len] = '\0';
//          msg_data.payload.dynamic_data.len = len;
//      } else {
//          msg_data.payload.dynamic_data.data = NULL;
//          msg_data.payload.dynamic_data.len = 0;
//      }
 
//      return tal_queue_post(sg_display.queue_hdl, &msg_data, 0xFFFFFFFF);
//  }
 
//  OPERATE_RET app_display_set_rotation(int16_t rotation)
//  {
//      DISPLAY_MSG_T msg;
//      msg.type = DISPLAY_MSG_SET_BACKGROUND_ROTATION;
//      msg.payload.rotation_data.rotation = rotation;
//      return tal_queue_post(sg_display.queue_hdl, &msg, 0);
//  }
 
//  /******************** MPU6050 替代方案 ********************/
//  // 全局变量用于存储方向状态
//  static float sg_current_angle = 0.0f;
//  static uint32_t sg_last_update = 0;
 
//  OPERATE_RET app_mpu6050_init(void)
//  {
//      // 初始化MPU6050的I2C连接
//      // 这里需要实现实际的MPU6050初始化代码
//      PR_DEBUG("MPU6050 initialized");
//      sg_last_update = tal_system_get_millisecond();
//      return OPRT_OK;
//  }
 
//  OPERATE_RET app_compass_get_angle(float *angle)
//  {
//      float accel_x, accel_y, accel_z;
     
//      // 这里应该是从MPU6050读取加速度数据的实际代码
//      // 示例值 - 实际实现需要替换为真实传感器读取
//      accel_x = 0.0f;
//      accel_y = 0.0f;
//      accel_z = 1.0f;  // 假设设备垂直放置
     
//      // 计算基于加速度的方向角度
//      // 使用atan2计算设备在XY平面上的倾斜角度
//      *angle = atan2f(accel_y, accel_x) * 180.0f / M_PI;
     
//      // 转换为0-360度范围
//      if (*angle < 0) {
//          *angle += 360.0f;
//      }
     
//      // 添加平滑滤波
//      uint32_t current_time = tal_system_get_millisecond();
//      float dt = (current_time - sg_last_update) / 1000.0f;
//      sg_last_update = current_time;
     
//      // 低通滤波减少抖动
//      float alpha = 0.2f;  // 滤波系数
//      sg_current_angle = alpha * (*angle) + (1 - alpha) * sg_current_angle;
//      *angle = sg_current_angle;
     
//      return OPRT_OK;
//  }

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
 
 // 全局变量
 static float sg_accel_scale = 1.0f; // 加速度计比例因子
 static float sg_current_angle = 0.0f;
 static uint32_t sg_last_update = 0;
 
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
             int16_t rotation;  // 旋转角度 (0-3600, 单位0.1度)
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
 static lv_obj_t *sg_background_img = NULL;
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
     
     // 配置I2C引脚
     tkl_io_pinmux_config(I2C_SCL_PIN, TUYA_IIC0_SCL);
     tkl_io_pinmux_config(I2C_SDA_PIN, TUYA_IIC0_SDA);
     
     // 初始化I2C
     TUYA_IIC_BASE_CFG_T i2c_cfg = {
         .role = TUYA_IIC_MODE_MASTER,
         .speed = I2C_SPEED, // 400kHz
         .addr_width = QMC_ADDR_MODE,
     };
     tkl_i2c_init(0, &i2c_cfg);
     
     // 唤醒MPU6050
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
     
     // 读取原始加速度数据
     if (mpu6050_read_accel(&accel_x, &accel_y, &accel_z) != OPRT_OK) {
         PR_ERR("Failed to read accelerometer data");
         return OPRT_COM_ERROR;
     }
     
     // 转换为g单位
     float ax = accel_x / sg_accel_scale;
     float ay = accel_y / sg_accel_scale;
     //float az = accel_z / sg_accel_scale;
     
     // 计算设备在XY平面上的倾斜角度
     *angle = atan2f(ay, ax) * 180.0f / M_PI;
     
     // 转换为0-360度范围
     if (*angle < 0) {
         *angle += 360.0f;
     }
     
     // 添加平滑滤波
     uint32_t current_time = tal_system_get_millisecond();
     //float dt = (current_time - sg_last_update) / 1000.0f;
     sg_last_update = current_time;
     
     // 低通滤波减少抖动
     float alpha = 0.2f;  // 滤波系数
     sg_current_angle = alpha * (*angle) + (1 - alpha) * sg_current_angle;
     *angle = sg_current_angle;
     
     return OPRT_OK;
 }
 
 static void __app_display_msg_handle(DISPLAY_MSG_T *msg_data)
 {
     if (msg_data == NULL) {
         return;
     }
     
     // 只释放动态分配的数据
     if (msg_data->type != DISPLAY_MSG_SET_BACKGROUND_ROTATION && 
         msg_data->payload.dynamic_data.data != NULL) {
         tkl_system_psram_free(msg_data->payload.dynamic_data.data);
         msg_data->payload.dynamic_data.data = NULL;
     }
 }
 
 // 指南针定时器回调
 static void __compass_timer_cb(lv_timer_t *timer)
 {
     (void)timer;
     
     float angle_deg;
     if (app_compass_get_angle(&angle_deg) == OPRT_OK) {
         // 创建旋转消息
         DISPLAY_MSG_T msg;
         msg.type = DISPLAY_MSG_SET_BACKGROUND_ROTATION;
         msg.payload.rotation_data.rotation = (int16_t)(angle_deg * 10);
         
         // 发送消息到显示队列
         tal_queue_post(sg_display.queue_hdl, &msg, 0);
     }
 }
 
 static void __display_task(void *args)
 {
     (void)args;
 
     tuya_lvgl_mutex_lock();
     
     // 创建背景图对象
     sg_background_img = lv_image_create(lv_scr_act());
     if (!sg_background_img) {
         PR_ERR("Failed to create background image object");
     } else {
         // 设置背景图并居中
         lv_image_set_src(sg_background_img, &background);
         lv_obj_align(sg_background_img, LV_ALIGN_CENTER, 0, 0);
         
         // 设置图片缩放模式
         lv_img_set_zoom(sg_background_img, 256); // 256 = 100% 缩放
         lv_image_set_rotation(sg_background_img, 0);
     }
     
 #if defined(BOARD_CHOICE_WAVESHARE_ESP32_S3_TOUCH_AMOLED_1_8)
     extern void lcd_sh8601_set_backlight(uint8_t brightness);
     lcd_sh8601_set_backlight(80); // set backlight to 80%
 #endif
     
     tuya_lvgl_mutex_unlock();
     PR_DEBUG("Display init success");
 
     // 初始化MPU6050模块
     if (app_mpu6050_init() != OPRT_OK) {
         PR_ERR("MPU6050 initialization failed");
     }
     
     // 创建指南针定时器
     sg_compass_timer = lv_timer_create(__compass_timer_cb, 200, NULL);
     if (sg_compass_timer) {
         lv_timer_set_repeat_count(sg_compass_timer, -1);  // 无限重复
     } else {
         PR_ERR("Failed to create compass timer");
     }
 
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