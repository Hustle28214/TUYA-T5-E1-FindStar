/**
 * @file app_display.h
 * @brief Header file for Tuya Display System
 *
 * This header file provides the declarations for initializing the display system
 * and sending messages to the display. It includes the necessary data types and
 * function prototypes for interacting with the display functionality.
 *
 * @copyright Copyright (c) 2021-2025 Tuya Inc. All Rights Reserved.
 *
 */

#ifndef __APP_DISPLAY_H__
#define __APP_DISPLAY_H__

#include "tuya_cloud_types.h"
#include "app_compass.h"
#include "lang_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/***********************************************************
************************macro define************************
***********************************************************/
// display network status
typedef uint8_t UI_WIFI_STATUS_E;
#define UI_WIFI_STATUS_DISCONNECTED 0
#define UI_WIFI_STATUS_GOOD         1
#define UI_WIFI_STATUS_FAIR         2
#define UI_WIFI_STATUS_WEAK         3

#define EMOJI_NEUTRAL        "NEUTRAL"
#define EMOJI_SAD            "SAD"
#define EMOJI_ANGRY          "ANGRY"
#define EMOJI_SURPRISE       "SURPRISE"
#define EMOJI_CONFUSED       "CONFUSED"
#define EMOJI_THINKING       "THINKING"
#define EMOJI_HAPPY          "HAPPY"
#define EMOJI_TOUCH          "TOUCH"
#define EMOJI_FEARFUL        "FEARFUL"
#define EMOJI_DISAPPOINTED   "DISAPPOINTED"
#define EMOJI_ANNOYED        "ANNOYED"

/***********************************************************
***********************typedef define***********************
***********************************************************/
typedef enum {
    TY_DISPLAY_TP_USER_MSG,
    TY_DISPLAY_TP_ASSISTANT_MSG,
    TY_DISPLAY_TP_ASSISTANT_MSG_STREAM_START,
    TY_DISPLAY_TP_ASSISTANT_MSG_STREAM_DATA,
    TY_DISPLAY_TP_ASSISTANT_MSG_STREAM_END,
    TY_DISPLAY_TP_SYSTEM_MSG,

    TY_DISPLAY_TP_EMOTION,

    // status bar
    TY_DISPLAY_TP_STATUS,
    TY_DISPLAY_TP_NOTIFICATION,
    TY_DISPLAY_TP_NETWORK,
    TY_DISPLAY_TP_CHAT_MODE,
    DISPLAY_MSG_UNKNOWN = 0,
    DISPLAY_MSG_SET_BACKGROUND_ROTATION,
    TY_DISPLAY_TP_MAX
} TY_DISPLAY_TYPE_E;

/***********************************************************
********************function declaration********************
***********************************************************/
/**
 * @brief Initialize the display system
 *
 * @param None
 * @return OPERATE_RET Initialization result, OPRT_OK indicates success
 */
OPERATE_RET app_display_init(void);

/**
 * @brief Send display message to the display system
 *
 * @param tp Type of the display message
 * @param data Pointer to the message data
 * @param len Length of the message data
 * @return OPERATE_RET Result of sending the message, OPRT_OK indicates success
 */
OPERATE_RET app_display_send_msg(TY_DISPLAY_TYPE_E tp, uint8_t *data, int len);
OPERATE_RET app_display_set_rotation(int16_t rotation);
// typedef struct {
//     float x_min;
//     float x_max;
//     float y_min;
//     float y_max;
// } MAG_CALIBRATION_T;
OPERATE_RET app_compass_init(void);  // 改为小写 void
OPERATE_RET app_compass_get_angle(float *angle);
void app_compass_calibrate(uint32_t duration);  // 改为小写 void

#define I2C_NUMBER TUYA_I2C_NUM_1
#define I2C_SPEED TUYA_IIC_BUS_SPEED_100K
/* 软件模拟I2C接口，保留20/21引脚 */
#define I2C_SCL_PIN TUYA_IO_PIN_19
#define I2C_SDA_PIN TUYA_IO_PIN_18
#define MPU6050_ADDRESS 0x68
#define M_PI 3.14159265358979323846264338327950288
/* QMC5883L Config */

#define QMC5883L_ADDRESS 0x0D
#define QMC_ADDR_MODE TUYA_IIC_ADDRESS_7BIT
#define QMC5883L_REG_X_LSB 0x00
#define QMC5883L_REG_X_MSB 0x01
#define QMC5883L_REG_Y_LSB 0x02
#define QMC5883L_REG_Y_MSB 0x03
#define QMC5883L_REG_Z_LSB 0x04
#define QMC5883L_REG_Z_MSB 0x05
#define QMC5883L_REG_STATUS 0x06
#define QMC5883L_REG_TEMPERATURE_LSB 0x07
#define QMC5883L_REG_TEMPERATURE_HSB 0x08
#define QMC5883L_REG_CONTROL 0x09
#define QMC5883L_REG_CONTROL_2 0x0A
#define QMC5883L_REG_RESET 0x0B
#define QMC5883L_REG_RESERVED 0x0C
#define QMC5883L_CHIP_ID 0x0D
/* 注意它们的值存取都是在06H这个寄存器里面 */
/* DRDY 数据就绪标志位 这是一个引脚*/
#define QMC5883L_STATUS_DRDY 0x01
#define QMC5883L_STATUS_OVL 0x02
/* DOR: Any data has been missed (“1”) or not (“0”) */
#define QMC5883L_STATUS_DOR 0x04

// 修改 app_display.h 中的宏定义
/* Range values */
#define QMC5883L_CONFIG_2GAUSS 0x00  // 原0x0b00000000
#define QMC5883L_CONFIG_8GAUSS 0x10  // 原0x0b00010000

/* Oversampling */
#define QMC5883L_CONFIG_OS512 0x00   // 原0x0b00000000
#define QMC5883L_CONFIG_OS256 0x20   // 原0x0b01000000
#define QMC5883L_CONFIG_OS128 0x40   // 原0x0b10000000
#define QMC5883L_CONFIG_OS64  0x60   // 原0x0b11000000

/* Mode */
#define QMC5883L_CONFIG_STANDBY 0x00 // 原0x0b00000000
#define QMC5883L_CONFIG_CONT    0x01 // 原0x0b00000001

/* Output data rate */
#define QMC5883L_CONFIG_10HZ   0x00  // 原0x0b00000000
#define QMC5883L_CONFIG_50HZ   0x04  // 原0x0b00000100
#define QMC5883L_CONFIG_100HZ  0x08  // 原0x0b00001000
#define QMC5883L_CONFIG_200HZ  0x0C  // 原0x0b00001100


void qmc5883_i2c_init(void);
void qmc5883l_init(void);
void qmc5883l_device_init(void);
//OPERATE_RET qmc5883l_read_reg(uint8_t addr, uint8_t reg, uint8_t *value);
//OPERATE_RET qmc5883l_write_reg(uint8_t addr, uint8_t reg, uint8_t value);
int16_t qmc5883l_read_16bit_data(uint8_t addr, uint8_t lsb_reg, uint8_t msb_reg);
//void qmc5883l_write_data(uint8_t addr, uint8_t reg, uint8_t value);
int16_t getX(void);
int16_t getY(void);
int16_t getZ(void);
void Reset_Range(int range);
void Reset_qmc5883l_samplingrate(int sample_rate);
void Reset_qmc5883l_odr(int odr);
void Reset_QMC5883L(void);
bool qmc5883l_Ready(void);
void Reset_I2C(void);
void Reset_qmc5883l_mode(int mode);
void qmc5883l_reset(void);
void qmc5883l_reset_calibration(void);
void qmc5883l_update_calibration(float x, float y, float z);
void qmc5883l_calibration(float* x, float* y, float* z);
void test_compass(void);
#ifdef __cplusplus
}
#endif

#endif /* __APP_DISPLAY_H__ */
