#ifndef __QMC5883L_H__
#define __QMC5883L_H__

#include "tuya_cloud_types.h"
#include "tkl_i2c.h"
#include "tkl_gpio.h"
#include "tkl_pinmux.h"
#include "tkl_output.h"

void qmc5883_i2c_init(void);
void qmc5883l_init(void);
static void qmc5883l_read_reg(uint8_t addr, uint8_t reg);
static void qmc5883l_write_reg(uint8_t addr, uint8_t reg, uint8_t value);
uint8_t qmc5883l_read_data(uint8_t addr, uint8_t reg);
void qmc5883l_write_data(uint8_t addr, uint8_t reg, uint8_t value);
void getX(void);
void getY(void);
void getZ(void);
void Reset_qmc5883l_samplingrate(int sample_rate);
void Reset_QMC5883L(void);
void qmc5883l_Ready(void);
void Reset_I2C(void);
void qmc5883l_reset_calibration(void);
void qmc5883l_update_calibration(float x, float y, float z);
void qmc5883l_calibration(float* x, float* y, float* z);



#endif