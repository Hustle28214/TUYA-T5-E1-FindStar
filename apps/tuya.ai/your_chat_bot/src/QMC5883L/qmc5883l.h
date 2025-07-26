#ifndef __QMC5883L_H__
#define __QMC5883L_H__

#include "tuya_cloud_types.h"
#include "tkl_i2c.h"
#include "tkl_gpio.h"
#include "tkl_pinmux.h"
#include "tkl_output.h"

void qmc5883_i2c_init(void);
void qmc5883l_init(void);
void qmc5883l_device_init(void);
//OPERATE_RET qmc5883l_read_reg(uint8_t addr, uint8_t reg, uint8_t *value);
//OPERATE_RET qmc5883l_write_reg(uint8_t addr, uint8_t reg, uint8_t value);
uint8_t qmc5883l_read_16bit_data(uint8_t addr, uint8_t lsb_reg, uint8_t msb_reg);
//void qmc5883l_write_data(uint8_t addr, uint8_t reg, uint8_t value);
void getX(void);
void getY(void);
void getZ(void);
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
//void qmc5883l_calibration_example(void);


#endif