#include "mpu6050.h"
#include "tkl_i2c.h"
#include "config.h"

// 初始化mpu6050
// 设备地址为0x68（AD0引脚为高电平时地址为0x69）
void mpu6050_init(void)
{
    tkl_i2c_init(I2C_NUMBER,I2C_SPEED);// I2C0, 400KHz高速传输
    tkl_i2c_master_send(MPU6050_ADDRESS,0x6B,0x00); //
}

// 0x3B，加速度计的X轴分量ACC_X
// 0x3D，加速度计的Y轴分量ACC_Y
// 0x3F，加速度计的Z轴分量ACC_Z
// 0x41，当前温度TEMP
// 0x43，绕X轴旋转的角速度GYR_X
// 0x45，绕Y轴旋转的角速度GYR_Y
// 0x47，绕Z轴旋转的角速度GYR_Z 

// 读数据

void mpu6050_read(void)
{
    uint8_t data[14];
    tkl_i2c_read(TUYA_I2C_NUM_0,MPU6050_ADDRESS,0x3B,data,14);
}

void mpu6050_write(void)
{

}

// 互补滤波算法实现

void mpu6050_filter(void)
{
    uint8_t data[14];
    tkl_i2c_read(TUYA_I2C_NUM_0,MPU6050_ADDRESS,0x3B,data,14);
}

void mpu6050_deinit(void)
{
    tkl_i2c_deinit(TUYA_I2C_NUM_0);
}// 析构函数