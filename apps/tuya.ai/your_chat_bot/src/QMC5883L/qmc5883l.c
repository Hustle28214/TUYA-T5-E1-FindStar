#include "qmc5883l.h"
#include "tkl_i2c.h"
#include "config.h"
#include "tkl_pinmux.h"
#include "tkl_output.h"
#include "tkl_gpio.h"
#include "tuya_cloud_types.h"

// 注意这里示例中TUYA_IIC0_SCL是软件模拟的
// 通过io pinmux初始化I2C接口

// 注：About: tkl_i2c_master_send()/**
//  * @brief i2c master send
//  *
//  * @param[in] port: i2c port
//  * @param[in] dev_addr: iic addrress of slave device.
//  * @param[in] data: i2c data to send
//  * @param[in] size: Number of data items to send
//  * @param[in] xfer_pending: xfer_pending: TRUE : not send stop condition, FALSE : send stop condition.
//  * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
//  */
// OPERATE_RET tkl_i2c_master_send(TUYA_I2C_NUM_E port, uint16_t dev_addr, const void *data, uint32_t size,
//                                 BOOL_T xfer_pending);

// TDD_TOUCH_I2C_CFG_T qmc5883_cfg = {
//     .i2c_num = I2C_NUMBER,
//     .i2c_speed = I2C_SPEED,
//     .addr_width = QMC_ADDR_MODE,
// }; // 本来打算用TDD_TOUCH的结果发现有cloud的i2c接口，就不拿屏驱的函数做了

// 注：TUYA的I2C主设备发送函数
// OPERATE_RET tkl_i2c_master_send(TUYA_I2C_NUM_E port, uint16_t dev_addr, const void *data, uint32_t size,
//     BOOL_T xfer_pending);
//  * @param[in] size: Number of data items to send
// * @param[in] xfer_pending: xfer_pending: TRUE : not send stop condition, FALSE : send stop condition.
// 没默认值所以必须要指定是否发送停止位，这里不发
int odr=QMC5883L_CONFIG_OS512
int _range =QMC5883L_CONFIG_2GAUSS;
int sample_rate=QMC5883L_CONFIG_50HZ;
int mode = QMC5883L_CONFIG_CONT;//09H CONFIG 控制位需要的数值
void qmc5883l_i2c_init(void){
    tkl_i2c_init(I2C_NUMBER,qmc5883_cfg);// I2C0, 400KHz高速传输
    qmc5883l_write_reg(QMC5883L_ADDRESS,QMC5883L_REG_CONTROL,QMC5883L_CONFIG_OS512|QMC5883L_CONFIG_2GAUSS|QMC5883L_CONFIG_50HZ|QMC5883L_CONFIG_CONT);
    // 或者换一种方式，不使用寄存器写指令直接指定？这样也许更安全，可以当备份写法
}
void qmc5883l_device_init(void){
    qmc5883l_write_reg(QMC5883L_ADDRESS,QMC5883L_REG_CONTROL,odr|_range|sample_rate|mode);// 详见寄存器地址0x09的存储四个数值介绍
}

void qmc5883l_init(void)
{   
    tkl_io_pinmux_config(I2C_SCL_PIN, TUYA_IIC0_SCL);
    tkl_io_pinmux_config(I2C_SDA_PIN, TUYA_IIC0_SDA);
    TUYA_IIC_BASE_CFG_T i2c_cfg = {
        .role = TUYA_IIC_MODE_MASTER,
        .speed = I2C_SPEED,
        .addr_width = QMC_ADDR_MODE,
    };
    tkl_i2c_init(I2C_NUMBER,&i2c_cfg);// I2C0, 400KHz高速传输，看情况是否要单独弄一个config函数
    qmc5883l_reset();
    qmc5883l_device_init();
    tkl_i2c_master_send(I2C_NUMBER,QMC5883L_ADDRESS,QMC5883L_REG_CONTROL,0x00,FALSE); // 不发送停止位，TODO: 核实size该填什么
}
// 用静态标识定义设备底层读写指令
static OPERATE_RET qmc5883l_read_reg(uint8_t addr, uint8_t reg, uint8_t *value) {
    OPERATE_RET ret = tkl_i2c_master_send(I2C_NUMBER, addr, &reg, 1, TRUE);
    if (ret != OPRT_OK) {
        tkl_output_log("I2C send error: %d", ret);
        return ret;
    }
    
    ret = tkl_i2c_master_receive(I2C_NUMBER, addr, value, 1, FALSE);
    if (ret != OPRT_OK) {
        tkl_output_log("I2C receive error: %d", ret);
    }
    return ret;
}
static OPERATE_RET qmc5883l_write_reg(uint8_t addr, uint8_t reg, uint8_t value){
    // 搞清楚i2c的主设备send函数需要怎么封装数据？
    
    uint8_t data[2];
    data[0] = reg;
    data[1] = value;
    
    OPERATE_RET ret = tkl_i2c_master_send(I2C_NUMBER, QMC5883L_ADDRESS, data, 2, FALSE);
    if (ret != OPRT_OK) {
        tkl_output_log("I2C write error: %d", ret);
    }
    return ret;

}
// 调用指令层的函数其实也可以不用写addr，上面直接写死。不过为了作为模板独立开来使用还是每次指定一下addr
int16_t qmc5883l_read_16bit_data(uint8_t addr, uint8_t lsb_reg, uint8_t msb_reg){
    uint8_t data[6];
    uint8_t lsb,msb;
    if (qmc5883l_read_reg(addr,lsb_reg,&lsb)!= OPRT_OK) return 0;
    if (qmc5883l_read_reg(addr,msb_reg,&msb)!= OPRT_OK) return 0;
    return (int16_t)((msb << 8) | lsb);
}// 大小端组合的封装函数

// void qmc5883l_write_data(uint8_t addr, uint8_t reg, uint8_t value){
//     //uint8_t data[6];
//     // TODO: 还没写好
//     qmc5883l_write_reg(addr,reg,value);
// }

// 获取X轴数据
int16_t getX(void){
    // Reg: 0x00, 0x01
    // 相当于低位和高位组在一起发送
    // uint8_t data[2];
    // data[0] = qmc5883l_read_data(QMC5883L_ADDRESS,QMC5883L_REG_X_LSB);
    // data[1] = qmc5883l_read_data(QMC5883L_ADDRESS,QMC5883L_REG_X_MSB);
    // // xfer_pending: xfer_pending: TRUE : not send stop condition, FALSE : send stop condition.
    // return data;
    return read_16bit_data(QMC5883L_REG_X_LSB, QMC5883L_REG_X_MSB);
}

// 获取Y轴数据
int16_t getY(void){
    // // Reg: 0x02, 0x03
    // uint8_t data[2];
    // data[0] = qmc5883l_read_data(QMC5883L_ADDRESS,QMC5883L_REG_Y_LSB);
    // data[1] = qmc5883l_read_data(QMC5883L_ADDRESS,QMC5883L_REG_Y_MSB);
    // return data;
    return read_16bit_data(QMC5883L_REG_Y_LSB, QMC5883L_REG_Y_MSB);
}
// 获取Z轴数据
int16_t getZ(void){
    // // Reg: 0x02, 0x03
    // uint8_t data[2];
    // data[0] = qmc5883l_read_data(QMC5883L_ADDRESS,QMC5883L_REG_Z_LSB);
    // data[1] = qmc5883l_read_data(QMC5883L_ADDRESS,QMC5883L_REG_Z_MSB);
    // return data;
    return read_16bit_data(QMC5883L_REG_Z_LSB, QMC5883L_REG_Z_MSB);
}

//TODO: 或许可以添加一个平滑函数，用来处理磁力计数据的抖动，但暂时raw也够用
void Reset_Range(int range){
    switch(x) {
        case 2:
          range = QMC5883L_CONFIG_2GAUSS;
          break;
        case 8:
          range = QMC5883L_CONFIG_8GAUSS;
          break;
      }
      qmc5883l_device_init();
}
void Reset_qmc5883l_odr(int odr){
    switch(odr){
        case 64:
            odr = QMC5883L_CONFIG_OS64;
            break;
        case 128:
            odr = QMC5883L_CONFIG_OS128;
            break;
        case 256:
            odr = QMC5883L_CONFIG_OS256;
            break;
        case 512:
            odr = QMC5883L_CONFIG_OS512;
            break;
    }
    qmc5883l_device_init()
}
void Reset_qmc5883l_samplingrate(int sample_rate){
    // 采样速率选择，只能选择四个速率当中的一个
    switch (sample_rate)
    {
    case 10:
        sample_rate=QMC5883L_CONFIG_10HZ;
        break;
    case 50:
        sample_rate=QMC5883L_CONFIG_50HZ;
        break;
    case 100:
        sample_rate=QMC5883L_CONFIG_100HZ;
        break;
    case 200:
        sample_rate=QMC5883L_CONFIG_200HZ;
        break;
    }
    qmc5883l_device_init();
}
void Reset_qmc5883l_mode(int mode){
    switch (mode)
    {
    case 0:
        mode = QMC5883L_CONFIG_STANDBY;
        break;
    case 1:
        mode = QMC5883L_CONFIG_CONT;
        break;
    }
    qmc5883l_device_init();
}
void Reset_QMC5883L(void)
{
    tkl_i2c_master_send(QMC5883L_ADDRESS,QMC5883L_REG_RESET,0x01);
    // size: Number of data items to receive
}

// void qmc5883l_Ready(void){
//     if(!qmc5883l_read_data(QMC5883L_ADDRESS,QMC5883L_REG_STATUS))
//     {
//         return 0;
//     }
//     // TODO: 找到status状态位
// }

bool qmc5883l_ready(void) {
    uint8_t status;
    if (qmc5883l_read_reg(QMC5883L_ADDRESS, QMC5883L_REG_STATUS, &status) != OPRT_OK) {
        return false;
    }
    return (status & 0x01) != 0; // 根据数据手册调整
}


void Reset_I2C(void){
    // I2C的析构函数
    tkl_i2c_reset(I2C_NUMBER);
}
void qmc5883l_reset(void) {
    qmc5883l_write_reg(QMC5883L_ADDRESS, QMC5883L_REG_RESET, 0x01);
}
/* 校准磁力计部分 */

typedef struct {
    float x_min = 0,
    float x_max = 0,
    float y_min = 0,
    float y_max = 0,
    float z_min = 0,
    float z_max = 0,
} MagCalibration; // 磁力计校准结构体

MagCalibration mag_calibration = {0};

void qmc5883l_reset_calibration(void){
    mag_calibration.x_min = -32768.0f;
    mag_calibration.x_max = 32767.0f;
    mag_calibration.y_min = -32768.0f;
    mag_calibration.y_max = 32767.0f;
    mag_calibration.z_min = -32768.0f;
    mag_calibration.z_max = 32767.0f;
}

void qmc5883l_update_calibration(float x, float y, float z){
    float current_x = x;
    float current_y = y;
    float current_z = z;
    if(x < mag_calibration.x_min){
        mag_calibration.x_min = current_x;
    }
    if(x > mag_calibration.x_max){
        mag_calibration.x_max = current_x;
    }
    if(y < mag_calibration.y_min){
        mag_calibration.y_min = current_y;
    }
    if(y > mag_calibration.y_max){
        mag_calibration.y_max = current_y;
    }
    if(z < mag_calibration.z_min){
        mag_calibration.z_min = current_z;
    }
    if(z > mag_calibration.z_max){
        mag_calibration.z_max = current_z;
    }
}

void qmc5883l_calibration(float* x, float* y, float* z){
    qmc5883l_reset_calibration();// 先置0
    // 校准坐标和范围
    float offset_x = (mag_calibration.x_min + mag_calibration.x_max) / 2.0f;
    float offset_y = (mag_calibration.y_min + mag_calibration.y_max) / 2.0f;
    float offset_z = (mag_calibration.z_min + mag_calibration.z_max) / 2.0f;
    float range_x = mag_calibration.x_max - mag_calibration.x_min;
    float range_y = mag_calibration.y_max - mag_calibration.y_min;
    float range_z = mag_calibration.z_max - mag_calibration.z_min;
    float avg_range = (range_x + range_y) / 2.0f;// 不计算z轴
    float scale_x = avg_range / range_x;
    float scale_y = avg_range / range_y;
    float scale_z = 1.0f; // Z轴不需要缩放
    
    // 应用校准
    *x = (float)((*x - offset_x) * scale_x);
    *y = (float)((*y - offset_y) * scale_y);
    *z = (float)((*z - offset_z) * scale_z);
}


// 示例校准封装函数

// void qmc5883l_calibration_example(void){
//     qmc5883l_reset_calibration();

//     current_x = getX();
//     current_y = getY();
//     current_z = getZ();
// }

// 为了追求高精度可以结合MPU6050进行校准

// void Reset_qmc5883l_Addr(uint16_t new_addr){
//     // 重新指定QMC5883L的设备地址（应该用不上）
//     QMC5883L_ADDRESS = new_addr;
// }