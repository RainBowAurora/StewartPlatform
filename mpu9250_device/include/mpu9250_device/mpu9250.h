#ifndef __MPU9250_H__
#define __MPU9250_H__

#include "mpu9250_device/i2c.h"

#define IMU_DEG2RAD 0.017453293f
#define IMU_RAD2DEG 57.29578f

#define WHO_AMIK8963    0X00
#define INFO            0x10
#define AK8963_ST1      0x02
#define AK8963_XOUT_L   0x03
#define AK8963_XOUT_H   0x04
#define AK8963_YOUT_L   0X05
#define AK8963_YOUT_H   0X06
#define AK8963_ZOUT_L   0X07
#define AK8963_ZOUT_H   0X08
#define AK8971_ST2      0X09
#define AK8963_CNTL     0X0A
#define AK8963_CNTL2    0X0B
#define AK8963_ASTC     0X0C
#define AK8963_I2CDIS   0X0F
#define AK8963_ASAX     0X10
#define AK8963_ASAY     0x11
#define AK8963_ASAZ     0x12

#define SELF_TEST_X_GYRO    0x00
#define SELF_TEST_Y_GYRO    0X01
#define SELF_TEST_Z_GYRO    0x02

#define SELF_TEST_X_ACCEL   0x0D
#define SELF_TEST_Y_ACCEL   0x0E
#define SELF_TEST_Z_ACCEL   0X0F

#define SELF_TEST_A     0x10
#define XG_OFFSET_H     0x13
#define XG_OFFSET_L     0X14
#define YG_OFFSET_H     0X15
#define YG_OFFSET_L     0X16
#define ZG_OFFSET_H     0X17
#define ZG_OFFSET_L     0X18
#define SMPLRT_DIV      0x19
#define CONFIG          0X1A
#define GYRO_CONFIG     0X1B
#define ACCEL_CONFIG    0X1C
#define ACCEL_CONFIG2   0X1D
#define LP_ACCEL_ODR    0x1E
#define WOM_THR         0X1F

#define MOT_DUR         0X20
#define ZMOT_THR        0X21
#define ZRMOT_DUR       0X22

#define FIFO_EN         0X23

#define I2C_MST_CTRL    0X24
#define I2C_SLV0_ADDR   0X25
#define I2C_SLV0_REG    0X26
#define I2C_SLV0_CTRL   0X27

#define I2C_SLV1_ADDR   0X28
#define I2C_SLV1_REG    0X29
#define I2C_SLV1_CTRL   0X2A

#define I2C_SLV2_ADDR   0X2B
#define I2C_SLV2_REG    0X2C
#define I2C_SLV2_CTRL   0X2D

#define I2C_SLV3_ADDR   0X2E
#define I2C_SLV3_REG    0X2F
#define I2C_SLV3_CTRL   0X30

#define I2C_SLV4_ADDR   0X31
#define I2C_SLV4_REG    0X32
#define I2C_SLV4_DO     0x33
#define I2C_SLV4_CTRL   0X34
#define I2C_SLV4_DI     0X35

#define I2C_MST_STATUS  0X36
#define INT_PIN_CFG     0X37
#define INT_ENABLE      0X38
#define DMP_INT_STATUS  0X39
#define INT_STATUS      0X3A
#define ACCEL_XOUT_H    0X3B
#define ACCEL_XOUT_L    0X3C
#define ACCEL_YOUT_H    0X3D
#define ACCEL_YOUT_L    0X3E
#define ACCEL_ZOUT_H    0X3F
#define ACCEL_ZOUT_L    0X40
#define TEMP_OUT_H      0X41
#define TEMP_OUT_L      0X42
#define GYRO_XOUT_H     0X43
#define GYRO_XOUT_L     0X44
#define GYRO_YOUT_H     0X45
#define GYRO_YOUT_L     0X46
#define GYRO_ZOUT_H     0X47
#define GYRO_ZOUT_L     0X48

#define EXT_SENS_DATA_00    0X49
#define EXT_SENS_DATA_01    0X4A
#define EXT_SENS_DATA_02    0X4B
#define EXT_SENS_DATA_03    0X4C
#define EXT_SENS_DATA_04    0X4D
#define EXT_SENS_DATA_05    0X4E
#define EXT_SENS_DATA_06    0X4F
#define EXT_SENS_DATA_07    0X50
#define EXT_SENS_DATA_08    0X51
#define EXT_SENS_DATA_09    0X52
#define EXT_SENS_DATA_10    0X53
#define EXT_SENS_DATA_11    0X54
#define EXT_SENS_DATA_12    0X55
#define EXT_SENS_DATA_13    0X56
#define EXT_SENS_DATA_14    0X57
#define EXT_SENS_DATA_15    0X58
#define EXT_SENS_DATA_16    0X59
#define EXT_SENS_DATA_17    0X5A
#define EXT_SENS_DATA_18    0X5B
#define EXT_SENS_DATA_19    0X5C
#define EXT_SENS_DATA_20    0X5D
#define EXT_SENS_DATA_21    0X5E
#define EXT_SENS_DATA_22    0X5F
#define EXT_SENS_DATA_23    0X60
#define MOT_DETECT_STATUS   0X61

#define I2C_SLV0_DO         0X63
#define I2C_SLV1_DO         0X64
#define I2C_SLV2_DO         0X65
#define I2C_SLV3_DO         0X66
#define I2C_MST_DELAY_CTRL  0X67
#define SIGNAL_PATH_REST    0X68
#define MOT_DETECT_CTRL     0X69
#define USER_CTRL           0X6A
#define PWR_MGMT_1          0X6B
#define PWR_MGMT_2          0X6C
#define DMP_BANK            0X6D
#define DMP_RW_PNT          0X6E
#define DMP_REG             0X6F
#define DMP_REG_1           0X70
#define DMP_REG_2           0X71
#define FIFO_COUNTH         0X72
#define FIFO_COUNTL         0X73
#define FIFO_R_W            0X74
#define WHO_AM_I_MPU9250    0X75
#define XA_OFFSET_H         0X77
#define XA_OFFSET_L         0X78
#define YA_OFFSET_H         0X7A
#define YA_OFFSET_L         0X7B
#define ZA_OFFSET_H         0X7D
#define ZA_OFFSET_L         0X7E

#define AFS_2G  0
#define AFS_4G  1
#define AFS_8G  2
#define AFS_16G 3

#define GFS_250DPS  0
#define GFS_500DPS  1
#define GFS_1000DPS 2
#define GFS_2000DPS 3

#define MFS_14BITS  0   //0.6mG per LSB
#define MFS_16BITS  1   //0.15mG per LSB

#define M_8Hz   0x02
#define M_100Hz 0x06

#define ADO 0
#if ADO
#define MPU9250_ADDRESS 0X69    //Device address when ADO = 1
#define AK8963_ADDRESS  0X0C    //Address of magnetometer
#else
#define MPU9250_ADDRESS 0X68    //Device address when ADO = 0
#define AK8963_ADDRESS  0X0C    //Address of magnetometer
#endif


#define Ascale_ AFS_16G
#define Gscale_ GFS_500DPS
#define Mscale_ MFS_16BITS

template<class T>
struct _sensor{
    T x;
    T y;
    T z;
};

typedef struct {
    struct _sensor<int16_t> gyro_original;
    struct _sensor<int16_t> accel_original;
    struct _sensor<int16_t> mag_original;

    struct _sensor<float> gyro;
    struct _sensor<float> accel;
    struct _sensor<float> mag;

    int16_t temp_original; //temperature
}_ins;

class mpu9250{
public:
    mpu9250(std::string dev_name, int addr);
    ~mpu9250();
    void update_sensor(_ins *destination);
    void gyromagWake(uint8_t);
    void gyromagSleep();
    void accelWakeOnMotion();

protected:

private:
    i2c_dev _i2c;
    float _magCalibration[3];
    uint8_t _Mmode;
    float _aRes;
    float _mRes;
    float _gRes;
    uint8_t _mpu9250_id;
    uint8_t _ak8963_id;
    uint8_t _status;
    void resetMPU9250();
    void MPU9250_Init(uint8_t, uint8_t, uint8_t);
    void AK8963_Init(uint8_t, uint8_t);
    void AK8963Slave_Init(uint8_t, uint8_t);
    bool checkNewAccelGyroData();
    bool checkNewMagData();
    bool checkWakeOnMotion();
    uint8_t getAK8963CID();
    uint8_t getMPU9250ID();
    void getAres(uint8_t Ascale);
    void getGres(uint8_t Gscale);
    void getMres(uint8_t Mscale);
    void readMPU9250Data(_ins *);
    void readAccelData(_ins *);
    void readGyroData(_ins *);
    void readMagData(_ins *data);
    void readTempData(_ins *);

};

#endif /*__MPU9250_H__*/
