/*********************************************************************                                                                                * Software License Agreement (BSD License)
*
*  Copyright (c) 2018, RainBowAurora
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the copyright holder nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include "mpu9250_device/mpu9250.h"

mpu9250::mpu9250(std::string dev_name, int addr):_i2c(dev_name, addr)
{
    getMPU9250ID();

    if(_mpu9250_id != 0x73){
        ROS_ERROR("MPU9250 not accessed!~");
    }else{
        ROS_INFO("Get MPU9250 ID: 0x%02x", _mpu9250_id);
    }    
    MPU9250_Init(Ascale_, Gscale_, 0x04);
    AK8963Slave_Init(Mscale_, M_100Hz);

    getAK8963CID();

    if(_ak8963_id != 0x48){
        ROS_ERROR("AK8963 not accessed!~");
    }else{
        ROS_INFO("Get MPU9250 ID: 0x%02x", _ak8963_id);
    }  
    getAres(Ascale_);
    getGres(Gscale_);                                                                                                               
    getMres(Mscale_);
}

mpu9250::~mpu9250()
{

}


uint8_t mpu9250::getMPU9250ID()
{
    _mpu9250_id = _i2c.i2c_read(WHO_AM_I_MPU9250);
    return _mpu9250_id;
}

uint8_t mpu9250::getAK8963CID()
{
    _i2c.i2c_write(USER_CTRL, 0x20);
    _i2c.i2c_write(I2C_MST_CTRL, 0x0D);
    _i2c.i2c_write(I2C_SLV0_ADDR,  AK8963_ADDRESS | 0x80);
    _i2c.i2c_write(I2C_SLV0_REG, WHO_AMIK8963);
    _i2c.i2c_write(I2C_SLV0_CTRL, 0x81);
    _i2c.delay_ms(10);
    _ak8963_id = _i2c.i2c_read(EXT_SENS_DATA_00)   ;
    return _ak8963_id;
}


void mpu9250::MPU9250_Init(uint8_t Ascale, uint8_t Gscale, uint8_t sampleRate)
{
    //Clear sleep mode bit(6) enable all sensors
    _i2c.i2c_write(PWR_MGMT_1, 0x00);

    _i2c.delay_us(100);

    _i2c.i2c_write(PWR_MGMT_1, 0x01);

    _i2c.delay_us(200);

    _i2c.i2c_write(CONFIG, 0x03);

    _i2c.i2c_write(SMPLRT_DIV, sampleRate);

    uint8_t c;
    _i2c.i2c_read(SMPLRT_DIV, &c);
    c = c & ~0x02; //clear Fchoice bits[1:0]
    c = c & ~0x18; //clear AFS bits[4:3]
    c = c | Gscale << 3; //set full scale range for the gyro
    _i2c.i2c_write(GYRO_CONFIG, c); //write new GYRO_CONFIG value to register

    //set accelerometer full-scale range configuration
    _i2c.i2c_read(ACCEL_CONFIG, &c); //get current ACCEL_CONFIG register value
    c = c & ~0x18; //Clear AFS bits [4:3]
    c = c | Ascale << 3; //Set full scale range for the accelerometer
    _i2c.i2c_write(ACCEL_CONFIG, c); //Write new ACCEL_CONFIG register  value

    _i2c.i2c_read(ACCEL_CONFIG2, &c); //get current ACCEL_CONFIG2 register value
    c = c & ~0X0F; //Clear accel_fchoice_b (bit3) and A_DLPFG(bits[2:0])
    c = c | 0x03; //set accelerometer rate to 1KHz and bandwidth to 41Hz
    _i2c.i2c_write(ACCEL_CONFIG2, c); //Write new ACCEL_CONFIG2 register value

    _i2c.i2c_write(INT_PIN_CFG, 0x10); //INT is 50 microsecond pulse and any read to clear
    _i2c.i2c_write(INT_ENABLE, 0x01); // Enable data ready (bit 0) interrupt

    _i2c.delay_us(200); 

   _i2c.i2c_write(USER_CTRL, 0x20);  //Enable I2C Master mode
   _i2c.i2c_write(I2C_MST_CTRL, 0x1D); //I2C configuration STOP after each transaction master I2Cbus at 400Hz
   _i2c.i2c_write(I2C_MST_DELAY_CTRL, 0X81); //Use blocking data retreival and enable dely for mag sample rate mismatch
   _i2c.i2c_write(I2C_SLV4_CTRL, 0x01); //Delay mag data retrieval to once every other accel/gyro data sample    
}

void mpu9250::AK8963_Init(uint8_t Mscale, uint8_t Mmode)
{
    //First extract the factory calibration for each magnetometer axis
    unsigned char rawData[3];
    _i2c.i2c_write(AK8963_CNTL, 0x00);
    _i2c.delay_ms(10);
    _i2c.i2c_write(AK8963_CNTL, 0x0F);
    _i2c.delay_ms(10);
    _i2c.i2c_read(AK8963_ASAX, rawData, 3);
   
   _magCalibration[0] = (float)(rawData[0] - 128)/256.0f +1.0f;
   _magCalibration[1] = (float)(rawData[1] - 128)/256.0f +1.0f;
   _magCalibration[2] = (float)(rawData[2] - 128)/256.0f +1.0f;
  
  _Mmode = Mmode;     

  _i2c.i2c_write(AK8963_CNTL, 0x00);
  _i2c.delay_ms(10);
  _i2c.i2c_write(AK8963_CNTL, Mscale << 4 | Mmode);
  _i2c.delay_ms(10);  
}


void mpu9250::AK8963Slave_Init(uint8_t Mscale, uint8_t Mmode)
{
    uint8_t rawData[3];
    _i2c.i2c_write(I2C_SLV0_ADDR, AK8963_ADDRESS);
    _i2c.i2c_write(I2C_SLV0_REG, AK8963_CNTL2);
    _i2c.i2c_write(I2C_SLV0_DO, 0x01);
    _i2c.i2c_write(I2C_SLV0_CTRL, 0x81);
    _i2c.delay_ms(50);
    _i2c.i2c_write(I2C_SLV0_ADDR, AK8963_ADDRESS);
    _i2c.i2c_write(I2C_SLV0_REG, AK8963_CNTL);
    _i2c.i2c_write(I2C_SLV0_DO, 0x00);
    _i2c.i2c_write(I2C_SLV0_CTRL, 0x81);
    _i2c.delay_ms(50);
    _i2c.i2c_write(I2C_SLV0_ADDR, AK8963_ADDRESS);
    _i2c.i2c_write(I2C_SLV0_REG, AK8963_CNTL);
    _i2c.i2c_write(I2C_SLV0_DO, 0X0F);
    _i2c.i2c_write(I2C_SLV0_CTRL, 0x81);
    _i2c.delay_ms(50);

    _i2c.i2c_write(I2C_SLV0_ADDR, AK8963_ADDRESS | 0x80);
    _i2c.i2c_write(I2C_SLV0_REG, AK8963_ASAX);
    _i2c.i2c_write(I2C_SLV0_CTRL, 0x83);
    _i2c.delay_ms(50);

    _i2c.i2c_read(EXT_SENS_DATA_00, rawData, 3);
    _magCalibration[0] = (float)(rawData[0] - 128)/256.0f + 1.0f;
    _magCalibration[1] = (float)(rawData[1] - 128)/256.0f + 1.0f;
    _magCalibration[2] = (float)(rawData[2] - 128)/256.0f + 1.0f;
    _Mmode = Mmode;

    _i2c.i2c_write(I2C_SLV0_ADDR, AK8963_ADDRESS);
    _i2c.i2c_write(I2C_SLV0_REG, AK8963_CNTL);
    _i2c.i2c_write(I2C_SLV0_DO, 0x00);
    _i2c.i2c_write(I2C_SLV0_CTRL, 0x81);
    _i2c.delay_ms(50);
    _i2c.i2c_write(I2C_SLV0_ADDR, AK8963_ADDRESS);
    _i2c.i2c_write(I2C_SLV0_REG, AK8963_CNTL);

    _i2c.i2c_write(I2C_SLV0_DO, Mscale << 4 | Mmode);
    _i2c.i2c_write(I2C_SLV0_CTRL, 0x81);
    _i2c.delay_ms(50);

    _i2c.i2c_write(I2C_SLV0_ADDR, AK8963_ADDRESS | 0x80);
    _i2c.i2c_write(I2C_SLV0_REG, AK8963_CNTL);
    _i2c.i2c_write(I2C_SLV0_CTRL, 0x81);
    _i2c.delay_ms(50);
}

void mpu9250::resetMPU9250()                                                                                                            
{
    _i2c.i2c_write(PWR_MGMT_1, 0x80);
    _i2c.delay_ms(100);

}

void mpu9250::getAres(uint8_t Ascale)
{
    switch(Ascale){
    case AFS_2G:
        _aRes = 2.0f / 32768.0f;
        break;
    case AFS_4G:
       _aRes = 4.0f / 32768.0f;
        break;
    case AFS_8G:
        _aRes = 8.0f / 32768.0f;
        break;
    case AFS_16G:
        _aRes = 16.0f / 32768.0f;
        break;
    }
}

void mpu9250::getGres(uint8_t Gscale){
    switch(Gscale){
    case GFS_250DPS:
        _gRes = 250.0f / 32768.0f;
        break;
    case GFS_500DPS:
        _gRes = 500.0f / 32768.0f;
        break;
    case GFS_1000DPS:
         _gRes = 1000.0f / 32768.0f;
        break;
    case GFS_2000DPS:
        _gRes = 2000.0f / 32768.0f;
        break;
    }
}

void mpu9250::getMres(uint8_t Mscale)
{
    switch(Mscale){
        case MFS_14BITS:
            _mRes = 10.0 * 4912.0 / 8190.0;
        break;
        case MFS_16BITS:
            _mRes = 10.0 * 4912.0 / 32760.0;
        break;
    }
}

void mpu9250::update_sensor(_ins *destination)
{
    readMPU9250Data(destination);
    destination->gyro.x = (float)destination->gyro_original.x * _gRes * IMU_DEG2RAD;
    destination->gyro.y = (float)destination->gyro_original.y * _gRes * IMU_DEG2RAD;
    destination->gyro.z = (float)destination->gyro_original.z * _gRes * IMU_DEG2RAD;

    destination->accel.x = (float)destination->accel_original.x * _aRes;
    destination->accel.y = (float)destination->accel_original.y * _aRes;
    destination->accel.z = (float)destination->accel_original.z * _aRes;

    readMagData(destination);
    destination->mag.x = (float)destination->mag_original.x * _mRes * _magCalibration[0];
    destination->mag.y = (float)destination->mag_original.y * _mRes * _magCalibration[1];
    destination->mag.z = (float)destination->mag_original.z * _mRes * _magCalibration[2];
}

void mpu9250::readMPU9250Data(_ins *destination)
{
    uint8_t rawData[14];
    _i2c.i2c_read(ACCEL_XOUT_H, rawData, 14);

    destination->accel_original.x = ((int16_t)rawData[0]  << 8) | rawData[1];
    destination->accel_original.y = ((int16_t)rawData[2]  << 8) | rawData[3];
    destination->accel_original.z = ((int16_t)rawData[4]  << 8) | rawData[5];

    destination->temp_original = ((int16_t)rawData[6]  << 8) | rawData[7];

    destination->gyro_original.x = ((int16_t)rawData[8]  << 8) | rawData[9];
    destination->gyro_original.y = ((int16_t)rawData[10]  << 8) | rawData[11];
    destination->gyro_original.z = ((int16_t)rawData[12]  << 8) | rawData[13];    
}

void mpu9250::readAccelData(_ins *destination)
{
    uint8_t rawData[6];
    _i2c.i2c_read(ACCEL_XOUT_H, rawData, 6); 
    destination->accel_original.x = ((int16_t)rawData[0] << 8) | rawData[1];
    destination->accel_original.y = ((int16_t)rawData[2] << 8) | rawData[3];
    destination->accel_original.z = ((int16_t)rawData[4] << 8) | rawData[5];
}

void mpu9250::readGyroData(_ins *destination)
{
    uint8_t rawData[6];
    _i2c.i2c_read(GYRO_XOUT_H, rawData, 6);
    destination->gyro_original.x = ((int16_t)rawData[0] <<8 ) | rawData[1];
    destination->gyro_original.y = ((int16_t)rawData[2] <<8 ) | rawData[3];
    destination->gyro_original.z = ((int16_t)rawData[4] <<8 ) | rawData[5];
}

void  mpu9250::readTempData(_ins *data)
{
    uint8_t rawData[2];
    _i2c.i2c_read(TEMP_OUT_H, rawData, 2);
    data->temp_original = ((int16_t)rawData[1] << 8) | rawData[0];                                                                                   
}

void mpu9250::readMagData(_ins *data)
{
    uint8_t rawData[7];
    _i2c.i2c_write(I2C_SLV0_ADDR, AK8963_ADDRESS | 0X80);
    _i2c.i2c_write(I2C_SLV0_REG,  AK8963_XOUT_L);
    _i2c.i2c_write(I2C_SLV0_CTRL, 0x87);
    _i2c.delay_ms(2);
    _i2c.i2c_read(EXT_SENS_DATA_00, rawData,7);
    uint8_t c = rawData[6];
    if(!(c & 0x80)){
        data->mag_original.x = ((int16_t)rawData[1] << 8) | rawData[0];
        data->mag_original.y = ((int16_t)rawData[3] << 8) | rawData[2];
        data->mag_original.z = ((int16_t)rawData[5] << 8) | rawData[4];
    }
}


bool mpu9250::checkNewMagData()
{
    _i2c.i2c_write(I2C_SLV0_ADDR, AK8963_ADDRESS | 0X80);
    _i2c.i2c_write(I2C_SLV0_REG,  AK8963_ST1);
    _i2c.i2c_write(I2C_SLV0_CTRL, 0x81);
    _i2c.delay_ms(2);
    _status = _i2c.i2c_read(EXT_SENS_DATA_00);
    return (bool)(_status & 0x01);
}

bool mpu9250::checkNewAccelGyroData()
{
   _status = _i2c.i2c_read(INT_STATUS);
   return (bool)(_status & 0x01);
}

bool mpu9250::checkWakeOnMotion()
{
    _status = _i2c.i2c_read(INT_STATUS);
    return (bool)(_status & 0x40);
}

void mpu9250::gyromagSleep()
{
    uint8_t temp = 0;
    temp = _i2c.i2c_read(AK8963_CNTL);
    _i2c.i2c_write(AK8963_CNTL, temp & ~(0x0F));
    temp = _i2c.i2c_read(PWR_MGMT_1);
    _i2c.i2c_write(PWR_MGMT_1, temp | 0x10);
    _i2c.delay_ms(10);
}

void mpu9250::gyromagWake(uint8_t Mmode)
{
      uint8_t temp = 0;
      temp = _i2c.i2c_read(AK8963_CNTL);
      _i2c.i2c_write(AK8963_CNTL, temp | Mmode);
      temp = _i2c.i2c_read(PWR_MGMT_1);
      _i2c.i2c_write(PWR_MGMT_1, 0x01);
      _i2c.delay_ms(10);
}


void mpu9250::accelWakeOnMotion()
{
    uint8_t c;                                                                                                                              
    c = _i2c.i2c_read(ACCEL_CONFIG2);
    c = c & ~0X0F;
    c= c | 0x01;
    _i2c.i2c_write(ACCEL_CONFIG2, c);

    _i2c.i2c_write(INT_PIN_CFG, 0X12);
    _i2c.i2c_write(INT_ENABLE, 0x41);
    _i2c.i2c_write(MOT_DETECT_CTRL, 0xC0);
    _i2c.i2c_write(WOM_THR, 0x19);
    _i2c.i2c_write(LP_ACCEL_ODR, 0x02);
    c = _i2c.i2c_read(PWR_MGMT_1);
    _i2c.i2c_write(PWR_MGMT_1, c | 0x20);
    gyromagSleep();
    _i2c.delay_ms(100);
}


