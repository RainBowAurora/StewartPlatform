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

#include "pca9685_device/i2c.h"

i2c_dev::i2c_dev(std::string dev_name, int address):_address(address)
{
 i2c_open(dev_name);   
}

i2c_dev::~i2c_dev()
{
    i2c_close();
}

void i2c_dev::i2c_open(std::string dev_name)
{
    _fd = open(dev_name.data(), O_RDWR, _address);
    if(_fd < 0){
    ROS_ERROR("%s\n", strerror(errno));
    }    
    if(ioctl(_fd, I2C_SLAVE, _address) < 0){
    ROS_ERROR("Error opening address 0x%02x:\n %d: %s\n", _address, errno, strerror(errno));
    }
}

void i2c_dev::i2c_close()
{
    if(_fd > 0){
        close(_fd);
    }
}

bool i2c_dev::i2c_read(unsigned char reg, unsigned char *buf, size_t len)
{
    
    //First message is an empty write to the register (no payload)
    _message[0].addr = _address;
    _message[0].len = sizeof(reg);
    _message[0].buf = &reg;
    _message[0].flags = 0;
    //Second message will return the value
    _message[1].addr = _address;

    _message[1].len = (len > 1? len:sizeof(*buf));
    _message[1].buf = buf;
    _message[1].flags = I2C_M_RD;

    _data.msgs = _message;
    _data.nmsgs = 2;

    if(_fd >= 0 && ioctl(_fd, I2C_RDWR, &_data) < 0){
         ROS_WARN("Error opening address 0x%02x:\n %d: %s\n", _address, errno, strerror(errno));
        return false;
    }
    return true;

}

unsigned char i2c_dev::i2c_read(unsigned char reg)
{
    unsigned char buf;
    i2c_read(reg, &buf);
    return buf;
}

bool i2c_dev::i2c_write(unsigned char reg, unsigned char value)
{
    unsigned char buf[2];

    buf[0] = reg;   //register to write to 
    buf[1] = value; //value to write

    _message[0].addr = _address;
    _message[0].len = sizeof(buf);
    _message[0].buf = buf;
    _message[0].flags = 0;

    _data.msgs = _message;
    _data.nmsgs = 1;

    if(ioctl(_fd, I2C_RDWR, &_data) < 0){
        ROS_WARN("Error opening address 0x%02x:\n %d: %s\n", _address, errno, strerror(errno));
        return false;
    }

    return true;
}

void i2c_dev::delay_ms(long ms)
{
    for(long i = 0; i < ms; i++){
        delay_us(1000);
    }
}

void i2c_dev::delay_us(long us)
{
    struct timespec tv={
        .tv_sec = 0,
        .tv_nsec = us * 1000
    };
    while(nanosleep(&tv, &tv));
}
