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

#ifndef __I2C_H__
#define __I2C_H__

#include <ros/ros.h>

#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string>
#include <unistd.h>

#include <iostream>

#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/limits.h>

#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <time.h>

class i2c_dev{
    public:
       i2c_dev(std::string, int); 
       ~i2c_dev();
       bool i2c_read(unsigned char, unsigned char *, size_t len = 1);
       unsigned char i2c_read(const unsigned char);
       bool i2c_write(unsigned char, unsigned char);
       void delay_us(long);
       void delay_ms(long);
    private:
        int _fd = 0;
        int _address = 0;
        struct i2c_msg _message[2];
        struct i2c_rdwr_ioctl_data _data;
        void i2c_open(std::string);
        void i2c_close();
    protected:
};

#endif /*__I2C_H__*/
