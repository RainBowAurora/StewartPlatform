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