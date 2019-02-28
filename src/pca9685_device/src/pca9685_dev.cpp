#include <ros/ros.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdlib.h>
#include <string>
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/limits.h>
#include <sys/stat.h>
#include <sys/types.h>
#include "pca9685_device/pca9685_dev.h"

pca9685::pca9685(std::string dev_name, int addr):_i2c(dev_name, addr)
{
    pca9685_init();
}

pca9685::~pca9685()
{

}

 void pca9685::pca9685_init()
 {
    bool err; 
    err = pca9685_set_channel_pulse(PCA9685_ALL_CHANNELS, 0, 0);
    if(!err){
        ROS_ERROR("Could not reset all PWM fields !");
        return;
    }
    
    err = _i2c.i2c_write(PCA9685_MODE2, PCA9685_MODE2_OUTDRV);
    if(!err){
        ROS_ERROR("Write pca9685 register 0x%x is failed", PCA9685_MODE2);
        return;
    }

    err = _i2c.i2c_write(PCA9685_MODE1, PCA9685_MODE1_ALLCALL);
    if(!err){   
        ROS_ERROR("Write pca9685 register 0x%x is failed", PCA9685_MODE1);
        return;
    }
    //Spec says maximum of 500us for oscillator to be up once SLEEP is set to 0
    _i2c.delay_us(500);

    unsigned char mode = 0;
    err = _i2c.i2c_read(PCA9685_MODE1, &mode);
    if(!err){
        ROS_ERROR("Read pca9685 register 0x%x is failed", PCA9685_MODE1);
        return;
    }

    mode &= ~PCA9685_MODE1_SLEEP; //Clear SLEEP bit
    err = _i2c.i2c_write(PCA9685_MODE1, mode);
    if(!err){
        ROS_ERROR("Write pca9685 register 0x%x is failed", PCA9685_MODE1);
    }
    //Wait 500us again after clearing SLEEP for clock to get back
    _i2c.delay_us(500);
    
    for(int i = 0; i < sizeof(_channels) / sizeof(_channels[0]); i++){
        _channels[i].on =
        _channels[i].off = -1;
    }
 }

bool pca9685::pca9685_set_channel_pulse(int channel, unsigned int on, unsigned int off)
{
    int channel_offset;
    bool err;
    if(_frequency > 0){
        on = 4096 * on / (1000000L / _frequency);
        off = 4096 * off / (1000000L / _frequency);
    }else{
        on = off = 0;
    }
    if(channel == PCA9685_ALL_CHANNELS){
        for(int i = 0; i < sizeof(_channels)/sizeof(_channels[0]); i++){
            _channels[i].on = -1;
            _channels[i].off = -1;
        }
        channel_offset = PCA9685_ALL_OFFSET;
    }else{
        if(_channels[channel].on == on && _channels[channel].off == off){
            return true;
        }
        channel_offset = PCA9685_CHANNEL_0_OFFSET + channel * PCA9685_CHANNEL_SIZE;
    }

    err = _i2c.i2c_write(channel_offset + PCA9685_ON_L, on & 0x0ff);
    if(!err){   
        ROS_ERROR("Unable to set channel ON LOW to 0x%02x", on & 0xff);
        return err;        
    }

    err = _i2c.i2c_write(channel_offset + PCA9685_ON_H, on >> 8);
    if(!err){
         ROS_ERROR("Unable to set channel ON HI  to 0x%02x", on >> 8);
         return err;
    }

    err = _i2c.i2c_write(channel_offset + PCA9685_OFF_L, off & 0x0ff);
    if(!err){   
        ROS_ERROR("Unable to set channel ON LOW to 0x%02x", off & 0xff);
        return err;        
    }

    err = _i2c.i2c_write(channel_offset + PCA9685_OFF_H, off >> 8);
    if(!err){
         ROS_ERROR("Unable to set channel ON HI  to 0x%02x", off >> 8);
         return err;
    }

    if(channel == PCA9685_ALL_CHANNELS){
        for(int i = 0; i < sizeof(_channels) / sizeof(_channels[0]); i++){
            _channels[i].on = on;
            _channels[i].off = off;
        }        
    }else{
        _channels[channel].on = on;
        _channels[channel].off = off;
    }
    return true;
}


bool pca9685::pca9685_set_pulse_frequency(int frequency)
{
    if(frequency == 0){
        ROS_WARN("Attempt to set frequency to 0Hz");
        return -1;        
    }
    /* Determine PRE_SCALE value based on 7.3.5 PWM frequency PRE_SCALE
     * from http://www.nxp.com/documents/data_sheet/PCA9685.BASE_DISTANCEf */
    bool err;
    int prescale;
    unsigned char mode = 0, tmp;
    prescale = round((float)PCA9685_INTERNAL_OSCILLATOR / (4096.0 * frequency)) - 1;

    //The PRE_SCALE can only be set if SLEEP is set to 1
    err = _i2c.i2c_read(PCA9685_MODE1, &mode);
    if(!err){
        ROS_ERROR("Could not read mode form PCA9685");
        return err;
    }
    tmp = mode;
    mode &= ~PCA9685_MODE1_RESTART; //Clear the RESTART bit if set 
    mode |= PCA9685_MODE1_SLEEP; //Set the SLEEP bit
    err = _i2c.i2c_write(PCA9685_MODE1, mode); //Initiate SLEEP
    if(!err){
        ROS_ERROR("Could not write SLEEP mode to PCA9685");
        return err;
    }

    //Set the new PRE_SCALE
    err =_i2c.i2c_write(PCA9685_PRE_SCALE, prescale);
    if(!err){
        ROS_ERROR("Could not write PRE_SCALE to PCA9685");
        return err;
    }

    //Set the mode back to what it was
    err = _i2c.i2c_write(PCA9685_MODE1, tmp);
    if(!err){
        ROS_ERROR("Could not set mode back on PCA9685");
        return err;
    }

    //After 500us raise the RESTART bit to resume PWM values
    _i2c.delay_us(500);
    err = _i2c.i2c_write(PCA9685_MODE1, tmp | PCA9685_MODE1_RESTART);
    if(!err){
        ROS_ERROR("Could not raise RESTART on PCA9685");
        return err;
    }

    _frequency = frequency;
    _pulse_width = 1000000L / _frequency;
    return true;
}

int pca9685::pca9685_get_frequency()
{
    return _frequency;
}

float pca9685::pca9685_get_effective_frequency()
{
    int prescale = round((float)PCA9685_INTERNAL_OSCILLATOR / (4096.0 * _frequency)) - 1.0;
    return (float)PCA9685_INTERNAL_OSCILLATOR / (prescale * 4096.0);
}