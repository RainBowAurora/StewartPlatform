#ifndef __PCA9685_H__
#define __PCA9685_H__

#define PCA9685_ALL_CHANNELS -1
#define PULSE_WIDTH_FREQUENCY 100
#include "pca9685_device/i2c.h"

class pca9685{
public:
    pca9685(std::string dev_name, int addr);
    ~pca9685();
    bool pca9685_set_pulse_frequency(int);
    bool pca9685_set_channel_pulse(int, unsigned int, unsigned int);
    int pca9685_get_frequency();
    float pca9685_get_effective_frequency();
protected:
/*
 * PCA9685 registers from:
 *
 * http://www.nxp.com/documents/data_sheet/PCA9685.pdf
 *
 */
enum{
    PCA9685_MODE1       = 0x00,
    PCA9685_MODE2       = 0x01,
    PCA9685_SUBADR1     = 0x02,
    PCA9685_SUBADR2     = 0x03,
    PCA9685_SUBADR3     = 0x04,
    PCA9685_ALLCALLADR  = 0x05,

    PCA9685_CHANNEL_SIZE      = 4,
    PCA9685_CHANNEL_0_OFFSET  = 0x06,
    PCA9685_ALL_OFFSET        = 0xFA,
    PCA9685_ON_L              = 0,
    PCA9685_ON_H,
    PCA9685_OFF_L,
    PCA9685_OFF_H,

    PCA9685_PRE_SCALE   = 0xFE,

    PCA9685_MODE1_RESTART = 1 << 7,
    PCA9685_MODE1_SLEEP   = 1 << 4,
    PCA9685_MODE1_ALLCALL = 1 << 0,

    PCA9685_MODE2_INVRT   = 1 << 4,
    PCA9685_MODE2_OUTDRV  = 1 << 2,

    PCA9685_INTERNAL_OSCILLATOR = 25000000 /* 25MHz */
};

typedef struct{
    int on; //Last requested PWM ON
    int off;  
}PCA9685Channel;

private:    
    PCA9685Channel _channels[16];
    int _frequency;
    int _pulse_width;
    i2c_dev _i2c;
    void pca9685_init();
};

#endif /*__PCA9685_H__*/