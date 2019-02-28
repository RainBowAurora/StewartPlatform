#include <ros/ros.h>
#include "pca9685_device/pca9685_dev.h"
#include "pca9685_device/servo.h"

pca9685 servo("/dev/i2c-1", 0x40);

void servo_control_callback(const pca9685_device::servo::ConstPtr& servo);


int main(int argc, char *argv[])
{
    bool err;
    ros::init(argc, argv, "servo");
    ros::NodeHandle n;
    
    err = servo.pca9685_set_pulse_frequency(PULSE_WIDTH_FREQUENCY);
    if(!err){
        ROS_ERROR("Could not set PWM frequency!");
        return -1;
    }
    ros::Subscriber servo_control_sub = n.subscribe("servo_control", 1000, servo_control_callback);
    ros::spin();
    return 0;
}


void servo_control_callback(const pca9685_device::servo::ConstPtr& servo_data)
{
    for(int i = 0; i < 6; i++){
    servo.pca9685_set_channel_pulse(i, 0, servo_data->servo[i]);
    }
}
