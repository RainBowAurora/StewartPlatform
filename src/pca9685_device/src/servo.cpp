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
