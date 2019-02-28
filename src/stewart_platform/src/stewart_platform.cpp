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
#include "stewart_platform/stewart.h"
#include "ps2_joysticker/ps2_msg.h"
#include "geometry_msgs/Twist.h"
#include "pca9685_device/servo.h"

point_t translation = {0.0, 0.0, 0.0}, rotation = {0.0, 0.0, 0.0};
pca9685_device::servo steering_engine;

namespace normlize{
    template<class T1, class T2>
    T2 map(T1  value, T1  I_Min, T1  I_Max, T2  O_Min, T2  O_Max)
    {
        return (T2)((T2)value / (T2)(I_Max - I_Min) * (O_Max - O_Min));
    }
};

void ps2_Subscriber_callback(const ps2_joysticker::ps2_msg::ConstPtr &ps2_msg);
void Servo_Publisher(float *servosPosition);
void terminal_Subscriber_callback(const geometry_msgs::Twist::ConstPtr &terminal_msg);

int main(int argc, char *argv[])
{    
    StewartPlatform sp;

    steering_engine.servo.resize(6);
    int control_mode;
    float servosPosition[6];
    ros::init(argc, argv, "stewart");
    ros::NodeHandle command;
    ros::Rate loop_rate(100);
    ros::Subscriber commd_sub;

    command.param<int>("platform_control_mode",  control_mode, 0);
    switch(control_mode){
        case 0:
            commd_sub= command.subscribe("ps2", 1000, ps2_Subscriber_callback);   
        break;
        case 1:
            commd_sub = command.subscribe("terminal", 1000, terminal_Subscriber_callback);   
        break;
    }
    ros::Publisher control_pub = command.advertise<pca9685_device::servo>("servo_control", 1000);
         
    ros::spinOnce();

     while (ros::ok())
        {                         
        //计算平台的每个舵机需要转过的角度
        sp.getServoPosition(translation, rotation, servosPosition);
        Servo_Publisher(servosPosition);
        control_pub.publish(steering_engine);
        loop_rate.sleep();
        /* code for loop body */
         ros::spinOnce();
    }    
    return 0;
}

/*
 *装载控制指令
*/
void ps2_Subscriber_callback(const ps2_joysticker::ps2_msg::ConstPtr &ps2_msg)
{
    rotation.x = normlize::map((int)ps2_msg->lx, -32767 ,32767, -20.0, 20.0);
    rotation.y = normlize::map((int)ps2_msg->ly, -32767 ,32767, -20.0, 20.0);
    translation.x = normlize::map((int)ps2_msg->rx, -32767 ,32767, -35.0, 35.0);
    translation.y = normlize::map((int)ps2_msg->ry, -32767 ,32767, -35.0, 35.0);
}


void terminal_Subscriber_callback(const geometry_msgs::Twist::ConstPtr &terminal_msg)
{
    translation.x = terminal_msg->angular.x;
    translation.y = terminal_msg->angular.y;
    rotation.x = terminal_msg->linear.x;
    rotation.y = terminal_msg->linear.y;
}

void Servo_Publisher(float *servosPosition)
{
    for(int i =0; i < 6; i++){
    steering_engine.servo[i] = round(servosPosition[i]);
    }        
}
