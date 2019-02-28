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
#include "ps2_joysticker/ps2.h"
#include "ps2_joysticker/ps2_msg.h"

void update_ps2_pub(ps2_joysticker::ps2_msg &ps2_msg, ps2_map_t &ps2_map);

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ps2_public");
    ros::NodeHandle n;
    ros::Publisher ps2_pub = n.advertise<ps2_joysticker::ps2_msg>("ps2", 1000);
    ps2 ps2_joysticker("/dev/input/js0");
    ps2_map_t ps2_map;
    ps2_joysticker::ps2_msg ps2_msg;
    ros::Rate loop_rate(100);
    while (ros::ok())
    {        
        ps2_joysticker.ps2_map_update(&ps2_map);        

        update_ps2_pub(ps2_msg, ps2_map);
        
        ps2_pub.publish(ps2_msg);
        ros::spinOnce();
        loop_rate.sleep();        
    }
    return 0;
}


void update_ps2_pub(ps2_joysticker::ps2_msg &ps2_msg, ps2_map_t &ps2_map)
{    
    ps2_msg.xx = ps2_map.xx;
    ps2_msg.yy = ps2_map.yy;
    ps2_msg.lx = ps2_map.lx;
    ps2_msg.rx = ps2_map.rx;
    ps2_msg.ly = ps2_map.ly;
    ps2_msg.ry = ps2_map.ry;
    ps2_msg.a = ps2_map.a;
    ps2_msg.b = ps2_map.b;
    ps2_msg.c = ps2_map.c;
    ps2_msg.d = ps2_map.d;
    ps2_msg.l1 = ps2_map.l1;
    ps2_msg.l2 = ps2_map.l2;
    ps2_msg.r1 = ps2_map.r1;
    ps2_msg.r2 = ps2_map.r2;
    ps2_msg.select = ps2_map.select;
    ps2_msg.start = ps2_map.start;
    ps2_msg.home = ps2_map.home;    
}
