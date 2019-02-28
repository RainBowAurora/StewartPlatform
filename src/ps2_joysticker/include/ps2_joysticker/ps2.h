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

#ifndef __PS2_H__
#define __PS2_H__

#include <ros/ros.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <linux/input.h>
#include <linux/joystick.h>

#include <string>

typedef struct {
    int time;
    int xx;
    int yy;
    int lx;
    int ly;
    int l1;
    int l2;
    int home;
    int select;
    int start;
    int a;
    int b;
    int c;
    int d;
    int rx;
    int ry;
    int r1;
    int r2;
    int ro;
    int lo;    
}ps2_map_t;

/* you can input commind on your terminal
 * cat /dev/input/js0 | hexdump
 * jstest /dev/input/js0
 * jstest --event /dev/input/js0
*/

class ps2{
public:
    ps2(std::string);
    bool ps2_map_update(ps2_map_t*);
    ~ps2();
protected:
    enum{
        PS2_TYPE_BUTTON = 0x01,
        PS2_TYPE_AXIS,  

        PS2_BUTTON_A = 0x00,
        PS2_BUTTON_B,
        PS2_BUTTON_C,
        PS2_BUTTON_D,
        PS2_BUTTON_L1,
        PS2_BUTTON_R1,
        PS2_BUTTON_L2,
        PS2_BUTTON_R2,
        PS2_BUTTON_SELECT,
        PS2_BUTTON_START,
        PS2_BUTTON_LO,      //手柄左摇杆按键
        PS2_BUTTON_RO,     
        PS2_BUTTON_MODE,

        PS2_AXIS_LX = 0x00,
        PS2_AXIS_LY,
        PS2_AXIS_RX,
        PS2_AXIS_RY,
        PS2_AXIS_XX,
        PS2_AXIS_YY
    };

private:
    std::string _dev_name;
    ps2_map_t _ps2_map;
    int _ps_fd;
    struct js_event js;
    bool ps2_dev_open();
};



#endif /*__PS2_H__*/

