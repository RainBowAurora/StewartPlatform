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

#include "ps2_joysticker/ps2.h"

ps2::ps2(std::string dev_name):_dev_name(dev_name)
{
    ps2_dev_open();
}

ps2::~ps2()
{
    close(_ps_fd);
}

bool ps2::ps2_dev_open()
{
    // block O_NONBLOCK
    _ps_fd = open(_dev_name.data(), O_RDONLY | O_NONBLOCK);
    if(_ps_fd < 0){
       ROS_ERROR("open ps2  facility filed");
       perror("ERROR");
       return false;
    }
    return true;
}

bool ps2::ps2_map_update(ps2_map_t *map)
{
    int  len = read(_ps_fd, &js, sizeof(struct js_event));
    if(len < 0){
        //perror("error");
        return false;
    }
    map->time = js.time;
    if(js.type == JS_EVENT_BUTTON){
        switch(js.number){
            case PS2_BUTTON_A:
                map->a = js.value;
            break;

            case PS2_BUTTON_B:
                map->b = js.value;
            break;

            case PS2_BUTTON_C:
                map->c = js.value;
            break;

            case PS2_BUTTON_D:
                map->d = js.value;
            break;

            case PS2_BUTTON_L1:
                map->l1 = js.value;
            break;

            case PS2_BUTTON_R1:
                map->r1 = js.value;
            break;

            case PS2_BUTTON_L2:
                map->l2 = js.value;
            break;

            case PS2_BUTTON_R2:
                map->r2 = js.value;
            break;
            
            case PS2_BUTTON_SELECT:
                map->select = js.value;
            break;

            case PS2_BUTTON_START:
                map->start = js.value;
            break;

            case PS2_BUTTON_LO:
                map->lo = js.value;
            break;

            case PS2_BUTTON_RO:
                map->ro = js.value;
            break;

            case PS2_BUTTON_MODE:
                map->home = js.value;
            break;            

            default:
            break;
        }
    }else if(js.type == JS_EVENT_AXIS){
         switch(js.number){
            case PS2_AXIS_LX:
                map-> lx = js.value;
            break;

            case PS2_AXIS_LY:
                 map->ly = js.value;
            break;

            case PS2_AXIS_RX:
                map->rx = js.value;
            break;

            case PS2_AXIS_RY:
                map->ry = js.value;
            break;

            case PS2_AXIS_XX:
                map->xx = js.value;
            break;

            case PS2_AXIS_YY:
                map->yy = js.value;
            break;

            default: break;
            }
        }
    else{
        //Init do nothing 
    }

    return true;
}

