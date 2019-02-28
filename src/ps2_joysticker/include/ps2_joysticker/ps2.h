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

