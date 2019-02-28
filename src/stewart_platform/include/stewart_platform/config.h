/***************************************************************************
 *                               y+
 *                                ^
 *                                |
 *                            .-------.
 *                         4 /         \ 3
 *                          /           \
 *                       5 /             \ 2
 *                -x <--- <       o +z    >  ---> x+
 *                         \             /
 *                          \           /
 *                           \         /
 *                            `-------'
 *                              0 | 1
 *                                v
 *                               -y
 *
********************************************************************************/
#ifndef __CONFIG_H__
#define __CONFIG_H__

#ifndef PI
#define PI 3.14159265359
#endif /*PI end*/

/* This is a CONFIG File used to perform the Stewart Calculation                                                                            
*  Be sure you replace all this constants to yours Platform values.
*  
*  You can get this values looking your Stewart Platform and from your Servo Datasheet.
*
*/

#ifndef degToRad
#define degToRad(degree) (degree * PI / 180.0f)
#endif /*degToRad end*/

#ifndef radToDeg
#define radToDeg(radians) ((radians * 180.0f) / PI)
#endif /*radToDeg end*/

/*The max a min pulse width form my Servo*/
#define MAX_SERVO_PLUSE 2500
#define MIN_SERVO_PLUSE 500

/*Posittion of servos mounted in inverse direction*/
#define INVERSE_SERVO_1 0
#define INVERSE_SERVO_2 2
#define INVERSE_SERVO_3 4

/*Multiplier used to convert radian to pulses in us*/
#define  SERVO_MULT (400.0f/(PI/4.0f))

/*The max e min rang of Servos in radians*/
#define SERVO_MIN degToRad(-80)
#define SERVO_MAX degToRad(80)

/*Here you should put Your Platform Values in millimeters*/
//Here you put the length of your servos arm 
#define LENGTH_SERVO_ARM 24.0f
//Here you put the length of your rods length
#define LENGTH_SERVO_LEG 155.0f

/*Here you put the defalut Height of your platform.
 *This value should be colose o9 mn to yours rods length.
  */
#define PLATFORM_HEIGHT_DEFAULT 145.0f
//Here you put the radius of the top of your platform
#define PLATFORM_TOP_RADIUS 65.5f
//Here you put the radius of the base of your platform
#define PLATFORM_BASE_RADUIS 96.5f
//Here you put the angle between two servos axis points
#define THETA_P_ANGLE degToRad(40.0f)
//Here you put the angle between two platform attachment points
#define THETA_R_ANGLE degToRad(27.2f)
//Here you dont need to change
#define THETA_ANGLE ((PI/3.0 - THETA_P_ANGLE) / 2.0f)
//Here you put the puless of each of yours servos respective to their position in horizontal
#define SERVO_ZERO_POSITION 1500, 1500, 1500, 1500, 1500, 1500
//Here you put the rotation of servo arms in respect to axis X
#define BETA_ANGLES PI/2, -PI/2, -PI/6,  5*PI/6, -5*PI/6, PI/6

#endif /*_CONFIG_H__*/
