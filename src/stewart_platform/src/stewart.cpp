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

#include "stewart_platform/config.h"
#include "stewart_platform/stewart.h"

StewartPlatform::StewartPlatform(): baseJoint{ { } }
                                    , platformJoint{ { } }
                                    , legLength{ { } }
                                    , translation{ }                                    
                                    , rotation{ }
                                    , alpha{ }
                                    , beta{BETA_ANGLES}
                                    , servoHorPos{SERVO_ZERO_POSITION}
{
    getPlatformJoints();
}

StewartPlatform::~StewartPlatform()
{

}

void StewartPlatform::getPlatformJoints()
{
    baseJoint[0].x = -PLATFORM_BASE_RADUIS * cos(degToRad(30) - THETA_ANGLE);
    baseJoint[1].x = -PLATFORM_BASE_RADUIS * cos(degToRad(30) - THETA_ANGLE);
    baseJoint[2].x =  PLATFORM_BASE_RADUIS * sin(THETA_ANGLE);
    baseJoint[3].x =  PLATFORM_BASE_RADUIS * cos(degToRad(30) + THETA_ANGLE);
    baseJoint[4].x =  PLATFORM_BASE_RADUIS * cos(degToRad(30) + THETA_ANGLE);
    baseJoint[5].x =  PLATFORM_BASE_RADUIS * sin(THETA_ANGLE);

    baseJoint[0].y = -PLATFORM_BASE_RADUIS * sin(degToRad(30) - THETA_ANGLE);
    baseJoint[1].y =  PLATFORM_BASE_RADUIS * sin(degToRad(30) - THETA_ANGLE);
    baseJoint[2].y =  PLATFORM_BASE_RADUIS * cos(THETA_ANGLE);
    baseJoint[3].y =  PLATFORM_BASE_RADUIS * sin(degToRad(30) + THETA_ANGLE);
    baseJoint[4].y = -PLATFORM_BASE_RADUIS * sin(degToRad(30) + THETA_ANGLE);
    baseJoint[5].y = -PLATFORM_BASE_RADUIS * cos(THETA_ANGLE);

    platformJoint[0].x = -PLATFORM_TOP_RADIUS * sin(degToRad(30) + THETA_R_ANGLE / 2.0f);
    platformJoint[1].x = -PLATFORM_TOP_RADIUS * sin(degToRad(30) + THETA_R_ANGLE / 2.0f);
    platformJoint[2].x = -PLATFORM_TOP_RADIUS * sin(degToRad(30) - THETA_R_ANGLE / 2.0f);
    platformJoint[3].x =  PLATFORM_TOP_RADIUS * cos(THETA_R_ANGLE / 2.0f);
    platformJoint[4].x =  PLATFORM_TOP_RADIUS * cos(THETA_R_ANGLE / 2.0f);
    platformJoint[5].x = -PLATFORM_TOP_RADIUS * sin(degToRad(30) - THETA_R_ANGLE / 2.0f);
  
    platformJoint[0].y = -PLATFORM_TOP_RADIUS * cos(degToRad(30) + THETA_R_ANGLE / 2.0f);
    platformJoint[1].y =  PLATFORM_TOP_RADIUS * cos(degToRad(30) + THETA_R_ANGLE / 2.0f);
    platformJoint[2].y =  PLATFORM_TOP_RADIUS * cos(degToRad(30) - THETA_R_ANGLE / 2.0f);
    platformJoint[3].y =  PLATFORM_TOP_RADIUS * sin(THETA_R_ANGLE / 2.0f);
    platformJoint[4].y = -PLATFORM_TOP_RADIUS * sin(THETA_R_ANGLE / 2.0f);
    platformJoint[5].y = -PLATFORM_TOP_RADIUS * cos(degToRad(30) - THETA_R_ANGLE / 2.0f);
}

void StewartPlatform::setTranslation(const point_t pos)
{
    translation.x = pos.x;
    translation.y = pos.y;
    translation.z = pos.z + PLATFORM_HEIGHT_DEFAULT;
}

void StewartPlatform::setRotation(const point_t rot)
{
    rotation.x = rot.x;
    rotation.y = rot.y;
    rotation.z = rot.z;
}

void StewartPlatform::getRotationMartix(float rotationMatrix[3][3])
{
    float roll = rotation.x;
    float pitch = rotation.y;
    float yaw = rotation.z;

    rotationMatrix[0][0] =  cos(yaw) * cos(pitch);
    rotationMatrix[1][0] = -sin(yaw) * cos(roll) + cos(yaw) * sin(pitch) * sin(roll);
    rotationMatrix[2][0] =  sin(yaw) * sin(roll) + cos(yaw) * cos(roll) * sin(pitch);

    rotationMatrix[0][1] =  sin(yaw) * cos(pitch);
    rotationMatrix[1][1] =  cos(yaw) * cos(roll) + sin(yaw) * sin(pitch) * sin(roll);  
    rotationMatrix[2][1] =  cos(pitch) * sin(roll);

    rotationMatrix[0][2] =  -sin(pitch);
    rotationMatrix[1][2] =  -cos(yaw) * sin(roll)  + sin(yaw) * sin(pitch) * cos(roll);
    rotationMatrix[2][2] =   cos(pitch) * cos(roll);
}

void StewartPlatform::calcLegLength()
{
    float rotMatrix[3][3] = {};
    getRotationMartix(rotMatrix);
    for(int i = 0; i < 6; i++){
        legLength[i].x = (rotMatrix[0][0] * platformJoint[i].x) + (rotMatrix[0][1] * platformJoint[i].y) + (rotMatrix[0][2] * platformJoint[i].z);
        legLength[i].y = (rotMatrix[1][0] * platformJoint[i].x) + (rotMatrix[1][1] * platformJoint[i].y) + (rotMatrix[1][2] * platformJoint[i].z);
        legLength[i].z = (rotMatrix[2][0] * platformJoint[i].x) + (rotMatrix[2][1] * platformJoint[i].y) + (rotMatrix[2][2] * platformJoint[i].z);

        legLength[i].x += translation.x;
        legLength[i].y += translation.y;
        legLength[i].z += translation.z;
    }
}

void StewartPlatform::calcAlpha()
{
    point_t basePoint, Li;
    double min, max, dist;
    for(int i = 0; i < 6; i++){
        min = SERVO_MIN;
        max = SERVO_MAX;
        for(int j = 0; j < 20; j++){
            basePoint.x = LENGTH_SERVO_ARM * cos(alpha[i]) * cos(beta[i]) + baseJoint[i].x;
            basePoint.y = LENGTH_SERVO_ARM * cos(alpha[i]) * sin(beta[i]) + baseJoint[i].y;
            basePoint.z = LENGTH_SERVO_ARM * sin(alpha[i]);

            Li.x = legLength[i].x - basePoint.x;
            Li.y = legLength[i].y - basePoint.y;
            Li.z = legLength[i].z - basePoint.z;

            dist = sqrt(Li.x * Li.x + Li.y * Li.y + Li.z * Li.z);

            if(std::abs(LENGTH_SERVO_LEG - dist) < 0.01){
                break;
            }
            if(dist < LENGTH_SERVO_LEG){
                max = alpha[i];
            }
            else{
                min = alpha[i];
            }
            if(max == SERVO_MIN || min == SERVO_MAX){
                break;
            }
            alpha[i] = min + (max - min) / 2.0f;
        }
    }
}


void StewartPlatform::calcServoPos(float servoPos[6])
{
    for(int i = 0; i < 6; i++){
        if(i == INVERSE_SERVO_1 || i == INVERSE_SERVO_2 || i == INVERSE_SERVO_3){
            servoPos[i] = constrain(servoHorPos[i] - (alpha[i]) * SERVO_MULT, MIN_SERVO_PLUSE, MAX_SERVO_PLUSE);
        }
        else{
            servoPos[i] = constrain(servoHorPos[i] + (alpha[i]) * SERVO_MULT, MIN_SERVO_PLUSE, MAX_SERVO_PLUSE);
        }
    }
}

void StewartPlatform::getServoPosition(const point_t transl, const point_t rotat, float servoPos[6])
{
    setTranslation(transl);
    setRotation(rotat);
    calcLegLength();
    calcAlpha();
    calcServoPos(servoPos);
}
