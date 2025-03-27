/*
 This lib has been written & maneuverable test on 3 wheels omni robot
thanks to Northwestern Robotics documments <3
Video URL:https://www.youtube.com/watch?v=NcOT9hOsceE&t=220s
*/
#ifndef OMNI_3_FIELD_KIN_H
#define OMNI_3_FIELD_KIN_H

#include "stdint.h"
#include "math.h"
#include "WT901C.h"
#include "PS4_ESP.h"
#include "PID.h"

#define PI 3.14159f
#define WHEEL_RADIUS 0.75f
#define D 0.3f // Distance from wheel to centrer of robot
#define Robot_Max_Speed 3
#define Robot_Max_Omega 6

/*Robot setup
         1
        / \
       /   \
      2-----3
 120 deg
*/
typedef struct
{
    float vx;    // Linear speed (m/s)
    float vy;    // Linear speed (m/s)
    float omega; // Angular speed (rad/s)
    float theta; // Robot's angle compared to home position
    float max_speed;
    float max_omega;
    float u[3];         // Signal to control speed
    uint8_t is_yaw_fix; // Determine field control or fix yaw angle control
    double fix_angle;   // Angle of robot to fix
    double IMU_theta;
} ORb;

extern ORb Omni_3_Bot;
/*PID controllers*/
extern PID_TypeDef Omega_PID;
/*Init robot with maximum linear speed(m/s) & angular speed(rad/s)*/
void OmniRobot_Init(ORb *robot, float m_speed, float m_omg);
/*Calculate signal to control speed*/
void OmniRobot_Field_Control(ORb *robot, PS4_DATA *ps4_joy, float imu_theta, uint8_t neg_heading);

void Find_Closet_Angle(int16_t current_angle, int16_t *closest_angles);
#endif // OMNI_3_FIELD_KIN_H
