/*
 * This header file defines the kinematic model for an omni-directional robot.
 * The kinematic equations are based on the research paper:
 * "Kinematic Modeling and Control of Omni-Directional Mobile Robots" 
 * published in MDPI (https://www.mdpi.com/2076-3417/12/12/5798).
 * 
 * The model allows the calculation of individual wheel speeds based on 
 * the desired linear and angular velocities of the robot. It is particularly 
 * useful for robots with omni-wheels, enabling smooth and precise movement 
 * in any direction without the need to rotate the robot's body.
 * 
 * This file includes necessary dependencies for mathematical calculations, 
 * sensor integration (WT901C), controller input (PS4_ESP), and PID control.
 */
#ifndef Omni_4_Kinematic_H
#define Omni_4_Kinematic_H


#include "stdint.h"
#include "math.h"
//#include "WT901C.h"
#include "PS4_Connect_ESP.h"
#include "PID.h"






#define PI 3.14159265358979323846
#define WHEEL_RADIUS 0.05
#define D 0.15 // Distance from wheel to centrer of robot

#define Omni_Max_Speed 3
/**
 * @brief Defines the maximum angular velocity (Omni_Max_Omega) of the robot's wheels.
 * 
 * When the robot moves at its maximum speed (Omni_Max_Speed), the wheels will rotate
 * at an angular velocity (ω). This value is calculated based on the maximum speed
 * and the distance (D) between the center of the robot and the wheels.
 */
#define Omni_Max_Omega (Omni_Max_Speed / D)

/**
 * @brief Structure Omni_Kinematic to store all parameters required for controlling a 2D Omni-directional robot.
 * This structure includes parameters for linear velocities (Vx, Vy), angular velocity (Omega),
 * robot orientation (Theta), control mode (Is_Yaw_Fix), and other related parameters for
 * sensor integration and wheel speed control.
 */
typedef struct

{
    float Vx; //Speed in x direction (m/s)
    float Vy; //Speed in y direction (m/s)
    float Omega;//Angular velocity (rad/s)
    float Theta; // Robot's angle compared to home position
    float Max_Speed; // The maximum linear speed the robot can achieve (m/s)
    float Max_Omega; // The maximum angular speed the robot can achieve (rad/s)
    float temp[4]; // Temporary array for wheel speeds
    uint8_t Is_Yaw_Fix; // Flag to determine control mode: "Field control" or "Fix yaw angle control"
    double Fix_Angle; // Fix angle for yaw control
    double IMU_Theta; // Yaw angle from IMU
} Omni_Kinematic; 



extern Omni_Kinematic Omni_4_Robot;
/*PID Controllers*/
extern PID_TypeDef Omega_PID;
/*Init robot with maximum linear speed(m / s) & angular speed(rad / s) */
void OmniRobot_Init(Omni_Kinematic *robot, float m_speed, float m_omega);
/*Calculate signal to control speed*/
void OmniRobot_Calculate(Omni_Kinematic *robot,PS4_DATA *ps4_data, float imu_theta, uint8_t neg_heading);

void Find_Closest_Angle(float *current_angle, float *closest_angle);
#endif // KINEMATIC_OMNI_H