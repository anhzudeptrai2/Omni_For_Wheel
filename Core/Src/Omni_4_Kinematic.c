#include "Omni_4_Kinematic.h"
#include "math.h"
#define Dead_Zone_X_Axis 0.5f
#define Dead_Zone_Y_Axis 0.1f

double Omega_PID_Output;
double Kp_Omega = 0.5f, Ki_Omega = 0.1f, Kd_Omega = 0.1f;
double Speed_Omega_PID = 0.0f;
typedef struct
{
    float X_Axis;
    float Y_Axis;
    float Z_Axis;
} Joystick;
Joystick joystick;
float Rads_to_RPM(float rad)
{
    return rad * 9.5492965855137;
}

void OmniRobot_Init(Omni_Kinematic *robot, float m_speed, float m_omega)
{
    robot->Vx = 0.0f;
    robot->Vy = 0.0f;
    robot->Omega = 0.0f;
    robot->Theta = 0.0f;
    robot->Max_Speed = m_speed;
    robot->Max_Omega = m_omega;
    robot->Is_Yaw_Fix = 0;
    robot->Fix_Angle = 0.0f;

    /* PID Init for Omega control to fix robot angle
     *
     * _PID_P_ON_E: Kp dependent on error instead of integral error
     *
     * _PID_CD_REVERSE: Reverse control direction
     */
    PID(&Omega_PID, &robot->IMU_Theta, &Omega_PID_Output, &robot->Fix_Angle, Kp_Omega, Ki_Omega, Kd_Omega, _PID_P_ON_E, _PID_CD_REVERSE);
    PID_SetMode(&Omega_PID, _PID_MODE_AUTOMATIC);
    PID_SetSampleTime(&Omega_PID, 1);
    PID_SetOutputLimits(&Omega_PID, -Speed_Omega_PID, Speed_Omega_PID);
}

void Joystick_To_Velocites(Omni_Kinematic *robot, float max_speed, float max_omega)
{
    float mapped_x = (joystick.X_Axis) / 127.0f;
    float mapped_y = (joystick.Y_Axis) / 127.0f;
    float mapped_z = (joystick.Z_Axis) / 127.0f;
	
    if (fabs(mapped_x) < Dead_Zone_X_Axis)
    {
        mapped_x = 0.0f;
    }
    else
    {
        robot->Vx = mapped_x * max_speed;
    }

    if (fabs(mapped_y) < Dead_Zone_Y_Axis)
    {
        mapped_y = 0.0f;
    }
    else
    {
        robot->Vy = mapped_y * max_speed;
    }

    /* Check control mode field control or fixed yaw angle control */
    if (robot->Is_Yaw_Fix)
    {
        PID_Compute(&Omega_PID);
        robot->Omega = Omega_PID_Output;
    }
    else
    {
        robot->Omega = mapped_z * max_omega;
    }
}

void OmniRobot_Calculate_WheelSpeeds(Omni_Kinematic *robot, float *temp1_in, float *temp2_in, float *temp3_in, float *temp4_in)
{
    /* Calculate wheel speeds in a robot-centric coordinate system */
    float Vx_Robot = robot->Vx * cosf(robot->Theta) + robot->Vy * sinf(robot->Theta);
    float Vy_Robot = -robot->Vx * sinf(robot->Theta) + robot->Vy * cosf(robot->Theta);

    /* Wheel speed calculations based on kinematic equations */
    float temp1 = (Vx_Robot - D * robot->Omega) / WHEEL_RADIUS;                             // 1
    float temp2 = (-0.5f * Vx_Robot + 0.866f * Vy_Robot - D * robot->Omega) / WHEEL_RADIUS; // 2
    float temp3 = (-0.5f * Vx_Robot - 0.866f * Vy_Robot - D * robot->Omega) / WHEEL_RADIUS; // 3
    float temp4 = (Vx_Robot + D * robot->Omega) / WHEEL_RADIUS;                             // 4
    
    /*Change rad/s to RPM*/
    *temp1_in = Rads_to_RPM(temp1);
    *temp2_in = Rads_to_RPM(temp2);
    *temp3_in = Rads_to_RPM(temp3);
    *temp4_in = Rads_to_RPM(temp4);
}

void OmniRobot_Field_Control(Omni_Kinematic *robot, PS4_DATA *ps4_joy, float imu_theta, uint8_t neg_heading )
{
    robot-> IMU_Theta = imu_theta;
    joystick.X_Axis = -ps4_joy->l_stick_x;
    joystick.Y_Axis = -ps4_joy->l_stick_y;
    joystick.Z_Axis = ps4_joy->r_stick_x;

    Joystick_To_Velocites(robot, robot->Max_Speed, robot->Max_Omega);

    if (neg_heading) /*Negative heading or positive heading*/
    {
        robot->Theta = (180 + imu_theta) * (PI / 180.0f);
    }
    else
    {
        robot->Theta = imu_theta * (PI / 180.0f);
    }

    if (robot->Theta > 2 * PI)
    {
        robot->Theta -= 2 * PI;
    }
    else if (robot->Theta < 0)
    {
        robot->Theta += 2 * PI;
    }

    OmniRobot_Calculate_WheelSpeeds(robot, &robot->temp[0], &robot->temp[1], &robot->temp[2], &robot->temp[3]);
}

int16_t Round_Float(float num)
{
    return (int16_t)(num >= 0 ? num + 0.5 : num - 0.5);
}

void Find_Closet_Angle(int16_t current_angle, int16_t *closest_angles)
{
    int16_t target_angles[] = {90, -90, 180, -180};

    for (uint8_t i = 0; i < 4; i++)
    {


        
        closest_angles[i] = target_angles[i] + 360.0f * roundf((current_angle - target_angles[i]) / 360.0f);
    }
}