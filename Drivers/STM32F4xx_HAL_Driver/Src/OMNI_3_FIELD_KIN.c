#include "OMNI_3_FIELD_KIN.h"

double Omega_PID_Out;
double Kp_Omega = 0.5f, Ki_Omega = 0.001f, Kd_Omega = 0.0f;
double Speed_Omega_PID = 15.0f;
struct
{
    float x_axis; //(-1.0 -> 1.0)
    float y_axis; //(-1.0 -> 1.0)
    float z_axis; //(-1.0 -> 1.0)
} Joystick;

float rads_2_rpm(float rads_in)
{
    return (rads_in * 60) / (2 * PI);
}

void OmniRobot_Init(ORb *robot, float m_speed, float m_omg)
{
    robot->vx = 0.0f;
    robot->vy = 0.0f;
    robot->omega = 0.0f;
    robot->theta = 0.0f;
    robot->max_speed = m_speed;
    robot->max_omega = m_omg;
    robot->is_yaw_fix = 0;
    robot->fix_angle = 0;

    /*PID Init for Omega control to fix robot angle*/
    PID(&Omega_PID, &robot->IMU_theta, &Omega_PID_Out, &robot->fix_angle, Kp_Omega, Ki_Omega, Kd_Omega, _PID_P_ON_E, _PID_CD_REVERSE);
    PID_SetMode(&Omega_PID, _PID_MODE_AUTOMATIC);
    PID_SetSampleTime(&Omega_PID, 1);
    PID_SetOutputLimits(&Omega_PID, -Speed_Omega_PID, Speed_Omega_PID);
}

void Joystick_To_Velocity(ORb *robot, float max_speed, float max_omega)
{
    float mapped_x = (Joystick.x_axis) / 127.0;
    float mapped_y = (Joystick.y_axis) / 127.0;
    float mapped_z = (Joystick.z_axis) / 127.0;

    robot->vx = mapped_x * max_speed;
    robot->vy = mapped_y * max_speed;
    /*Check control mode field control or fixed yaw angle control*/
    if (robot->is_yaw_fix) 
    {
        PID_Compute(&Omega_PID);
        robot->omega = Omega_PID_Out;
    }
    else
    {
        robot->omega = mapped_z * max_omega;
    }
}

void OmniRobot_CalculateWheelSpeeds(ORb *robot, float *u1_in, float *u2_in, float *u3_in)
{
    // Set only Vx, Vy if control robot centric
    float vx_robot = robot->vx * cosf(robot->theta) + robot->vy * sinf(robot->theta);
    float vy_robot = -robot->vx * sinf(robot->theta) + robot->vy * cosf(robot->theta);

    float u1_temp = (vx_robot - D * robot->omega) / WHEEL_RADIUS;
    float u2_temp = (-0.5f * vx_robot + 0.866f * vy_robot - D * robot->omega) / WHEEL_RADIUS;
    float u3_temp = (-0.5f * vx_robot - 0.866f * vy_robot - D * robot->omega) / WHEEL_RADIUS;

    *u1_in = rads_2_rpm(u1_temp);
    *u2_in = rads_2_rpm(u2_temp);
    *u3_in = rads_2_rpm(u3_temp);
}

void OmniRobot_Field_Control(ORb *robot, PS4_DATA *ps4_joy, float imu_theta, uint8_t neg_heading)
{
    robot->IMU_theta = imu_theta;
    Joystick.x_axis = -ps4_joy->l_stick_x;
    Joystick.y_axis = -ps4_joy->l_stick_y;
    Joystick.z_axis = ps4_joy->r_stick_x;

    Joystick_To_Velocity(robot, robot->max_speed, robot->max_omega);

    if (neg_heading) /*Negative heading or positive heading*/
    {
        robot->theta = (180 + imu_theta) * (PI / 180.0f);
    }
    else
    {
        robot->theta = imu_theta * (PI / 180.0f);
    }

    if (robot->theta > 2 * PI)
    {
        robot->theta -= 2 * PI;
    }
    else if (robot->theta < 0)
    {
        robot->theta += 2 * PI;
    }

    OmniRobot_CalculateWheelSpeeds(robot, &robot->u[0], &robot->u[1], &robot->u[2]);
}

int16_t Round_Float(float x)
{
    return (int16_t)(x >= 0 ? x + 0.5f : x - 0.5f);
}

void Find_Closet_Angle(int16_t current_angle, int16_t *closest_angles)
{
    int16_t target_angles[] = {90, -90, 180, -180};

    for (uint8_t i = 0; i < 4; i++)
    {
        closest_angles[i] = target_angles[i] + 360.0f * roundf((current_angle - target_angles[i]) / 360.0f);
    }
}