#ifndef SWERVE_4_KIN_H
#define SWERVE_4_KIN_H

#include <math.h>

typedef struct
{
    float x;     // Vận tốc trên trục x
    float y;     // Vận tốc trên trục y
    float omega; // Tốc độ góc của robot (r/s)
} Robot_Velo;

typedef struct
{
    float speed; 
    float angle;
} Wheel_Pod;

typedef struct
{
    Wheel_Pod wheel[4];  
    float wheelbase;  // Khoảng cách giữa các bánh xe dọc theo trục x
    float trackwidth; // Khoảng cách giữa các bánh xe dọc theo trục y
} Swerve_Drive;

void Swerve_Drive_Init(Swerve_Drive *swerve, float wheelbase, float trackwidth);
void Swerve_Cal_Kinematic(Swerve_Drive *swerve, Robot_Velo *velocity);

#endif
