#include "SWERVE_4_KINEMATIC.h"

void Swerve_Drive_Init(Swerve_Drive *swerve, float wheelbase, float trackwidth)
{
    swerve->wheelbase = wheelbase;
    swerve->trackwidth = trackwidth;
    for (int i = 0; i < 4; i++)
    {
        swerve->wheel[i].speed = 0.0f;
        swerve->wheel[i].angle = 0.0f;
    }
}

void Swerve_Cal_Kinematic(Swerve_Drive *swerve, Robot_Velo *velocity)
{
    // Bán kính quay (radius) trên hai trục
    float A = velocity->x - velocity->omega * (swerve->wheelbase / 2.0f);
    float B = velocity->x + velocity->omega * (swerve->wheelbase / 2.0f);
    float C = velocity->y - velocity->omega * (swerve->trackwidth / 2.0f);
    float D = velocity->y + velocity->omega * (swerve->trackwidth / 2.0f);

    // Tính toán tốc độ và góc cho từng bánh xe
    swerve->wheel[0].speed = sqrtf(B * B + D * D);
    swerve->wheel[0].angle = atan2f(D, B);

    swerve->wheel[1].speed = sqrtf(B * B + C * C);
    swerve->wheel[1].angle = atan2f(C, B);

    swerve->wheel[2].speed = sqrtf(A * A + D * D);
    swerve->wheel[2].angle = atan2f(D, A);

    swerve->wheel[3].speed = sqrtf(A * A + C * C);
    swerve->wheel[3].angle = atan2f(C, A);
}
