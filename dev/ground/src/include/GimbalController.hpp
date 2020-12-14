/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG
 * Author     : Antariksh Narain
 * Description: Gimbal control commands for Ground Station
----------------------------------------------------------------- */

class GimbalController
{
private:
    float pitch_value_angle;
    float pitch_value_linear;
    float roll_value_angle;
    float roll_value_linear;

public:
    bool ChangePitch(float value);
    bool ChangeRoll(float value);
};