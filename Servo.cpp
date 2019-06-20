#include "Common.h"
//#include <mbed.h>

double Servo::angle(double angle_ref) //サーボの目標角度からPWM信号を出す関数
{
    if (angle_ref < angle_min)
    {
        //angle_ref = angle_min;
    }
    else if (angle_ref > angle_max)
    {
        //angle_ref = angle_max;
    }
    double output(0.0);
    output = (2000 / 270) * (angle_ref + angle_offset) + 500;
    return output;
}

Servo::Servo(PwmOut& _output):output(_output){
    output.period_ms(3.0);
}