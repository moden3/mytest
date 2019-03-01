#ifndef _SERVO_H
#define _SERVO_H

//#include <mbed.h>

#include "usingthread.h"

class Servo
{
public:
  Servo(PwmOut &);
  double angle_min;    //最小角度[deg]
  double angle_max;    //最大角度[deg]
  double angle_offset; //基準角度[deg]
  double angle_old;
  double angle_ref;
  double angle_now;
  double angle(double); //角度指令値からパルス幅を計算
  PwmOut &output;
};

#endif