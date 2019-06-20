#ifndef _SERVO_H
#define _SERVO_H

//#include <mbed.h>

#include "usingthread.h"

class Servo
{
public:
  Servo(PwmOut &);
  double angle_min;    //�ŏ��p�x[deg]
  double angle_max;    //�ő�p�x[deg]
  double angle_offset; //��p�x[deg]
  double angle_old;
  double angle_ref;
  double angle_now;
  double angle(double); //�p�x�w�ߒl����p���X�����v�Z
  PwmOut &output;
};

#endif