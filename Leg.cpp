//#include <mbed.h>
#include "Leg.h"

#define PI 3.14159265358979

double r0(0.07), r1(0.14301*1.6), r2(0.29795/1.4); //ƒŠƒ“ƒN‚Ì’·‚³[m]

void Leg::calc(double x, double y, double z, double height, double &angle0, double &angle1, double &angle2)
{
  double d;
  d = sqrt(pow(x, 2) + pow(y, 2)) - r0;
  angle0 = (360 / (2 * PI)) * atan(y / x);
  if (sqrt(d * d + (height - z) * (height - z)) < r1 + r2) {
	  angle1 = -(360 / (2 * PI)) * (acos((pow(r1, 2) + pow(height - z, 2) + pow(d, 2) - pow(r2, 2)) / (2 * r1 * sqrt(pow(height - z, 2) + pow(d, 2)))) - atan((height - z) / d));
	  angle2 = 180 - (360 / (2 * PI)) * acos((pow(r1, 2) + pow(r2, 2) - pow(height - z, 2) - pow(d, 2)) / (2 * r1 * r2));
  }
  else {
	  std::cout << "legpositionerror" << std::endl;
  }
}

Leg::Leg(Servo &_s0, Servo &_s1, Servo &_s2) : s0(_s0), s1(_s1), s2(_s2)
{
  adjust=0.0;
}