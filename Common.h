#ifndef _COMMON_H_
#define _COMMON_H_

//#include <mbed.h>
#include "Leg.h"
//#include "BNO055.h"

//プロトタイプ宣言

/****************main.cpp*********************/
extern Leg rf;
extern Leg rb;
extern Leg lb;
extern Leg lf;
extern Serial pc;
extern DigitalIn UB;
extern BNO055 bno;
extern double pitch, roll, yaw;
extern double pitch_offset, roll_offset, yaw_offset;
extern double all_height_old;
extern int mode;
extern int label;

void Set_Angle_Offset(double, double, double,
                      double, double, double,
                      double, double, double,
                      double, double, double);
void Set_Angle_Min(double, double, double,
                   double, double, double,
                   double, double, double,
                   double, double, double);
void Set_Angle_Max(double, double, double,
                   double, double, double,
                   double, double, double,
                   double, double, double);
void Wait_UB(void);
void Body_Up(double, double, double, double);
void Body_Down(double, double);
void Hip_Shaking(double, double, double,
                 double, int);
void Offset_Position(void);
void Store_Position(void);
void Set_Gyro_Offset(void);


/**********************************************/

#endif
