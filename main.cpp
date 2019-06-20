//#include <mbed.h>
//#include "Common.h"
#include "Creeping.h"
//#include "Output.h"
#include "Rotation.h"
#include "Trot.h"

#define PI 3.14159265358979

int mode(2);  //0�Ȃ畁�ʂ̕��s�A1�Ȃ�Sand�A2�Ȃ�Slope�A3�Ȃ�yaw�p�̏C���I�t�A4�Ȃ�3������
int label(3); //0�Ȃ�printf�Ȃ��A1�Ȃ�Check_Sand�A2�Ȃ�Chek_Yaw�A3�Ȃ�Check_Slope�A4�Ȃ畁�ʂ̕��s�̃f�o�b�N�����\��

/************************�e��s���ݒ�***********************/

Serial pc(USBTX, USBRX, 115200);
DigitalIn UB(USER_BUTTON);
BNO055 bno(PB_4, PA_8);
AnalogIn analog_in(PC_0);

DigitalOut out1(PB_9); //D14
DigitalOut out2(PB_3); //D3

/***********************PWM�s���o�͐ݒ�**********************/
PwmOut rf_output[3] = {PwmOut(PB_2), PwmOut(PA_1), PwmOut(PB_0)};   //�E�O
PwmOut rb_output[3] = {PwmOut(PA_11), PwmOut(PC_8), PwmOut(PA_6)};  //�E���
PwmOut lb_output[3] = {PwmOut(PA_7), PwmOut(PB_6), PwmOut(PB_5)};   //�����
PwmOut lf_output[3] = {PwmOut(PB_10), PwmOut(PA_10), PwmOut(PB_7)}; //���O

/***********************�O���[�o���ϐ�**********************/
double pitch, roll, yaw;
double pitch_offset, roll_offset, yaw_offset;
double all_height_old = 0;

/*************************Servo�̃I�u�W�F�N�g*****************/

Servo rf_servo[3] = {Servo(rf_output[0]), Servo(rf_output[1]), Servo(rf_output[2])};
Servo rb_servo[3] = {Servo(rb_output[0]), Servo(rb_output[1]), Servo(rb_output[2])};
Servo lb_servo[3] = {Servo(lb_output[0]), Servo(lb_output[1]), Servo(lb_output[2])};
Servo lf_servo[3] = {Servo(lf_output[0]), Servo(lf_output[1]), Servo(lf_output[2])};

/*************************Leg�̃I�u�W�F�N�g*******************/

Leg rf(rf_servo[0], rf_servo[1], rf_servo[2]);
Leg rb(rb_servo[0], rb_servo[1], rb_servo[2]);
Leg lb(lb_servo[0], lb_servo[1], lb_servo[2]);
Leg lf(lf_servo[0], lf_servo[1], lf_servo[2]);

/************************�^�C�}�[���荞��*********************/
Ticker interrupt;

/****************************�I�t�Z�b�g�ݒ�֐�***************************/
void Set_Angle_Offset(double offset_a_s0, double offset_a_s1, double offset_a_s2,
                      double offset_b_s0, double offset_b_s1, double offset_b_s2,
                      double offset_c_s0, double offset_c_s1, double offset_c_s2,
                      double offset_d_s0, double offset_d_s1, double offset_d_s2)
{
  rf.s0.angle_offset = offset_a_s0*0;
  rf.s1.angle_offset = offset_a_s1*0;
  rf.s2.angle_offset = offset_a_s2*0;
  rb.s0.angle_offset = offset_b_s0*0;
  rb.s1.angle_offset = offset_b_s1*0;
  rb.s2.angle_offset = offset_b_s2*0;
  lb.s0.angle_offset = offset_c_s0*0;
  lb.s1.angle_offset = offset_c_s1*0;
  lb.s2.angle_offset = offset_c_s2*0;
  lf.s0.angle_offset = offset_d_s0*0;
  lf.s1.angle_offset = offset_d_s1*0;
  lf.s2.angle_offset = offset_d_s2*0;
}

/****************************�ŏ��p�x�ݒ�֐�***************************/
void Set_Angle_Min(double min_a_s0, double min_a_s1, double min_a_s2,
                   double min_b_s0, double min_b_s1, double min_b_s2,
                   double min_c_s0, double min_c_s1, double min_c_s2,
                   double min_d_s0, double min_d_s1, double min_d_s2)
{
  rf.s0.angle_min = min_a_s0;
  rf.s1.angle_min = min_a_s1;
  rf.s2.angle_min = min_a_s2;
  rb.s0.angle_min = min_b_s0;
  rb.s1.angle_min = min_b_s1;
  rb.s2.angle_min = min_b_s2;
  lb.s0.angle_min = min_c_s0;
  lb.s1.angle_min = min_c_s1;
  lb.s2.angle_min = min_c_s2;
  lf.s0.angle_min = min_d_s0;
  lf.s1.angle_min = min_d_s1;
  lf.s2.angle_min = min_d_s2;
}
/****************************�ő�p�x�ݒ�֐�***************************/
void Set_Angle_Max(double max_a_s0, double max_a_s1, double max_a_s2,
                   double max_b_s0, double max_b_s1, double max_b_s2,
                   double max_c_s0, double max_c_s1, double max_c_s2,
                   double max_d_s0, double max_d_s1, double max_d_s2)
{
  rf.s0.angle_max = max_a_s0;
  rf.s1.angle_max = max_a_s1;
  rf.s2.angle_max = max_a_s2;
  rb.s0.angle_max = max_b_s0;
  rb.s1.angle_max = max_b_s1;
  rb.s2.angle_max = max_b_s2;
  lb.s0.angle_max = max_c_s0;
  lb.s1.angle_max = max_c_s1;
  lb.s2.angle_max = max_c_s2;
  lf.s0.angle_max = max_d_s0;
  lf.s1.angle_max = max_d_s1;
  lf.s2.angle_max = max_d_s2;
}

/**************************���[�U�[�{�^�������ܑҋ@����֐�*******************/
/*void Wait_UB(void)
{

  pc.printf("Please push user button.\n");

  while (UB)
    ;
  while (!UB)
    ;
  wait(0.5);
}*/

/************************���̂��グ��(�N���[�s���O�p)*************************************/
void Body_Up(double x0, double y0, double y1, double all_height)
{
  Output_Coordinate(x0, -y1, 0,
                    x0, -y1, 0,
                    x0, y0, 0,
                    x0, y0, 0,
                    0,
                    0, 1); //xo�Ayo�ő��𕂂��������
  wait(0.5);

  Output_Coordinate(x0, -y1, 0,
					x0, -y1, 0,
					x0, y0, 0,
					x0, y0, 0,
					all_height,
					20, 0.05); //(1)
}

/************************���̂��グ��(��̎�)*************************************/
void Body_Up_Dainozi(double x0, double y0, double y1, double all_height)
{
  Output_Coordinate(x0, y0, 0,
                    x0, y0, 0,
                    x0, y0, 0,
                    x0, y0, 0,
                    0,
                    0, 1); //xo�Ayo�ő��𕂂��������
  wait(0.5);
  
  Output_Coordinate(x0, y0, 0,
                    x0, y0, 0,
                    x0, y0, 0,
                    x0, y0, 0,
                    all_height,
                    20, 0.05); //(1)
  
}

/*****************************���̂�������*******************************/
void Body_Down(double x0, double y0)
{
  Output_Coordinate(x0, y0, 0.01,
                    x0, y0, 0.01,
                    x0, y0, 0.01,
                    x0, y0, 0.01,
                    0,
                    100, 0.05); //xo�Ayo�ő��𕂂��������
}

/**************************���U��**********************************/
void Hip_Shaking(double x0, double y0, double all_height,
                 double shaking_angle, int shaking_times) //rotation_angle�͉�]�p�x[deg]
{

  Output_Angle(rf.s0.angle_old + shaking_angle, rf.s1.angle_old, rf.s2.angle_old,
               rb.s0.angle_old - shaking_angle, rb.s1.angle_old, rb.s2.angle_old,
               lb.s0.angle_old + shaking_angle, lb.s1.angle_old, lb.s2.angle_old,
               lf.s0.angle_old - shaking_angle, lf.s1.angle_old, lf.s2.angle_old,
               10, 0.1);

  for (int i(1); i <= shaking_times; i++)
  {
    Output_Angle(rf.s0.angle_old - shaking_angle * 2, rf.s1.angle_old, rf.s2.angle_old,
                 rb.s0.angle_old + shaking_angle * 2, rb.s1.angle_old, rb.s2.angle_old,
                 lb.s0.angle_old - shaking_angle * 2, lb.s1.angle_old, lb.s2.angle_old,
                 lf.s0.angle_old + shaking_angle * 2, lf.s1.angle_old, lf.s2.angle_old,
                 10, 0.1);

    Output_Angle(rf.s0.angle_old + shaking_angle * 2, rf.s1.angle_old, rf.s2.angle_old,
                 rb.s0.angle_old - shaking_angle * 2, rb.s1.angle_old, rb.s2.angle_old,
                 lb.s0.angle_old + shaking_angle * 2, lb.s1.angle_old, lb.s2.angle_old,
                 lf.s0.angle_old - shaking_angle * 2, lf.s1.angle_old, lf.s2.angle_old,
                 10, 0.1);
  }

  Output_Angle(rf.s0.angle_old - shaking_angle, rf.s1.angle_old, rf.s2.angle_old,
               rb.s0.angle_old + shaking_angle, rb.s1.angle_old, rb.s2.angle_old,
               lb.s0.angle_old - shaking_angle, lb.s1.angle_old, lb.s2.angle_old,
               lf.s0.angle_old + shaking_angle, lf.s1.angle_old, lf.s2.angle_old,
               10, 0.1);
}

/***************************�I�t�Z�b�g�̎p��************************/
void Offset_Position(void)
{
  Output_Angle(0, 0, 0,
               0, 0, 0,
               0, 0, 0,
               0, 0, 0,
               0, 1);
  pc.printf("Offset_Position!\n");
}

/*************************���i�[�p��**************************/
void Store_Position(void)
{
  Output_Angle(45, -90, -30,
               45, -90, -30,
               45, -90, -30,
               45, -90, -30,
               0, 1);
  pc.printf("Store_Position!\n");
}

/********************�W���C���Z���T�̃I�t�Z�b�g�ݒ�**********************/
void Set_Gyro_Offset(void)
{
  bno.get_angles();
  pitch_offset = bno.euler.pitch;
  roll_offset = bno.euler.roll;
  yaw_offset = bno.euler.yaw;
  pc.printf("OFFSET pitch : %lf, roll : %lf, yaw : %lf\n", pitch_offset, roll_offset, yaw_offset);
}

/*********************�����Z���T**********************/
/*double Distance(void)
{
  double distance;
  distance = (160 / 7) * (35 / (35 * analog_in.read() * 3.3 - 4)); //[cm]
  pc.printf("distance= %lf\n", distance);
  return distance;
}*/

/*************************���C���֐�****************************/
void mainsimulation()
{
	rf.adjust = 0.0; lf.adjust = 0.0; rb.adjust = 0.0; lb.adjust = 0.0;
  pc.baud(9600);
  bno.setmode(OPERATION_MODE_NDOF);

  out1 = 0; //�Q���Q�󂯓n���p�}�C�R���ւ̒ʐM
  out2 = 0;

  /*************************�e��ݒ�**************************/
  Set_Angle_Offset(96, 125, 135,
                   139, 126, 129,
                   147, 130, 138,
                   122, 138, 145); //�I�t�Z�b�g�ݒ�
 
  Set_Angle_Min(-100, -100, -100,
                -100, -100, -100,
                -100, -100, -100,
                -100, -100, -100); //�ŏ��p�x�ݒ�
  
  Set_Angle_Max(100, 100, 100,
                100, 100, 100,
                100, 100, 100,
                100, 100, 100); //�ő�p�x�ݒ�

  Set_Gyro_Offset(); //�W���C���̃I�t�Z�b�g�ݒ�
  
  interrupt.attach(&Gyro, 1.0); //1�b���ƂɃW���C���Z���T�̒l��ǂݎ��
  interrupt.attach(&print_adjust, 1);

  /***********************************************************/

  //Wait_UB();

  //Offset_Position();

  /*for (int i(0); i <= 3; i++)
  {
    Output_Angle(0, -90, -90,
                 0, -90, -90,
                 0, -90, -90,
                 0, -90, -90,
                 20, 0.1);
    wait(1);

    Output_Angle(0, -90, 90,
                 0, -90, 90,
                 0, -90, 90,
                 0, -90, 90,
                 20, 0.1);
    wait(1);
  }*/

  /*while (1)
  {
    pc.printf("distance= %lf\n", Distance());
  }*/

  /*Wait_UB();

  Creeping(0.15, 0.20, 0.020, 0.17,
             0.40, 0.20,
             10, 0.01,
             0.0, 20);
   */

  //Wait_UB();

  wait(2);
  
  if (mode == 0 || mode == 1 || mode == 3) //���ʂ̕��s�܂���slope
  {
    while (Distance() > 20 || Distance() < 10)
    {
      if (label == 4)
      {
        pc.printf("Please hold out your hand on the sensor.\n");
      }
	  wait(0.01);
    }
	
    pc.printf("Start Walking\n");

    /*********���[�v�N���A�����p�����[�^*****/
    /*
    Body_Up(0.15, 0.20, 0.020, 0.25);
    Creeping(0.15, 0.20, 0.020, 0.25,
             0.40, 0.25,
             10, 0.01,
             0.0, 15);
    */
    /*******�������N���A�����p�����[�^******/
    //rf.adjust = lf.adjust = 0.1;
	Wait_UB();
    Body_Up(0.15, 0.20, 0.020, 0.20);
	Wait_UB();
    Creeping(0.15, 0.20, 0.020, 0.20,
             0.4, 0.30,
             7, 0.01,
             0.0, 15);
    /**************************************/
  }

  if (mode == 2) //��o��
  {
    while (Distance() > 20 || Distance() < 10)
    {
      if (label == 3)
      {
        pc.printf("Please hold out your hand on the sensor.\n");
      }
	  wait(0.01);
    }
    pc.printf("Start Climbing\n");
    Body_Up(0.15, 0.25, 0.020, 0.20);
    //Body_Up_Dainozi(0.15, 0.15, 0.02, 0.30);

    Creeping(0.15, 0.25, 0.020, 0.20,
             0.40, 0.20,
             7, 0.01,
             0.0, 2); //��o��̃_���_���p�����[�^(���o��I������break����)
			 
    pc.printf("Finish climbing\n");

    /*��o��I��*/

    /*yaw�p�����]�p�𐳊m�ɋ��߂�ꍇ*/
    /*bno.get_angles();
      yaw = bno.euler.yaw - yaw_offset;
      double rotate_angle = yaw - initial_yaw;*/
    /*
    Output_Coordinate(0.15, 0.15, 0,
                      0.15, 0.15, 0,
                      0.15, 0.15, 0,
                      0.15, 0.15, 0,
                      0.30,
                      20, 0.05);

    */

    Rotation_Deg(0.15, 0.15, 0.30,
                 45, 4); //90�x��]??
    wait(1);

    out1 = 1;
    out2 = 1;
  }

  if (mode == 4)
  {
    Wait_UB();
	Body_Up(0.15, 0.25, 0.020, 0.20);

    Output_Coordinate(0.15, 0.15, 0,
                      0.15, 0.15, 0,
                      0.15, -0.02, 0,
                      0.15, -0.02, 0,
                      0.20,
                      10, 0.01);

    Wait_UB();

    Output_Coordinate(0.15, 0.15, 0,
                      0.15, 0.15, 0,
                      0.15, -0.02, 0.29,
                      0.15, -0.02, 0,
                      0.20,
                      10, 0.1);

    Output_Coordinate(0.15, 0.15, 0,
                      0.15, 0.15, 0,
                      0.15, 0.15, 0.3,
                      0.15, -0.02, 0,
                      0.20,
                      10, 0.1);

    Output_Coordinate(0.15, 0.15, 0,
                      0.15, 0.15, 0,
                      0.15, 0.15, 0.3,
                      0.15, -0.02, 0,
                      0.20,
                      10, 0.1);
  }

  if (mode == 5) {
	  Body_Up_Dainozi(0.15, 0.25, 0.020, 0.20);
	  Trot(0.15, 0.25, 0.2,
		  0.4, 0.05,
		  1.0, 1.0,
		  5);
  }

  if (mode == 6) {
	  Output_Coordinate(0.20, 0.20, 0.3,
		  0.20, 0.20, 0.3,
		  0.20, 0.20, 0.3,
		  0.20, 0.20, 0.3,
		  0.10,
		  0, 0.05);
	  while (1) {
		  Output_Coordinate(0.20, 0.20, 0,
			  0.20, 0.20, 0,
			  0.20, 0.20, 0,
			  0.20, 0.20, 0,
			  0.10,
			  20, 0.05);
		  /*Output_Coordinate(0.15, -0.02, 0,
			  0.15, -0.02, 0,
			  0.15, 0.20, 0,
			  0.15, 0.20, 0,
			  0.15,
			  20, 0.02);
		  Output_Coordinate(0.15, -0.12, 0,
			  0.15, 0.08, 0,
			  0.15, 0.30, 0,
			  0.15, 0.10, 0,
			  0.15,
			  20, 0.02);
		  Output_Coordinate(0.15, 0.08, 0.50,
			  0.15, -0.12, 0,
			  0.15, 0.10, 0,
			  0.15, 0.30, 0,
			  0.15,
			  20, 0.02);
		  Output_Coordinate(0.15, -0.02, 0,
			  0.15, -0.02, 0,
			  0.15, 0.20, 0,
			  0.15, 0.20, 0,
			  0.15,
			  20, 0.02);
		  Output_Coordinate(0.15, -0.02, 0.40,
			  0.15, -0.02, 0,
			  0.15, 0.20, 0,
			  0.15, 0.20, 0,
			  0.15,
			  20, 0.02);*/
	  }
  }

  /*Output_Coordinate( 0.25, 0.25, 0,
                    0.25, 0.25, 0,
                    0.25, 0.25, 0,
                    0.25, 0.25, 0,
                    0.25,
                    20, 0.05);*/

  Wait_UB();

  Body_Down(0.15, 0.20);

  //Store_Position();

  //return 0;
}