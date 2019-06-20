#include "Common.h"

/*****************************ステップ出力関数(角度指定)***************************/
/*
void Output_Angle(double angle_a_s0, double angle_a_s1, double angle_a_s2,
                  double angle_b_s0, double angle_b_s1, double angle_b_s2,
                  double angle_c_s0, double angle_c_s1, double angle_c_s2,
                  double angle_d_s0, double angle_d_s1, double angle_d_s2,
                  int step_times, double wait_time)
//step_timesはステップ回数(20以上)
//wait_timeは各ステップの待ち時間[s]
//step_timesを0にするとステップなしで即座にangle_refを出力

{
  rf.s0.angle_ref = angle_a_s0;
  rf.s1.angle_ref = angle_a_s1;
  rf.s2.angle_ref = angle_a_s2;
  rb.s0.angle_ref = angle_b_s0;
  rb.s1.angle_ref = angle_b_s1;
  rb.s2.angle_ref = angle_b_s2;
  lb.s0.angle_ref = angle_c_s0;
  lb.s1.angle_ref = angle_c_s1;
  lb.s2.angle_ref = angle_c_s2;
  lf.s0.angle_ref = angle_d_s0;
  lf.s1.angle_ref = angle_d_s1;
  lf.s2.angle_ref = angle_d_s2;

  if (step_times == 0)
  {
    //pc.printf("step_times==0\n");
    rf.s0.output.pulsewidth_us(rf.s0.angle(rf.s0.angle_ref));
    rf.s1.output.pulsewidth_us(rf.s1.angle(rf.s1.angle_ref));
    rf.s2.output.pulsewidth_us(rf.s2.angle(rf.s2.angle_ref));
    rb.s0.output.pulsewidth_us(rb.s0.angle(-rb.s0.angle_ref));
    rb.s1.output.pulsewidth_us(rb.s1.angle(rb.s1.angle_ref));
    rb.s2.output.pulsewidth_us(rb.s2.angle(rb.s2.angle_ref));
    lb.s0.output.pulsewidth_us(lb.s0.angle(lb.s0.angle_ref));
    lb.s1.output.pulsewidth_us(lb.s1.angle(lb.s1.angle_ref));
    lb.s2.output.pulsewidth_us(lb.s2.angle(lb.s2.angle_ref));
    lf.s0.output.pulsewidth_us(lf.s0.angle(-lf.s0.angle_ref));
    lf.s1.output.pulsewidth_us(lf.s1.angle(lf.s1.angle_ref));
    lf.s2.output.pulsewidth_us(lf.s2.angle(lf.s2.angle_ref));
	
    wait(wait_time);
  }
  else
  {
	  //std::cout << "angle" << std::endl;
    //pc.printf("step_times!=0\n");
    for (int i(1); i <= step_times; i++)
    {
      rf.s0.angle_now = (rf.s0.angle_ref - rf.s0.angle_old) * ((double)i / (double)step_times) + rf.s0.angle_old;
      rf.s1.angle_now = (rf.s1.angle_ref - rf.s1.angle_old) * ((double)i / (double)step_times) + rf.s1.angle_old;
      rf.s2.angle_now = (rf.s2.angle_ref - rf.s2.angle_old) * ((double)i / (double)step_times) + rf.s2.angle_old;
      rb.s0.angle_now = (rb.s0.angle_ref - rb.s0.angle_old) * ((double)i / (double)step_times) + rb.s0.angle_old;
      rb.s1.angle_now = (rb.s1.angle_ref - rb.s1.angle_old) * ((double)i / (double)step_times) + rb.s1.angle_old;
      rb.s2.angle_now = (rb.s2.angle_ref - rb.s2.angle_old) * ((double)i / (double)step_times) + rb.s2.angle_old;
      lb.s0.angle_now = (lb.s0.angle_ref - lb.s0.angle_old) * ((double)i / (double)step_times) + lb.s0.angle_old;
      lb.s1.angle_now = (lb.s1.angle_ref - lb.s1.angle_old) * ((double)i / (double)step_times) + lb.s1.angle_old;
      lb.s2.angle_now = (lb.s2.angle_ref - lb.s2.angle_old) * ((double)i / (double)step_times) + lb.s2.angle_old;
      lf.s0.angle_now = (lf.s0.angle_ref - lf.s0.angle_old) * ((double)i / (double)step_times) + lf.s0.angle_old;
      lf.s1.angle_now = (lf.s1.angle_ref - lf.s1.angle_old) * ((double)i / (double)step_times) + lf.s1.angle_old;
      lf.s2.angle_now = (lf.s2.angle_ref - lf.s2.angle_old) * ((double)i / (double)step_times) + lf.s2.angle_old;
	  
      rf.s0.output.pulsewidth_us(rf.s0.angle(rf.s0.angle_now));
      rf.s1.output.pulsewidth_us(rf.s1.angle(rf.s1.angle_now));
      rf.s2.output.pulsewidth_us(rf.s2.angle(rf.s2.angle_now));
      rb.s0.output.pulsewidth_us(rb.s0.angle(-rb.s0.angle_now));
      rb.s1.output.pulsewidth_us(rb.s1.angle(rb.s1.angle_now));
      rb.s2.output.pulsewidth_us(rb.s2.angle(rb.s2.angle_now));
      lb.s0.output.pulsewidth_us(lb.s0.angle(lb.s0.angle_now));
      lb.s1.output.pulsewidth_us(lb.s1.angle(lb.s1.angle_now));
      lb.s2.output.pulsewidth_us(lb.s2.angle(lb.s2.angle_now));
      lf.s0.output.pulsewidth_us(lf.s0.angle(-lf.s0.angle_now));
      lf.s1.output.pulsewidth_us(lf.s1.angle(lf.s1.angle_now));
      lf.s2.output.pulsewidth_us(lf.s2.angle(lf.s2.angle_now));

      wait(wait_time);
    }
  }

  rf.s0.output.pulsewidth_us(rf.s0.angle(rf.s0.angle_ref));
  rf.s1.output.pulsewidth_us(rf.s1.angle(rf.s1.angle_ref));
  rf.s2.output.pulsewidth_us(rf.s2.angle(rf.s2.angle_ref));
  rb.s0.output.pulsewidth_us(rb.s0.angle(-rb.s0.angle_ref));
  rb.s1.output.pulsewidth_us(rb.s1.angle(rb.s1.angle_ref));
  rb.s2.output.pulsewidth_us(rb.s2.angle(rb.s2.angle_ref));
  lb.s0.output.pulsewidth_us(lb.s0.angle(lb.s0.angle_ref));
  lb.s1.output.pulsewidth_us(lb.s1.angle(lb.s1.angle_ref));
  lb.s2.output.pulsewidth_us(lb.s2.angle(lb.s2.angle_ref));
  lf.s0.output.pulsewidth_us(lf.s0.angle(-lf.s0.angle_ref));
  lf.s1.output.pulsewidth_us(lf.s1.angle(lf.s1.angle_ref));
  lf.s2.output.pulsewidth_us(lf.s2.angle(lf.s2.angle_ref));

  rf.s0.angle_old = rf.s0.angle_ref;
  rf.s1.angle_old = rf.s1.angle_ref;
  rf.s2.angle_old = rf.s2.angle_ref;
  rb.s0.angle_old = rb.s0.angle_ref;
  rb.s1.angle_old = rb.s1.angle_ref;
  rb.s2.angle_old = rb.s2.angle_ref;
  lb.s0.angle_old = lb.s0.angle_ref;
  lb.s1.angle_old = lb.s1.angle_ref;
  lb.s2.angle_old = lb.s2.angle_ref;
  lf.s0.angle_old = lf.s0.angle_ref;
  lf.s1.angle_old = lf.s1.angle_ref;
  lf.s2.angle_old = lf.s2.angle_ref;
}
*/
/**************************ステップ出力関数(足先座標指定)***********************/
/*
void Output_Coordinate(double x_a, double y_a, double z_a,
                       double x_b, double y_b, double z_b,
                       double x_c, double y_c, double z_c,
                       double x_d, double y_d, double z_d,
                       double all_height,
                       int step_times, double wait_time)
{
  rf.x = x_a;
  rf.y = y_a;
  rf.z = z_a;
  rb.x = x_b;
  rb.y = y_b;
  rb.z = z_b;
  lb.x = x_c;
  lb.y = y_c;
  lb.z = z_c;
  lf.x = x_d;
  lf.y = y_d;
  lf.z = z_d;

  all_height_old = all_height;
  
  rf.calc(rf.x, rf.y, rf.z, all_height, rf.theta0, rf.theta1, rf.theta2);
  rb.calc(rb.x, rb.y, rb.z, all_height, rb.theta0, rb.theta1, rb.theta2);
  lb.calc(lb.x, lb.y, lb.z, all_height, lb.theta0, lb.theta1, lb.theta2);
  lf.calc(lf.x, lf.y, lf.z, all_height, lf.theta0, lf.theta1, lf.theta2);
  
  Output_Angle(rf.theta0, rf.theta1, rf.theta2,
               rb.theta0, rb.theta1, rb.theta2,
               lb.theta0, lb.theta1, lb.theta2,
               lf.theta0, lf.theta1, lf.theta2,
               step_times, wait_time);
  
  //pc.printf("Leg fr : %lf, %lf, %lf\n", fr.theta0, fr.theta1, fr.theta2);
  //pc.printf("Leg br : %lf, %lf, %lf\n", br.theta0, br.theta1, br.theta2);
  //pc.printf("Leg bl : %lf, %lf, %lf\n", bl.theta0, bl.theta1, bl.theta2);
  //pc.printf("Leg fl : %lf, %lf, %lf\n", fl.theta0, fl.theta1, fl.theta2);
  //pc.printf("*********************************************\n");
}
*/