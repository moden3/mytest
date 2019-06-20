//#include <mbed.h>
//#include "Common.h"
//#include "Creeping.h"
#include "Output.h"

#define PI 3.14159265358979

double ratio_right(1.0), ratio_left(0.85); //ratio_right�F�E���̕��������B0 < ratio_right < 1�Bratio_left�F�����̕��������B0 < ratio_left < 1�B
double initial_yaw;
bool slope_flag(0);

/************************�i�s�����m�F**********************************/

void Check_Yaw(double curve, double initial_yaw)
{
	bno.get_angles();
	yaw = bno.euler.yaw - yaw_offset;
    if (label == 2)
    {
        //pc.printf("get_yaw= %lf\n", bno.euler.yaw);
        //pc.printf("yaw= %lf\n", yaw);
        //pc.printf("yaw-initial_yaw= %lf\n", yaw - initial_yaw);
    }

    double alpha(10); //�Y���̋��e�l[deg]
    double delta_yaw(0.0);

    delta_yaw = yaw - initial_yaw;
	
    if (delta_yaw > 100)
    {
        delta_yaw -= 360;
    }
    else if (delta_yaw < -100)
    {
        delta_yaw += 360;
    }

    if (-(3 / 2) * alpha < delta_yaw && delta_yaw < -(1 / 2) * alpha)
    {
        if (label == 2)
        {
            //pc.printf("Litte Left\n");
        }
        ratio_right = 0.8;
        ratio_left = 1.0;
    }
    else if (-(5 / 2) * alpha < delta_yaw && delta_yaw < -(3 / 2) * alpha)
    {
        if (label == 2)
        {
            //pc.printf("Left\n");
        }
        ratio_right = 0.7;
        ratio_left = 1.0;
    }
    else if (delta_yaw < -(5 / 2) * alpha)
    {
        if (label == 2)
        {
            //pc.printf("Very Left\n");
        }
        ratio_right = 0.5;
        ratio_left = 1.0;
    }
    else if ((1 / 2) * alpha < delta_yaw && delta_yaw < (3 / 2) * alpha)
    {
        if (label == 2)
        {
            //pc.printf("Litte Right\n");
        }
        ratio_right = 1.0;
        ratio_left = 0.8;
    }
    else if ((3 / 2) * alpha < delta_yaw && delta_yaw < (5 / 2) * alpha)
    {
        if (label == 2)
        {
            //pc.printf("Right\n");
        }
        ratio_right = 1.0;
        ratio_left = 0.7;
    }
    else if ((5 / 2) * alpha < delta_yaw)
    {
        if (label == 2)
        {
            //pc.printf("Very Right\n");
        }
        ratio_right = 1.0;
        ratio_left = 0.5;
    }
    else
    {
        if (label == 2)
        {
            //pc.printf("Straight\n");
        }
        ratio_right = 1.0;
        ratio_left = 1.0;
    }
	//std::cout << "yaw=" << delta_yaw << std::endl;
}

/*************************Sand�m�F************************************/
void Check_Sand(void)
{
    bno.get_angles();
    pitch = bno.euler.pitch - pitch_offset;
    roll = bno.euler.roll - roll_offset;
    if (label == 1)
    {
        //pc.printf("pitch : %lf, roll : %lf\n", pitch, roll);
    }

    if (roll > 15)
    {
        if (label == 1)
        {
            //pc.printf("front Up\n");
        }
        if (pitch > 0)
        {
            if (rf.adjust == 0.0)
            {
				rf.adjust = 0.12;
                Output_Coordinate(rf.x, rf.y, rf.z + rf.adjust,
                                  rb.x, rb.y, rb.z,
                                  lb.x, lb.y, lb.z,
                                  lf.x, lf.y, lf.z,
                                  all_height_old,
                                  3, 0.1);
                if (label == 1)
                {
                    //pc.printf("rf adjust\n");
                }
            }
            else
            {
                if (label == 1)
                {
                    //pc.printf("error\n");
                }
            }
        }
        else
        {
            if (lf.adjust == 0.0)
            {
                lf.adjust = 0.12;
                Output_Coordinate(rf.x, rf.y, rf.z,
                                  rb.x, rb.y, rb.z,
                                  lb.x, lb.y, lb.z,
                                  lf.x, lf.y, lf.z + lf.adjust,
                                  all_height_old,
                                  3, 0.1);
                if (label == 1)
                {
                    //pc.printf("lf adjust\n");
                }
            }
            else
            {
                if (label == 1)
                {
                    //pc.printf("error\n");
                }
            }
        }
    }
    if (roll < -15)
    {
        //pc.printf("back up\n");
        if (pitch > 0)
        {
            if (rb.adjust == 0.0)
            {
                rb.adjust = 0.12;
                Output_Coordinate(rf.x, rf.y, rf.z,
                                  rb.x, rb.y, rb.z + rb.adjust,
                                  lb.x, lb.y, lb.z,
                                  lf.x, lf.y, lf.z,
                                  all_height_old,
                                  3, 0.1);
                if (label == 1)
                {
                    //pc.printf("rb adjust\n");
                }
            }
            else
            {
                if (label == 1)
                {
                    //pc.printf("error\n");
                }
            }
        }
        else
        {
            if (lb.adjust == 0.0)
            {
                lb.adjust = 0.12;
                Output_Coordinate(rf.x, rf.y, rf.z,
                                  rb.x, rb.y, rb.z,
                                  lb.x, lb.y, lb.z + lb.adjust,
                                  lf.x, lf.y, lf.z,
                                  all_height_old,
                                  3, 0.1);
                if (label == 1)
                {
                    //pc.printf("lb adjust\n");
                }
            }
            else
            {
                if (label == 1)
                {
                    //pc.printf("error\n");
                }
            }
        }
    }
    if (rf.adjust == 0.12 && rb.adjust == 0.12 && lf.adjust == 0.12 && lb.adjust == 0.12)
    {
        rf.adjust = rb.adjust = lf.adjust = lb.adjust = 0.0;
        Output_Coordinate(rf.x, rf.y, rf.z - 0.12,
                          rb.x, rb.y, rb.z - 0.12,
                          lb.x, lb.y, lb.z - 0.12,
                          lf.x, lf.y, lf.z - 0.12,
                          all_height_old,
                          3, 0.1);
        if (label == 1)
        {
            //pc.printf("Reset all adjust!\n");
        }
    }
}

/***************************Slope �m�F*********************************/
void Check_Slope(double y)
{
    bno.get_angles();
    roll = bno.euler.roll - roll_offset;
    rb.adjust = lb.adjust = -(2 * y + 0.195) * tan(roll * PI / 180);
    //pc.printf("rb= %lf, lb= %lf\n", rb.adjust, lb.adjust);
    if (abs(rb.adjust) >= 0.15) //���o��n�߂��t���O
    {
        slope_flag = 1;
        if (label == 3)
        {
            //pc.printf("slope_flag=1\n");
        }
    }
}

/************************�N���[�s���O���e***********************************/

void Creeping(double x0, double y0, double y1, double all_height,
              double step_length, double step_height,
              double creeping_step_times, double creeping_wait_time,
              double curve, int walk_times)
//x0,y0�F�����xy���W
//y1�F3�������̂Ƃ���y���W
//step_length�F����
//step_height�F�����グ���Ƃ��̍���
//curve�F�͂��ߌ����Ă�������Ɛi�s�����̂Ȃ��p�x[deg]
//walk_times�F�J��Ԃ��񐔁B���̉񐔂�n�Ƃ���ƁA2n + step_length�����i�ނ͂�(���ۂɂ͑S�R�i�܂Ȃ�)
{
    if (label == 1)
    {
        //pc.printf("Start Creeping!\n");
    }

    bno.get_angles();
    initial_yaw = bno.euler.yaw - yaw_offset;

    for (int i(1); i <= walk_times; i++)
    {
        if (mode == 1)
        {
            Check_Sand();
        }

        if (mode == 2)
        {
            Check_Slope(y0);
        }

        Output_Coordinate(x0, -y1, rf.adjust,
                          x0, -y1, rb.adjust,
                          x0, y0 * ratio_left, lb.adjust,
                          x0, y0 * ratio_left, lf.adjust,
                          all_height,
                          creeping_step_times * 4, creeping_wait_time); //(2)
		
        if (mode == 1)
        {
            Check_Sand();
        }
        if (mode == 2)
        {
            Check_Slope(y0);
        }
		
        Output_Coordinate(x0, -y1, rf.adjust + step_height,
                          x0, -y1, rb.adjust,
                          x0, y0 * ratio_left, lb.adjust,
                          x0, y0 * ratio_left, lf.adjust,
                          all_height,
                          creeping_step_times, creeping_wait_time); //(3)-1
		
        if (mode == 1)
        {
            Check_Sand();
        }
        if (mode == 2)
        {
            Check_Slope(y0);
        }

        if (mode != 3)
        {
            Check_Yaw(curve, initial_yaw);
        }
		
        Output_Coordinate(x0, -y1 + step_length * ratio_right, rf.adjust + step_height,
                          x0, -y1, rb.adjust,
                          x0, y0 * ratio_left, lb.adjust,
                          x0, y0 * ratio_left, lf.adjust,
                          all_height,
                          creeping_step_times * 4, creeping_wait_time); //(3)-2
		
        //pc.printf("ratio_right= %lf\n", ratio_right);
        //pc.printf("ratio_left= %lf\n", ratio_left);

        if (mode == 1)
        {
            Check_Sand();
        }
        if (mode == 2)
        {
            Check_Slope(y0);
        }
		
        Output_Coordinate(x0, -y1 + step_length * ratio_right, rf.adjust,
                          x0, -y1, rb.adjust,
                          x0, y0 * ratio_left, lb.adjust,
                          x0, y0 * ratio_left, lf.adjust,
                          all_height,
                          creeping_step_times * 4, creeping_wait_time); //(4)
		
        //pc.printf("(4) : %lf", -y1 + step_length * ratio_right);

        if (mode == 1)
        {
            Check_Sand();
        }
        if (mode == 2)
        {
            Check_Slope(y0);
        }
		
        if (mode != 3)
        {
            Check_Yaw(curve, initial_yaw);
        }
		
        Output_Coordinate(x0, y0 * ratio_right, rf.adjust,
                          x0, y0 * ratio_right, rb.adjust,
                          x0, -y1 + step_length * ratio_left, lb.adjust,
                          x0, -y1, lf.adjust,
                          all_height,
                          creeping_step_times * 4 * 2, creeping_wait_time); //(5)*/
		
        /***********************************************************************/
        if (mode == 2 && slope_flag == 1 && abs(rb.adjust) <= 0.02) //���o��I����1�T�C�N���o������break
        {
            break;

            if (label == 3)
            {
                //pc.printf("Finish climbing\n");
            }
        }
        /****************************************************************************/
		
        if (mode == 1)
        {
            Check_Sand();
        }
        if (mode == 2)
        {
            Check_Slope(y0);
        }

        Output_Coordinate(x0, y0 * ratio_right, rf.adjust,
                          x0, y0 * ratio_right, rb.adjust,
                          x0, -y1 + step_length * ratio_left, lb.adjust + step_height,
                          x0, -y1, lf.adjust,
                          all_height,
                          creeping_step_times, creeping_wait_time); //(6)-1

        if (mode == 1)
        {
            Check_Sand();
        }
        if (mode == 2)
        {
            Check_Slope(y0);
        }

        Output_Coordinate(x0, y0 * ratio_right, rf.adjust,
                          x0, y0 * ratio_right, rb.adjust,
                          x0, -y1, lb.adjust + step_height,
                          x0, -y1, lf.adjust,
                          all_height,
                          creeping_step_times * 4, creeping_wait_time); //(6)-2

        if (mode == 1)
        {
            Check_Sand();
        }
        if (mode == 2)
        {
            Check_Slope(y0);
        }

        Output_Coordinate(x0, y0 * ratio_right, rf.adjust,
                          x0, y0 * ratio_right, rb.adjust,
                          x0, -y1, lb.adjust,
                          x0, -y1, lf.adjust,
                          all_height,
                          creeping_step_times * 4, creeping_wait_time); //(7)

        if (mode == 1)
        {
            Check_Sand();
        }
        if (mode == 2)
        {
            Check_Slope(y0);
        }

        Output_Coordinate(x0, y0 * ratio_right, rf.adjust,
                          x0, y0 * ratio_right, rb.adjust,
                          x0, -y1, lb.adjust,
                          x0, -y1, lf.adjust + step_height,
                          all_height,
                          creeping_step_times, creeping_wait_time); //(8)-1

        if (mode == 1)
        {
            Check_Sand();
        }
        if (mode == 2)
        {
            Check_Slope(y0);
        }

        if (mode != 3)
        {
            Check_Yaw(curve, initial_yaw);
        }
		
        Output_Coordinate(x0, y0 * ratio_right, rf.adjust,
                          x0, y0 * ratio_right, rb.adjust,
                          x0, -y1, lb.adjust,
                          x0, -y1 + step_length * ratio_left, lf.adjust + step_height,
                          all_height,
                          creeping_step_times * 4, creeping_wait_time); //(8)-2

        if (mode == 1)
        {
            Check_Sand();
        }
        if (mode == 2)
        {
            Check_Slope(y0);
        }
		
        Output_Coordinate(x0, y0 * ratio_right, rf.adjust,
                          x0, y0 * ratio_right, rb.adjust,
                          x0, -y1, lb.adjust,
                          x0, -y1 + step_length * ratio_left, lf.adjust,
                          all_height,
                          creeping_step_times * 4, creeping_wait_time); //(9)
		
        if (mode == 1)
        {
            Check_Sand();
        }
        if (mode == 2)
        {
            Check_Slope(y0);
        }

        if (mode != 3)
        {
            Check_Yaw(curve, initial_yaw);
        }
		
        Output_Coordinate(x0, -y1, rf.adjust,
                          x0, -y1 + step_length * ratio_right, rb.adjust,
                          x0, y0 * ratio_left, lb.adjust,
                          x0, y0 * ratio_left, lf.adjust,
                          all_height,
                          creeping_step_times * 4 * 2 , creeping_wait_time); //(10)
		
        if (mode == 1)
        {
            Check_Sand();
        }
        if (mode == 2)
        {
            Check_Slope(y0);
        }
		
        Output_Coordinate(x0, -y1, rf.adjust,
                          x0, -y1 + step_length * ratio_right, rb.adjust + step_height,
                          x0, y0 * ratio_left, lb.adjust,
                          x0, y0 * ratio_left, lf.adjust,
                          all_height,
                          creeping_step_times, creeping_wait_time); //(11)-1

        if (mode == 1)
        {
            Check_Sand();
        }
        if (mode == 2)
        {
            Check_Slope(y0);
        }
		
        Output_Coordinate(x0, -y1, rf.adjust,
                          x0, -y1, rb.adjust + step_height,
                          x0, y0 * ratio_left, lb.adjust,
                          x0, y0 * ratio_left, lf.adjust,
                          all_height,
                          creeping_step_times * 4, creeping_wait_time); //(11)-2
		
        /***************************************************************/
        if (mode == 2 && slope_flag == 1 && abs(rb.adjust) <= 0.02) //���o��I����1�T�C�N���o������break
        {
            break;

            if (label == 3)
            {
                //pc.printf("Finish climbing\n");
            }
        }
        /*******************************************************/
    }
    if (mode == 1)
    {
        Check_Sand();
    }
    if (mode == 2)
    {
        Check_Slope(y0);
    }

    Output_Coordinate(x0, -y1, rf.adjust,
                      x0, -y1, rb.adjust,
                      x0, y0, lb.adjust,
                      x0, y0, lf.adjust,
                      all_height,
                      creeping_step_times * 4, creeping_wait_time); //(2)

    if (mode == 1)
    {
        Check_Sand();
    }
    if (mode == 2)
    {
        Check_Slope(y0);
    }

    Output_Coordinate(x0, y0, rf.adjust,
                      x0, y0, rb.adjust,
                      x0, y0, lb.adjust,
                      x0, y0, lf.adjust,
                      all_height,
                      creeping_step_times * 4, creeping_wait_time); //(1)

    if (label == 1)
    {
        //pc.printf("Finish Creeping!\n");
    }
}
