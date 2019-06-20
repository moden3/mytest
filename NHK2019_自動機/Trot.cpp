//#include <mbed.h>
//#include "Common.h"
#include "Output.h"

/**************************�g���b�g���e****************************/
void Trot(double x0, double y0, double all_height,
          double step_length, double step_height,
          double ratio_right, double ratio_left,
          int walk_times)
//a���i�s�������O�B�I�t�Z�b�g�̕����̊֌W��Aa��fr��bl�̂݁B
//x0,y0�F�����xy���W
//step_length�F����
//step_height�F�����グ���Ƃ��̍���
//ratio_right�F�E���̕��������B0 < ratio_right < 1�B
//ratio_left�F�����̕��������B0 < ratio_left < 1�B
//walk_times�F�J��Ԃ��񐔁B���̉񐔂�n�Ƃ���ƁA2n + step_length�����i�ނ͂�(���ۂɂ͑S�R�i�܂Ȃ�)
{
    //pc.printf("Start Trot!\n");

    /*Output_Coordinate(x0, y0, 0,
                      x0, y0, 0,
                      x0, y0, 0,
                      x0, y0, 0,
                      0,
                      0, 1); //xo�Ayo�ő��𕂂��������
    wait(0.10);*/

    /*Output_Coordinate(x0, y0, 0,
                      x0, y0, 0,
                      x0, y0, 0,
                      x0, y0, 0,
                      all_height,
                      30, 0.05); //(1)*/

    //Wait_UB();

    for (int i(1); i <= walk_times; i++)
    {
        Output_Coordinate(x0, y0 - step_length * ratio_right, 0,
                          x0, y0 - step_length * ratio_right, 0,
                          x0, y0, 0,
                          x0, y0, 0,
                          all_height,
                          10, 0.03); //(2)

        Output_Coordinate(x0, y0 - (step_length * ratio_right / 2), step_height,
                          x0, y0 - (step_length * ratio_right / 2), 0,
                          x0, y0 - (step_length * ratio_left / 2), step_height,
                          x0, y0 - (step_length * ratio_left / 2), 0,
                          all_height,
                          10, 0.03); //(3)

        Output_Coordinate(x0, y0, 0,
                          x0, y0, 0,
                          x0, y0 - step_length * ratio_left, 0,
                          x0, y0 - step_length * ratio_left, 0,
                          all_height,
                          10, 0.03); //(4)

        Output_Coordinate(x0, y0 - (step_length * ratio_right / 2), 0,
                          x0, y0 - (step_length * ratio_right / 2), step_height,
                          x0, y0 - (step_length * ratio_left / 2), 0,
                          x0, y0 - (step_length * ratio_left / 2), step_height,
                          all_height,
                          10, 0.03); //(10)
    }

    Output_Coordinate(x0, y0 - step_length * ratio_right, 0,
                      x0, y0 - step_length * ratio_right, 0,
                      x0, y0, 0,
                      x0, y0, 0,
                      all_height,
                      10, 0.03); //(2)

    Output_Coordinate(x0, y0, 0,
                      x0, y0, 0,
                      x0, y0, 0,
                      x0, y0, 0,
                      all_height,
                      10, 0.03); //(1)

    Output_Coordinate(x0, y0, 0,
                      x0, y0, 0,
                      x0, y0, 0,
                      x0, y0, 0,
                      0,
                      30, 0.05); //xo�Ayo�ő��𕂂��������

    //pc.printf("Finish Trot!\n");
}