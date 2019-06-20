#include "Rotation.h"
#include "Creeping.h"

using namespace std;

// ���̃t�@�C���Œ�`�ς݂̕ϐ��Ɗ֐�
extern double mainbodysize[3];

// �O���ϐ�
double eulersum[3] = { 0,0,0 };									// �{�̂ɑ΂���r�̊p�x
double relativepos[4][3] = { {0,0,0},{0,0,0},{0,0,0},{0,0,0} };	// �r�̑��΍��W
double legposition[4][3] = { {0,0,0},{0,0,0},{0,0,0},{0,0,0} };	// �r�̐�΍��W
double bodyposition[3] = { 0,0,0 };								// �{�̂̐�΍��W
int legflag[4] = { 0,0,0,0 };									// �ڒn���Ă���r�̃t���O
int redrawflag = 0;												// opengl�`��p�̃t���O

// �s��̕\��
void showmatrix(double matrix[9]) {
	cout << "|" << matrix[0] << " 	" << matrix[1] << "		" << matrix[2] << "|" << endl
		<< "|" << matrix[3] << "	" << matrix[4] << "		" << matrix[5] << "|" << endl
		<< "|" << matrix[6] << "	" << matrix[7] << "		" << matrix[8] << "|" << endl << endl;
}

// 3�~3�s��̏�Z
void calcmatrix3(double solution[9], double a[9], double b[9]) {
	for (int i = 0; i < 9; i++) {
		solution[i] = 0;
	}
	for (int i = 0; i < 9; i++) {
		for (int j = 0; j < 3; j++) {
			solution[i] += a[(i / 3) * 3 + j] * b[i % 3 + 3 * j];
		}
	}
}

// ��]�s��̐ݒ�
void rotationmatrix(double matrix3[9], double roll_, double pitch_, double yaw_) {
	double theta[3] = { roll_*M_PI / 180.0,pitch_*M_PI / 180.0,yaw_*M_PI / 180.0 };
	double rotx[9] = { 1            ,  0            ,  0            ,
					   0            ,  cos(theta[0]), -sin(theta[0]),
					   0            ,  sin(theta[0]),  cos(theta[0]) };
	double roty[9] = { cos(theta[1]),  0            ,  sin(theta[1]),
					   0            ,  1            ,  0            ,
					  -sin(theta[1]),  0            ,  cos(theta[1]) };
	double rotz[9] = { cos(theta[2]), -sin(theta[2]),  0            ,
					   sin(theta[2]),  cos(theta[2]),  0            ,
					   0            ,  0            ,  1             };

	double rottemp[9];
	calcmatrix3(rottemp, roty, rotz);
	calcmatrix3(matrix3, rotx, rottemp);	// x->y->z�^�I�C���[�p
}

// ���W�̉�]
void rotatecoordinate(double matrix3_[], double* xx, double* yy, double* zz) {
	double pos[3] = { *xx,*yy,*zz };
	*xx = matrix3_[0] * pos[0] + matrix3_[1] * pos[1] + matrix3_[2] * pos[2];
	*yy = matrix3_[3] * pos[0] + matrix3_[4] * pos[1] + matrix3_[5] * pos[2];
	*zz = matrix3_[6] * pos[0] + matrix3_[7] * pos[1] + matrix3_[8] * pos[2];
}

// �i�������z����֐�
double adjustz[4] = { 0,0,0,0 };		// z���W�̏C��
double adjustflag[4] = { 0,0,0,0 };		// �o��Ɖ���̐؂�ւ��t���O
double zi[4] = { 0,0,0,0 };				// �؂�ւ����̌J��Ԃ��񐔂��L�^
void sandwalking(int ii) {
	double adjustKp[3] = { 0.008,0.008,0.001 };		// z���W�␳��P�Q�C��
	double sandheight = 0.1;			// �i���̍���[m]
	if (eulersum[0] > 0) {
		if (eulersum[1] > 0) {
			if (adjustflag[3] == 0) {
				adjustz[3] += sandheight;
				adjustflag[3] = 1;
				zi[3] = ii;
				cout << "���Oup" << endl;
			}
			if (adjustflag[3] == 2 && adjustflag[1] == 1) {
				adjustz[1] -= sandheight;
				adjustflag[1] = 2;
				zi[1] = ii;
				cout << "�E��down" << endl;
			}
		}
		else {
			if (adjustflag[0] == 0) {
				adjustz[0] += sandheight;
				adjustflag[0] = 1;
				zi[0] = ii;
				cout << "�E�Oup" << endl;
			}
			if (adjustflag[0] == 2 && adjustflag[2] == 1) {
				adjustz[2] -= sandheight;
				adjustflag[2] = 2;
				zi[2] = ii;
				cout << "����down" << endl;
			}
		}
	}
	else {
		if (eulersum[1] > 0) {
			if (adjustflag[2] == 0 && adjustflag[0] >= 1) {
				if (adjustflag[3] >= 1) {
					adjustz[2] += sandheight;
					adjustflag[2] = 1;
					zi[2] = ii;
					cout << "����up" << endl;
				}
			}
			if (adjustflag[0] == 1) {
				adjustz[0] -= sandheight;
				adjustflag[0] = 2;
				zi[0] = ii;
				cout << "�E�Odown" << endl;
			}
		}
		else {
			if (adjustflag[1] == 0 && adjustflag[3] >= 1) {
				if (adjustflag[0] >= 1) {
					adjustz[1] += sandheight;
					adjustflag[1] = 1;
					zi[1] = ii;
					cout << "�E��up" << endl;
				}
			}
			if (adjustflag[3] == 1) {
				adjustz[3] -= sandheight;
				adjustflag[3] = 2;
				zi[3] = ii;
				cout << "���Odown" << endl;
			}
		}
	}
}

// �t�^���w,"Output.cpp"�̒u������
/*****************************�X�e�b�v�o�͊֐�(�p�x�w��)***************************/

void Output_Angle(double angle_a_s0, double angle_a_s1, double angle_a_s2,
	double angle_b_s0, double angle_b_s1, double angle_b_s2,
	double angle_c_s0, double angle_c_s1, double angle_c_s2,
	double angle_d_s0, double angle_d_s1, double angle_d_s2,
	int step_times, double wait_time)
	//step_times�̓X�e�b�v��(20�ȏ�)
	//wait_time�͊e�X�e�b�v�̑҂�����[s]
	//step_times��0�ɂ���ƃX�e�b�v�Ȃ��ő�����angle_ref���o��

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
	
	pc.printf("step_times==0\n");
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
}

/**************************�X�e�b�v�o�͊֐�(������W�w��)***********************/
void Output_Coordinate(double x_a, double y_a, double z_a,
	double x_b, double y_b, double z_b,
	double x_c, double y_c, double z_c,
	double x_d, double y_d, double z_d,
	double all_height,
	int step_times, double wait_time)
{
	//wait_time *= 1.5;
	double posnow[4][3], all__height;
	double temp = mainbodysize[0] / 2000;			// ���̂̕��̔���(�t�����̃T�[�{�Ԃ̋���)
	static double rotmatrix3[9];
	//double sandflag = 0;
	int deltastep;									// ��莞�Ԓu�����v�����邽�߂̕ϐ�
	if (wait_time == 0)deltastep = 2;
	else deltastep = int(0.5 / (wait_time));
	if (deltastep == 0)deltastep = 2;
	//for (int j = 0; j < 4; j++)zi[j] = 0;

	// �r���猩�����W�ɐݒ肷��Ƃ��p
	x_a = x_a + temp;
	y_a = y_a + temp;
	z_a = z_a;
	x_b = x_b + temp;
	y_b = -y_b - temp;
	z_b = z_b;
	x_c = -x_c - temp;
	y_c = -y_c - temp;
	z_c = z_c;
	x_d = -x_d - temp;
	y_d = y_d + temp;
	z_d = z_d;

	// �O��̑���̈ʒu
	static double oldlegpos[4][3] = { {temp * 3,temp * 3,0},
									{temp * 3,-temp * 3,0},
									{-temp * 3,-temp * 3,0},
									{-temp * 3,temp * 3,0} };

	// �ݒu���Ă���r���m�F
	int num = 0;					// �r�̐�
	for (int j = 0; j < 4; j++)legflag[j] = 0;
	if (oldlegpos[0][2] == 0) {
		if (z_a == 0) {
			legflag[0] = 1;
			num++;
		}
	}
	if (oldlegpos[1][2] == 0) {
		if (z_b ==0) {
			legflag[1] = 1;
			num++;
		}
	}
	if (oldlegpos[2][2] == 0) {
		if (z_c == 0) {
			legflag[2] = 1;
			num++;
		}
	}
	if (oldlegpos[3][2] == 0) {
		if (z_d == 0) {
			legflag[3] = 1;
			num++;
		}
	}

	for (int i(1); i <= step_times + 1; i++)
	{
		bno.get_angles();

		// ��]���W��ݒ�
		double angleKp[3] = { 0.5,0.5,0.3 };	// �p���␳��P����̃Q�C��,(���[���p,�s�b�`�p,���[�p)
		double euler3param[3];
		euler3param[0] = bno.euler.roll - roll_offset;
		euler3param[1] = bno.euler.pitch - pitch_offset;
		euler3param[2] = bno.euler.yaw - yaw_offset;

		// z�̕␳�l�̐ݒ�
		/*if (adjustflag[1] < 2 && adjustflag[2] < 2)
			if (sandflag == 0) {
				if (abs(eulersum[0]) > 3 || abs(eulersum[1]) > 3) {
					//if (abs(euler3param[0]) > 2.5 || abs(euler3param[1]) > 2.5) {
					sandwalking(i - 1);
					sandflag = 1;
				}
			}
		double adjustrate[4] = {1,1,1,1};	// �␳�̔䗦
		if (sandflag == 1) {
			for (int j = 0; j < 4; j++)
				if (zi[j] != 0)
					adjustrate[j] = (i - zi[0]) / (step_times + 1 - zi[0]);
		}*/

		//if (i % 2 == 1) {
			for (int j = 0; j < 3; j++) {
				eulersum[j] += euler3param[j]* angleKp[j];
			}
			rotationmatrix(rotmatrix3, eulersum[0], eulersum[1], -eulersum[2]);
		//}
		//cout << (int)eulersum[0] << "	" << (int)eulersum[1] << "	" << -(int)eulersum[2] << endl;
		
		// �X�e�b�v���Ƃ̍��W�ݒ��z�̕␳
		posnow[0][0] = oldlegpos[0][0] + (x_a - oldlegpos[0][0])*i / (step_times + 1);
		posnow[0][1] = oldlegpos[0][1] + (y_a - oldlegpos[0][1])*i / (step_times + 1);
		posnow[0][2] = oldlegpos[0][2] + (z_a - oldlegpos[0][2])*i / (step_times + 1);
		posnow[1][0] = oldlegpos[1][0] + (x_b - oldlegpos[1][0])*i / (step_times + 1);
		posnow[1][1] = oldlegpos[1][1] + (y_b - oldlegpos[1][1])*i / (step_times + 1);
		posnow[1][2] = oldlegpos[1][2] + (z_b - oldlegpos[1][2])*i / (step_times + 1);
		posnow[2][0] = oldlegpos[2][0] + (x_c - oldlegpos[2][0])*i / (step_times + 1);
		posnow[2][1] = oldlegpos[2][1] + (y_c - oldlegpos[2][1])*i / (step_times + 1);
		posnow[2][2] = oldlegpos[2][2] + (z_c - oldlegpos[2][2])*i / (step_times + 1);
		posnow[3][0] = oldlegpos[3][0] + (x_d - oldlegpos[3][0])*i / (step_times + 1);
		posnow[3][1] = oldlegpos[3][1] + (y_d - oldlegpos[3][1])*i / (step_times + 1);
		posnow[3][2] = oldlegpos[3][2] + (z_d - oldlegpos[3][2])*i / (step_times + 1);
		all__height = all_height_old + (all_height - all_height_old)*i / (step_times + 1);

		// ���W�ϊ�
		rotatecoordinate(rotmatrix3, &posnow[0][0], &posnow[0][1], &posnow[0][2]);
		rotatecoordinate(rotmatrix3, &posnow[1][0], &posnow[1][1], &posnow[1][2]);
		rotatecoordinate(rotmatrix3, &posnow[2][0], &posnow[2][1], &posnow[2][2]);
		rotatecoordinate(rotmatrix3, &posnow[3][0], &posnow[3][1], &posnow[3][2]);
		//showmatrix(rotmatrix3);

		// �{�̂̍��W�ړ����L�^
		double delta[4][3];
		static double oldbodyheight = 0.0;	// �O��̖{�̍���
		if ((step_times + 1 - i) % deltastep == 0) {
			for (int j = 0; j < 4; j++) {
				for (int k = 0; k < 3; k++) {
					if (legflag[j] == 1)
						bodyposition[k] -= (posnow[j][k] - relativepos[j][k]) / num;
					relativepos[j][k] = posnow[j][k];
					// �r�̐�΍��W���L�^
					legposition[j][k] = bodyposition[k] + posnow[j][k];
					legposition[j][2] += all_height;
				}
			}
			bodyposition[2] += (all__height - oldbodyheight);
			oldbodyheight = all__height;
			//cout << int(bodyposition[0] * 100) << "	" << int(bodyposition[1] * 100) << "	" << int(bodyposition[2] * 100) << endl;
			
			if (redrawflag == 0)redrawflag = 1;
		}

		rf.x = posnow[0][0] - temp;
		rf.y = posnow[0][1] - temp;
		rf.z = posnow[0][2] + adjustz[0];
		rb.x = posnow[1][0] - temp;
		rb.y = -posnow[1][1] - temp;
		rb.z = posnow[1][2] + adjustz[1];
		lb.x = -posnow[2][0] - temp;
		lb.y = -posnow[2][1] - temp;
		lb.z = posnow[2][2] + adjustz[2];
		lf.x = -posnow[3][0] - temp;
		lf.y = posnow[3][1] - temp;
		lf.z = posnow[3][2] + adjustz[3];

		rf.calc(rf.x, rf.y, rf.z, all__height, rf.theta0, rf.theta1, rf.theta2);
		rb.calc(rb.x, rb.y, rb.z, all__height, rb.theta0, rb.theta1, rb.theta2);
		lb.calc(lb.x, lb.y, lb.z, all__height, lb.theta0, lb.theta1, lb.theta2);
		lf.calc(lf.x, lf.y, lf.z, all__height, lf.theta0, lf.theta1, lf.theta2);

		Output_Angle(rf.theta0, rf.theta1, rf.theta2,
			rb.theta0, rb.theta1, rb.theta2,
			lb.theta0, lb.theta1, lb.theta2,
			lf.theta0, lf.theta1, lf.theta2,
			step_times, wait_time);

		wait(wait_time);
	}
	// �O��̑�����W�̍X�V
	oldlegpos[0][0] = x_a;
	oldlegpos[0][1] = y_a;
	oldlegpos[0][2] = z_a;
	oldlegpos[1][0] = x_b;
	oldlegpos[1][1] = y_b;
	oldlegpos[1][2] = z_b;
	oldlegpos[2][0] = x_c;
	oldlegpos[2][1] = y_c;
	oldlegpos[2][2] = z_c;
	oldlegpos[3][0] = x_d;
	oldlegpos[3][1] = y_d;
	oldlegpos[3][2] = z_d;
	all_height_old = all_height;

	// �r�̐�΍��W���L�^
	/*for (int j = 0; j < 4; j++) {
		for (int k = 0; k < 3; k++) {
			legposition[j][k] = bodyposition[k] + posnow[j][k];
		}
		legposition[j][2] += all_height;
	}*/
	wait(wait_time);
	//if (redrawflag == 0)redrawflag = 1;
}