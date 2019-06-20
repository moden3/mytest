#include "Rotation.h"
#include "Creeping.h"

using namespace std;

// 他のファイルで定義済みの変数と関数
extern double mainbodysize[3];

// 行列の表示
void showmatrix(double matrix[9]) {
	cout << "|" << matrix[0] << " 	" << matrix[1] << "		" << matrix[2] << "|" << endl
		<< "|" << matrix[3] << "	" << matrix[4] << "		" << matrix[5] << "|" << endl
		<< "|" << matrix[6] << "	" << matrix[7] << "		" << matrix[8] << "|" << endl << endl;
}

// 3×3行列の乗算
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


// 回転行列の設定
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
					   0            ,  0            ,  1 };

	double rottemp[9];
	calcmatrix3(rottemp, roty, rotz);
	calcmatrix3(matrix3, rotx, rottemp);	// x->y->z型オイラー角
}

// 座標の回転
void rotatecoordinate(double matrix3_[], double* xx, double* yy, double* zz) {
	double pos[3] = { *xx,*yy,*zz };
	*xx = matrix3_[0] * pos[0] + matrix3_[1] * pos[1] + matrix3_[2] * pos[2];
	*yy = matrix3_[3] * pos[0] + matrix3_[4] * pos[1] + matrix3_[5] * pos[2];
	*zz = matrix3_[6] * pos[0] + matrix3_[7] * pos[1] + matrix3_[8] * pos[2];
}

// 段差を乗り越える関数
double adjustz[4] = { 0,0,0,0 };		// z座標の修正
double adjustflag[4] = { 0,0,0,0 };		// 登りと下りの切り替えフラグ
void sandwalking() {
	double adjustKp[3] = { 0.008,0.008,0.001 };		// z座標補正のPゲイン
	double sandheight = 0.1;			// 段差の高さ[m]
	if (bno.euler.roll > 0) {
		if (bno.euler.pitch > 0) {
			if (adjustflag[3] == 0) {
				adjustz[0] -= sandheight / 4;
				adjustz[1] -= sandheight / 4;
				adjustz[2] -= sandheight / 4;
				adjustz[3] += sandheight * 3 / 4;
				adjustflag[3] = 1;
				cout << "左前up" << endl;
			}
			if (adjustflag[3] == 1) {
				adjustz[0] += sandheight / 4;
				adjustz[1] -= sandheight * 3 / 4;
				adjustz[2] += sandheight / 4;
				adjustz[3] += sandheight / 4;
				adjustflag[1] = 2;
				cout << "右後down" << endl;
			}
		}
		else {
			if (adjustflag[0] == 0) {
				adjustz[0] += sandheight * 3 / 4;
				adjustz[1] -= sandheight / 4;
				adjustz[2] -= sandheight / 4;
				adjustz[3] -= sandheight / 4;
				adjustflag[0] = 1;
				cout << "右前up" << endl;
			}
			if (adjustflag[0] == 1) {
				adjustz[0] += sandheight / 4;
				adjustz[1] += sandheight / 4;
				adjustz[2] -= sandheight * 3 / 4;
				adjustz[3] += sandheight / 4;
				adjustflag[2] = 2;
				cout << "左後down" << endl;
			}
		}
	}
	else {
		if (bno.euler.pitch > 0) {
			if (adjustflag[2] == 0) {
				adjustz[0] -= sandheight / 4;
				adjustz[1] -= sandheight / 4;
				adjustz[2] += sandheight * 3 / 4;
				adjustz[3] -= sandheight / 4;
				adjustflag[2] = 1;
				cout << "左後up" << endl;
			}
			if (adjustflag[2] == 1) {
				adjustz[0] -= sandheight * 3 / 4;
				adjustz[1] += sandheight / 4;
				adjustz[2] += sandheight / 4;
				adjustz[3] += sandheight / 4;
				adjustflag[0] = 2;
				cout << "右前down" << endl;
			}
		}
		else {
			if (adjustflag[1] == 0) {
				adjustz[0] -= sandheight / 4;
				adjustz[1] += sandheight * 3 / 4;
				adjustz[2] -= sandheight / 4;
				adjustz[3] -= sandheight / 4;
				adjustflag[1] = 1;
				cout << "左後up" << endl;
			}
			if (adjustflag[1] == 1) {
				adjustz[0] += sandheight / 4;
				adjustz[1] += sandheight / 4;
				adjustz[2] += sandheight / 4;
				adjustz[3] -= sandheight * 3 / 4;
				adjustflag[3] = 2;
				cout << "左前down" << endl;
			}
		}
	}
}

// 逆運動学,"Output.cpp"の置き換え
/*****************************ステップ出力関数(角度指定)***************************/

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
}

/**************************ステップ出力関数(足先座標指定)***********************/
void Output_Coordinate(double x_a, double y_a, double z_a,
	double x_b, double y_b, double z_b,
	double x_c, double y_c, double z_c,
	double x_d, double y_d, double z_d,
	double all_height,
	int step_times, double wait_time)
{
	double temp = mainbodysize[0] / 2000;		// 胴体の幅の半分(付け根のサーボ間の距離)

	// 脚から見た座標に設定するとき用
	x_a = x_a + temp;
	y_a = y_a + temp;
	z_a = z_a - all_height;
	x_b = x_b + temp;
	y_b = -y_b - temp;
	z_b = z_b - all_height;
	x_c = -x_c - temp;
	y_c = -y_c - temp;
	z_c = z_c - all_height;
	x_d = -x_d - temp;
	y_d = y_d + temp;
	z_d = z_d - all_height;

	// 前回の足先の位置
	static double oldlegpos[4][3] = { {temp * 3,temp * 3,0},
									{temp * 3,-temp * 3,0},
									{-temp * 3,-temp * 3,0},
									{-temp * 3,temp * 3,0} };

	for (int i(1); i <= step_times + 1; i++)
	{
		bno.get_angles();

		// 回転座標を設定
		double rotmatrix3[9];
		double angleKp[3] = { 0.3*0,0.3*0,0.3*0 };	// 姿勢補正のP制御のゲイン,(ロール角,ピッチ角,ヨー角)
		double euler3param[3];
		euler3param[0] = bno.euler.roll - roll_offset;
		euler3param[1] = bno.euler.pitch - pitch_offset;
		euler3param[2] = bno.euler.yaw - yaw_offset;
		static double eulersum[3] = { 0,0,0 };

		// zの補正値の設定
		if (abs(euler3param[0]) > 3 || abs(euler3param[1]) > 3) {
			sandwalking();
		}
		for (int i = 0; i < 3; i++) eulersum[i] += euler3param[i] * angleKp[i];
		rotationmatrix(rotmatrix3, eulersum[0], eulersum[1], -eulersum[2]);
		//cout << (int)eulersum[0] << "	" << (int)eulersum[1] << "	" << -(int)eulersum[2] << endl;
		
		// ステップごとの座標設定とzの補正
		double x__a, y__a, z__a, x__b, y__b, z__b, x__c, y__c, z__c, x__d, y__d, z__d;
		x__a = oldlegpos[0][0] + (x_a - oldlegpos[0][0])*i / (step_times + 1);
		y__a = oldlegpos[0][1] + (y_a - oldlegpos[0][1])*i / (step_times + 1);
		z__a = oldlegpos[0][2] + (z_a - oldlegpos[0][2])*i / (step_times + 1) + adjustz[0];
		x__b = oldlegpos[1][0] + (x_b - oldlegpos[1][0])*i / (step_times + 1);
		y__b = oldlegpos[1][1] + (y_b - oldlegpos[1][1])*i / (step_times + 1);
		z__b = oldlegpos[1][2] + (z_b - oldlegpos[1][2])*i / (step_times + 1) + adjustz[1];
		x__c = oldlegpos[2][0] + (x_c - oldlegpos[2][0])*i / (step_times + 1);
		y__c = oldlegpos[2][1] + (y_c - oldlegpos[2][1])*i / (step_times + 1);
		z__c = oldlegpos[2][2] + (z_c - oldlegpos[2][2])*i / (step_times + 1) + adjustz[2];
		x__d = oldlegpos[3][0] + (x_d - oldlegpos[3][0])*i / (step_times + 1);
		y__d = oldlegpos[3][1] + (y_d - oldlegpos[3][1])*i / (step_times + 1);
		z__d = oldlegpos[3][2] + (z_d - oldlegpos[3][2])*i / (step_times + 1) + adjustz[3];
		
		// 座標変換
		rotatecoordinate(rotmatrix3, &x__a, &y__a, &z__a);
		rotatecoordinate(rotmatrix3, &x__b, &y__b, &z__b);
		rotatecoordinate(rotmatrix3, &x__c, &y__c, &z__c);
		rotatecoordinate(rotmatrix3, &x__d, &y__d, &z__d);
		//showmatrix(rotmatrix3);

		rf.x = x__a - temp;
		rf.y = y__a - temp;
		rf.z = z__a + all_height;
		rb.x = x__b - temp;
		rb.y = -y__b - temp;
		rb.z = z__b + all_height;
		lb.x = -x__c - temp;
		lb.y = -y__c - temp;
		lb.z = z__c + all_height;
		lf.x = -x__d - temp;
		lf.y = y__d - temp;
		lf.z = z__d + all_height;

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

		wait(wait_time);

		/*pc.printf("Leg fr : %lf, %lf, %lf\n", fr.theta0, fr.theta1, fr.theta2);
		pc.printf("Leg br : %lf, %lf, %lf\n", br.theta0, br.theta1, br.theta2);
		pc.printf("Leg bl : %lf, %lf, %lf\n", bl.theta0, bl.theta1, bl.theta2);
		pc.printf("Leg fl : %lf, %lf, %lf\n", fl.theta0, fl.theta1, fl.theta2);
		pc.printf("*********************************************\n");*/
	}
	// 前回の足先座標の更新
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
	wait(wait_time);
}