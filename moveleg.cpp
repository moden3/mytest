#include "Rotation.h"
#include "Creeping.h"

using namespace std;

// 他のファイルで定義済みの変数と関数
extern double mainbodysize[3];

// 外部変数
double eulersum[3] = { 0,0,0 };									// 本体に対する脚の角度
double relativepos[4][3] = { {0,0,0},{0,0,0},{0,0,0},{0,0,0} };	// 脚の相対座標
double legposition[4][3] = { {0,0,0},{0,0,0},{0,0,0},{0,0,0} };	// 脚の絶対座標
double bodyposition[3] = { 0,0,0 };								// 本体の絶対座標
int legflag[4] = { 0,0,0,0 };									// 接地している脚のフラグ
int redrawflag = 0;												// opengl描画用のフラグ

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
					   0            ,  0            ,  1             };

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
double zi[4] = { 0,0,0,0 };				// 切り替え時の繰り返し回数を記録
void sandwalking(int ii) {
	double adjustKp[3] = { 0.008,0.008,0.001 };		// z座標補正のPゲイン
	double sandheight = 0.1;			// 段差の高さ[m]
	if (eulersum[0] > 0) {
		if (eulersum[1] > 0) {
			if (adjustflag[3] == 0) {
				adjustz[3] += sandheight;
				adjustflag[3] = 1;
				zi[3] = ii;
				cout << "左前up" << endl;
			}
			if (adjustflag[3] == 2 && adjustflag[1] == 1) {
				adjustz[1] -= sandheight;
				adjustflag[1] = 2;
				zi[1] = ii;
				cout << "右後down" << endl;
			}
		}
		else {
			if (adjustflag[0] == 0) {
				adjustz[0] += sandheight;
				adjustflag[0] = 1;
				zi[0] = ii;
				cout << "右前up" << endl;
			}
			if (adjustflag[0] == 2 && adjustflag[2] == 1) {
				adjustz[2] -= sandheight;
				adjustflag[2] = 2;
				zi[2] = ii;
				cout << "左後down" << endl;
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
					cout << "左後up" << endl;
				}
			}
			if (adjustflag[0] == 1) {
				adjustz[0] -= sandheight;
				adjustflag[0] = 2;
				zi[0] = ii;
				cout << "右前down" << endl;
			}
		}
		else {
			if (adjustflag[1] == 0 && adjustflag[3] >= 1) {
				if (adjustflag[0] >= 1) {
					adjustz[1] += sandheight;
					adjustflag[1] = 1;
					zi[1] = ii;
					cout << "右後up" << endl;
				}
			}
			if (adjustflag[3] == 1) {
				adjustz[3] -= sandheight;
				adjustflag[3] = 2;
				zi[3] = ii;
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

/**************************ステップ出力関数(足先座標指定)***********************/
void Output_Coordinate(double x_a, double y_a, double z_a,
	double x_b, double y_b, double z_b,
	double x_c, double y_c, double z_c,
	double x_d, double y_d, double z_d,
	double all_height,
	int step_times, double wait_time)
{
	//wait_time *= 1.5;
	double posnow[4][3], all__height;
	double temp = mainbodysize[0] / 2000;			// 胴体の幅の半分(付け根のサーボ間の距離)
	static double rotmatrix3[9];
	//double sandflag = 0;
	int deltastep;									// 一定時間置きを計測するための変数
	if (wait_time == 0)deltastep = 2;
	else deltastep = int(0.5 / (wait_time));
	if (deltastep == 0)deltastep = 2;
	//for (int j = 0; j < 4; j++)zi[j] = 0;

	// 脚から見た座標に設定するとき用
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

	// 前回の足先の位置
	static double oldlegpos[4][3] = { {temp * 3,temp * 3,0},
									{temp * 3,-temp * 3,0},
									{-temp * 3,-temp * 3,0},
									{-temp * 3,temp * 3,0} };

	// 設置している脚を確認
	int num = 0;					// 脚の数
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

		// 回転座標を設定
		double angleKp[3] = { 0.5,0.5,0.3 };	// 姿勢補正のP制御のゲイン,(ロール角,ピッチ角,ヨー角)
		double euler3param[3];
		euler3param[0] = bno.euler.roll - roll_offset;
		euler3param[1] = bno.euler.pitch - pitch_offset;
		euler3param[2] = bno.euler.yaw - yaw_offset;

		// zの補正値の設定
		/*if (adjustflag[1] < 2 && adjustflag[2] < 2)
			if (sandflag == 0) {
				if (abs(eulersum[0]) > 3 || abs(eulersum[1]) > 3) {
					//if (abs(euler3param[0]) > 2.5 || abs(euler3param[1]) > 2.5) {
					sandwalking(i - 1);
					sandflag = 1;
				}
			}
		double adjustrate[4] = {1,1,1,1};	// 補正の比率
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
		
		// ステップごとの座標設定とzの補正
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

		// 座標変換
		rotatecoordinate(rotmatrix3, &posnow[0][0], &posnow[0][1], &posnow[0][2]);
		rotatecoordinate(rotmatrix3, &posnow[1][0], &posnow[1][1], &posnow[1][2]);
		rotatecoordinate(rotmatrix3, &posnow[2][0], &posnow[2][1], &posnow[2][2]);
		rotatecoordinate(rotmatrix3, &posnow[3][0], &posnow[3][1], &posnow[3][2]);
		//showmatrix(rotmatrix3);

		// 本体の座標移動を記録
		double delta[4][3];
		static double oldbodyheight = 0.0;	// 前回の本体高さ
		if ((step_times + 1 - i) % deltastep == 0) {
			for (int j = 0; j < 4; j++) {
				for (int k = 0; k < 3; k++) {
					if (legflag[j] == 1)
						bodyposition[k] -= (posnow[j][k] - relativepos[j][k]) / num;
					relativepos[j][k] = posnow[j][k];
					// 脚の絶対座標を記録
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
	all_height_old = all_height;

	// 脚の絶対座標を記録
	/*for (int j = 0; j < 4; j++) {
		for (int k = 0; k < 3; k++) {
			legposition[j][k] = bodyposition[k] + posnow[j][k];
		}
		legposition[j][2] += all_height;
	}*/
	wait(wait_time);
	//if (redrawflag == 0)redrawflag = 1;
}