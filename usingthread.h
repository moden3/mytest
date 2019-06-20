// 歩行プログラムのソースファイルでインクルードする
#pragma once

#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>

// 変数
extern double startangle[4][3];
extern double setangle[4][3];
extern double sensorangle[3];
extern double realangle, between, prev, sum, slope, imputspeed;


// クラスの置き換え
// オイラー角
class Euler {
public:
	double roll = 0.0, pitch = 0.0, yaw = 0.0;
};

// ジャイロ
#define PB_4 0
#define PA_8 0
#define OPERATION_MODE_NDOF 0
extern int mode;
extern int modenum;
class BNO055 {
public:
	Euler euler;
	BNO055(int s1, int s2) {};
	void get_angles() {
		euler.roll = sensorangle[0] * 180 / M_PI;
		euler.pitch = sensorangle[1] * 180 / M_PI;
		euler.yaw = -sensorangle[2] * 180 / M_PI;
		//std::cout << "roll = " << (int)euler.roll << ":	pitch = " << (int)euler.pitch << ":	yaw = " << (int)euler.yaw << std::endl;
	};
	void setmode(double a) {};
};


// パルス出力
#define PB_2 0
#define PA_1 4
#define PB_0 8
#define PA_11 9
#define PC_8 1
#define PA_6 5
#define PA_7 6
#define PB_6 10
#define PB_5 14
#define PB_10 3
#define PA_10 7
#define PB_7 11
class PwmOut {
public:
	int num[2] = { 0,0 };
	void pulsewidth_us(double d) { setangle[num[0]][num[1]] = (d - 500) / (2000 / 270); };
	void period_ms(double ms) {};
	PwmOut(int n) { num[0] = n % 4; num[1] = n % 3; };
	PwmOut() {};
};

// pc出力
#define USBTX 0
#define USBRX 1
class Serial {
public:
	Serial(double a, double b, double c) {};
	void printf(char* s, double a = 1000, double b = 1000, double c = 1000)
	{
		//if (a == 1000)std::cout << s << std::endl;
		//else if (b == 1000)std::cout << s << a << std::endl << std::endl;
		//else if (c == 1000)std::cout << s << a << "  " << b << std::endl << std::endl;
		//else std::cout << s << a << "  " << b << "  " << c << std::endl << std::endl;
	};
	void baud(double a) {};

};

// タイマー割り込み
extern int Gyro;
extern int print_adjust;
class Ticker {
public:
	void attach(int* a, double b) {};
};

// その他ボタン
#define USER_BUTTON 0
class DigitalIn {
public:
	DigitalIn(double a) {};
};
#define PC_0 0
class AnalogIn {
public:
	AnalogIn(double a) {};
};
#define PB_9 9
#define PB_3 3
class DigitalOut {
public:
	DigitalOut(double a) {};
};



// 関数の置き換え
void wait(double);
void Wait_UB();
double Distance();