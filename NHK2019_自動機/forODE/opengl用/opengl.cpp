#include <math.h>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string.h>
#include <vector>
#include <sstream>
#include <windows.h>
//#include <GL/glut.h>
#include "sdglib.h"

using namespace std;

//#define OFF

#ifndef OFF
// 他のファイルで定義済みの変数と関数
extern double mainbodysize[3];
extern double bodyposition[3];
extern double legposition[4][3];
extern double eulersum[3];
extern int legflag[4];
extern int redrawflag;

char str[] = " - 2D Simulation (61717325　松尾　瑛) - ";

vector<double> drawheight;
vector<double> drawlength;
int num = 0;

void displayfunc() {
	while (redrawflag == 0) {
		Sleep(50);
	}
	redrawflag = 0;

	using namespace SDGLibF;
	double drawscale = 135;
	double startpos[2] = { -100,-310 };
	double vertex[4][2] = { {bodyposition[0] + mainbodysize[0] / 2000,bodyposition[1] + mainbodysize[1] / 2000},
	{bodyposition[0] + mainbodysize[0] / 2000,bodyposition[1] - mainbodysize[1] / 2000},
	{bodyposition[0] - mainbodysize[0] / 2000,bodyposition[1] - mainbodysize[1] / 2000},
	{bodyposition[0] - mainbodysize[0] / 2000,bodyposition[1] + mainbodysize[1] / 2000} };

	Before();

	// 本体
	SetColor(1.0, 0.0, 0.0);
	DrawLine(1.0, vertex[0][0] * drawscale + startpos[0], vertex[0][1] * drawscale + startpos[1], vertex[1][0] * drawscale + startpos[0], vertex[1][1] * drawscale + startpos[1]);
	DrawLine(1.0, vertex[1][0] * drawscale + startpos[0], vertex[1][1] * drawscale + startpos[1], vertex[2][0] * drawscale + startpos[0], vertex[2][1] * drawscale + startpos[1]);
	DrawLine(1.0, vertex[2][0] * drawscale + startpos[0], vertex[2][1] * drawscale + startpos[1], vertex[3][0] * drawscale + startpos[0], vertex[3][1] * drawscale + startpos[1]);
	DrawLine(1.0, vertex[3][0] * drawscale + startpos[0], vertex[3][1] * drawscale + startpos[1], vertex[0][0] * drawscale + startpos[0], vertex[0][1] * drawscale + startpos[1]);

	// 脚
	SetColor(0.0, 0.0, 0.0);
	for (int i = 0; i < 4; i++)
		DrawLine(1.0, vertex[i][0] * drawscale + startpos[0], vertex[i][1] * drawscale + startpos[1], legposition[i][0] * drawscale + startpos[0], legposition[i][1] * drawscale + startpos[1]);

	// 足先
	SetColor(1.0, 0.0, 0.0);
	for (int i = 0; i < 4; i++) {
		if (legflag[i] == 1)
			DrawCircle(1.0, legposition[i][0] * drawscale + startpos[0], legposition[i][1] * drawscale + startpos[1], 0.05*drawscale);
	}

	// 高さ
	drawlength.push_back(bodyposition[1]);
	drawheight.push_back(bodyposition[2]);
	double heightposition = 80;
	double heightscale = 200;
	num++;
	SetColor(0.0, 0.0, 0.0);
	for (int i = 1; i < num; i++) {
		DrawLine(1.0, drawheight[i - 1] * heightscale + heightposition, drawlength[i - 1] * drawscale + startpos[1], drawheight[i] * heightscale + heightposition, drawlength[i] * drawscale + startpos[1]);
	}

	// 座標軸
	SetColor(0.5, 0.5, 0.5);
	DrawLine(0.2, startpos[0], startpos[1], startpos[0], 300);
	DrawLine(0.2, -300, startpos[1], heightposition-100, startpos[1]);
	DrawLine(0.3, heightposition, startpos[1], heightposition, 300);

	// 角度表示
	string x, y, z;
	stringstream comment, showangle[3];
	x = "x";
	y = "y";
	z = "z";
	comment << "ground angle";
	showangle[0] << "Roll  = " << int(eulersum[0]);
	showangle[1] << "Pitch= " << int(eulersum[1]);
	showangle[2] << "Yaw = " << int(eulersum[2]);
	SetColor(0.0, 0.0, 0.0);
	DrawString(heightposition-95, startpos[1]-10, x);
	DrawString(startpos[0]-5, 320, y);
	DrawString(heightposition, startpos[1] - 15, z);
	DrawString(-340, 340, comment.str());
	DrawString(-340, 310, showangle[0].str());
	DrawString(-340, 290, showangle[1].str());
	DrawString(-340, 270, showangle[2].str());

	After();

	ReDraw();
}
#endif

void threadfunc2()
{
#ifndef OFF
	while (redrawflag == 0) {
		Sleep(50);
	}
	SDGLib mygraphic(700, 740, "- 2D Graphics - (61717325　松尾　瑛)", -350.0, 350.0, -370.0, 370.0);
	mygraphic.SetCursor(GLUT_CURSOR_WAIT);
	mygraphic.Display(displayfunc);
	mygraphic.Start();
#endif
}
