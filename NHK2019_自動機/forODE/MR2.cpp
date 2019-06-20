// 自動機　サーボ式
// version 2

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"
#include <thread>		// 並列処理実行用
#include <windows.h>	// 一時停止"Sleep"実行用の関数
#include <iostream>

#define _USE_MATH_DEFINES
#include <math.h>

//#define OPENGL	// openglでシミュレーション
//#define THISONLY	// このファイルのみを実行するとき

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

//#include "icosahedron_geom.h"

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#define dsDrawConvex dsDrawConvexD
#endif

#define DENSITY 5.0		// 密度
#define MAX_CONTACTS 3		// 接触点の最大値
#define GRAVITY         REAL(9.8)
#define SCALE1 100.0

///////////////////////////////////////////////////////////////////////////////////
/**********************************設定パラメータ*********************************/
// 全質量4.8kg
// 寸法
double servosize[3] = { 65.0,30.0,48.0 };		// サーボモータの大きさ[mm]
double servooffset = 50 - 65.0 / 2.0;			// サーボモータの中心と回転中心のずれ[mm]
double servodistance = 70;						// 付け根の2つのサーボの回転中心間の距離[mm]
double servomass = (163.0 + 20) / 1000;			// サーボモータ(とサーボホーン)の質量[kg]
double servocolor[3] = { 0.1,0.3,0.3 };			// サーボモータの色

double leglength[3] = { 10,143.01,297.95 };		// 脚の長さ[mm]、{角柱の一辺の長さ,脚の上下の関節から膝の関節までの長さ、膝の関節から足先までの長さ}
//double leglength[3] = { 10,216,234.6 };
double legmass = 30.0 / 1000;					// 脚のアルミ角パイプの質量[kg]
double legcolor[3] = { 0.8,0.8,0.8 };			// 脚の色

double toesize[2] = { 15,40 };					// 足先の滑り止めの円柱の大きさ[mm]、{半径,長さ}
double toemass = 30.0 / 1000;					// 足先の質量[kg]
double toecolor[3] = { 1.0,0.4,0.4 };			// 足先の色

double mainbodysize[3] = { 190.1,190.1,56.0 };	// 本体の大きさ[mm]
double mainbodymass = 800.0 / 1000;				// 本体の質量[kg],(残り質量2124g)
double mainbodycolor[3] = { 1.0,0.4,0.4 };		// 本体の色

double subbodysize[3] = { 50,250,70 };			// 本体についてる重りのサイズ[mm]
double subbodymass = 1324.0 / 1000;				// 本体についてる重りの質量[kg]
double subbodypos[3] = { 0,0,80 };				// 本体についてる重りの位置[mm]
double subbodycolor[3] = { 0.5,0.5,0.5 };		// 重りの色

double startz = 10;								// 初期高さ[mm]

double slopepos[3] = { 0,1700,10000 };				// スロープの位置
double slopecolor[3] = { 1.0,0.85,0.7 };		// スロープの色

double sandpos[3] = { 0,850,10000 };				// サンドの位置
double sandcolor[3] = { 1.0,0.85,0.7 };			// サンドの色

// サーボ角度の設定
double startangle[4][3] = { {45,0.0,0.0},
							{-45,0.0,0.0},
							{45,0.0,0.0},
							{-45,0.0,0.0},};// サーボの初期角度(オフセット)
int rotdirection[4][3] = { {1,1,1},
						   {1,1,1},
						   {1,1,1},
						   {1,1,1}, };		// サーボの回転方向(0か1)
double servotorque = 60.0 *GRAVITY* 0.01;	// サーボのトルク[N*m]
double Kp = 50.0, Ki = 0.05, Kd = 0.02;		// サーボのPID制御のゲイン

// その他パラメータ
double dt = 0.008;							// 刻み時間[s]
/*********************************************************************************/
///////////////////////////////////////////////////////////////////////////////////


// 直方体型の物体
class SetBox {
public:
	dReal size[3] = { 0,0,0 }, m = 0, pos[3] = { 0,0,0 };
	dMatrix3 R;			// 回転行列を格納する構造体
	dMass mass;			// 質量パラメータを格納する変数
	dBodyID body;
	dGeomID geom;
	void setsize(double, double, double, double);
	void setposition(double, double, double);
	void setrotation(double, double, double, double);
	SetBox() { dRSetIdentity(R); };
};

void SetBox::setsize(double xl_, double yl_, double zl_, double m_ = 0.05) {
	size[0] = xl_ / SCALE1;
	size[1] = yl_ / SCALE1;
	size[2] = zl_ / SCALE1;
	m = m_;
	dMassSetBox(&mass, DENSITY, size[0], size[1], size[2]);
	dMassAdjust(&mass, m);
	dMassRotate(&mass, R);
}

void SetBox::setposition(double x_, double y_, double z_) {
	pos[0] = x_ / SCALE1;
	pos[1] = y_ / SCALE1;
	pos[2] = z_ / SCALE1;
}

void SetBox::setrotation(double ax, double ay, double az, double angle) {
	dRFromAxisAndAngle(R, ax, ay, az, angle);	// 回転行列を取得
}

// 円柱型の物体
class SetCylinder {
public:
	dReal size[2] = { 0,0 }, m = 0, pos[3] = { 0,0,0 };
	dMatrix3 R;			// 回転行列を格納する構造体
	dMass mass;			// 質量パラメータを格納する変数
	dBodyID body;
	dGeomID geom;
	void setsize(double, double, double);
	void setposition(double, double, double);
	void setrotation(double, double, double, double);
};

void SetCylinder::setsize(double r_, double l_, double m_ = 0.05) {
	size[0] = r_ / SCALE1;
	size[1] = l_ / SCALE1;
	m = m_;
	dMassSetCylinder(&mass, DENSITY, 3, size[0], size[1]);
	dMassAdjust(&mass, m);
	dMassRotate(&mass, R);
}

void SetCylinder::setposition(double x_, double y_, double z_) {
	pos[0] = x_ / SCALE1;
	pos[1] = y_ / SCALE1;
	pos[2] = z_ / SCALE1;
}

void SetCylinder::setrotation(double ax, double ay, double az, double angle) {
	dRFromAxisAndAngle(R, ax, ay, az, angle);	// 回転行列を取得
}

static dWorldID world;

// 直方体型のボディとジオメトリを生成
void createbox(SetBox* box, dSpaceID& spacename) {
	box->body = dBodyCreate(world);
	dBodySetPosition(box->body, box->pos[0], box->pos[1], box->pos[2]);
	dBodySetRotation(box->body, box->R);
	dBodySetMass(box->body, &(box->mass));
	box->geom = dCreateBox(spacename, box->size[0], box->size[1], box->size[2]);
	dGeomSetBody(box->geom, box->body);
}

// 円柱型のボディとジオメトリを生成
void createcylinder(SetCylinder* cylinder, dSpaceID& spacename) {
	cylinder->body = dBodyCreate(world);
	dBodySetPosition(cylinder->body, cylinder->pos[0], cylinder->pos[1], cylinder->pos[2]);
	dBodySetRotation(cylinder->body, cylinder->R);
	dBodySetMass(cylinder->body, &(cylinder->mass));
	cylinder->geom = dCreateCylinder(spacename, cylinder->size[0], cylinder->size[1]);
	dGeomSetBody(cylinder->geom, cylinder->body);
}

class Leg {
public:
	double jointpos[3][2], jointvec[3];
	SetBox legobj[3], servo[3];
	SetCylinder toeobj;
	dJointID fixedjoint[4], hingejoint[3];
	void initialposition(double);
};

void Leg::initialposition(double angle) {
	double temp[7];
	temp[0] = sqrt(mainbodysize[0] * mainbodysize[0] + mainbodysize[1] * mainbodysize[1]) / 2 - servooffset;
	temp[1] = temp[0] + servodistance + servooffset * 2;
	temp[2] = temp[1] - servooffset + leglength[1] / 2;
	temp[3] = temp[2] + leglength[1] / 2 - servooffset;
	temp[4] = temp[3] + servooffset + leglength[2] / 2;
	temp[5] = temp[4] + leglength[2] / 2;
	temp[6] = (temp[0] + temp[1]) / 2;

	// オブジェクトの座標を回転して設定
	servo[0].setposition(temp[0] * cos(angle), temp[0] * sin(angle), mainbodysize[2] / 2 + startz);
	servo[1].setposition(temp[1] * cos(angle), temp[1] * sin(angle), mainbodysize[2] / 2 + startz);
	servo[2].setposition(temp[3] * cos(angle), temp[3] * sin(angle), mainbodysize[2] / 2 + startz);
	legobj[0].setposition(temp[6] * cos(angle), temp[6] * sin(angle), mainbodysize[2] / 2 + startz);
	legobj[1].setposition(temp[2] * cos(angle), temp[2] * sin(angle), mainbodysize[2] / 2 + startz);
	legobj[2].setposition(temp[4] * cos(angle), temp[4] * sin(angle), mainbodysize[2] / 2 + startz);
	toeobj.setposition(temp[5] * cos(angle), temp[5] * sin(angle), mainbodysize[2] / 2 + startz);

	// オブジェクトの回転行列を設定
	for (int i = 0; i < 3; i++) {
		servo[i].setrotation(0, 0, 1, angle);
		legobj[i].setrotation(0, 0, 1, angle);
	}
	toeobj.setrotation(cos(angle), sin(angle), 0, M_PI / 2);

	// オブジェクトの寸法を設定
	servo[0].setsize(servosize[0], servosize[1], servosize[2], servomass);
	servo[1].setsize(servosize[0], servosize[2], servosize[1], servomass);
	servo[2].setsize(servosize[0], servosize[2], servosize[1], servomass);
	legobj[0].setsize(servodistance, leglength[0], leglength[0], legmass);
	legobj[1].setsize(leglength[1], leglength[0], leglength[0], legmass);
	legobj[2].setsize(leglength[2], leglength[0], leglength[0], legmass);
	toeobj.setsize(toesize[0], toesize[1], toemass);

	// 関節の位置と向きを設定
	jointpos[0][0] = (temp[0] + servooffset)*cos(angle) / SCALE1;	jointpos[0][1] = (temp[0] + servooffset)*sin(angle) / SCALE1;
	jointpos[1][0] = (temp[1] - servooffset)*cos(angle) / SCALE1;	jointpos[1][1] = (temp[1] - servooffset)*sin(angle) / SCALE1;
	jointpos[2][0] = (temp[3] + servooffset)*cos(angle) / SCALE1;	jointpos[2][1] = (temp[3] + servooffset)*sin(angle) / SCALE1;
	jointvec[0] = -sin(angle); jointvec[1] = cos(angle); jointvec[2] = 0;
}

class MR2 {
public:
	SetBox obj[2];
	dJointID joint;
	Leg leg[4];
	void setparam();
};

static dSpaceID space;
static dSpaceID mr2space;
static dJointGroupID contactgroup;
static dGeomID ground;
static MR2 mr2;

static double Time = 0.0;					// 時間
static double sleeptime = 0.0;				// 停止時間を格納
static int timeflag = 0;					// 時間経過を停止するフラグ
static int threadflag = 0;					// 並列処理実行開始のフラグ
double setangle[4][3];						// サーボの角度を格納
double sensorangle[3] = { 0.0,0.0,0.0 };	// 本体の角度を格納、{roll,pitch,yaw}
double sensoroffset[3] = { 0.0,0.0,0.0 };	// センサー角度操作用の変数、{roll,pitch,yaw}

// 他のファイルで定義済みの変数と関数
int Gyro = 0;
int print_adjust = 1;

#ifndef THISONLY
extern int mode;
void main2();
void threadfunc2();
#endif


// 角度補正確認用の土台オブジェクト
static SetBox tableobj[3];
static dJointID tablejoint[3];
static double tableangle[3] = { 0.0,0.0,0.0 };
void createtable() {
	// ボディとジオメトリの生成
	for (int i = 0; i < 3; i++) {
		tableobj[i].setposition(0, 0, startz - 150);
		tableobj[i].setrotation(0, 0, 1, 0);
	}
	tableobj[0].setsize(10, 10, 10, 0.01);
	tableobj[1].setsize(10, 10, 10, 0.01);
	tableobj[2].setsize(1300, 1300, 100, 1.0);
	for (int i = 0; i < 3; i++) createbox(&(tableobj[i]), space);
	
	// 関節の生成
	for (int i = 0; i < 3; i++)	tablejoint[i] = dJointCreateHinge(world, 0);
	dJointAttach(tablejoint[0], tableobj[0].body, 0);
	dJointAttach(tablejoint[1], tableobj[0].body, tableobj[1].body);
	dJointAttach(tablejoint[2], tableobj[1].body, tableobj[2].body);
	for (int i = 0; i < 3; i++) dJointSetHingeAnchor(tablejoint[i], 0, 0, tableobj[i].pos[2]);
	dJointSetHingeAxis(tablejoint[0], 1, 0, 0);
	dJointSetHingeAxis(tablejoint[1], 0, 1, 0);
	dJointSetHingeAxis(tablejoint[2], 0, 0, 1);
}
void movetable() {
	double anglespeed[3] = { 0,0,0 };
	for (int i = 0; i < 3; i++) {
		anglespeed[i] = tableangle[i] * M_PI / 180 - dJointGetHingeAngle(tablejoint[i]);
		dJointSetHingeParam(tablejoint[i], dParamVel, anglespeed[i]*5);
		dJointSetHingeParam(tablejoint[i], dParamFMax, 100000);
		dJointSetHingeParam(tablejoint[i], dParamFudgeFactor, 0.7);	// 関節の挙動がジャンプしていたら小さくする
	}
	// 描画
	dsSetTexture(DS_WOOD);
	dsSetColor(0.8, 0.8, 0.8);
	dsDrawBoxD(dGeomGetPosition(tableobj[2].geom), dGeomGetRotation(tableobj[2].geom), tableobj[2].size);
}

// 段差用のオブジェクト
static SetBox sandobj;
void createsand() {
	double sandlength[3] = { 1710,300,100 };
	double direction = M_PI * 0 / 4;
	sandobj.setposition(sandpos[0] * (-sin(direction)), sandpos[1] * cos(direction), sandpos[2] + sandlength[2] / 2);
	sandobj.setrotation(0, 0, 1, direction);
	sandobj.setsize(sandlength[0], sandlength[1], sandlength[2]);
	sandobj.geom = dCreateBox(space, sandobj.size[0], sandobj.size[1], sandobj.size[2]);
	dGeomSetPosition(sandobj.geom, sandobj.pos[0], sandobj.pos[1], sandobj.pos[2]);
	dGeomSetRotation(sandobj.geom, sandobj.R);
}

// 坂道用のオブジェクト
static SetBox slopeobj[2];
void createslope() {
	double slopelength[3] = { 1455,1500,1125 };	// スロープの幅と長さ,平坦部分の長さ
	double angle = atan((double)4 / (double)15);
	double direction = M_PI * 0 / 4;
	// 傾斜部分
	slopeobj[0].setposition(slopepos[0] + slopelength[1] / (2 * cos(angle)*cos(angle)) * (-sin(direction)), slopepos[1] + slopelength[1] / (2 * cos(angle)*cos(angle)) * cos(direction), slopepos[2]);
	dRFromEulerAngles(slopeobj[0].R, -M_PI / 2, -M_PI / 2 - direction, M_PI / 2 - angle);		// オイラー格を設定,(xyz系)
	slopeobj[0].setsize(slopelength[1] * tan(angle) / cos(angle), slopelength[1] / cos(angle), slopelength[0]);
	slopeobj[0].geom = dCreateBox(space, slopeobj[0].size[0], slopeobj[0].size[1], slopeobj[0].size[2]);
	dGeomSetPosition(slopeobj[0].geom, slopeobj[0].pos[0], slopeobj[0].pos[1], slopeobj[0].pos[2]);
	dGeomSetRotation(slopeobj[0].geom, slopeobj[0].R);
	// 平坦部分
	slopeobj[1].setposition(slopepos[0] + (slopelength[1] + slopelength[2] / 2)  * (-sin(direction)), slopepos[1] + (slopelength[1] + slopelength[2] / 2) * cos(direction), slopepos[2] + slopelength[1] * tan(angle) / 2);
	slopeobj[1].setrotation(0, 0, 1, direction);
	slopeobj[1].setsize(slopelength[0], slopelength[2], slopelength[1] * tan(angle));
	slopeobj[1].geom = dCreateBox(space, slopeobj[1].size[0], slopeobj[1].size[1], slopeobj[1].size[2]);
	dGeomSetPosition(slopeobj[1].geom, slopeobj[1].pos[0], slopeobj[1].pos[1], slopeobj[1].pos[2]);
	dGeomSetRotation(slopeobj[1].geom, slopeobj[1].R);
}


void MR2::setparam() {
	// 本体の座標を設定
	obj[0].setposition(0, 0, mainbodysize[2] / 2 + startz);
	obj[1].setposition(subbodypos[0], subbodypos[1], subbodypos[2] + startz);
	
	// 本体の寸法を設定
	obj[0].setsize(mainbodysize[0] - servooffset * sqrt(2), mainbodysize[1] - servooffset * sqrt(2), mainbodysize[2], mainbodymass);
	obj[1].setsize(subbodysize[0], subbodysize[1], subbodysize[2], subbodymass);

	// 脚のパラメータを設定
	leg[0].initialposition(M_PI / 4);
	leg[1].initialposition(-M_PI / 4);
	leg[2].initialposition(-M_PI * 3 / 4);
	leg[3].initialposition(M_PI * 3 / 4);
}


// 衝突判定を行う関数
static void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
	// 処理を行わない物体を除外
	/*int g1 = (o1 == slopeobj[0].geom || o1 == slopeobj[1].geom);
	int g2 = (o2 == slopeobj[0].geom || o2 == slopeobj[1].geom);
	if (!(g1 ^ g2)) return;*/
	int g1 = (o1 == sandobj.geom || o1 == slopeobj[0].geom || o1 == slopeobj[1].geom || o1 == ground || o1 == tableobj[0].geom || o1 == tableobj[1].geom || o1 == tableobj[2].geom);
	int g2 = (o2 == sandobj.geom || o2 == slopeobj[0].geom || o2 == slopeobj[1].geom || o2 == ground || o2 == tableobj[0].geom || o2 == tableobj[1].geom || o2 == tableobj[2].geom);
	if (g1*g2 != 0)return;

	const int N = 40;			// ジオメトリが交差した際の最大接触点数を指定
	dContact contact[N];		// 接触面の定義
	int n = dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));		// ジオメトリo1とo2の接触情報を生成
	if (n > 0) {
		for (int i = 0; i < n; i++) {
			contact[i].surface.mode = dContactSlip1 | dContactSlip2 |	// surface.mode 値を指定する衝突状態の宣言
				dContactSoftERP | dContactSoftCFM | dContactApprox1;		// 摩擦円錐を摩擦四角錘で近似する
			contact[i].surface.mu = dInfinity;		// 摩擦係数:0～∞(dInfinity)の値で設定
			contact[i].surface.slip1 = 0.02/2;	// FDS摩擦係数
			contact[i].surface.slip2 = 0.02/2;	// FDS摩擦係数
			contact[i].surface.soft_erp = 0.5;		// 粘弾性質
			contact[i].surface.soft_cfm = 0.01;		// 粘弾性質
			//contact[i].surface.mode = dContactBounce | dContactSoftCFM;
			//contact[i].surface.bounce = 0.1 * 5;		// 弾性係数
			//contact[i].surface.bounce_vel = 0.1 * 5;	// 弾性
			dJointID c = dJointCreateContact(world, contactgroup, &contact[i]);	//接触ジョイントを形成,衝突判定のみで使用
			dJointAttach(c,							// 関節と物体を結合させる、関節を付ける2物体を選択
				dGeomGetBody(contact[i].geom.g1),	// 引数のジオメトリに対応する物体のID を取得する
				dGeomGetBody(contact[i].geom.g2));
		}
	}
}


// 開始時の視点を設定
static void start()
{
	dAllocateODEDataForThread(dAllocateMaskAll);
	static float xyz[3] = { 6.1640f,-18.0079f,10.000f };
	static float hpr[3] = { 105.5000f,-18.0000f,0.0000f };
	dsSetViewpoint(xyz, hpr);
}


// オブジェクトの生成
static void createobj() {
	// パラメータを取得
	mr2.setparam();
	// 本体
	for (int i = 0; i < 2; i++) {
		createbox(&(mr2.obj[i]), mr2space);
	}

	// 脚
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 3; j++) {
			createbox(&(mr2.leg[i].servo[j]), mr2space);
		}
		for (int j = 0; j < 3; j++) {
			createbox(&(mr2.leg[i].legobj[j]), mr2space);
		}
		createcylinder(&(mr2.leg[i].toeobj), mr2space);
	}

	// 関節の設定
	// 本体
	mr2.joint = dJointCreateFixed(world, 0);
	dJointAttach(mr2.joint, mr2.obj[0].body, mr2.obj[1].body);
	dJointSetFixed(mr2.joint);

	// 脚の関節
	for (int i = 0; i < 4; i++) {
		// 固定ジョイント
		for (int j = 0; j < 4; j++) {
			mr2.leg[i].fixedjoint[j] = dJointCreateFixed(world, 0);
		}
		dJointAttach(mr2.leg[i].fixedjoint[0], mr2.obj[0].body, mr2.leg[i].servo[0].body);
		dJointAttach(mr2.leg[i].fixedjoint[1], mr2.leg[i].servo[1].body, mr2.leg[i].legobj[1].body);
		dJointAttach(mr2.leg[i].fixedjoint[2], mr2.leg[i].legobj[1].body, mr2.leg[i].servo[2].body);
		dJointAttach(mr2.leg[i].fixedjoint[3], mr2.leg[i].legobj[2].body, mr2.leg[i].toeobj.body);
		for (int j = 0; j < 4; j++) {
			dJointSetFixed(mr2.leg[i].fixedjoint[j]);
			dJointSetFixedParam(mr2.leg[i].fixedjoint[j], dParamStopERP, 0.01);
			dJointSetFixedParam(mr2.leg[i].fixedjoint[j], dParamStopCFM, 0.001);
			dJointSetFixedParam(mr2.leg[i].fixedjoint[j], dParamCFM, 0.001);
		}

		// 可動ジョイント
		for (int j = 0; j < 3; j++) {
			mr2.leg[i].hingejoint[j] = dJointCreateHinge(world, 0);
		}
		dJointAttach(mr2.leg[i].hingejoint[0], mr2.leg[i].servo[0].body, mr2.leg[i].legobj[0].body);
		dJointAttach(mr2.leg[i].hingejoint[1], mr2.leg[i].legobj[0].body, mr2.leg[i].servo[1].body);
		dJointAttach(mr2.leg[i].hingejoint[2], mr2.leg[i].servo[2].body, mr2.leg[i].legobj[2].body);
		for (int j = 0; j < 3; j++) {
			dJointSetHingeAnchor(mr2.leg[i].hingejoint[j], mr2.leg[i].jointpos[j][0], mr2.leg[i].jointpos[j][1], (mainbodysize[2] / 2 + startz) / SCALE1);
		}
		dJointSetHingeAxis(mr2.leg[i].hingejoint[0], 0, 0, 1);
		dJointSetHingeAxis(mr2.leg[i].hingejoint[1], mr2.leg[i].jointvec[0], mr2.leg[i].jointvec[1], mr2.leg[i].jointvec[2]);
		dJointSetHingeAxis(mr2.leg[i].hingejoint[2], mr2.leg[i].jointvec[0], mr2.leg[i].jointvec[1], mr2.leg[i].jointvec[2]);

		for (int j = 0; j < 3; j++) {
			dJointSetHingeParam(mr2.leg[i].hingejoint[j], dParamStopERP, 0.1);
			dJointSetHingeParam(mr2.leg[i].hingejoint[j], dParamCFM, 0.01);
		}
	}

	// サーボ角度の初期化
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 3; j++) {
			setangle[i][j] = startangle[i][j];
		}
	}
}


// サーボのPID制御
double realangle = 0, between = 0, prev = 0, sum = 0, slope = 0, imputspeed = 0;
void servo(int lnum, int snum, double angle) {
	if (abs(angle) < 1000) {
		//if (angle >= 180)angle -= 180;
		//else if (angle <= -180)angle += 180;
		if (rotdirection[lnum][snum] == 0)realangle = -startangle[lnum][snum] + angle;
		else realangle = startangle[lnum][snum] - angle;
		// P制御
		between = realangle * M_PI / 180 - dJointGetHingeAngle(mr2.leg[lnum].hingejoint[snum]);
		// I制御
		sum += (prev + between) / 2 * dt;
		// D制御
		slope = (between - prev) / dt;
		prev = between;
		// 入力速度
		imputspeed = between * Kp +sum * Ki + slope * Kd;
		double speedmax = 9;
		if (imputspeed > speedmax) imputspeed = speedmax;
		if (imputspeed < -speedmax) imputspeed = -speedmax;
	}
	else {
		std::cout << "error " << lnum << ": " << snum << std::endl;
		imputspeed = 0;
	}
	dJointSetHingeParam(mr2.leg[lnum].hingejoint[snum], dParamVel, imputspeed);
	dJointSetHingeParam(mr2.leg[lnum].hingejoint[snum], dParamFMax, servotorque * 1000000 / (SCALE1*SCALE1));
	dJointSetHingeParam(mr2.leg[lnum].hingejoint[snum], dParamFudgeFactor, 0.7);	// 関節の挙動がジャンプしていたら小さくする
}

// それぞれのサーボに角度を入力
void setservo() {
	servo(0, 0, setangle[0][0]);
	servo(0, 1, setangle[0][1]);
	servo(0, 2, setangle[0][2]);
	servo(1, 0, setangle[1][0]);
	servo(1, 1, setangle[1][1]);
	servo(1, 2, setangle[1][2]);
	servo(2, 0, setangle[2][0]);
	servo(2, 1, setangle[2][1]);
	servo(2, 2, setangle[2][2]);
	servo(3, 0, setangle[3][0]);
	servo(3, 1, setangle[3][1]);
	servo(3, 2, setangle[3][2]);
}


// 待ち時間を実現する関数
void wait(double waittime) {
	sleeptime += waittime;
	while (Time < sleeptime) {
		Sleep((int)(dt * 1000));
	}
}

void wait_ms(double waittime) {
	sleeptime += (double)waittime / 1000;
	while (Time < sleeptime) {
		Sleep((int)(dt * 1000));
	}
}

// ボタンを待機する関数
void Wait_UB() {
	timeflag = 1;
	wait(0.1);
}

// 本体の角度を取得する関数
void anglesensor() {
	const dReal *rot;
	rot = dGeomGetRotation(mr2.obj[0].geom);
	sensorangle[0] = sensoroffset[0] + atan2(rot[9], rot[10]);										// roll,(r32,r33)
	sensorangle[1] = sensoroffset[1] + atan2(-rot[8], sqrt(rot[9] * rot[9] + rot[10] * rot[10]));	// pitch,(-r31, sqrt(r32*r32 + r33 * r33));
	sensorangle[2] = sensoroffset[2] + atan2(rot[4], rot[0]);										// yaw,(r21, r11)
}

// 並列処理関数
void threadfunc1(){
	while (1) {
		// フラグが開始するまで待つ
		while (threadflag == 0) {
			Sleep((int)(dt * 1000));
		}
		
#ifndef THISONLY
		main2();
#endif
		// 終了処理
		threadflag = 0;
	}
}


// オブジェクトの破棄と再生成
void clearobj() {
	// オブジェクトと関節の破棄
	for (int i = 0; i < 2; i++) {
		dBodyDestroy(mr2.obj[i].body);
		dGeomDestroy(mr2.obj[i].geom);
	}
	dJointDestroy(mr2.joint);
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 3; j++) {
			dBodyDestroy(mr2.leg[i].servo[j].body);
			dGeomDestroy(mr2.leg[i].servo[j].geom);
			dBodyDestroy(mr2.leg[i].legobj[j].body);
			dGeomDestroy(mr2.leg[i].legobj[j].geom);
			dJointDestroy(mr2.leg[i].hingejoint[j]);
		}
		dBodyDestroy(mr2.leg[i].toeobj.body);
		dGeomDestroy(mr2.leg[i].toeobj.geom);
		for (int j = 0; j < 4; j++) {
			dJointDestroy(mr2.leg[i].fixedjoint[j]);
		}
	}

	// オブジェクトの生成
	createobj();
}

// キーボードによるコマンド
static void command(int cmd)
{
	// モードを選択して実行
	if (cmd >= 48 && cmd <= 48+6) {
		if (threadflag == 0) {
			if (cmd == 48 + 6) {
				startz = 600;
				for (int i = 0; i < 3; i++) tableangle[i] = 0.0;
				createtable();
			}
#ifndef THISONLY
			mode = cmd - 48;
#endif
			clearobj();
			Time = 0.0;
			sleeptime = 0.0;
			threadflag = 1;
		}
	}
	switch (cmd) {
		// オブジェクトの再設定
	case' ':
		if (threadflag == 0)clearobj();
		for (int i = 0; i < 3; i++) {
			tableangle[i] = 0.0;
			sensoroffset[i] = 0.0;
		}
		break;
		// ジャイロの角度操作
	case'r':
		sensoroffset[0] += M_PI / 36;
		break;
	case'R':
		sensoroffset[0] -= M_PI / 36;
		break;
	case'p':
		sensoroffset[1] += M_PI / 36;
		break;
	case'P':
		sensoroffset[1] -= M_PI / 36;
		break;
	case'y':
		sensoroffset[2] += M_PI / 36;
		break;
	case'Y':
		sensoroffset[2] -= M_PI / 36;
		break;
		// 角度調整用の台
	case'/':
		tableangle[0] += 5.0;
		break;
	case'@':
		tableangle[0] -= 5.0;
		break;
	case';':
		tableangle[1] += 5.0;
		break;
	case':':
		tableangle[1] -= 5.0;
		break;
	case',':
		tableangle[2] += 5.0;
		break;
	case'.':
		tableangle[2] -= 5.0;
		break;
		// 手動操作
	case'u':
		if (timeflag == 1)timeflag = 0;
		break;
	case'z':
		for (int i = 0; i < 4; i++)setangle[i][0] += 10.0;
		break;
	case'Z':
		for (int i = 0; i < 4; i++)setangle[i][0] -= 10.0;
		break;
	case'x':
		for (int i = 0; i < 4; i++)setangle[i][1] += 10.0;
		break;
	case'X':
		for (int i = 0; i < 4; i++)setangle[i][1] -= 10.0;
		break;
	case'c':
		for (int i = 0; i < 4; i++)setangle[i][2] += 10.0;
		break;
	case'C':
		for (int i = 0; i < 4; i++)setangle[i][2] -= 10.0;
		break;
	}
}

// シミュレーションの1ステップで行われる処理
static void simLoop(int pause)
{
	if (!pause) {
		setservo();		// サーボの角度を設定して制御
		anglesensor();	// ロボットの角度を取得

		dSpaceCollide(space, 0, &nearCallback);		// 衝突判定
		dWorldStep(world, dt);
		//dWorldQuickStep(world, dt);
		
		dJointGroupEmpty(contactgroup);

		// 時間の更新
		if (timeflag == 0)Time += dt;
		if (Time > 3600)Time = 0.0;
	}
	

	// オブジェクトの描画
	// 本体と重り
	for (int i = 0; i < 2; i++) {
		if (i == 0)dsSetColor(mainbodycolor[0], mainbodycolor[1], mainbodycolor[2]);
		else dsSetColor(subbodycolor[0], subbodycolor[1], subbodycolor[2]);
		dsDrawBoxD(dGeomGetPosition(mr2.obj[i].geom), dGeomGetRotation(mr2.obj[i].geom), mr2.obj[i].size);
	}
	// 脚
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 3; j++) {
			dsSetColor(servocolor[0], servocolor[1], servocolor[2]);
			dsDrawBoxD(dGeomGetPosition(mr2.leg[i].servo[j].geom), dGeomGetRotation(mr2.leg[i].servo[j].geom), mr2.leg[i].servo[j].size);
			dsSetColor(legcolor[0], legcolor[1], legcolor[2]);
			dsDrawBoxD(dGeomGetPosition(mr2.leg[i].legobj[j].geom), dGeomGetRotation(mr2.leg[i].legobj[j].geom), mr2.leg[i].legobj[j].size);
		}
		dsSetColor(toecolor[0], toecolor[1], toecolor[2]);
		dsDrawCylinderD(dGeomGetPosition(mr2.leg[i].toeobj.geom), dGeomGetRotation(mr2.leg[i].toeobj.geom), mr2.leg[i].toeobj.size[1], mr2.leg[i].toeobj.size[0]);
	}
	// サンド
	dsSetTexture(DS_WOOD);
	dsSetColor(sandcolor[0], sandcolor[1], sandcolor[2]);
	dsDrawBoxD(dGeomGetPosition(sandobj.geom), dGeomGetRotation(sandobj.geom), sandobj.size);
	// スロープ
	dsSetTexture(DS_WOOD);
	dsSetColor(slopecolor[0], slopecolor[1], slopecolor[2]);
	for (int i = 0; i < 2; i++) {
		dsDrawBoxD(dGeomGetPosition(slopeobj[i].geom), dGeomGetRotation(slopeobj[i].geom), slopeobj[i].size);
	}
	// 角度調整用の台
	if (startz > 499)movetable();
}


// メイン関数
int main(int argc, char **argv)
{
	// setup pointers to drawstuff callback functions
	dsFunctions fn;
	fn.version = DS_VERSION;
	fn.start = &start;
	fn.step = &simLoop;
	fn.command = &command;
	fn.stop = 0;
	fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;

	// create world
	dInitODE2(0);
	world = dWorldCreate();
	space = dHashSpaceCreate(0);
	contactgroup = dJointGroupCreate(0);
	dWorldSetGravity(world, 0, 0, -GRAVITY * 1000 / SCALE1);
	dWorldSetCFM(world, 1e-5);
	//dWorldSetAutoDisableFlag(world, 1);	// 一定時間たつと勝手にシミュレーションが終了する

	mr2space = dSimpleSpaceCreate(space);
	dSpaceSetCleanup(mr2space, 0);
	
	ground = dCreatePlane(space, 0, 0, 1, 0);	// 地面の生成

	// オブジェクトの生成
	createobj();
	createsand();
	createslope();

	// 並列処理の関数を宣言
	std::thread thread1(threadfunc1);
#ifdef OPENGL
	std::thread thread2(threadfunc2);
#endif
	// run simulation
	dsSimulationLoop(argc, argv, 352 * 4, 288 * 3, &fn);

	dJointGroupDestroy(contactgroup);
	dSpaceDestroy(space);
	dWorldDestroy(world);
	dCloseODE();

	// 並列処理が終了するのを待つ
	thread1.join();
#ifdef OPENGL
	thread2.join();
#endif

	return 0;
}
