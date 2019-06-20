// �����@�@�T�[�{��

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"
#include <thread>		// ���񏈗����s�p
#include <windows.h>	// �ꎞ��~"Sleep"���s�p�̊֐�
#include <iostream>

#define _USE_MATH_DEFINES
#include <math.h>

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

//#include "icosahedron_geom.h"

// select correct drawing functions

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#define dsDrawConvex dsDrawConvexD
#endif

#define DENSITY (5.0)		// ���x
#define MAX_CONTACTS 3		// �ڐG�_�̍ő�l
#define GRAVITY         REAL(9.8)
#define SCALE1 100.0

///////////////////////////////////////////////////////////////////////////////////
/**********************************�ݒ�p�����[�^*********************************/
// �S����4.8kg
// ���@
double servosize[3] = { 65.0,30.0,48.0 };		// �T�[�{���[�^�̑傫��[mm]
double servooffset = 50 - 65.0 / 2.0;			// �T�[�{���[�^�̒��S�Ɖ�]���S�̂���[mm]
double servodistance = 70;						// �t������2�̃T�[�{�̉�]���S�Ԃ̋���[mm]
double servomass = (163.0 + 20) / 1000;			// �T�[�{���[�^(�ƃT�[�{�z�[��)�̎���[kg]
double servocolor[3] = { 0.1,0.3,0.3 };			// �T�[�{���[�^�̐F

double leglength[3] = { 10,143.01*1.6,297.95/1.4 };		// �r�̒���[mm]�A{�p���̈�ӂ̒���,�r�̏㉺�̊֐߂���G�̊֐߂܂ł̒����A�G�̊֐߂��瑫��܂ł̒���}
//double leglength[3] = { 10,216,234.6 };
double legmass = 30.0 / 1000;					// �r�̃A���~�p�p�C�v�̎���[kg]
double legcolor[3] = { 0.8,0.8,0.8 };			// �r�̐F

double toesize[2] = { 15,40 };					// ����̊���~�߂̉~���̑傫��[mm]�A{���a,����}
double toemass = 30.0 / 1000;					// ����̎���[kg]
double toecolor[3] = { 1.0,0.4,0.4 };			// ����̐F

double mainbodysize[3] = { 190.1,190.1,56.0 };	// �{�̂̑傫��[mm]
double mainbodymass = 800.0 / 1000;				// �{�̂̎���[kg],(�c�莿��2124g)
double mainbodycolor[3] = { 1.0,0.4,0.4 };		// �{�̂̐F

double subbodysize[3] = { 50,250,70 };			// �{�̂ɂ��Ă�d��̃T�C�Y[mm]
double subbodymass = 1324.0 / 1000;				// �{�̂ɂ��Ă�d��̎���[kg]
double subbodypos[3] = { 0,0,80 };				// �{�̂ɂ��Ă�d��̈ʒu[mm]
double subbodycolor[3] = { 0.5,0.5,0.5 };		// �d��̐F

double startz = 10;								// ��������[mm]

double slopepos[3] = { 0,1700,0 };				// �X���[�v�̈ʒu
double slopecolor[3] = { 1.0,0.85,0.7 };		// �X���[�v�̐F

double sandpos[3] = { 0,850,0 };				// �T���h�̈ʒu
double sandcolor[3] = { 1.0,0.85,0.7 };			// �T���h�̐F

// �T�[�{�p�x�̐ݒ�
double startangle[4][3] = { {45,0.0,0.0},
							{-45,0.0,0.0},
							{45,0.0,0.0},
							{-45,0.0,0.0},};// �T�[�{�̏����p�x(�I�t�Z�b�g)
int rotdirection[4][3] = { {1,1,1},
						   {1,1,1},
						   {1,1,1},
						   {1,1,1}, };		// �T�[�{�̉�]����(0��1)
double servotorque = 60.0 *GRAVITY* 0.01;	// �T�[�{�̃g���N[N*m]
double Kp = 50.0, Ki = 0.05, Kd = 0.02;		// �T�[�{��PID����̃Q�C��

// ���̑��p�����[�^
double dt = 0.008;							// ���ݎ���[s]
/*********************************************************************************/
///////////////////////////////////////////////////////////////////////////////////


// �����̌^�̕���
class SetBox {
public:
	dReal size[3] = { 0,0,0 }, m = 0, pos[3] = { 0,0,0 };
	dMatrix3 R;			// ��]�s����i�[����\����
	dMass mass;			// ���ʃp�����[�^���i�[����ϐ�
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
	dRFromAxisAndAngle(R, ax, ay, az, angle);	// ��]�s����擾
}

// �~���^�̕���
class SetCylinder {
public:
	dReal size[2] = { 0,0 }, m = 0, pos[3] = { 0,0,0 };
	dMatrix3 R;			// ��]�s����i�[����\����
	dMass mass;			// ���ʃp�����[�^���i�[����ϐ�
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
	dRFromAxisAndAngle(R, ax, ay, az, angle);	// ��]�s����擾
}

static dWorldID world;

// �����̌^�̃{�f�B�ƃW�I���g���𐶐�
void createbox(SetBox* box, dSpaceID& spacename) {
	box->body = dBodyCreate(world);
	dBodySetPosition(box->body, box->pos[0], box->pos[1], box->pos[2]);
	dBodySetRotation(box->body, box->R);
	dBodySetMass(box->body, &(box->mass));
	box->geom = dCreateBox(spacename, box->size[0], box->size[1], box->size[2]);
	dGeomSetBody(box->geom, box->body);
}

// �~���^�̃{�f�B�ƃW�I���g���𐶐�
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

	// �I�u�W�F�N�g�̍��W����]���Đݒ�
	servo[0].setposition(temp[0] * cos(angle), temp[0] * sin(angle), mainbodysize[2] / 2 + startz);
	servo[1].setposition(temp[1] * cos(angle), temp[1] * sin(angle), mainbodysize[2] / 2 + startz);
	servo[2].setposition(temp[3] * cos(angle), temp[3] * sin(angle), mainbodysize[2] / 2 + startz);
	legobj[0].setposition(temp[6] * cos(angle), temp[6] * sin(angle), mainbodysize[2] / 2 + startz);
	legobj[1].setposition(temp[2] * cos(angle), temp[2] * sin(angle), mainbodysize[2] / 2 + startz);
	legobj[2].setposition(temp[4] * cos(angle), temp[4] * sin(angle), mainbodysize[2] / 2 + startz);
	toeobj.setposition(temp[5] * cos(angle), temp[5] * sin(angle), mainbodysize[2] / 2 + startz);

	// �I�u�W�F�N�g�̉�]�s���ݒ�
	for (int i = 0; i < 3; i++) {
		servo[i].setrotation(0, 0, 1, angle);
		legobj[i].setrotation(0, 0, 1, angle);
	}
	toeobj.setrotation(cos(angle), sin(angle), 0, M_PI / 2);

	// �I�u�W�F�N�g�̐��@��ݒ�
	servo[0].setsize(servosize[0], servosize[1], servosize[2], servomass);
	servo[1].setsize(servosize[0], servosize[2], servosize[1], servomass);
	servo[2].setsize(servosize[0], servosize[2], servosize[1], servomass);
	legobj[0].setsize(servodistance, leglength[0], leglength[0], legmass);
	legobj[1].setsize(leglength[1], leglength[0], leglength[0], legmass);
	legobj[2].setsize(leglength[2], leglength[0], leglength[0], legmass);
	toeobj.setsize(toesize[0], toesize[1], toemass);

	// �֐߂̈ʒu�ƌ�����ݒ�
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

static double Time = 0.0;					// ����
static double sleeptime = 0.0;				// ��~���Ԃ��i�[
static int timeflag = 0;					// ���Ԍo�߂��~����t���O
static int threadflag = 0;					// ���񏈗����s�J�n�̃t���O
double setangle[4][3];						// �T�[�{�̊p�x���i�[
double sensorangle[3] = { 0.0,0.0,0.0 };	// �{�̂̊p�x���i�[�A{roll,pitch,yaw}
double sensoroffset[3] = { 0.0,0.0,0.0 };	// �Z���T�[�p�x����p�̕ϐ��A{roll,pitch,yaw}

// ���̃t�@�C���Œ�`�ς݂̕ϐ��Ɗ֐�
extern int mode;
int Gyro = 0;
int print_adjust = 1;
void mainsimulation();
void threadfunc2();


// �p�x�␳�m�F�p�̓y��I�u�W�F�N�g
static SetBox tableobj[3];
static dJointID tablejoint[3];
static double tableangle[3] = { 0.0,0.0,0.0 };
void createtable() {
	// �{�f�B�ƃW�I���g���̐���
	for (int i = 0; i < 3; i++) {
		tableobj[i].setposition(0, 0, startz - 150);
		tableobj[i].setrotation(0, 0, 1, 0);
	}
	tableobj[0].setsize(10, 10, 10, 0.01);
	tableobj[1].setsize(10, 10, 10, 0.01);
	tableobj[2].setsize(1300, 1300, 100, 1.0);
	for (int i = 0; i < 3; i++) createbox(&(tableobj[i]), space);
	
	// �֐߂̐���
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
		dJointSetHingeParam(tablejoint[i], dParamFudgeFactor, 0.7);	// �֐߂̋������W�����v���Ă����珬��������
	}
	// �`��
	dsSetTexture(DS_WOOD);
	dsSetColor(0.8, 0.8, 0.8);
	dsDrawBoxD(dGeomGetPosition(tableobj[2].geom), dGeomGetRotation(tableobj[2].geom), tableobj[2].size);
}

// �i���p�̃I�u�W�F�N�g
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

// �⓹�p�̃I�u�W�F�N�g
static SetBox slopeobj[2];
void createslope() {
	double slopelength[3] = { 1455,1500,1125 };	// �X���[�v�̕��ƒ���,���R�����̒���
	double angle = atan((double)4 / (double)15);
	double direction = M_PI * 0 / 4;
	// �X�Ε���
	slopeobj[0].setposition(slopepos[0] + slopelength[1] / (2 * cos(angle)*cos(angle)) * (-sin(direction)), slopepos[1] + slopelength[1] / (2 * cos(angle)*cos(angle)) * cos(direction), slopepos[2]);
	dRFromEulerAngles(slopeobj[0].R, -M_PI / 2, -M_PI / 2 - direction, M_PI / 2 - angle);		// �I�C���[�i��ݒ�,(xyz�n)
	slopeobj[0].setsize(slopelength[1] * tan(angle) / cos(angle), slopelength[1] / cos(angle), slopelength[0]);
	slopeobj[0].geom = dCreateBox(space, slopeobj[0].size[0], slopeobj[0].size[1], slopeobj[0].size[2]);
	dGeomSetPosition(slopeobj[0].geom, slopeobj[0].pos[0], slopeobj[0].pos[1], slopeobj[0].pos[2]);
	dGeomSetRotation(slopeobj[0].geom, slopeobj[0].R);
	// ���R����
	slopeobj[1].setposition(slopepos[0] + (slopelength[1] + slopelength[2] / 2)  * (-sin(direction)), slopepos[1] + (slopelength[1] + slopelength[2] / 2) * cos(direction), slopepos[2] + slopelength[1] * tan(angle) / 2);
	slopeobj[1].setrotation(0, 0, 1, direction);
	slopeobj[1].setsize(slopelength[0], slopelength[2], slopelength[1] * tan(angle));
	slopeobj[1].geom = dCreateBox(space, slopeobj[1].size[0], slopeobj[1].size[1], slopeobj[1].size[2]);
	dGeomSetPosition(slopeobj[1].geom, slopeobj[1].pos[0], slopeobj[1].pos[1], slopeobj[1].pos[2]);
	dGeomSetRotation(slopeobj[1].geom, slopeobj[1].R);
}


void MR2::setparam() {
	// �{�̂̍��W��ݒ�
	obj[0].setposition(0, 0, mainbodysize[2] / 2 + startz);
	obj[1].setposition(subbodypos[0], subbodypos[1], subbodypos[2] + startz);
	
	// �{�̂̐��@��ݒ�
	obj[0].setsize(mainbodysize[0] - servooffset * sqrt(2), mainbodysize[1] - servooffset * sqrt(2), mainbodysize[2], mainbodymass);
	obj[1].setsize(subbodysize[0], subbodysize[1], subbodysize[2], subbodymass);

	// �r�̃p�����[�^��ݒ�
	leg[0].initialposition(M_PI / 4);
	leg[1].initialposition(-M_PI / 4);
	leg[2].initialposition(-M_PI * 3 / 4);
	leg[3].initialposition(M_PI * 3 / 4);
}


// �Փ˔�����s���֐�
static void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
	// �������s��Ȃ����̂����O
	/*int g1 = (o1 == slopeobj[0].geom || o1 == slopeobj[1].geom);
	int g2 = (o2 == slopeobj[0].geom || o2 == slopeobj[1].geom);
	if (!(g1 ^ g2)) return;*/
	int g1 = (o1 == sandobj.geom || o1 == slopeobj[0].geom || o1 == slopeobj[1].geom || o1 == ground || o1 == tableobj[0].geom || o1 == tableobj[1].geom || o1 == tableobj[2].geom);
	int g2 = (o2 == sandobj.geom || o2 == slopeobj[0].geom || o2 == slopeobj[1].geom || o2 == ground || o2 == tableobj[0].geom || o2 == tableobj[1].geom || o2 == tableobj[2].geom);
	if (g1*g2 != 0)return;

	const int N = 40;			// �W�I���g�������������ۂ̍ő�ڐG�_�����w��
	dContact contact[N];		// �ڐG�ʂ̒�`
	int n = dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));		// �W�I���g��o1��o2�̐ڐG���𐶐�
	if (n > 0) {
		for (int i = 0; i < n; i++) {
			contact[i].surface.mode = dContactSlip1 | dContactSlip2 |	// surface.mode �l���w�肷��Փˏ�Ԃ̐錾
				dContactSoftERP | dContactSoftCFM | dContactApprox1;		// ���C�~���𖀎C�l�p���ŋߎ�����
			contact[i].surface.mu = dInfinity;		// ���C�W��:0�`��(dInfinity)�̒l�Őݒ�
			contact[i].surface.slip1 = 0.02/2;	// FDS���C�W��
			contact[i].surface.slip2 = 0.02/2;	// FDS���C�W��
			contact[i].surface.soft_erp = 0.5;		// �S�e����
			contact[i].surface.soft_cfm = 0.01;		// �S�e����
			//contact[i].surface.mode = dContactBounce | dContactSoftCFM;
			//contact[i].surface.bounce = 0.1 * 5;		// �e���W��
			//contact[i].surface.bounce_vel = 0.1 * 5;	// �e��
			dJointID c = dJointCreateContact(world, contactgroup, &contact[i]);	//�ڐG�W���C���g���`��,�Փ˔���݂̂Ŏg�p
			dJointAttach(c,							// �֐߂ƕ��̂�����������A�֐߂�t����2���̂�I��
				dGeomGetBody(contact[i].geom.g1),	// �����̃W�I���g���ɑΉ����镨�̂�ID ���擾����
				dGeomGetBody(contact[i].geom.g2));
		}
	}
}


// �J�n���̎��_��ݒ�
static void start()
{
	dAllocateODEDataForThread(dAllocateMaskAll);
	static float xyz[3] = { 6.1640f,-18.0079f,10.000f };
	static float hpr[3] = { 105.5000f,-18.0000f,0.0000f };
	dsSetViewpoint(xyz, hpr);
}


// �I�u�W�F�N�g�̐���
static void createobj() {
	// �p�����[�^���擾
	mr2.setparam();
	// �{��
	for (int i = 0; i < 2; i++) {
		createbox(&(mr2.obj[i]), mr2space);
	}

	// �r
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 3; j++) {
			createbox(&(mr2.leg[i].servo[j]), mr2space);
		}
		for (int j = 0; j < 3; j++) {
			createbox(&(mr2.leg[i].legobj[j]), mr2space);
		}
		createcylinder(&(mr2.leg[i].toeobj), mr2space);
	}

	// �֐߂̐ݒ�
	// �{��
	mr2.joint = dJointCreateFixed(world, 0);
	dJointAttach(mr2.joint, mr2.obj[0].body, mr2.obj[1].body);
	dJointSetFixed(mr2.joint);

	// �r�̊֐�
	for (int i = 0; i < 4; i++) {
		// �Œ�W���C���g
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

		// ���W���C���g
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

	// �T�[�{�p�x�̏�����
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 3; j++) {
			setangle[i][j] = startangle[i][j];
		}
	}
}


// �T�[�{��PID����
double realangle = 0, between = 0, prev = 0, sum = 0, slope = 0, imputspeed = 0;
void servo(int lnum, int snum, double angle) {
	if (abs(angle) < 1000) {
		//if (angle >= 180)angle -= 180;
		//else if (angle <= -180)angle += 180;
		if (rotdirection[lnum][snum] == 0)realangle = -startangle[lnum][snum] + angle;
		else realangle = startangle[lnum][snum] - angle;
		// P����
		between = realangle * M_PI / 180 - dJointGetHingeAngle(mr2.leg[lnum].hingejoint[snum]);
		// I����
		sum += (prev + between) / 2 * dt;
		// D����
		slope = (between - prev) / dt;
		prev = between;
		// ���͑��x
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
	dJointSetHingeParam(mr2.leg[lnum].hingejoint[snum], dParamFudgeFactor, 0.7);	// �֐߂̋������W�����v���Ă����珬��������
}

// ���ꂼ��̃T�[�{�Ɋp�x�����
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


// �҂����Ԃ���������֐�
void wait(double waittime) {
	sleeptime += waittime;
	while (Time < sleeptime) {
		Sleep((int)(dt * 1000));
	}
}

// �{�^����ҋ@����֐�
void Wait_UB() {
	timeflag = 1;
	wait(0.1);
}

// �����Z���T��ҋ@����֐�
double Distance() {
	return 15.0;
}

// �{�̂̊p�x���擾����֐�
void anglesensor() {
	const dReal *rot;
	rot = dGeomGetRotation(mr2.obj[0].geom);
	sensorangle[0] = sensoroffset[0] + atan2(rot[9], rot[10]);										// roll,(r32,r33)
	sensorangle[1] = sensoroffset[1] + atan2(-rot[8], sqrt(rot[9] * rot[9] + rot[10] * rot[10]));	// pitch,(-r31, sqrt(r32*r32 + r33 * r33));
	sensorangle[2] = sensoroffset[2] + atan2(rot[4], rot[0]);										// yaw,(r21, r11)
}

// ���񏈗��֐�
void threadfunc1(){
	while (1) {
		// �t���O���J�n����܂ő҂�
		while (threadflag == 0) {
			Sleep((int)(dt * 1000));
		}
		
		mainsimulation();

		// �I������
		threadflag = 0;
	}
}


// �I�u�W�F�N�g�̔j���ƍĐ���
void clearobj() {
	// �I�u�W�F�N�g�Ɗ֐߂̔j��
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

	// �I�u�W�F�N�g�̐���
	createobj();
}

// �L�[�{�[�h�ɂ��R�}���h
static void command(int cmd)
{
	// ���[�h��I�����Ď��s
	if (cmd >= 48 && cmd <= 48+6) {
		if (threadflag == 0) {
			if (cmd == 48 + 6) {
				startz = 600;
				for (int i = 0; i < 3; i++) tableangle[i] = 0.0;
				createtable();
			}
			mode = cmd - 48;
			clearobj();
			Time = 0.0;
			sleeptime = 0.0;
			threadflag = 1;
		}
	}
	switch (cmd) {
		// �I�u�W�F�N�g�̍Đݒ�
	case' ':
		if (threadflag == 0)clearobj();
		for (int i = 0; i < 3; i++) {
			tableangle[i] = 0.0;
			sensoroffset[i] = 0.0;
		}
		break;
		// �W���C���̊p�x����
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
		// �p�x�����p�̑�
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
		// �蓮����
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

// �V�~�����[�V������1�X�e�b�v�ōs���鏈��
static void simLoop(int pause)
{
	if (!pause) {
		setservo();		// �T�[�{�̊p�x��ݒ肵�Đ���
		anglesensor();	// ���{�b�g�̊p�x���擾

		dSpaceCollide(space, 0, &nearCallback);		// �Փ˔���
		dWorldStep(world, dt);
		//dWorldQuickStep(world, dt);
		
		dJointGroupEmpty(contactgroup);

		// ���Ԃ̍X�V
		if (timeflag == 0)Time += dt;
		if (Time > 3600)Time = 0.0;
	}
	

	// �I�u�W�F�N�g�̕`��
	// �{�̂Əd��
	for (int i = 0; i < 2; i++) {
		if (i == 0)dsSetColor(mainbodycolor[0], mainbodycolor[1], mainbodycolor[2]);
		else dsSetColor(subbodycolor[0], subbodycolor[1], subbodycolor[2]);
		dsDrawBoxD(dGeomGetPosition(mr2.obj[i].geom), dGeomGetRotation(mr2.obj[i].geom), mr2.obj[i].size);
	}
	// �r
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
	// �T���h
	dsSetTexture(DS_WOOD);
	dsSetColor(sandcolor[0], sandcolor[1], sandcolor[2]);
	dsDrawBoxD(dGeomGetPosition(sandobj.geom), dGeomGetRotation(sandobj.geom), sandobj.size);
	// �X���[�v
	dsSetTexture(DS_WOOD);
	dsSetColor(slopecolor[0], slopecolor[1], slopecolor[2]);
	for (int i = 0; i < 2; i++) {
		dsDrawBoxD(dGeomGetPosition(slopeobj[i].geom), dGeomGetRotation(slopeobj[i].geom), slopeobj[i].size);
	}
	// �p�x�����p�̑�
	if (startz > 499)movetable();
}


// ���C���֐�
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
	//dWorldSetAutoDisableFlag(world, 1);	// ��莞�Ԃ��Ə���ɃV�~�����[�V�������I������

	mr2space = dSimpleSpaceCreate(space);
	dSpaceSetCleanup(mr2space, 0);
	
	ground = dCreatePlane(space, 0, 0, 1, 0);	// �n�ʂ̐���

	// �I�u�W�F�N�g�̐���
	createobj();
	createsand();
	createslope();
	//if (startz > 499)createtable();

	// ���񏈗��̊֐���錾
	std::thread thread1(threadfunc1);
	std::thread thread2(threadfunc2);

	// run simulation
	dsSimulationLoop(argc, argv, 352 * 4, 288 * 3, &fn);

	dJointGroupDestroy(contactgroup);
	dSpaceDestroy(space);
	dWorldDestroy(world);
	dCloseODE();

	// ���񏈗����I������̂�҂�
	thread1.join();
	thread2.join();

	return 0;
}
