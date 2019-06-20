///////////////////////////////////////////////////////////////////
//                                                               //
//         Sample Program of 2-D Graphics  sdglib.h              //
//                 Orignal  2013/Oct/16 Nishi                    //
//                 Modified 2013/Oct/20 Nakazawa                 //
//                                                               //
///////////////////////////////////////////////////////////////////
#ifndef __sdglib_h__
#define __sdglib_h__

//#include <math.h>
//#include <iostream>
//#include <stdexcept>
#include <string>
//#include <string.h>
//#include <GL/glut.h>

#if defined(WINDOWS) || defined(_WIN32) || defined(_WIN64)
#include <GL/glut.h>
#endif
#if defined(linux) || defined(__linux) || defined(__linux__)
#include <GL/glut.h>
#endif
#if defined(__APPLE__)
#include <GLUT/glut.h>
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


class SDGLib {
public:
	SDGLib(int, int, std::string, double, double, double, double);
	//	~SDGLib();
	//void Before(void);
	//void After(void);
	//void ReDraw(void);
	void Start(void);
	void SetCursor(int);
	//void SetColor(float, float, float);
	//void DrawPoint(float, double, double);
	//void DrawLine(float, double, double, double, double);
	//void DrawCircle(float, double, double, double);
	//void DrawString(double, double, char *);
	void Display(void (*)(void));
private:
	int size_x, size_y;
	std::string title;
	double small_x, large_x, small_y, large_y;
};

void SDGLib::Display(void (*func)(void)) {
	glutDisplayFunc(func);
};

SDGLib::SDGLib(int size_x=600, int size_y=600, std::string title="No Title", double small_x=-300.0, double large_x=-300.0, double small_y=-300.0, double large_y=300.0) {
	int i=1;
	//char cv[1] = { 'SD' };
	char cv_s = 'S';
	char *cv_pt = &cv_s;
	char **cv = &cv_pt;
	glutInit(&i,cv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
	glutInitWindowSize(size_x, size_y);
	glutInitWindowPosition(0, 0);
	glutCreateWindow(title.c_str());
	glClearColor(1.0, 1.0, 1.0, 1.0);
	gluOrtho2D(small_x, large_x, small_y, large_y);
	glutSetCursor(GLUT_CURSOR_HELP);
}
void SDGLib::SetCursor(int cursor_type){
	glutSetCursor(cursor_type);
}
void SDGLib::Start(void){
	glutMainLoop();
}


namespace SDGLibF {
	void Before(void){
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	}
	void After(void){
		glutSwapBuffers();
	}
	void SetCursor(int cursor_type){
		glutSetCursor(cursor_type);
	}
	void SetColor(float r, float g, float b){
		glColor3f((GLfloat)r, (GLfloat)g, (GLfloat)b);
	}
	void DrawPoint(float size=10, double x=0, double y=0){
		glPointSize((GLfloat)size);
		glBegin(GL_POINTS);
		glVertex2d((GLdouble)x, (GLdouble)y);
		glEnd();
	}
	void DrawLine(float width=2, double x1=100, double y1=100, double x2=-100, double y2=-100){
		glLineWidth((GLfloat)width);
		glBegin(GL_LINES);
		glVertex2d((GLdouble)x1,(GLdouble)y1);
		glVertex2d((GLdouble)x2,(GLdouble)y2);
		glEnd();
	}
	void DrawCircle(float width, double x, double y, double r){
		int i;
		GLdouble x1, x2, y1, y2;

		for(i=0; i<360; i++){
			x1=x+r*sin(M_PI*i/180.0); y1=y+r*cos(M_PI*i/180.0);
			x2=x+r*sin(M_PI*(i+1)/180.0); y2=y+r*cos(M_PI*(i+1)/180.0);
			DrawLine((GLfloat)width, x1, y1, x2, y2);
		}
	}
	void DrawString(double x, double y, std::string s){
		glRasterPos2f(x, y);
		for(unsigned int i=0; i<s.length(); i++){
			//glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, s.at(i));
			glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, s.at(i));
		}
	}
	void ReDraw(void){
		glutPostRedisplay();
	}
	void IdleFunc(void (*func)(void)) {
		glutIdleFunc(func);
	}
}
# endif
