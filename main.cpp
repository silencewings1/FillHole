#include <iostream>
#include "Mesh.h"
#include "GLProjector.h"

// the way the mesh is rendered
enum EnumDisplayMode {
	WIREFRAME,
	HIDDENLINE,
	FLATSHADED,
	SMOOTHSHADED
};

// variables
int displayMode = SMOOTHSHADED;                     // current display mode
int mainMenu, displayMenu;                          // glut menu handlers
int winWidth, winHeight;                            // window width and height
double winAspect;                                   // winWidth / winHeight;
int lastX, lastY;                                   // last mouse motion position
bool leftDown, middleDown, shiftDown;               // mouse down and shift down flags
double sphi = 90.0, stheta = 45.0, sdepth = 10;     // for simple trackball
double xpan = 0.0, ypan = 0.0;                      // for simple trackball
double zNear = 1.0, zFar = 100.0;                   // clipping
double g_fov = 45.0;
Eigen::Vector3d g_center;
double g_sdepth;
Mesh mesh; // our mesh

// editing mode
enum Mode {
	Viewing,
	Selection,
	Moving
};

Mode currentMode = Viewing;
int downX, downY; // mouse down position
int selectedHandleIndex = -1; // the index of the handle

// functions
void SetBoundaryBox(const Eigen::Vector3d& bmin, const Eigen::Vector3d& bmax);
void InitGL();
void InitMenu();
void InitGeometry();
void FillMeshHole();

// window related 
void MenuCallback(int value);
void ReshapeFunc(int width, int height);

// rendering functions
void DisplayFunc();
void DrawWireframe();
void DrawHiddenLine();
void DrawFlatShaded();
void DrawSmoothShaded();

// input related glut functions
void KeyboardFunc(unsigned char ch, int x, int y);
void MouseFunc(int button, int state, int x, int y);
void MotionFunc(int x, int y);


void SetBoundaryBox(const Eigen::Vector3d& bmin, const Eigen::Vector3d& bmax) {
	double PI = 3.14159265358979323846;
	double radius = (bmax - bmin).norm();
	g_center = 0.5 * (bmin + bmax);
	zNear = 0.2 * radius / sin(0.5 * g_fov * PI / 180.0);
	zFar = zNear + 2.0 * radius;
	g_sdepth = zNear + radius;
	zNear *= 0.1;
	zFar *= 10;
	sdepth = g_sdepth;
}

// init openGL environment
void InitGL() {
	GLfloat light0Position[] = { 0, 1, 0, 1.0 };

	// initialize GLUT stuffs
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(500, 500);
	glutCreateWindow("Comp5411 Mesh Viewer");

	glClearColor(0.0, 0.0, 0.0, 0.0);
	glPolygonOffset(1.0, 1.0);
	glDepthFunc(GL_LEQUAL);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	glEnable(GL_COLOR_MATERIAL);
	glColorMaterial(GL_FRONT, GL_DIFFUSE);
	glLightfv(GL_LIGHT0, GL_POSITION, light0Position);
	glEnable(GL_LIGHT0);

	glutReshapeFunc(ReshapeFunc);
	glutDisplayFunc(DisplayFunc);
	glutKeyboardFunc(KeyboardFunc);
	glutMouseFunc(MouseFunc);
	glutMotionFunc(MotionFunc);
}

// init right-click menu
void InitMenu() {
	displayMenu = glutCreateMenu(MenuCallback);
	glutAddMenuEntry("Wireframe", WIREFRAME);
	glutAddMenuEntry("Hidden Line", HIDDENLINE);
	glutAddMenuEntry("Flat Shaded", FLATSHADED);
	glutAddMenuEntry("Smooth Shaded", SMOOTHSHADED);

	mainMenu = glutCreateMenu(MenuCallback);
	glutAddSubMenu("Display", displayMenu);
	glutAddMenuEntry("Fill Hole", 123);
	glutAddMenuEntry("Exit", 99);
	glutAttachMenu(GLUT_RIGHT_BUTTON);
}

// init geometry (if no input argument is provided)
void InitGeometry() {
	const int VSIZE = 4;
	const int HESIZE = 12;
	const int FSIZE = 4;
	int i;
	Vertex* v[VSIZE];
	HEdge* he[HESIZE];
	Face* f[FSIZE];

	for (i = 0; i < VSIZE; i++) {
		v[i] = new Vertex();
		mesh.vList.push_back(v[i]);
	}
	v[0]->SetPosition(Eigen::Vector3d(0.0, 0.0, 0.0));
	v[1]->SetPosition(Eigen::Vector3d(10.0, 0.0, 0.0));
	v[2]->SetPosition(Eigen::Vector3d(0.0, 10.0, 0.0));
	v[3]->SetPosition(Eigen::Vector3d(0.0, 0.0, 10.0));

	v[0]->SetNormal(Eigen::Vector3d(-0.577, -0.577, -0.577));
	v[1]->SetNormal(Eigen::Vector3d(0.0, -0.7, -0.7));
	v[2]->SetNormal(Eigen::Vector3d(-0.7, 0.0, -0.7));
	v[3]->SetNormal(Eigen::Vector3d(-0.7, -0.7, 0.0));

	for (i = 0; i < FSIZE; i++) {
		f[i] = new Face();
		mesh.fList.push_back(f[i]);
	}

	for (i = 0; i < HESIZE; i++) {
		he[i] = new HEdge();
		mesh.heList.push_back(he[i]);
	}
	for (i = 0; i < FSIZE; i++) {
		int base = i * 3;
		Mesh::SetPrevNext(he[base], he[base + 1]);
		Mesh::SetPrevNext(he[base + 1], he[base + 2]);
		Mesh::SetPrevNext(he[base + 2], he[base]);
		Mesh::SetFace(f[i], he[base]);
	}
	Mesh::SetTwin(he[0], he[4]);
	Mesh::SetTwin(he[1], he[7]);
	Mesh::SetTwin(he[2], he[10]);
	Mesh::SetTwin(he[3], he[8]);
	Mesh::SetTwin(he[5], he[9]);
	Mesh::SetTwin(he[6], he[11]);
	he[0]->SetStart(v[1]);
	he[1]->SetStart(v[2]);
	he[2]->SetStart(v[3]);
	he[3]->SetStart(v[0]);
	he[4]->SetStart(v[2]);
	he[5]->SetStart(v[1]);
	he[6]->SetStart(v[0]);
	he[7]->SetStart(v[3]);
	he[8]->SetStart(v[2]);
	he[9]->SetStart(v[0]);
	he[10]->SetStart(v[1]);
	he[11]->SetStart(v[3]);
	v[0]->SetHalfEdge(he[3]);
	v[1]->SetHalfEdge(he[0]);
	v[2]->SetHalfEdge(he[1]);
	v[3]->SetHalfEdge(he[2]);
}


// GLUT menu callback function
void MenuCallback(int value) {
	switch (value) {
	case 99:
		exit(0);
		break;
	case 123:
		FillMeshHole();
		glutPostRedisplay();
		break;
	default:
		displayMode = value;
		glutPostRedisplay();
		break;
	}
}


// GLUT reshape callback function
void ReshapeFunc(int width, int height) {
	winWidth = width;
	winHeight = height;
	winAspect = (double)width / (double)height;
	glViewport(0, 0, width, height);
	glutPostRedisplay();
}


// GLUT display callback function
void DisplayFunc() {
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(g_fov, winAspect, zNear, zFar);
	//glOrtho(-2.0, 2.0, -2.0, 2.0, zNear, zFar);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef(xpan, ypan, -sdepth);
	glRotatef(-stheta, 1.0, 0.0, 0.0);
	glRotatef(sphi, 0.0, 1.0, 0.0);
	glTranslatef(-g_center[0], -g_center[1], -g_center[2]);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	switch (displayMode) {
	case WIREFRAME:
		DrawWireframe();
		break;
	case HIDDENLINE:
		DrawHiddenLine();
		break;
	case FLATSHADED:
		DrawFlatShaded();
		break;
	case SMOOTHSHADED:
		DrawSmoothShaded();
		break;
	}
	glutSwapBuffers();
}

// Wireframe render function
void DrawWireframe() {
	HEdgeList heList = mesh.Edges();
	HEdgeList bheList = mesh.BoundaryEdges();
	glColor3f(1.0f, 1.0f, 1.0f);
	glBegin(GL_LINES);
	size_t i;
	for (i = 0; i < heList.size(); i++) {
		glVertex3dv(heList[i]->Start()->Position().data());
		glVertex3dv(heList[i]->End()->Position().data());
	}
	glColor3f(1.0f, 0.0f, 0.0f);
	for (i = 0; i < bheList.size(); i++) {
		glVertex3dv(bheList[i]->Start()->Position().data());
		glVertex3dv(bheList[i]->End()->Position().data());
	}
	glEnd();

	HEdgeList bhefhList = mesh.bhefhList;
	glPointSize(10.0);
	glBegin(GL_POINTS);
	glColor3f(1.0f, 1.0f, 0.0f);
	for (i = 0; i < bhefhList.size(); i++) {
		if (bhefhList[i]->IsHole()) {
			glVertex3dv(bhefhList[i]->End()->Position().data());
		}
	}
	glEnd();
	glPointSize(1.0);

	FaceList ffhList = mesh.ffhList;
	glBegin(GL_TRIANGLES);
	glColor3f(0.0f, 1.0f, 0.0f);
	for (int i = 0; i < ffhList.size(); i++) {
		Face* f = ffhList[i];
		const Eigen::Vector3d& pos1 = f->HalfEdge()->Start()->Position();
		const Eigen::Vector3d& pos2 = f->HalfEdge()->End()->Position();
		const Eigen::Vector3d& pos3 = f->HalfEdge()->Next()->End()->Position();
		glVertex3dv(pos1.data());
		glVertex3dv(pos2.data());
		glVertex3dv(pos3.data());
	}
	glEnd();

	HEdgeList holeList = mesh.heholeList;
	glBegin(GL_LINES);
	glColor3f(0.0f, 0.0f, 0.0f);
	for (int i = 0; i < holeList.size(); i++) {
		glVertex3dv(mesh.heholeList[i]->Start()->Position().data());
		glVertex3dv(mesh.heholeList[i]->End()->Position().data());
	}
	glEnd();

	//VertexList vfhList = mesh.vfhList;
	//glPointSize(10.0);
	//glBegin(GL_POINTS);
	//glColor3f(0.0f, 0.0f, 0.0f);
	//for (i = 0; i < vfhList.size(); i++) {
	//	glVertex3dv(vfhList[i]->Position().data());
	//}
	//glEnd();
	//glPointSize(1.0);

	//// draw vertex normal
	//glBegin(GL_LINES);
	//glColor3f(1.0f, 1.0f, 0.0f);
	//for (int i = 0; i < bheList.size(); i++) {
	//	glVertex3dv(bheList[i]->Start()->Position().data());
	//	Eigen::Vector3d n = bheList[i]->Start()->Position() + bheList[i]->Start()->Normal() / 20.0;
	//	glVertex3dv(n.data());
	//}
	//glEnd();
}

// Hidden Line render function
void DrawHiddenLine() {
	FaceList fList = mesh.Faces();
	glShadeModel(GL_FLAT);
	glEnable(GL_POLYGON_OFFSET_FILL);
	glColor3f(0, 0, 0);
	glBegin(GL_TRIANGLES);
	for (size_t i = 0; i < fList.size(); i++) {
		Face* f = fList[i];
		const Eigen::Vector3d& pos1 = f->HalfEdge()->Start()->Position();
		const Eigen::Vector3d& pos2 = f->HalfEdge()->End()->Position();
		const Eigen::Vector3d& pos3 = f->HalfEdge()->Next()->End()->Position();
		glVertex3dv(pos1.data());
		glVertex3dv(pos2.data());
		glVertex3dv(pos3.data());
	}
	glEnd();
	glDisable(GL_POLYGON_OFFSET_FILL);

	DrawWireframe();
}

// Flat Shaded render function
void DrawFlatShaded() {
	FaceList fList = mesh.Faces();
	glShadeModel(GL_FLAT);
	glEnable(GL_LIGHTING);
	glColor3f(0.4f, 0.4f, 1.0f);
	glBegin(GL_TRIANGLES);
	for (size_t i = 0; i < fList.size(); i++) {
		Face* f = fList[i];
		const Eigen::Vector3d& pos1 = f->HalfEdge()->Start()->Position();
		const Eigen::Vector3d& pos2 = f->HalfEdge()->End()->Position();
		const Eigen::Vector3d& pos3 = f->HalfEdge()->Next()->End()->Position();
		Eigen::Vector3d normal = (pos2 - pos1).cross(pos3 - pos1);
		normal.normalize();
		glNormal3dv(normal.data());
		glVertex3dv(pos1.data());
		glVertex3dv(pos2.data());
		glVertex3dv(pos3.data());
	}
	glEnd();
	glDisable(GL_LIGHTING);
}

// Smooth Shaded render function
void DrawSmoothShaded() {
	FaceList fList = mesh.Faces();
	glShadeModel(GL_SMOOTH);
	glEnable(GL_LIGHTING);
	glColor3f(0.4f, 0.4f, 1.0f);
	glBegin(GL_TRIANGLES);
	for (size_t i = 0; i < fList.size(); i++) {
		Face* f = fList[i];
		Vertex* v1 = f->HalfEdge()->Start();
		Vertex* v2 = f->HalfEdge()->End();
		Vertex* v3 = f->HalfEdge()->Next()->End();

		glNormal3dv(v1->Normal().data());
		glVertex3dv(v1->Position().data());
		glNormal3dv(v2->Normal().data());
		glVertex3dv(v2->Position().data());
		glNormal3dv(v3->Normal().data());
		glVertex3dv(v3->Position().data());
	}
	glEnd();
	glDisable(GL_LIGHTING);
}

void DrawColorSmoothShaded() {
	FaceList fList = mesh.Faces();
	glShadeModel(GL_SMOOTH);
	glEnable(GL_LIGHTING);
	glColor3f(0.4f, 0.4f, 1.0f);
	glBegin(GL_TRIANGLES);
	for (size_t i = 0; i < fList.size(); i++) {
		Face* f = fList[i];
		Vertex* v1 = f->HalfEdge()->Start();
		Vertex* v2 = f->HalfEdge()->End();
		Vertex* v3 = f->HalfEdge()->Next()->End();
		glNormal3dv(v1->Normal().data());
		glColor3dv(v1->Color().data());
		glVertex3dv(v1->Position().data());
		glNormal3dv(v2->Normal().data());
		glColor3dv(v2->Color().data());
		glVertex3dv(v2->Position().data());
		glNormal3dv(v3->Normal().data());
		glColor3dv(v3->Color().data());
		glVertex3dv(v3->Position().data());
	}
	glEnd();
	glDisable(GL_LIGHTING);
}


// GLUT keyboard callback function
void KeyboardFunc(unsigned char ch, int x, int y) {
	switch (ch) {
	case '1':
		displayMode = WIREFRAME;
		mesh.GeneratePatchMesh();
		break;
	case '2':
		mesh.OptimizePatchMesh();
		break;
	case '3':
		mesh.GenerateImplicitSurface();
		break;
	case '4':
		mesh.ProjectVertex();
		break;
	case '5':
		mesh.MergeMesh();
		displayMode = SMOOTHSHADED;
		break;
	case 'd':
		mesh.initFillHole();
		break;
	case 'f':
		mesh.FillHoleStep();
		break;
	case '8':
		mesh.ComputeBoundaryVertexNormals();
		break;
	case '0':
		displayMode = SMOOTHSHADED;
		mesh.GeneratePatchMesh();
		mesh.OptimizePatchMesh();
		mesh.GenerateImplicitSurface();
		mesh.ProjectVertex();
		mesh.MergeMesh();
		break;
	case 27: // Esc char
		exit(0);
		break;
	}
	glutPostRedisplay();
}


// GLUT mouse callback function
void MouseFunc(int button, int state, int x, int y) {
	if (state == GLUT_DOWN) {
		downX = x;
		downY = y;
	}
	lastX = x;
	lastY = y;
	leftDown = (button == GLUT_LEFT_BUTTON) && (state == GLUT_DOWN);
	middleDown = (button == GLUT_MIDDLE_BUTTON) && (state == GLUT_DOWN);
	shiftDown = (glutGetModifiers() & GLUT_ACTIVE_SHIFT);
}


// GLUT mouse motion callback function
void MotionFunc(int x, int y) {
	if (leftDown) {
		if (!shiftDown) { // rotate
			sphi += (double)(x - lastX) / 4.0;
			stheta += (double)(lastY - y) / 4.0;
		}
		else { // pan with shift key
			xpan += (double)(x - lastX) * sdepth / zNear / winWidth;
			ypan += (double)(lastY - y) * sdepth / zNear / winHeight;
		}
	}
	// scale
	if (middleDown)
		sdepth += (double)(lastY - y) / 10.0;

	lastX = x;
	lastY = y;
	glutPostRedisplay();
}

void FillMeshHole() {
	displayMode = SMOOTHSHADED;
	mesh.GeneratePatchMesh();
	mesh.OptimizePatchMesh();
	mesh.GenerateImplicitSurface();
	mesh.ProjectVertex();
	mesh.MergeMesh();
}

// main function
void main(int argc, char** argv) {
	glutInit(&argc, argv);
	InitGL();
	InitMenu();
	if (argc >= 2)
		mesh.LoadObjFile(argv[1]);
	else
		InitGeometry();
	SetBoundaryBox(mesh.MinCoord(), mesh.MaxCoord());
	mesh.ComputeVertexNormals();

	//mesh.DisplayMeshInfo();

	std::cout << "Press 'right' button and Select 'Fill Hole'" << std::endl;
	std::cout << "OR Press '0' Key" << std::endl;

	glutMainLoop();
}
