#ifndef __GL_PROJECTOR_H__
#define __GL_PROJECTOR_H__

#ifdef WIN32
#include <windows.h>
#endif
#ifdef __APPLE__
#   include <OpenGL/gl.h>
#   include <GLUT/glut.h>
#else
#   include "GL/glut.h"                   // GLUT
#endif
#include "Eigen/Dense"

class GLProjector {
private:
    double modelView[16];
    double projection[16];
    int viewport[4];

public:
    double* ModelViewMatrix() {
        return modelView;
    }

    double* ProjectionMatrix() {
        return projection;
    }

    int* Viewport() {
        return viewport;
    }

    GLProjector() {
        glGetDoublev(GL_MODELVIEW_MATRIX, modelView);
        glGetDoublev(GL_PROJECTION_MATRIX, projection);
        glGetIntegerv(GL_VIEWPORT, viewport);
    }

    Eigen::Vector3d UnProject(double inX, double inY, double inZ) const {
        double x, y, z;
        gluUnProject(inX, inY, inZ, modelView, projection, viewport, &x, &y, &z);
        return Eigen::Vector3d(x, y, z);
    }

    Eigen::Vector3d UnProject(Eigen::Vector3d p) const {
        double x, y, z;
        gluUnProject(p(0), p(1), p(2), modelView, projection, viewport, &x, &y, &z);
        return Eigen::Vector3d(x, y, z);
    }

    Eigen::Vector3d Project(double inX, double inY, double inZ) const {
        double x, y, z;
        gluProject(inX, inY, inZ, modelView, projection, viewport, &x, &y, &z);
        return Eigen::Vector3d(x, y, z);
    }

    Eigen::Vector3d Project(Eigen::Vector3d p) const {
        double x, y, z;
        gluProject(p(0), p(1), p(2), modelView, projection, viewport, &x, &y, &z);
        return Eigen::Vector3d(x, y, z);
    }
};

#endif // __GL_PROJECTOR_H__
