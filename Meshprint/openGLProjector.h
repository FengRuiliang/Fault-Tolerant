#ifndef __OPENGLPROJECTOR__
#define __OPENGLPROJECTOR__

#ifdef WIN32
	#include <windows.h>
#endif
#include <gl/glut.h>
#include <cstdlib>
#include "HE_mesh/Vec.h"

typedef trimesh::point point;
typedef trimesh::vec3  Vec3f;
typedef trimesh::vec4  Vec4f;

class OpenGLProjector 
{
private:
	double  modelView[16];
	double  projection[16];
	int	    viewport[4];
	float* depthBuffer = NULL;

public:
	double* ModelViewMatrix() { return modelView; }
	double* ProjectionMatrix() { return projection; }
	int* Viewport() { return viewport; }
	float* DepthBuffer() { return depthBuffer; }

	OpenGLProjector()
	{
		glGetDoublev(GL_MODELVIEW_MATRIX, modelView);
		glGetDoublev(GL_PROJECTION_MATRIX, projection);
		glGetIntegerv(GL_VIEWPORT, viewport);
		
		SafeDeletes(depthBuffer);
		unsigned int bufferSize = viewport[2] * viewport[3];
		depthBuffer = new float[bufferSize];

		glReadPixels(viewport[0], viewport[1], viewport[2], viewport[3], GL_DEPTH_COMPONENT, GL_FLOAT, depthBuffer);
	}
	~OpenGLProjector() { SafeDeletes(depthBuffer); }
	Vec3f UnProject(double inX, double inY, double inZ)
	{
		double x,y,z;
		gluUnProject(inX, inY, inZ, modelView, projection, viewport, &x, &y, &z);
		return Vec3f(x,y,z);
	}
	Vec3f UnProject(Vec3f p)
	{
		double x,y,z;
		gluUnProject((double)p.x(), (double)p.y(), (double)p.z(), modelView, projection, viewport, &x, &y, &z);
		return Vec3f(x,y,z);
	}
	Vec3f Project(double inX, double inY, double inZ)
	{
		double x,y,z;
		gluProject(inX, inY, inZ, modelView, projection, viewport, &x, &y, &z);
		return Vec3f(x,y,z);
	}
	Vec3f Project(Vec3f p)
	{
		double x,y,z;
		gluProject(p.x(), p.y(), p.z(), modelView, projection, viewport, &x, &y, &z);
		return Vec3f(x,y,z);
	}
	float GetDepthValue(int x, int y) {
		float depth;
		glReadPixels(x, y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);
		return depth;
	}
};

#endif // __OPENGLPROJECTOR__