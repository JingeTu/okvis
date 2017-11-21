#include <okvis/MapDrawer.hpp>
#include <pangolin/pangolin.h>
#include <mutex>
#include <glog/logging.h>

namespace okvis
{

MapDrawer::MapDrawer()
{
    mPointSize = 2;
    mCameraSize = 0.08;
    mCameraLineWidth = 3;
}

void MapDrawer::DrawMapPoints(const okvis::MapPointVector &mapPointVector)
{
    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);

    for(size_t i=0, iend=mapPointVector.size(); i<iend;i++)
    {
        Eigen::Vector4d pos = mapPointVector[i].point;
        pos = pos/pos(3);
        glVertex3f(pos(0),pos(1),pos(2));
    }
    glEnd();
}

void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float &w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

    glLineWidth(mCameraLineWidth);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}

pangolin::OpenGlMatrix MapDrawer::GetCurrentOpenGLCameraMatrix(const okvis::kinematics::Transformation &T_WS)
{

    pangolin::OpenGlMatrix M;
    const Eigen::Matrix3d &Rwc = T_WS.C();
    const Eigen::Map<Eigen::Vector3d> &twc = T_WS.r();

    M.m[0] = Rwc(0,0);
    M.m[1] = Rwc(1,0);
    M.m[2] = Rwc(2,0);
    M.m[3]  = 0.0;

    M.m[4] = Rwc(0,1);
    M.m[5] = Rwc(1,1);
    M.m[6] = Rwc(2,1);
    M.m[7]  = 0.0;

    M.m[8] = Rwc(0,2);
    M.m[9] = Rwc(1,2);
    M.m[10] = Rwc(2,2);
    M.m[11]  = 0.0;

    M.m[12] = twc(0);
    M.m[13] = twc(1);
    M.m[14] = twc(2);
    M.m[15]  = 1.0;
    return M;
}

} //namespace okvis
