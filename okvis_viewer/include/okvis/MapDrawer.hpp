#ifndef OKVIS_SOURCE_FILE_MAPDRAWER_H
#define OKVIS_SOURCE_FILE_MAPDRAWER_H

#include <pangolin/pangolin.h>
#include <opencv/cv.h>
#include <okvis/ThreadedKFVio.hpp>

#include <mutex>

namespace okvis
{

class MapDrawer
{
public:
    MapDrawer();

    void DrawMapPoints(const okvis::MapPointVector &mapPointVector);
    // void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph);
    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);
    pangolin::OpenGlMatrix GetCurrentOpenGLCameraMatrix(const okvis::kinematics::Transformation &T_WS);

private:

    float mPointSize;
    float mCameraSize;
    float mCameraLineWidth;
};

} //namespace okvis

#endif // OKVIS_SOURCE_FILE_MAPDRAWER_H
