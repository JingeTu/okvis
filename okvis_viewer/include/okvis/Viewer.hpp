#ifndef OKVIS_SOURCE_FILE_VIEWER_H
#define OKVIS_SOURCE_FILE_VIEWER_H

#include <pangolin/pangolin.h>

#include <memory>
#include <mutex>

#include <okvis/MapDrawer.hpp>
#include <okvis/ThreadedKFVio.hpp>

namespace okvis {

class Viewer { 
public:
    Viewer();
    void run();
    void setLandmarks(
        const okvis::Time &t, const okvis::MapPointVector &mapPointVector,
         const okvis::MapPointVector &transferredMapPointVector);
    void setT_WS(const okvis::Time &t,
         const okvis::kinematics:: Transformation &T_WS);
    void requestFinish();
private:
    float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;
    std::shared_ptr<MapDrawer> mpMapDrawer;
    
    okvis::MapPointVector mMapPointVector;
    std::mutex mMutexLandMarks;
    okvis::kinematics::Transformation mT_WS;
    std::mutex mMutexT_WS;

    bool checkFinish();
    bool mbFinishRequested;
    std::mutex mMutexFinishRequested;
};

} //namespace okvis

#endif // OKVIS_SOURCE_FILE_VIEWER_H