#include <okvis/Viewer.hpp>
#include <glog/logging.h>
#include <mutex>

namespace okvis {

Viewer::Viewer(): mbFinishRequested(false) {
    mViewpointX = 0;
    mViewpointY = -0.7;
    mViewpointZ = -1.8;
    mViewpointF = 500;
    mpMapDrawer = std::make_shared<MapDrawer>();
}

void Viewer::run() {
    pangolin::CreateWindowAndBind("OKVIS: Map Viewer",1024,768);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000),
                pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
                );

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();

    cv::namedWindow("ORB-SLAM2: Current Frame");


    while(1)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
        {
            std::unique_lock<std::mutex> lock(mMutexT_WS);
            Twc = mpMapDrawer->GetCurrentOpenGLCameraMatrix(mT_WS);
        }

        {
            std::unique_lock<std::mutex> lock(mMutexLandMarks);
            mpMapDrawer->DrawMapPoints(mMapPointVector);
        }

        mpMapDrawer->DrawCurrentCamera(Twc);

        // s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
        s_cam.Follow(Twc);

        d_cam.Activate(s_cam);
        glClearColor(1.0f,1.0f,1.0f,1.0f);

        pangolin::FinishFrame();
        if(checkFinish())
            break;
    }
}

void Viewer::requestFinish() {
    std::unique_lock<std::mutex> lock(mMutexFinishRequested);
    mbFinishRequested = true;
}

bool Viewer::checkFinish() {
    std::unique_lock<std::mutex> lock(mMutexFinishRequested);
    return mbFinishRequested;
}

void Viewer::setT_WS(const okvis::Time &t, const okvis::kinematics:: Transformation &T_WS) {
    std::unique_lock<std::mutex> locheckFinishck(mMutexT_WS);
    mT_WS = T_WS;
}

void Viewer::setLandmarks(
    const okvis::Time &t, const okvis::MapPointVector &mapPointVector,
     const okvis::MapPointVector &transferredMapPointVector) {
        std::unique_lock<std::mutex> lock(mMutexLandMarks);
        mMapPointVector = mapPointVector;
}
    
} // namespace okvis