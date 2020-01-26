#ifndef SIMPLEARCLASS_H
#define SIMPLEARCLASS_H

#include "myLogger.h"
#include "myGLFunctions.h"
#include "myGLCamera.h"
#include "textPicture.h"
#include "backTexture.h"
#include <sstream>
#include <iostream>
#include <stdio.h>
#include <string>
#include <mutex>

#define MIN_KPS_IN_FRAME            300     // need to detect at least these keypoints in reference image
#define MIN_INLIER_COUNT            15      // should have at least these many matches
#define CAM_DISTANCE_FROM_WALL      75      // assumed distance between device and wall
#define NN_MATCH_RATIO              0.8f    // Nearest-neighbour matching ratio
#define RANSAC_THRESH               2.5f    // RANSAC inlier threshold for solvePnp

class SimpleARClass {
public:
    SimpleARClass();
    ~SimpleARClass();

    void    PerformGLInits();
    void    Render();
    void    SetViewport(int width, int height);
    void    ProcessCameraImage(cv::Mat cameraRGBImage);
    void    SetCameraParams(int cameraPreviewWidth, int cameraPreviewHeight, float cameraFOV);
    void    UpdateGravity(float gx, float gy, float gz);

private:
    void    DrawMatchedKeypoints(std::vector<cv::KeyPoint> keyPoints);
    void    DetectAndHighlightCorners();
    bool    DetectKeypointsInReferenceImage();
    bool    MatchKeypointsInQueryImage();
    void    TrackKeypointsAndUpdatePose();
    void    LoadMarkFiles();
    void    KeepGravityVector();
    void    DrawFrameAlongShiftedCorners(float vertices[8]);

    BackTexture * myBack;
    MyGLCamera  * myGLCamera;
    TextPicture * myTextPic;

    cv::Ptr<cv::DescriptorMatcher> matcher;
    cv::Mat referenceDescriptors;
    std::vector<cv::KeyPoint> referenceKeypoints;
    cv::Mat cameraImageForBack;

    int     screenWidth, screenHeight;
    int     cameraPreviewWidth, cameraPreviewHeight;
    int     markImgWidth,markImgHeight;
    bool    debugIsOn;
    bool    reduceMarkShake;
    bool    initsDone;
    bool    newCameraImage;
    bool    trackingIsOn;
    bool    renderPicture;
    bool    pnpResultIsMatch;
    bool    newPnpResult;
    float   previewScaleFactor;
    float   cameraFOV;

    std::mutex  cameraMutex;
    std::mutex  pnpMutex;
    std::mutex  gravityMutex;

    cv::Ptr<cv::Feature2D> cornerDetector;
    std::vector<cv::KeyPoint> keyPoints, sourceInlierKeypoints, queryInlierKeypoints;
    std::vector<cv::Point2f> sourceInlierPoints, queryInlierPoints;
    std::vector<cv::Point3f> sourceKeypointLocationsIn3D;
    cv::Mat translationVector, rotationVector;
    cv::Mat translationVectorCopy, rotationVectorCopy;
    float matchedVertices[8];

    std::vector <float> gravity;
    glm::vec3   sourceGravityVector;
};

#endif //SIMPLEARCLASS_H
