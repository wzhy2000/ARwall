#include <vector>
#include "myShader.h"
#include "simpleARClass.h"
#include <myJNIHelper.h>

/**
 * Class constructor
 */
SimpleARClass::SimpleARClass() {

    MyLOGD("SimpleARClass::SimpleARClass");
    initsDone           = false;
    debugIsOn           = false;
    reduceMarkShake     = true;
    trackingIsOn        = false;
    renderPicture       = false;
    pnpResultIsMatch    = false;
    newPnpResult        = false;
    previewScaleFactor  = 0.75; // camera image is downscaled to half its original size

    myBack = NULL;
    myTextPic = NULL;

    cornerDetector = cv::ORB::create(750); // choosing ORB detector with default parameters
    matcher        = cv::DescriptorMatcher::create("BruteForce-Hamming");

    gravityMutex.unlock();
    gravity.assign(3, 0);
    gravity[2] = 1.0f;

    translationVector       = cv::Mat::zeros(3,1,CV_32F);
    translationVectorCopy   = cv::Mat::zeros(3,1,CV_32F);
    rotationVector          = cv::Mat::zeros(3,1,CV_32F);
    rotationVectorCopy      = cv::Mat::zeros(3,1,CV_32F);

    for(int i=0;i<8;i++) matchedVertices[i] = 0.0f;
}

SimpleARClass::~SimpleARClass() {

    MyLOGD("SimpleARClass::SimpleARClass");
    if(myBack) {
        delete myBack;
        myBack = NULL;
    }
    if (myGLCamera) {
        delete myGLCamera;
    }

    if(myTextPic)
        delete myTextPic;
}

/**
 * Perform inits, create objects for detecting corners and rendering image
 */
void SimpleARClass::PerformGLInits() {

    MyLOGD("SimpleARClass::PerformGLInits");

    MyGLInits();

    // create MyGLCamera object and set default position for the object
    myGLCamera = new MyGLCamera(cameraFOV,0);

    myBack = new BackTexture(cameraPreviewWidth*previewScaleFactor,
                           cameraPreviewHeight*previewScaleFactor);

    // extract the OBJ and companion files from assets
    std::string objMarkFile, objDisplayFile;
    bool isMarksPresent =
            gHelperObject->ExtractAssetReturnFilename("marks/MK_0_1_0b.jpg", objMarkFile) &&
            gHelperObject->ExtractAssetReturnFilename("displays/display_b0.jpg", objDisplayFile);

    if( !isMarksPresent ) {
        MyLOGE("Mark file %s, %s does not exist!", objMarkFile.c_str(), objDisplayFile.c_str() );
        return;
    }

    LoadMarkFiles();

    myTextPic = new TextPicture(cameraPreviewWidth*previewScaleFactor,
                                cameraPreviewHeight*previewScaleFactor);

    myTextPic->Load(objDisplayFile);

    CheckGLError("SimpleARClass::PerformGLInits");
    newCameraImage = false;
    initsDone = true;
}

void SimpleARClass::LoadMarkFiles()
{
    std::string objMarkFile;
    gHelperObject->ExtractAssetReturnFilename("marks/MK_0_1_0b.jpg", objMarkFile);

    MyLOGI("Loading texture %s", objMarkFile.c_str());

    cv::Mat markImage = cv::imread(objMarkFile);
    if (!markImage.empty()) {

        // resize the camera preview image to a smaller size to speedup processing
        float xScaleFactor = cameraPreviewWidth*previewScaleFactor/markImage.cols;
        float yScaleFactor = cameraPreviewHeight*previewScaleFactor/markImage.rows;

        cv::resize(markImage,markImage, cv::Size(), xScaleFactor, yScaleFactor);

        // OpenCV image needs to be flipped for OpenGL
        cv::flip(markImage, markImage, 0);

        markImgWidth = markImage.cols;
        markImgHeight = markImage.rows;

        //Detect feature points and descriptors in reference image
        cornerDetector->detectAndCompute(markImage, cv::noArray(),
                                         referenceKeypoints, referenceDescriptors);

        MyLOGD("Number of feature points in mark file %d", (int) referenceKeypoints.size());

        if (referenceKeypoints.size() < MIN_KPS_IN_FRAME) {
            return;
        }

        trackingIsOn = true;
    }
}

/**
 * set the viewport, function is also called when user changes device orientation
 */
void SimpleARClass::SetViewport(int width, int height) {

    screenHeight = height;
    screenWidth = width;
    glViewport(0, 0, width, height);
    CheckGLError("Cube::SetViewport");

    myGLCamera->SetAspectRatio((float) 1280 / 720);
}

/**
 * Camera preview dimensions are saved -- used later to initialize BackTexture object
 */
void SimpleARClass::SetCameraParams(int cameraPreviewWidth, int cameraPreviewHeight,
                                    float cameraFOV) {

    this->cameraPreviewWidth = cameraPreviewWidth;
    this->cameraPreviewHeight = cameraPreviewHeight;
    this->cameraFOV = cameraFOV;
}

/**
 * Copy gravity vector from sensor into private variable
 */
void SimpleARClass::UpdateGravity(float gx, float gy, float gz) {

    gravityMutex.try_lock();
    gravity[0] = gx;
    gravity[1] = gy;
    gravity[2] = gz;
    gravityMutex.unlock();
    return;
}

void SimpleARClass::KeepGravityVector() {
    gravityMutex.lock();
    sourceGravityVector.x = gravity[0];
    sourceGravityVector.y = gravity[1];
    sourceGravityVector.z = gravity[2];
    gravityMutex.unlock();
}

/**
 * draw a red rectangle corresponding to reference frame used to compute homography
 */
void SimpleARClass::DrawFrameAlongShiftedCorners(float vertices[8]){

    std::vector< cv::Point2f > sceneCorners(4);
    sceneCorners[3].x = vertices[0];
    sceneCorners[3].y = vertices[1];
    sceneCorners[0].x = vertices[2];
    sceneCorners[0].y = vertices[3];
    sceneCorners[2].x = vertices[4];
    sceneCorners[2].y = vertices[5];
    sceneCorners[1].x = vertices[6];
    sceneCorners[1].y = vertices[7];

    //-- Draw lines between the corners (the mapped object in the scene - image_2 )
    cv::line(cameraImageForBack, sceneCorners[0], sceneCorners[1], cv::Scalar(255), 4 );
    cv::line(cameraImageForBack, sceneCorners[1], sceneCorners[2], cv::Scalar(255), 4 );
    cv::line(cameraImageForBack, sceneCorners[3], sceneCorners[0], cv::Scalar(255), 4 );
    cv::line(cameraImageForBack, sceneCorners[2], sceneCorners[3], cv::Scalar(255), 4 );
}

/**
 * Render to the display
 */
void SimpleARClass::Render() {

    // clear the screen
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // render the camera image as the Background texture
    cameraMutex.try_lock();
    if(newCameraImage) {
        myBack->LoadBackImg(cameraImageForBack);
    }
    newCameraImage = false;
    cameraMutex.unlock();
    myBack->Render();

    // render the Text Picture if we have tracked reference marker in query image
    if(trackingIsOn) {

        pnpMutex.try_lock();

        // new result has been generated,
        if(newPnpResult) {
            // new match results are available.
            if (pnpResultIsMatch) {

                // make a copy of pnp result, it will be retained till result is updated again
                translationVectorCopy = translationVector.clone();
                rotationVectorCopy = rotationVector.clone();

                // flip OpenCV results to be consistent with OpenGL's coordinate system
                translationVectorCopy.at<double>(2, 0) = -translationVectorCopy.at<double>(2, 0);
                rotationVectorCopy.at<double>(0, 0) = -rotationVectorCopy.at<double>(0, 0);
                rotationVectorCopy.at<double>(1, 0) = -rotationVectorCopy.at<double>(1, 0);

                // need to call render to draw display texture, Call TextPicture in this loop.
                renderPicture = true;

            } else {
                renderPicture = false;
            }

            //reset the status of new result
            newPnpResult = false;
        }
        pnpMutex.unlock();


        cv::Mat defaultModelPosition = cv::Mat::zeros(3,1,CV_64F);
        defaultModelPosition.at<double>(2,0) = -CAM_DISTANCE_FROM_WALL;
        myGLCamera->UpdateModelMat(translationVectorCopy, rotationVectorCopy, defaultModelPosition);

        gravityMutex.lock();
        glm::mat4 mvpMat = myGLCamera->GetMVPAlignedWithGravity(gravity);
        gravityMutex.unlock();

        if (renderPicture) {
             myTextPic->Render(&mvpMat, matchedVertices, 1.0);
        } else {
            //For debug
            //MyLOGD("***not rendering model***");
        }
    }

    CheckGLError("SimpleARClass::Render");

}

/**
 * Save camera image, detect feature points in it, highlight them, match them
 */
void SimpleARClass::ProcessCameraImage(cv::Mat cameraRGBImage) {

    cameraMutex.lock();

    cameraImageForBack = cameraRGBImage.clone();

    // resize the camera preview image to a smaller size to speedup processing
    cv::resize(cameraImageForBack,cameraImageForBack, cv::Size(),
               previewScaleFactor, previewScaleFactor);

    // OpenCV image needs to be flipped for OpenGL
    cv::flip(cameraImageForBack, cameraImageForBack, 0);

    // if the marks are loaded and need to match the camera preview.
    if(trackingIsOn) {

        // if enough feature points are detected in query image, then try to match them
        // else indicate to render loop that matching has failed for this frame
        if (MatchKeypointsInQueryImage()) {

            KeepGravityVector();
            TrackKeypointsAndUpdatePose();

        } else {
            pnpMutex.lock();
            newPnpResult = true;        // new frame was processed ...
            pnpResultIsMatch = false;   // ... but no match.
            pnpMutex.unlock();
        }
    }

    // simply highlight corners in the image
    // For debug, this will slow down the match and display
    if (debugIsOn)
        DetectAndHighlightCorners();

    newCameraImage = true; // indicate to Render() that a new image is available

    cameraMutex.unlock();
}

/**
 * Use the corner detector to find feature points and draw small circles around them
 */
void SimpleARClass::DetectAndHighlightCorners(){

    cornerDetector->detect(cameraImageForBack, keyPoints);

    for(int i=0;i<keyPoints.size();i++){
        cv::circle(cameraImageForBack, keyPoints[i].pt, 3, cv::Scalar(255,0,0));
    }
}

/**
 * Use OpenCV's feature detector to compute locations and descriptors of keypoints
 */
bool SimpleARClass::DetectKeypointsInReferenceImage() {

    //Detect feature points and descriptors in reference image
    cornerDetector->detectAndCompute(cameraImageForBack, cv::noArray(),
                                     referenceKeypoints, referenceDescriptors);
    MyLOGD("Number of feature points in source frame %d", (int)referenceKeypoints.size());

    if(referenceKeypoints.size() < MIN_KPS_IN_FRAME){
        return false;
    }

    // source gravity vector used to project keypoints on imaginary wall at certain depth
    gravityMutex.lock();
    sourceGravityVector.x = gravity[0];
    sourceGravityVector.y = gravity[1];
    sourceGravityVector.z = gravity[2];
    gravityMutex.unlock();

    return true;
}

/**
 * Match keypoints in new image with reference frame. Compute homography to determine inliers
 */
bool SimpleARClass::MatchKeypointsInQueryImage() {

    std::vector<cv::KeyPoint> queryKeypoints;
    cv::Mat queryDescriptors;

    // compute keypoints and their descriptors in the query image
    cameraMutex.lock();
    cornerDetector->detectAndCompute(cameraImageForBack, cv::noArray(), queryKeypoints,
                                     queryDescriptors);
    cameraMutex.unlock();

    MyLOGD("Number of kps in query frame %d", (int) queryKeypoints.size());
    if (queryKeypoints.size() == 0) {
        MyLOGD("Not enough feature points in query image");
        return false;
    }

    std::vector<std::vector<cv::DMatch> > descriptorMatches;
    std::vector<cv::KeyPoint> sourceMatches, queryMatches;
    // knn-match with k = 2
    matcher->knnMatch(referenceDescriptors, queryDescriptors, descriptorMatches, 2);

    // save matches within a certain distance threshold
    for (unsigned i = 0; i < descriptorMatches.size(); i++) {
        if (descriptorMatches[i][0].distance < NN_MATCH_RATIO * descriptorMatches[i][1].distance) {
            sourceMatches.push_back(referenceKeypoints[descriptorMatches[i][0].queryIdx]);
            queryMatches.push_back(queryKeypoints[descriptorMatches[i][0].trainIdx]);
        }
    }
    MyLOGD("Number of kps whose descriptors match = %d", (int) sourceMatches.size());

    // compute homography to further prune outlier keypoints
    cv::Mat homography, inlierMask;
    if (sourceMatches.size() >= MIN_INLIER_COUNT) {
        homography = cv::findHomography(Keypoint2Point(sourceMatches),
                                        Keypoint2Point(queryMatches),
                                        cv::RANSAC, RANSAC_THRESH, inlierMask);
    } else {
        MyLOGD("Very few kps match, cannot proceed further!, sourceMatches=%d", (int) sourceMatches.size());
        return false;
    }

    if (homography.empty()) {
        MyLOGD("Could not determine homography!");
        return false;
    }

    sourceInlierKeypoints.clear();
    queryInlierKeypoints.clear();
    // retain inliers after computing homography
    for (unsigned i = 0; i < sourceMatches.size(); i++) {
        if (inlierMask.at<uchar>(i)) {
            sourceInlierKeypoints.push_back(sourceMatches[i]);
            queryInlierKeypoints.push_back(queryMatches[i]);
        }
    }
    MyLOGD("Number of kps match after homography filter = %d", (int) sourceInlierKeypoints.size());

    if (sourceInlierKeypoints.size() < MIN_INLIER_COUNT) {
        MyLOGD("Not enough kps match after homography filter!");
        return false;
    }
    MyLOGD("Success, Number of keypoint matches = %d", (int)sourceInlierKeypoints.size());

    // draw blue bigger circles in current image for the matched key points
    if(debugIsOn)
       DrawMatchedKeypoints( queryInlierKeypoints);

    // matched vertices are calculated for the text picture positioning.
    float newMatchVertices[8]={0};
    CalcShiftedCorners( homography, markImgWidth, markImgHeight, newMatchVertices);

    // in order to reduce the shaking of text picture, we ignore the slight movements.
    if (!reduceMarkShake)
       for(int i=0;i<8;i++) matchedVertices[i] = newMatchVertices[i];
    else {
        float deltPos = 0.0f, absDelt = 0.0f;
        for (int i = 0; i < 8; i++) {
            deltPos += newMatchVertices[i] - matchedVertices[i];
            absDelt += std::abs(newMatchVertices[i] - matchedVertices[i]);
        }

        // We can ignore the movements caused by shaking.
        if (deltPos > 8*1 || absDelt > 8 * 15)
            for (int i = 0; i < 8; i++) matchedVertices[i] = newMatchVertices[i];
    }

    // draw a rectangle marking reference frame in current image
    if (debugIsOn)
        DrawFrameAlongShiftedCorners(matchedVertices);

    return true;
}

void SimpleARClass::DrawMatchedKeypoints(std::vector<cv::KeyPoint> keyPoints){

    for(int i=0;i<keyPoints.size();i++){
        cv::circle(cameraImageForBack, keyPoints[i].pt, 6, cv::Scalar(0,0,255), 3);
    }
}
/**
 * Call the pnp-based solver to estimate new pose
 */
void SimpleARClass::TrackKeypointsAndUpdatePose() {

    // Use inliers as reference points
    sourceInlierPoints  = Keypoint2Point(sourceInlierKeypoints);
    queryInlierPoints   = Keypoint2Point(queryInlierKeypoints);
    int num_reference_pts = sourceInlierKeypoints.size();

    // Project inlier points onto an imaginary wall to get their 3D locations
    sourceKeypointLocationsIn3D.clear();
    sourceKeypointLocationsIn3D = myGLCamera->GetProjectedPointsOnWall(sourceInlierPoints,
                                                                        sourceGravityVector,
                                                                        CAM_DISTANCE_FROM_WALL,
                                                                        cameraImageForBack.cols,
                                                                        cameraImageForBack.rows);

    // construct the camera intrinsic matrix
    cv::Mat cameraMatrix = myGLCamera->ConstructCameraIntrinsicMatForCV(cameraImageForBack.cols,
                                                                        cameraImageForBack.rows);


    // estimate pose of query frame with solvepnp
    std::vector<float> distCoeffs; //null vector
    pnpMutex.lock();
    pnpResultIsMatch = false;
    pnpResultIsMatch = cv::solvePnP(sourceKeypointLocationsIn3D, queryInlierPoints,
                                    cameraMatrix, distCoeffs,
                                    rotationVector, translationVector);
    // if pnpResultIsMatch is true, new rotation and translation vector are calculated,
    // currently we draw Text Picture using transformation, not using normal mapping
    // so these two vectors are not used NOW!
    newPnpResult = true; // indicate to render loop that a new result is available
    pnpMutex.unlock();

    if(!pnpResultIsMatch) {
        MyLOGD("No solution to pnp!");
    }
    else{
        PrintCVMat(translationVector.t());
        PrintCVMat(rotationVector.t());
    }
    return;
}
