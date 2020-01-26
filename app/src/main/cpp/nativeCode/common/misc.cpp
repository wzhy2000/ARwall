
#include "misc.h"

/**
 * Strip out the path and return just the filename
 */
std::string GetFileName(std::string fileName) {

    // assume filename is of the form "<path>/<name>.<type>"
    std::string::size_type slashIndex = fileName.find_last_of("/");

    std::string onlyName;
    if (slashIndex == std::string::npos) {
        onlyName = fileName.substr(0, std::string::npos);
    } else {
        onlyName = fileName.substr(slashIndex + 1, std::string::npos);
    }

    return onlyName;
}

/**
 * Extract only the directory part from the file name
 */
std::string GetDirectoryName(std::string fullFileName) {

    std::string::size_type slashIndex = fullFileName.find_last_of("/");
    std::string directoryName;
    if (slashIndex == std::string::npos) {
        directoryName = ".";
    } else if (slashIndex == 0) {
        directoryName = "/";
    } else {
        directoryName = fullFileName.substr(0, slashIndex);
    }
    return directoryName;
}

/**
 * Print the contents of a Glm 4x4 matrix
 */
void PrintGLMMat4(glm::mat4 testMat) {

    MyLOGD("%f %f %f %f", testMat[0][0],testMat[1][0],testMat[2][0],testMat[3][0]);
    MyLOGD("%f %f %f %f", testMat[0][1],testMat[1][1],testMat[2][1],testMat[3][1]);
    MyLOGD("%f %f %f %f", testMat[0][2],testMat[1][2],testMat[2][2],testMat[3][2]);
    MyLOGD("%f %f %f %f", testMat[0][3],testMat[1][3],testMat[2][3],testMat[3][3]);

}

/**
 * Print opencv mat as a string (mainly for android)
 */
void PrintCVMat(cv::Mat matToPrint) {

    std::ostringstream ss;
    ss << matToPrint;
    std::string matAsString(ss.str());
    MyLOGD("%s", matAsString.c_str());

}


/**
 * convert vector of keypoint to vector of Point2f
 */
std::vector<cv::Point2f> Keypoint2Point(std::vector<cv::KeyPoint> keypoints)
{
    std::vector<cv::Point2f> vectorOfPoints;
    for(unsigned i = 0; i < keypoints.size(); i++) {

        vectorOfPoints.push_back(keypoints[i].pt);

    }
    return vectorOfPoints;
}

/**
 * calculate a rectangle corresponding to reference frame used to compute homography
 */
void CalcShiftedCorners(cv::Mat homography, int nWidth, int nHeight, float* vertices){

    //-- Get the corners from the image_1 ( the object to be "detected" )
    std::vector< cv::Point2f > imageCorners(4);
    imageCorners[0] = cv::Point2f(0, 0);
    imageCorners[1] = cv::Point2f(nWidth, 0 );
    imageCorners[2] = cv::Point2f(nWidth, nHeight );
    imageCorners[3] = cv::Point2f(0, nHeight );
    std::vector< cv::Point2f > sceneCorners(4);

    cv::perspectiveTransform(imageCorners, sceneCorners, homography);

    vertices[0] = (float)sceneCorners[3].x;
    vertices[1] = (float)sceneCorners[3].y;
    vertices[2] = (float)sceneCorners[0].x;
    vertices[3] = (float)sceneCorners[0].y;
    vertices[4] = (float)sceneCorners[2].x;
    vertices[5] = (float)sceneCorners[2].y;
    vertices[6] = (float)sceneCorners[1].x;
    vertices[7] = (float)sceneCorners[1].y;
}
