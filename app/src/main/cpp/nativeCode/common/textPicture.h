
#ifndef TEXT_PICTURE_H
#define TEXT_PICTURE_H

#include "myGLM.h"
#include "myGLFunctions.h"
#include <opencv2/opencv.hpp>

#define TRI_WIN_WIDTH   250
#define TRI_WIN_HEIGHT  250

class Triangle{
public:
    Triangle(int x, int y, int z)
    {
        this->x = x;
        this->y = y;
        this->z = z;
        this->draw =false;
    }
    int x;
    int y;
    int z;
    bool draw;
};
class TextPicture{
public:
    TextPicture(int screenWidth, int screenHeight, int markerWidth, int markerHeight);
    ~TextPicture();

    void Render(glm::mat4 *mvpMat);
    bool Load(std::string fileImage);
    bool MappingTrianglePoints(cv::Mat matHomography);

protected:
    void DetectTrianglePoints();
    void BuildTrianglePoints();
    unsigned int getOutsideBitMask(cv::Point2f ptx);

private:
    int screenWidth, screenHeight;
    int markerWidth, markerHeight;
    int imageWidth, imageHeight;
    std::vector< cv::Point2f > imageVertexes, screenVertexes, tempVertexes;
    std::vector< Triangle* > triangles;

    GLfloat* pVertices;
    GLuint* pIndices;
    int nTriangleShow;

    bool isObjectLoaded;
    GLuint  textureNameInGL;
    GLuint  shaderProgramID;
    GLint   dspTextureInShader;
    GLint   mvpMatInShader;
    glm::mat4 transform;
};
#endif //TEXT_PICTURE_H
