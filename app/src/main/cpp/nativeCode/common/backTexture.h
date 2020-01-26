
#ifndef BACKTEXTURE_H
#define BACKTEXTURE_H

#include "myGLFunctions.h"
#include <opencv2/opencv.hpp>

class BackTexture{
public:
    BackTexture(int width, int height);
    void Render();
    bool LoadBackImg(cv::Mat backImage);
    int  GetWidth(){return width;}
    int  GetHeight(){return height;}

private:
    int width, height;

    GLuint  vertexBuffer;
    GLuint  vertexAttribute;
    GLuint  shaderProgramID;
    GLint   textureSamplerLocation;
    GLuint  textureNameInGL;

};
#endif //BACKTEXTURE_H
