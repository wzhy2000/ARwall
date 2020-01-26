
#ifndef TEXT_PICTURE_H
#define TEXT_PICTURE_H

#include "myGLM.h"
#include "myGLFunctions.h"
#include <opencv2/opencv.hpp>

class TextPicture{
public:
    TextPicture(int width, int height);
    void Render(glm::mat4 *mvpMat, float* verticesPixel, float ratio);
    bool Load(std::string fileImage);
    int  GetWidth(){return width;}
    int  GetHeight(){return height;}

private:
    int width, height;

    bool isObjectLoaded;
    GLuint  textureNameInGL;
    GLuint  shaderProgramID;
    GLint   dspTextureInShader;
    GLint   mvpMatInShader;
    glm::mat4 transform;
};
#endif //TEXT_PICTURE_H
