#include "textPicture.h"
#include "myShader.h"

/**
 * Create a RGB image from camera's preview data and send it to native class
 */
TextPicture::TextPicture(int width, int height) {

    isObjectLoaded = false;

    // create an empty texture that will be filled with image to be displayed
    this->width = width;
    this->height = height;

    shaderProgramID         = LoadShaders("shaders/picture.vsh", "shaders/picture.fsh");
    dspTextureInShader      = GetUniformLocation(shaderProgramID, "dspTexture");
    mvpMatInShader          = GetUniformLocation(shaderProgramID, "mvpMat");

    // create fixed transformation matrix.
    transform = glm::translate(transform, glm::vec3(0.0f, 0.0f, 0.0f));
    transform = glm::rotate(transform, (GLfloat)0*50.f, glm::vec3(0,0,1.0f));

    CheckGLError("TextPicture::TextPicture");
}

/**
 * Load opencv image into the texture
 */
bool TextPicture::Load(std::string fileDisplay) {

    cv::Mat textureImage = cv::imread(fileDisplay);
    if (!textureImage.empty()) {
        // opencv reads textures in BGR format, change to RGB for GL
        cv::cvtColor(textureImage, textureImage, CV_BGR2RGB);
        // opencv reads image from top-left, while GL expects it from bottom-left
        // vertically flip the image
        cv::flip(textureImage, textureImage, 0);

        glGenTextures(1, &textureNameInGL);
        // bind the texture
        glBindTexture(GL_TEXTURE_2D, textureNameInGL);
        // specify linear filtering
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

        // load the OpenCV Mat into GLES
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, textureImage.cols,
                     textureImage.rows, 0, GL_RGB, GL_UNSIGNED_BYTE,
                     textureImage.data);
        glGenerateMipmap(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, 0);

        isObjectLoaded = true;
        CheckGLError("TextPicture::loadGLTexGen");
    }

    return true;
}


/**
 * Render a quad and send texture to shader
 * mpvMat is calculated for 3D model, not used now
 * Currently we draw picture using transformation, the model-view-project Matrix is fixed.
 */
void TextPicture::Render(glm::mat4 *mvpMat, float* verticesPixel, float ratio) {

    if (!isObjectLoaded) {
        return;
    }

    //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    //Activate shader
    glUseProgram(shaderProgramID);

    GLfloat verticesNorm[] = {-1.0f, 1.0f, -1.0f, -1.0f, 1.0f, 1.0f, 1.0f, -1.0f };
    verticesNorm[0] = verticesPixel[0]*ratio/width*2-1;
    verticesNorm[2] = verticesPixel[2]*ratio/width*2-1;
    verticesNorm[4] = verticesPixel[4]*ratio/width*2-1;
    verticesNorm[6] = verticesPixel[6]*ratio/width*2-1;
    verticesNorm[1] = verticesPixel[1]*ratio/height*2-1;
    verticesNorm[3] = verticesPixel[3]*ratio/height*2-1;
    verticesNorm[5] = verticesPixel[5]*ratio/height*2-1;
    verticesNorm[7] = verticesPixel[7]*ratio/height*2-1;

    GLfloat vertices[]={
            //positions                              //Texture Coords
            verticesNorm[0], verticesNorm[1], 0.0f,  0.0f, 1.0f, //top Left
            verticesNorm[6], verticesNorm[7], 0.0f,  1.0f, 0.0f, //bottom Right
            verticesNorm[2], verticesNorm[3], 0.0f,  0.0f, 0.0f, //bottom Left
            verticesNorm[4], verticesNorm[5], 0.0f,  1.0f, 1.0f, //top Right

    };
    GLuint indices[] = { // Note that we start from 0!
            2, 0, 1, //First Triangle
            0, 1, 3  //second Triangle
    };

    // create VAO, VBO, EBO for openGL
    GLuint VBO, VAO, EBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

    // position attributes
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5*sizeof(GLfloat), (GLvoid*)0);
    glEnableVertexAttribArray(0);
    // TexCoords attributes
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5*sizeof(GLfloat), (GLvoid*)(3*sizeof(GLfloat)));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);

    // bind textures using texture units
    glActiveTexture(GL_TEXTURE0);
    glBindTexture( GL_TEXTURE_2D, textureNameInGL);
    glUniform1i( dspTextureInShader, 0);

    //We don't draw picture in 3D mode
    //glUniformMatrix4fv(mvpMatInShader, 1, GL_FALSE, (const GLfloat *) mvpMat);
    //We draw text picture by transformation using the fixed mvpMatrix.
    glUniformMatrix4fv(mvpMatInShader, 1, GL_FALSE, glm::value_ptr(transform));

    // load vertices of the quad
    glBindVertexArray(VAO);
    // Draw the quad
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
    // unbind buffers
    glBindVertexArray(0);
    // unbind texture
    glBindTexture(GL_TEXTURE_2D, 0);

    // release all resources allocated in this loop
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteBuffers(1, &EBO);

    CheckGLError("TextPicture::Render");
}

