#include "textPicture.h"
#include "myShader.h"
#include <math.h>

/**
 * Create a RGB image from camera's preview data and send it to native class
 */
TextPicture::TextPicture(int screenWidth, int screenHeight, int markerWidth, int markerHeight) {

    isObjectLoaded = false;
    nTriangleShow = 0;

    // create an empty texture that will be filled with image to be displayed
    this->screenWidth = screenWidth;
    this->screenHeight = screenHeight;

    this->markerWidth = markerWidth;
    this->markerHeight = markerHeight;

    this->imageWidth = 0;
    this->imageHeight = 0;

    shaderProgramID         = LoadShaders("shaders/picture.vsh", "shaders/picture.fsh");
    dspTextureInShader      = GetUniformLocation(shaderProgramID, "dspTexture");
    mvpMatInShader          = GetUniformLocation(shaderProgramID, "mvpMat");

    // create fixed transformation matrix.
    transform = glm::translate(transform, glm::vec3(0.0f, 0.0f, 0.0f));
    transform = glm::rotate(transform, (GLfloat)0*50.f, glm::vec3(0,0,1.0f));

    CheckGLError("TextPicture::TextPicture");
}

TextPicture::~TextPicture()
{
    if(pVertices) delete pVertices;
    if(pIndices) delete pIndices;
}

/**
 * Load opencv image into the texture
 */
bool TextPicture::Load(std::string fileDisplay) {

    cv::Mat textureImage = cv::imread(fileDisplay);
    if (!textureImage.empty()) {
        // opencv reads textures in BGR format, change to RGB for GL
        cv::cvtColor( textureImage, textureImage, CV_BGR2RGB );
        cv::resize( textureImage, textureImage, cv::Size(),
                    markerWidth*1.0/textureImage.cols,
                    markerHeight*1.0/textureImage.rows);
        // opencv reads image from top-left, while GL expects it from bottom-left
        // vertically flip the image
        cv::flip( textureImage, textureImage, 0);

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

        this->imageWidth = textureImage.cols;
        this->imageHeight = textureImage.rows;

        BuildTrianglePoints();

        isObjectLoaded = true;
        CheckGLError("TextPicture::loadGLTexGen");
    }

    return true;
}

void TextPicture::BuildTrianglePoints() {
    //Put the left-bottom as the 1st point
    imageVertexes.push_back(cv::Point2f(0.0, 0.0));
    //Put the left-top as the 2nd point
    for (int j=1; j<=ceil(imageHeight*1.0f / TRI_WIN_HEIGHT);j++)
        if(j*TRI_WIN_HEIGHT>=imageHeight)
            imageVertexes.push_back(cv::Point2f(0.0, imageHeight * 1.0));
        else
            imageVertexes.push_back(cv::Point2f(0.0, TRI_WIN_HEIGHT*j*1.0));

    int nVertexPerCol = imageVertexes.size();

    for (int i = 1; i <= ceil(imageWidth*1.0f / TRI_WIN_WIDTH); i++) {
        float x = TRI_WIN_WIDTH*1.0f * i;
        if (i == ceil(imageWidth / TRI_WIN_WIDTH))
            x = imageWidth * 1.0;

        imageVertexes.push_back(cv::Point2f(x, 0.0f));

        for (int j=1; j<=ceil(imageHeight*1.0f / TRI_WIN_HEIGHT);j++)
            if(j*TRI_WIN_HEIGHT>=imageHeight)
                imageVertexes.push_back(cv::Point2f(x, imageHeight * 1.0));
            else
                imageVertexes.push_back(cv::Point2f(x, TRI_WIN_HEIGHT*j*1.0));

        int offset = (i - 1) * nVertexPerCol;
        for( int k=0; k< (nVertexPerCol-1);k++) {
            triangles.push_back(new Triangle(0 + k+ offset, k+ 1 + offset, k + nVertexPerCol + offset));
            triangles.push_back(new Triangle(1 + k+ offset, k + nVertexPerCol + offset, k+ nVertexPerCol + 1 + offset));
        }
    }

    for (int i = 0; i < imageVertexes.size(); i++) {
        screenVertexes.push_back(cv::Point2f(-1.0, -1.0));
        tempVertexes.push_back(cv::Point2f(-1.0, -1.0));
    }

    pVertices = new GLfloat[(imageVertexes.size()+1)*5 ];
    pIndices = new GLuint[(ceil(imageVertexes.size()/2) +1 ) *3];
}


/**
 * Render a quad and send texture to shader
 * mpvMat is calculated for 3D model, not used now
 * Currently we draw picture using transformation, the model-view-project Matrix is fixed.
 */
void TextPicture::Render(glm::mat4 *mvpMat) {

    if (!isObjectLoaded) {
        return;
    }

    DetectTrianglePoints();

    if(nTriangleShow<=0)
        return;

    //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    //Activate shader
    glUseProgram(shaderProgramID);

    // create VAO, VBO, EBO for openGL
    GLuint VBO, VAO, EBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, imageVertexes.size()*5*sizeof(GLfloat), pVertices, GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, nTriangleShow*3*sizeof(GLuint), pIndices, GL_STATIC_DRAW);

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
    glDrawElements(GL_TRIANGLES, nTriangleShow*3, GL_UNSIGNED_INT, 0);
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

unsigned int TextPicture::getOutsideBitMask(cv::Point2f ptx)
{
    unsigned int nMask=0;
    if(ptx.x <= 0) nMask = 0x02;
    if(ptx.x >= screenWidth) nMask =0x08;
    if(ptx.y <= 0) nMask += 0x01;
    if(ptx.y >= screenHeight) nMask += 0x04;

    //abnormal position, ignore
    if(ptx.x > 20000 || ptx.x < -20000 ) nMask=0x10;
    if(ptx.y > 20000 || ptx.y < -20000 ) nMask=0x10;

    return nMask;
}

bool TextPicture::MappingTrianglePoints(cv::Mat matHomography) {

    cv::perspectiveTransform(imageVertexes, tempVertexes, matHomography);

    float xmin=0.0, xmax=0.0f, ymin=0.0f, ymax = 0.0f;
    xmin = xmax = tempVertexes[0].x;
    ymin = ymax = tempVertexes[0].y;

    for (unsigned int i = 1; i < tempVertexes.size(); i++) {
        if (tempVertexes[i].x > xmax) xmax = tempVertexes[i].x;
        if (tempVertexes[i].x < xmin) xmin = tempVertexes[i].x;
        if (tempVertexes[i].y > ymax) ymax = tempVertexes[i].y;
        if (tempVertexes[i].y < ymin) ymin = tempVertexes[i].y;
    }

    //the error in Homography make very strange transformation position.
    if (xmax - xmin <= 200 || ymax - ymin <= 200)
        return false;

    for (unsigned int i = 0; i < tempVertexes.size(); i++) {
        screenVertexes[i].x = tempVertexes[i].x;
        screenVertexes[i].y = tempVertexes[i].y;
    }

    return true;
}

void TextPicture::DetectTrianglePoints()
{
    nTriangleShow=0;
    for(int i=0; i<triangles.size();i++) {
        Triangle *pTri = triangles.at(i);
        unsigned int s1 = getOutsideBitMask(screenVertexes.at(pTri->x));
        unsigned int s2 = getOutsideBitMask(screenVertexes.at(pTri->y));
        unsigned int s3 = getOutsideBitMask(screenVertexes.at(pTri->z));

        if (s1 == 0x10 || s2 == 0x10 || s3 == 0x10 || (s1 & s2 & s3) != 0) {
            // in the same outside area, no need to draw, skip
            pTri->draw = false;
        } else {
            float x0=0,x1=0,y0=0,y1=0;
            x0 = screenVertexes.at(pTri->x).x - screenVertexes.at(pTri->y).x;
            x1 = screenVertexes.at(pTri->y).x - screenVertexes.at(pTri->z).x;
            y0 = screenVertexes.at(pTri->x).y - screenVertexes.at(pTri->y).y;
            y1 = screenVertexes.at(pTri->y).y - screenVertexes.at(pTri->z).y;

            if(fabs(x0)<5.0f && fabs(x1)<5.0f && fabs(y0)<=5.0f && fabs(y1)<=5.0f) {
                nTriangleShow = 0;
                return;
            }

            pTri->draw = true;
            pIndices[nTriangleShow*3] = pTri->x;
            pIndices[nTriangleShow*3+1] = pTri->y;
            pIndices[nTriangleShow*3+2] = pTri->z;
            nTriangleShow += 1;
        }
    }

    // positions                                 //Texture Coords
    //  verticesNorm[0], verticesNorm[1], 0.0f,  0.0f, 1.0f, //top Left
    //  verticesNorm[6], verticesNorm[7], 0.0f,  1.0f, 0.0f, //bottom Right
    //  verticesNorm[2], verticesNorm[3], 0.0f,  0.0f, 0.0f, //bottom Left
    //  verticesNorm[4], verticesNorm[5], 0.0f,  1.0f, 1.0f, //top Right
    if(nTriangleShow>0)
    for(int i=0;i<imageVertexes.size();i++) {
        pVertices[i*5+0] = screenVertexes[i].x/screenWidth*2-1.0;
        pVertices[i*5+1] = screenVertexes[i].y/screenHeight*2-1.0;
        pVertices[i*5+2] = 0.0f;
        pVertices[i*5+3] = imageVertexes[i].x*1.0f/imageWidth;
        pVertices[i*5+4] = imageVertexes[i].y*1.0f/imageHeight;
    };
}

