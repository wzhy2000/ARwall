// shader associated with TextPicture

attribute   vec3 position;
attribute   vec2 vertexUV;
varying     vec2 textureCoords;
uniform     mat4 mvpMat;

void main()
{
    gl_Position = mvpMat * vec4(position, 1.0);
    textureCoords = vertexUV;
}