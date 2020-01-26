// shader associated with BackTexture

attribute   vec2 vertexPosition;
varying     vec2 textureCoords;
const float EPS = 10e-3;

void main()
{
    // Z coordinate of gl_Position is chosen so that image lies at the back
    gl_Position     = vec4(vertexPosition.xy, 1.0 - EPS, 1.0);

    // vertexPosition range is [-1,1]. Convert to range [0,1] for reading texture
    textureCoords   = (vertexPosition + 1.) * 0.5;
}
