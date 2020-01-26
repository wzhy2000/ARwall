// shader associated with TextPicture

precision mediump float; // required in GLSL ES 1.00

varying vec2      textureCoords;
uniform sampler2D dspTexture;

void main()
{
    gl_FragColor.xyz = texture2D( dspTexture, textureCoords ).xyz;
}
