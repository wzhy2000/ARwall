// shader associated with BackTexture

precision mediump float; // required in GLSL ES 1.00
uniform sampler2D   textureSampler;
varying vec2        textureCoords;

void main()
{
	gl_FragColor.xyz = texture2D( textureSampler, textureCoords ).xyz;
}
