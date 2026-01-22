#version 330 core

in vec2 TexCoord;

out vec4 FragColor;

uniform sampler2D tSprite;

void main() {
	vec4 texColor = texture2D(tSprite, TexCoord);
	if(texColor.a < 0.1)
        discard;
	FragColor = texColor;

	//make color be affected by light
}