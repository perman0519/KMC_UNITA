#version 330 core

in vec2 TexCoord;
in vec3 Color;

out vec4 FragColor;

uniform sampler2D atlas;
//uniform vec4 bgColor;
//uniform vec4 fgColor;

const float screenPxRange = 4.5; 

float median(float r, float g, float b) {
    return max(min(r, g), min(max(r, g), b));
}

void main() {
    vec3 msd = texture(atlas, TexCoord).rgb;
    float sd = median(msd.r, msd.g, msd.b);
    float screenPxDistance = screenPxRange*(sd - 0.5);
    float opacity = clamp(screenPxDistance + 0.5, 0.0, 1.0);

    FragColor = vec4(Color, opacity);
}