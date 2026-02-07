#version 330 core

in vec2 vUv;

out vec4 FragColor;

uniform sampler2D tColor;

void main() {
    FragColor = texture(tColor, vUv);
}