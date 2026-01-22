#version 330 core

layout(location = 0) in vec3 vertex;
layout(location = 1) in vec3 normal; 
layout(location = 2) in mat4 instance;
layout(location = 6) in vec3 color;

out vec3 FragPos;
out vec3 Normal;
out vec3 Color;

uniform mat4 view;
uniform mat4 projection;

void main() {
    gl_Position = projection * view * instance * vec4(vertex, 1.0);

    FragPos = vec3(instance * vec4(vertex, 1.0));
    Normal = mat3(instance) * normal;
    Color = color;
}