#version 330 core

layout(location = 0) in vec3 position; 
layout(location = 1) in vec2 vertex; 
layout(location = 2) in vec4 instance;
layout(location = 3) in vec2 spriteCoord;

out vec2 TexCoord;
out vec3 Normal;

uniform mat4 view;
uniform mat4 projection;
uniform vec2 spriteSheetGrid;

void main() {
    vec3 cameraRight = vec3(view[0][0], view[1][0], view[2][0]);
    vec3 cameraUp    = vec3(view[0][1], view[1][1], view[2][1]);
    vec3 worldPos = position + instance.xyz
                    + cameraRight * vertex.x * instance.w
                    + cameraUp * (vertex.y + 0.5) * instance.w;

    gl_Position = projection * view * vec4(worldPos, 1.0);

    vec2 spriteSizeUV = vec2(1.0) / spriteSheetGrid;
    vec2 uvBase = spriteCoord * spriteSizeUV;
    TexCoord = (vertex + vec2(0.5)) * spriteSizeUV + uvBase;
    Normal = vec3(1.0);
}