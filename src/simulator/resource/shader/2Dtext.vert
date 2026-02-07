#version 330 core

layout (location = 0) in vec2 texCoord;
layout (location = 1) in vec4 uv;
layout (location = 2) in vec3 color;
layout (location = 3) in vec2 position;
layout (location = 4) in vec2 size;

out vec2 TexCoord;
out vec3 Color;

uniform mat4 projection;

void main() {
	vec2 worldPos = position + texCoord * size;
    gl_Position = projection * vec4(worldPos, 0.0, 1.0);

	TexCoord = vec2(
		mix(uv.x, uv.z, texCoord.x),  // mix(left, right, texCoord.x)
		mix(uv.y, uv.w, texCoord.y)   // mix(bottom, top, texCoord.y)
	);

	Color = color;
}