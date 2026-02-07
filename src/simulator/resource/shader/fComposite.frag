#version 330 core

in vec2 vUv;

out vec4 FragColor;

uniform sampler2D tTextColor;   
uniform sampler2D tMeshColor;  
uniform sampler2D tMeshDepth;  
uniform sampler2D tSpriteColor;  
uniform sampler2D tSpriteDepth;  

uniform vec4 resolution;     

float linearizeDepth(float d, float near, float far) {
    float z = d * 2.0 - 1.0; // Back to NDC
    return (2.0 * near * far) / (far + near - z * (far - near));
}

void main() {

    float nearPlane = 50.0f;
    float farPlane = 800.0f;

    float meshDepthRaw = texture(tMeshDepth, vUv).r;
    float spriteDepthRaw = texture(tSpriteDepth, vUv).r;
    float meshDepth = linearizeDepth(meshDepthRaw, nearPlane, farPlane);
    float spriteDepth = linearizeDepth(spriteDepthRaw, nearPlane, farPlane);

    //vec4 baseColor = (spriteDepth < meshDepth) ? texture(tSpriteColor, vUv) : texture(tMeshColor, vUv);
    vec4 baseColor = (meshDepth < spriteDepth) ? texture(tMeshColor, vUv) : texture(tSpriteColor, vUv);
    vec4 textColor = texture(tTextColor, vUv);
    FragColor = mix(baseColor, textColor, textColor.a);
}