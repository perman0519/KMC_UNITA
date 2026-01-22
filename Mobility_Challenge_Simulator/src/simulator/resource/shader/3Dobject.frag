#version 330 core
in vec3 Normal;  
in vec3 FragPos;  
in vec3 Color;

out vec4 FragColor;
out vec4 FragNormal;

void main() {
    vec3 lightPos = vec3(0.0, 0.0, 50.0);
    vec3 lightColor = vec3(1.0, 1.0, 1.0);
    float ambientStrength = 0.1;
    vec3 ambient = ambientStrength * lightColor;
    
    vec3 lightDir = normalize(lightPos - FragPos);
            
    float diff = max(dot(Normal, lightDir), 0.0);
    vec3 diffuse = diff * lightColor;
    vec3 result = (ambient + diffuse);
    
    FragColor = vec4(result, 1.0) * vec4(Color, 1.0);;
    FragNormal = vec4(Normal, 1.0);
}