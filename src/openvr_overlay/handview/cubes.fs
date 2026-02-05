
#version 330 core
layout(location=0) out vec4 fColor;

in vec4 gColor;
in vec3 gNormal;

uniform mat4 projectionMat;
uniform mat4 cameraMat;
uniform vec4 colour;

void main() {
    vec3 lightDir = normalize(vec3(1, 1, 1));
    float lf = dot(gNormal, lightDir);
    lf = clamp(lf, 0.4, 1.0);
    fColor = gColor; 
    fColor.xyz *= lf;
}