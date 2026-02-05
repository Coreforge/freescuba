
#version 330 core
layout(location=0) out vec4 fColor;

in vec4 gColor;
in vec3 gNormal;

uniform mat4 projectionMat;
uniform mat4 cameraMat;
uniform vec4 colour;

void main() {
    fColor = gColor; 
}