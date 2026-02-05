#version 330 core
layout(location = 0) in vec4 vPosition;
layout(location = 1) in vec3 vNormal;

out vec4 gColor;
out vec3 gNormal;

uniform mat4 projectionMat;
uniform mat4 cameraMat;
uniform vec4 colour;

#define MAX_CUBES 16    // monado XRT_HAND_JOINT_COUNT
layout(std140) uniform TransformBlock{
    mat4 transformMat[MAX_CUBES];
} transforms;

void main(){ 
    vec4 pos = vPosition;
    mat4 transformMat = transforms.transformMat[gl_InstanceID];
    gl_Position = projectionMat * cameraMat * transformMat * pos;
    
    gColor = colour;
    gNormal = mat3(transformMat) * vNormal;
}