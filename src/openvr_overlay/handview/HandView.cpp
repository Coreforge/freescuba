#include <GL/glew.h>
#include "HandView.hpp"
#include <GL/gl.h>
#include <GL/glext.h>
#include <cmath>
#include <cstring>
#include <fstream>
#include <glm/ext/matrix_clip_space.hpp>
#include <glm/ext/matrix_float4x4.hpp>
#include <glm/ext/matrix_transform.hpp>
#include <glm/ext/quaternion_geometric.hpp>
#include <glm/ext/vector_float3.hpp>
#include <glm/ext/vector_float4.hpp>
#include <glm/ext.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/trigonometric.hpp>
#include <iostream>
#include <sstream>

#include <imgui.h>
#include "../imgui_extensions.hpp"

#include "monado/u_hand_simulation.h"

#define JOINT_COUNT 26

std::string readFile(std::string path){
    std::ifstream stream(path);
    if(!stream.is_open()){
        std::cerr << "Failed to read file " << path << std::endl;
        return "";
    }
    std::stringstream buf;
    buf << stream.rdbuf();
    return buf.str();
}

// GL_TRIANGLES, without indices
GLfloat cubeVerts[] = {
    // bottom face
    -1.f, -1.f, -1.f,       0.f, 0.f, -1.f,
    1.f, -1.f, -1.f,       0.f, 0.f, -1.f,
    -1.f, 1.f, -1.f,       0.f, 0.f, -1.f,
    1.f, -1.f, -1.f,       0.f, 0.f, -1.f,
    -1.f, 1.f, -1.f,       0.f, 0.f, -1.f,
    1.f, 1.f, -1.f,       0.f, 0.f, -1.f,

    // top face
    -1.f, -1.f, 1.f,       0.f, 0.f, 1.f,
    1.f, -1.f, 1.f,       0.f, 0.f, 1.f,
    -1.f, 1.f, 1.f,       0.f, 0.f, 1.f,
    1.f, -1.f, 1.f,       0.f, 0.f, 1.f,
    -1.f, 1.f, 1.f,       0.f, 0.f, 1.f,
    1.f, 1.f, 1.f,       0.f, 0.f, 1.f,

    // left face
    -1.f, -1.f, -1.f,       -1.f, 0.f, 0.f,
    -1.f, -1.f, 1.f,       -1.f, 0.f, 0.f,
    -1.f, 1.f, -1.f,       -1.f, 0.f, 0.f,
    -1.f, 1.f, -1.f,       -1.f, 0.f, 0.f,
    -1.f, -1.f, 1.f,       -1.f, 0.f, 0.f,
    -1.f, 1.f, 1.f,       -1.f, 0.f, 0.f,

    // right face
    1.f, -1.f, -1.f,       1.f, 0.f, 0.f,
    1.f, -1.f, 1.f,       1.f, 0.f, 0.f,
    1.f, 1.f, -1.f,       1.f, 0.f, 0.f,
    1.f, 1.f, -1.f,       1.f, 0.f, 0.f,
    1.f, -1.f, 1.f,       1.f, 0.f, 0.f,
    1.f, 1.f, 1.f,       1.f, 0.f, 0.f,

    // back face
    -1.f, -1.f, -1.f,       0.f, -1.f, 0.f,
    -1.f, -1.f, 1.f,       0.f, -1.f, 0.f,
    1.f, -1.f,-1.f,       0.f, -1.f, 0.f,
    1.f, -1.f,-1.f,       0.f, -1.f, 0.f,
    -1.f, -1.f, 1.f,       0.f, -1.f, 0.f,
    1.f, -1.f,1.f,       0.f, -1.f, 0.f,

    // front face
    -1.f, 1.f, -1.f,       0.f, 1.f, 0.f,
    -1.f, 1.f, 1.f,       0.f, 1.f, 0.f,
    1.f, 1.f,-1.f,       0.f, 1.f, 0.f,
    1.f, 1.f,-1.f,       0.f, 1.f, 0.f,
    -1.f, 1.f, 1.f,       0.f, 1.f, 0.f,
    1.f, 1.f,1.f,       0.f, 1.f, 0.f,
};

HandView::HandView(bool isLeft) : isLeft(isLeft){
    std::string vs = readFile("cubes.vs");
    std::string fs = readFile("cubes.fs");

    cubeShader.compile(vs, fs);

    cubeVAO.bind();
    cubeVBO.setData(cubeVerts, sizeof(cubeVerts), GL_STATIC_DRAW);
    cubeVAO.setAttrib(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), 0, cubeVBO);
    cubeVAO.setAttrib(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat),
        (void*)(sizeof(GLfloat) * 3), cubeVBO);
    cubeVAO.enableAttrib(0, true);
    cubeVAO.enableAttrib(1, true);

    std::memset(&gloveState, 0, sizeof(gloveState));

    // set up initial parameters
    setColour(glm::vec4(0.8f, 0.f, 0.f, 1.f));
    projectiomMatrix = glm::perspective(glm::radians(70.f), 1.f, 0.001f, 10.f);
    setCameraParams();
    setCameraPose();
    setTransforms();
    cubeShader.setUniformBlockBinding(cubeTransformUBO, 0, "TransformBlock");
    cubeVAO.unbind();

    setSize(256, 256);
    zoom = 1.f;
    rotation = 0.f;
    updateViewMatrix();

}

void HandView::setCameraParams(){
    GLuint loc = cubeShader.getUniformLocation("projectionMat");
    if(loc == -1) return;
    cubeShader.use();
    glUniformMatrix4fv(loc, 1, GL_FALSE, (GLfloat*)&projectiomMatrix);
}

void HandView::setCameraPose(){
    GLuint loc = cubeShader.getUniformLocation("cameraMat");
    if(loc == -1) return;
    cubeShader.use();
    glUniformMatrix4fv(loc, 1, GL_FALSE, (GLfloat*)&viewMatrix);
}

void HandView::setColour(glm::vec4 colour){
    GLuint loc = cubeShader.getUniformLocation("colour");
    if(loc == -1) return;
    cubeShader.use();
    glUniform4fv(loc, 1, (GLfloat*)&colour);
}

void HandView::setTransforms(){

    u_hand_tracking_values simValues;
    std::memset(&simValues, 0, sizeof(simValues));
    #define DEG_TO_RAD(DEG) (DEG * M_PI / 180.)
    #define APPLY_FINGER(finger, fingerSrc) \
    simValues.finger.joint_curls[0] = gloveState.fingerSrc##Root * DEG_TO_RAD(5); \
    simValues.finger.joint_curls[1] = gloveState.fingerSrc##Root * M_PI_2; \
    simValues.finger.joint_curls[2] = (gloveState.fingerSrc##Tip * 0.75f + gloveState.fingerSrc##Root * 0.25f) * DEG_TO_RAD(80); \
    simValues.finger.joint_curls[3] = gloveState.fingerSrc##Tip * DEG_TO_RAD(80); \
    simValues.finger.splay = gloveState.fingerSrc##Splay * DEG_TO_RAD(15);

    // 1.859
    // 1.43
    // 0.572

    APPLY_FINGER(thumb, thumb)
    APPLY_FINGER(index, index)
    APPLY_FINGER(middle, middle)
    APPLY_FINGER(ring, ring)
    APPLY_FINGER(little, pinky)
    simValues.thumb.splay = gloveState.thumbBase * -DEG_TO_RAD(30);
    simValues.thumb.joint_curls[0] = gloveState.thumbSplay / 0.08f * DEG_TO_RAD(50);
    simValues.thumb.joint_curls[2] = simValues.thumb.joint_curls[3];
    #undef APPLY_FINGER
    #undef DEG_TO_RAD

    xrt_space_relation relation;
    relation.pose.position = glm::vec3(0.f);
    relation.pose.orientation = glm::quat(0.f, 0.f, 0.f, 1.f);

    xrt_hand_joint_set simOut;
    u_hand_sim_simulate_generic(&simValues, isLeft ? XRT_HAND_LEFT : XRT_HAND_RIGHT, &relation, &simOut);

    glm::mat4 jointTransforms[JOINT_COUNT];
    for(size_t i = 0; i < JOINT_COUNT; i++){
        const xrt_hand_joint_value& fingerVals = simOut.values.hand_joint_set_default[i];
        auto& mat = jointTransforms[i];
        mat = glm::mat4(1.f);
        glm::mat4 cubeMat = glm::mat4(1.f);
        cubeMat = glm::scale(cubeMat, glm::vec3(0.005f));
        mat = glm::translate(mat, glm::vec3(0.f, 0.f, 0.095f));
        mat = glm::translate(mat, fingerVals.relation.pose.position);
        mat = mat * glm::mat4_cast(fingerVals.relation.pose.orientation);
        mat = mat * cubeMat;
        /*std::cout << "Joint " << i << 
            " position " << 
            fingerVals.relation.pose.position.x << " " <<
            fingerVals.relation.pose.position.y << " " <<
            fingerVals.relation.pose.position.z << " " <<
            " orientation " << 
            fingerVals.relation.pose.orientation.x << " " <<
            fingerVals.relation.pose.orientation.y << " " <<
            fingerVals.relation.pose.orientation.z << " " <<
            fingerVals.relation.pose.orientation.w << " " <<
            std::endl;*/
    }
    cubeTransformUBO.setData(jointTransforms, sizeof(jointTransforms), GL_DYNAMIC_DRAW);
}

void HandView::setSize(int width, int height){
    if(width != this->width || height != this->height){
        fbo.setResolution(width, height);
        this->width = width;
        this->height = height;
        projectiomMatrix = glm::perspective(glm::radians(70.f), width/(float)height, 0.001f, 10.f);
        setCameraParams();
    }
}

void HandView::render(){
    fbo.bind(GL_FRAMEBUFFER);

    glEnable(GL_DEPTH_TEST);
    glClearColor(0.1, 0.1, 0.1, 0.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glViewport(0, 0, width, height);

    cubeVAO.bind();
    cubeShader.use();
    updateViewMatrix();
    setCameraPose();
    setTransforms();
    cubeShader.setUniformBlockBinding(cubeTransformUBO, 0, "TransformBlock");
    glDrawArraysInstanced(GL_TRIANGLES, 0, 36, JOINT_COUNT);
    cubeVAO.unbind();

    glDisable(GL_DEPTH_TEST);
    fbo.unbind(GL_FRAMEBUFFER);
}

void HandView::updateViewMatrix(){
    float r = .3f / zoom;
    viewMatrix = glm::lookAt(
        glm::vec3(-cosf(-rotation + M_PI_2) * r, sinf(-rotation + M_PI_2) * r, 0.f),
        glm::vec3(0.f), 
        glm::vec3(0.f, 0.f, -1.f));
}

void HandView::drawImGui(){
    ImGui::BeginGroupPanel("Hand Preview");
    ImGui::SliderFloat("Rotation", &rotation, 0, M_PI * 2);
    ImGui::SliderFloat("Zoom", &zoom, 0.5f, 3.f);
    auto space = ImGui::GetContentRegionAvail();
    const int minWidth = 512;
    const int minHeight = 512;
    const float minAspect = 0.4f;
    const float maxAspect = 2.f;
    space.x = std::max((int)space.x, minWidth);
    space.y = std::max((int)space.y, minHeight);
    float aspect = space.x / space.y;
    if(aspect > maxAspect){
        // too wide
        // either make the viewport higher, or thinner. Thinner is safer, but higher probably looks better
        // just means the window can't be ultrawide without issues
        space.y = space.x / maxAspect;
    } else if(aspect < minAspect){
        // too thin
        // this probably means the window is too thin, make the viewport less tall
        space.y = space.x / minAspect;
    }
    setSize(space.x, space.y);

    render();
    ImGui::Image((ImTextureID)fbo.getColourBufferID(), ImVec2(space.x, space.y), {0,1}, {1,0});
    ImGui::EndGroupPanel();
}

void HandView::setZoom(float zoom){
    this->zoom = zoom;
}

void HandView::setRotation(float ang){
    this->rotation = ang;
}

GLuint HandView::getColourOutbufBufferID(){
    return fbo.getColourBufferID();
}

void HandView::setGloveState(const protocol::ContactGloveState_t& state){
    this->gloveState = state;
}