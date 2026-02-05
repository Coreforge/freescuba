#pragma once

#include "ShaderProgram.hpp"
#include "VAO.hpp"
#include "VBO.hpp"
#include "FBO.hpp"

#include <glm/ext/matrix_float4x4.hpp>
#include <glm/glm.hpp>

#include "../../ipc_protocol.hpp"

class HandView{
public:
    HandView(bool isLeft = true);

    // this only renders to the internal FBO
    void render();
    void setSize(int width, int height);
    void setZoom(float zoom);
    void setRotation(float ang);
    void drawImGui();   // render and ImGui widgets
    GLuint getColourOutbufBufferID();   // quicker/easier for now, might be useful to move imgui code into here instead though

    void setGloveState(const protocol::ContactGloveState_t& state);

private:

    void setColour(glm::vec4 colour);
    void setCameraParams();
    void setCameraPose();

    void setTransforms();

    void updateViewMatrix();

    ShaderProgram cubeShader;
    VBO cubeVBO;
    VBO cubeTransformUBO;
    VAO cubeVAO;
    FBO fbo;

    int width, height;

    float zoom, rotation;
    bool isLeft;

    glm::mat4 projectiomMatrix;
    glm::mat4 viewMatrix;

    protocol::ContactGloveState_t gloveState;

};