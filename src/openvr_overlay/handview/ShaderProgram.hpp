#pragma once
#include "VBO.hpp"
#include <GL/gl.h>

#include <string>
#include <unordered_map>

class ShaderProgram{
public:
    ShaderProgram(){}
    ShaderProgram(ShaderProgram& other) = delete;
    bool compile(std::string vertex, std::string fragment);
    GLuint getUniformLocation(std::string name);
    void setUniformBlockBinding(VBO& buffer, GLuint index, std::string name);
    void use();

private:
    bool compileShader(std::string shader, GLuint glshader);
    GLuint id = 0;
    std::unordered_map<std::string, GLuint> uniforms;
    std::unordered_map<std::string, GLuint> uniformBlocks;
};