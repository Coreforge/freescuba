#include <GL/glew.h>
#include "ShaderProgram.hpp"
#include <GL/gl.h>
#include <GL/glext.h>
#include <cstddef>
#include <iostream>

bool ShaderProgram::compileShader(std::string shader, GLuint glshader){
    const char* code = shader.c_str();
    GLint size = shader.size();
    glShaderSource(glshader, 1, &code, &size);
    glCompileShader(glshader);
    GLint success;
    glGetShaderiv(glshader, GL_COMPILE_STATUS, &success);
    if(success == GL_TRUE){
        return true;
    }

    glGetShaderiv(glshader, GL_INFO_LOG_LENGTH, &success);
    std::string errmsg(success, ' ');
    glGetShaderInfoLog(glshader, errmsg.size(), NULL, errmsg.data());
    std::cerr << "Error compiling shader: " << std::endl << errmsg << std::endl;
    return false;
}

bool ShaderProgram::compile(std::string vertex, std::string fragment){
    GLuint shaders[2];
    shaders[0] = glCreateShader(GL_VERTEX_SHADER);
    shaders[1] = glCreateShader(GL_FRAGMENT_SHADER);
    bool cmpSuccess = true;
    cmpSuccess = compileShader(vertex, shaders[0]) && cmpSuccess;
    cmpSuccess = compileShader(fragment, shaders[1]) && cmpSuccess;
    if(cmpSuccess){
        id = glCreateProgram();
        glAttachShader(id, shaders[0]);
        glAttachShader(id, shaders[1]);
        glLinkProgram(id);

        GLint buf;
        glGetProgramiv(id, GL_LINK_STATUS, &buf);
        if(buf == GL_TRUE){
            glDeleteShader(shaders[0]);
            glDeleteShader(shaders[1]);
            return true;
        }

        glGetProgramiv(id, GL_INFO_LOG_LENGTH, &buf);
        std::string errmsg(buf, ' ');
        glGetProgramInfoLog(id, errmsg.size(), NULL, errmsg.data());
        std::cerr << "Error linking shader: " << std::endl << errmsg << std::endl;
        glDeleteProgram(id);
        id = 0;
    }
    glDeleteShader(shaders[0]);
    glDeleteShader(shaders[1]);
    return false;
}

GLuint ShaderProgram::getUniformLocation(std::string name){
    if(!uniforms.count(name)){
        uniforms[name] = glGetUniformLocation(id, name.c_str());
    }
    if(uniforms[name] == -1){
        std::cerr << "Unknown shader parameter: " << name << std::endl;
    }
    return uniforms[name];
}

void ShaderProgram::setUniformBlockBinding(VBO& buffer, GLuint index, std::string name){
    if(!uniformBlocks.count(name)){
        uniformBlocks[name] = glGetUniformBlockIndex(id, name.c_str());
    }
    GLuint ubi = uniformBlocks[name];
    if(ubi == -1){
        std::cerr << "Unknown uniform block: " << name << std::endl;
        return;
    }
    buffer.bindIndexed(GL_UNIFORM_BUFFER, index);
    glUniformBlockBinding(id, ubi, index);
}

void ShaderProgram::use(){
    if(id != 0){
        glUseProgram(id);
    }
}