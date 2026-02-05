#pragma once

#include <GL/gl.h>
#include <cstddef>

#include "VBO.hpp"

class VAO{
public: 
    VAO();
    VAO(VAO& other) = delete;
    ~VAO();

    void bind();
    void unbind();
    // binds the VAO
    void enableAttrib(GLuint index, bool enabled);
    // binds the VAO
    void setAttrib(GLuint index, GLint size, GLenum type, GLboolean normalized, GLsizei stride, const void* offset, const VBO& vbo);


private:
    GLuint id;
};