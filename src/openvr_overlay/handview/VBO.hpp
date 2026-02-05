#pragma once

#include <GL/gl.h>
#include <cstddef>

class VBO{
public: 
    VBO();
    VBO(VBO& other) = delete;
    ~VBO();

    void bind(GLenum target) const;
    void unbind(GLenum target) const;
    void bindIndexed(GLenum target, GLuint index) const;
    // binds the buffer to GL_ARRAY_BUFFER
    void setData(void* data, size_t size, GLenum usage);


private:
    GLuint id;
};