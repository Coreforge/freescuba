#include <GL/glew.h>

#include "VAO.hpp"

VAO::VAO(){
    glGenVertexArrays(1, &id);
}

VAO::~VAO(){
    glDeleteVertexArrays(1, &id);
}

void VAO::bind(){
    glBindVertexArray(id);
}
void VAO::unbind(){
    glBindVertexArray(0);
}
void VAO::enableAttrib(GLuint index, bool enabled){
    bind();
    if(enabled)
        glEnableVertexAttribArray(index);
    else
        glDisableVertexAttribArray(index);
}

void VAO::setAttrib(GLuint index, GLint size, GLenum type, GLboolean normalized, GLsizei stride, const void* offset, const VBO& vbo){
    vbo.bind(GL_ARRAY_BUFFER);
    glVertexAttribPointer(index, size, type, normalized, stride, offset);
}
