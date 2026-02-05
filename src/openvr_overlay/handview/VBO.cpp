#include <GL/glew.h>
#include <GL/glext.h>

#include "VBO.hpp"

VBO::VBO(){
    glGenBuffers(1, &id);
}

VBO::~VBO(){
    glDeleteBuffers(1, &id);
}

void VBO::bind(GLenum target) const{
    glBindBuffer(target, id);
}

void VBO::unbind(GLenum target) const{
    glBindBuffer(target, 0);
}

void VBO::bindIndexed(GLenum target, GLuint index) const{
    glBindBufferBase(target, index, id);
}

void VBO::setData(void* data, size_t size, GLenum usage){
    bind(GL_ARRAY_BUFFER);
    glBufferData(GL_ARRAY_BUFFER, size, data, usage);
    unbind(GL_ARRAY_BUFFER);
}