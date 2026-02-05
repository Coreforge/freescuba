#include <GL/glew.h>
#include <GL/glext.h>
#include <cstddef>
#include "FBO.hpp"

FBO::FBO(){
    glGenFramebuffers(1, &fboid);
    glGenRenderbuffers(RBO_COUNT, rbos);
    glGenTextures(1, &colourTex);

    setResolution(256, 256);    // dummy values
    bind(GL_FRAMEBUFFER);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, colourTex, 0);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, rbos[RBO_DEPTH]);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void FBO::bind(GLenum target) const{
    glBindFramebuffer(target, fboid);
}

void FBO::unbind(GLenum target) const{
    glBindFramebuffer(target, 0);
}

void FBO::setResolution(GLuint x, GLuint y){
    // set up storage for the renderbuffers/textures
    glBindTexture(GL_TEXTURE_2D, colourTex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, x, y, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glBindTexture(GL_TEXTURE_2D, 0);

    glBindRenderbuffer(GL_RENDERBUFFER, rbos[RBO_DEPTH]);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, x, y);
}

GLuint FBO::getColourBufferID(){
    return colourTex;
}

FBO::~FBO(){
    glDeleteTextures(1, &colourTex);
    glDeleteRenderbuffers(RBO_COUNT, rbos);
    glDeleteFramebuffers(1, &fboid);
}