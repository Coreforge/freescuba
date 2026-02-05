#pragma once


#include <GL/gl.h>
#include <cstddef>

class FBO{
public: 
    FBO();
    FBO(FBO& other) = delete;
    ~FBO();

    void bind(GLenum target) const;
    void unbind(GLenum target) const;
    void setResolution(GLuint x, GLuint y);
    GLuint getColourBufferID();


private:
    GLuint fboid;

    enum RBO_ID{
        RBO_DEPTH,

        RBO_COUNT
    };
    GLuint rbos[RBO_COUNT];
    GLuint colourTex;
};