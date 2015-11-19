/*
 * Copyright (c) 2013, Dan Sledz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "opts.h"
#include "driver/emulator.h"

#include <future>
#include <mutex>

#include "TargetConditionals.h"
#if TARGET_OS_IPHONE || TARGET_IPHONE_SIMULATOR
#include <OpenGLES/ES2/gl.h>
#else
#include <OpenGL/OpenGL.h>
#include <OpenGL/gl.h>
#endif

using namespace EMU;

struct Vec3 {
    Vec3(void) = default;
    ~Vec3(void) = default;
    Vec3(float x, float y, float z): x(x), y(y), z(z) { }
    float x=0.0, y=0.0, z=0.0;
};

struct Vec2 {
    Vec2(void) = default;
    ~Vec2(void) = default;
    Vec2(float x, float y): x(x), y(y) { }
    float x=0.0, y=0.0;
};

struct TextureVertex {
    TextureVertex(void) = default;
    ~TextureVertex(void) = default;
    Vec3     vert;
    Vec2     tex;
};

template<class destructor>
struct ScopedObject
{
    ScopedObject(void): value(0)
    {
    }
    ScopedObject(GLuint value): value(value)
    {
        if (value == 0) {
            /* XXX: Throw an error */
        }
    }
    ScopedObject(const ScopedObject &obj) = delete;

    ~ScopedObject(void) {
        if (value != 0)
            destructor()(value);
    }

    const operator GLuint () const {
        return value;
    }

    GLuint release() {
        GLuint tmp = value;
        value = 0;
        return tmp;
    }

    GLuint value;
};

struct shader_cleanup {
    void operator()(GLuint value) {
        glDeleteShader(value);
    }
};
typedef ScopedObject<shader_cleanup> scoped_shader;

struct program_cleanup {
    void operator()(GLuint value) {
        glDeleteProgram(value);
    };
};
typedef ScopedObject<program_cleanup> scoped_program;

struct texture_cleanup {
    void operator()(GLuint value) {
        glDeleteTextures(1, &value);
    }
};
typedef ScopedObject<texture_cleanup> scoped_texture;

/* XXX: Grab GLError() */
struct OpenGLError: public CoreException
{
    OpenGLError(void): CoreException("OpenGL Error: ")
    {
        std::stringstream ss;
        ss << glGetError();
        msg += ss.str();
    }
};

/**
 * OpenGL Shader (Vertex or Fragment)
 */
class Shader
{
public:
    Shader(GLenum type, const std::string &source);
    ~Shader(void);

    void attach(GLuint program);

    const operator GLuint() const {
        return _shader;
    }

private:
    scoped_shader _shader;
    GLuint _program;
};

class ShaderProgram
{
public:
    ShaderProgram(void);
    ~ShaderProgram(void);

    void build(const std::string &frag_source, const std::string &vert_source,
               const std::vector<std::pair<std::string, GLuint> > &attribs);

    const operator GLuint() const {
        return _program;
    }

private:
    scoped_program _program;
};

class GfxTransform
{
public:
    GfxTransform(void) { }
    virtual ~GfxTransform(void) { }

    short width(void) {
        return _width;
    }

    short height(void) {
        return _height;
    }

    short pitch(void) {
        return _pitch;
    }

    void *fb(void) {
        return _fb.data();
    }

    virtual void resize(short width, short height) = 0;
    virtual void render(FrameBuffer *screen) = 0;

protected:
    void init_fb(short width, short height) {
        _width = width;
        _height = height;
        _pitch = _width * sizeof(uint32_t);
        _fb.resize(_pitch * _height);
    }

private:

    short _width;
    short _height;
    short _pitch;
    bvec _fb;
};

enum class GfxScale {
    None = 0,
    Scale2x = 1,
    Nearest2x = 2,
    Scaneline2x = 3,
};

typedef std::unique_ptr<GfxTransform> gfx_transform_ptr;
gfx_transform_ptr get_transform(GfxScale scale);

class GLSLFrameBuffer: public FrameBuffer
{
public:

    GLSLFrameBuffer(void);
    virtual ~GLSLFrameBuffer(void);

    virtual void resize(short width, short height);
    virtual void render(void);
    virtual void flip(void);

    void render_screen(float *modelview);

    void init(void);

private:

    std::unique_ptr<GfxTransform> _transform;
    GfxScale _scale;

    ShaderProgram _program;
    GLuint _screen_buffer;
    GLuint _var_Position;
    GLuint _var_TexCoordIn;
    GLuint _var_Texture;
    GLuint _var_Projection;
    GLuint _var_Modelview;
};

typedef std::lock_guard<std::mutex> mtx_lock;

#if OPENGL_LEGACY

class GLFrameBuffer: public FrameBuffer
{
public:

    GLFrameBuffer(void);
    virtual ~GLFrameBuffer(void);

    virtual void resize(short width, short height);
    virtual void render(void);
    virtual void flip(void);

    void init(void);
private:

    std::unique_ptr<GfxTransform> _transform;
    GfxScale _scale;
};
#endif

