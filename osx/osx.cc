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

#include "machine/emulator.h"
#include "osx/osx.h"

#include <future>

#include <OpenGLES/ES2/gl.h>

using namespace EMU;

struct Vec3 {
    Vec3(void) = default;
    Vec3(float x, float y, float z): x(x), y(y), z(z) { }
    float x, y, z;
};

struct Vec2 {
    Vec2(void) = default;
    Vec2(float x, float y): x(x), y(y) { }
    float x, y;
};

struct TextureVertex {
    Vec3     vert;
    RGBColor color;
    Vec2     tex;
};

/* XXX: Our screen should be dynamic based on the game's screen. */
static TextureVertex gScreenData[] = {
    { {  0.5f,  0.5f, 1.0f }, { }, { 1.0f, 0.0f } },
    { { -0.5f,  0.5f, 1.0f }, { }, { 0.0f, 0.0f } },
    { {  0.5f, -0.5f, 1.0f }, { }, { 1.0f, 1.0f } },
    { {  0.5f, -0.5f, 1.0f }, { }, { 1.0f, 1.0f } },
    { { -0.5f,  0.5f, 1.0f }, { }, { 0.0f, 0.0f } },
    { { -0.5f, -0.5f, 1.0f }, { }, { 0.0f, 1.0f } },
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

struct OpenGLError: public EmuException
{
    OpenGLError(void): EmuException("OpenGL Error") { }
};

/**
 * Defines a opengl shader
 */
class Shader
{
public:
    Shader(GLenum type, const std::string &source):
        _shader(glCreateShader(type)),
        _program(0)
    {
        const GLchar *shader_source = source.c_str();

        glShaderSource(_shader, 1, &shader_source, NULL);
        glCompileShader(_shader);

        GLint length;
        glGetShaderiv(_shader, GL_INFO_LOG_LENGTH, &length);
        if (length > 0) {
            std::vector<char> log;
            log.resize(length);
            glGetShaderInfoLog(_shader, length, &length, log.data());
            std::cout << log.data();
        }

        GLint status;
        glGetShaderiv(_shader, GL_COMPILE_STATUS, &status);
        if (status == 0)
            throw OpenGLError();
    }

    ~Shader(void)
    {
        if (_shader != 0 && _program != 0)
            glDetachShader(_program, _shader);
    }

    void attach(GLuint program)
    {
        glAttachShader(program, _shader);
        _program = program;
    }

private:
    scoped_shader _shader;
    GLuint _program;
};

class ShaderProgram
{
public:
    ShaderProgram(void):
        _program(glCreateProgram())
    {
    }

    ~ShaderProgram(void)
    {
        /* XXX: delete program */
    }

    void build(const std::string &frag_source,
               const std::string &vert_source,
               const std::vector<std::pair<std::string, GLuint> > &attribs)
    {
        Shader frag(GL_FRAGMENT_SHADER, frag_source);
        Shader vert(GL_VERTEX_SHADER, vert_source);

        frag.attach(_program);
        vert.attach(_program);

        for (auto it = attribs.begin(); it != attribs.end(); it++)
            glBindAttribLocation(_program, it->second, it->first.c_str());

        glLinkProgram(_program);

        GLint status;
        glGetProgramiv(_program, GL_LINK_STATUS, &status);
        if (status == 0)
            throw OpenGLError();
    }

    const operator GLuint() const {
        return _program;
    }

private:
    scoped_program _program;
};

static std::string frag_source = R"(
varying lowp vec4 colorVarying;
varying lowp vec2 TexCoordOut;
uniform sampler2D Texture;

void main(void)
{
    gl_FragColor = texture2D(Texture, TexCoordOut);
})";

static std::string vert_source = R"(
attribute vec4 Position;
attribute vec2 TexCoordIn;

uniform mat4 Projection;
uniform mat4 Modelview;

varying lowp vec4 colorVarying;
varying vec2 TexCoordOut;

void main()
{
    colorVarying = vec4(1.0, 1.0, 1.0, 1.0);

    gl_Position = Projection * Modelview * Position;
    TexCoordOut = TexCoordIn;
})";

class OSXEmulator: public Emulator
{
public:
    OSXEmulator(const Options &options):
        Emulator(options)
    {
        glEnable(GL_BLEND);
        glEnable(GL_TEXTURE_2D);
        glEnable(GL_DEPTH_TEST);

        glClearColor(0.0,0.0,0.0,0.0);

        init_screen();
        init_shader();

        machine()->set_render(
            [&](RasterScreen *screen) {
                make_texture(screen);
            });
    }

    ~OSXEmulator(void)
    {
        /* XXX: Free _frame */
    }

    void init_screen(void)
    {
        RasterScreen *screen = machine()->screen();

        _width = screen->width;
        _height = screen->height;

        glGenBuffers(1, &_screen_buffer);
        glBindBuffer(GL_ARRAY_BUFFER, _screen_buffer);
        glBufferData(GL_ARRAY_BUFFER, sizeof(gScreenData), gScreenData,
                     GL_STATIC_DRAW);

        glGenTextures(1, &_frame);
        glBindTexture(GL_TEXTURE_2D, _frame);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    }

    void init_shader(void)
    {
        std::vector<std::pair<std::string, GLuint> > attribs;
        //attribs.push_back(std::make_pair("position", _var_Position));
        //attribs.push_back(std::make_pair("normal", _var_Normal));

        _program.build(frag_source, vert_source, attribs);

        _var_Position = glGetAttribLocation(_program, "Position");
        glEnableVertexAttribArray(_var_Position);

        _var_TexCoordIn = glGetAttribLocation(_program, "TexCoordIn");
        glEnableVertexAttribArray(_var_TexCoordIn);

        _var_Texture = glGetUniformLocation(_program, "Texture");
        _var_Modelview = glGetUniformLocation(_program, "Modelview");
        _var_Projection = glGetUniformLocation(_program, "Projection");
    }

    void render(float *modelview)
    {

        glUseProgram(_program);

        float projection[] = { 1.0, 0.0, 0.0, 0.0,
                               0.0, 1.0, 0.0, 0.0,
                               0.0, 0.0, 1.0, 0.0,
                               0.0, 0.0, 0.0, 1.0 };

        glUniformMatrix4fv(_var_Modelview, 1, 0, modelview);
        glUniformMatrix4fv(_var_Projection, 1, 0, projection);

        glBindBuffer(GL_ARRAY_BUFFER, _screen_buffer);
        glVertexAttribPointer(_var_Position, 3, GL_FLOAT, GL_FALSE,
            sizeof(TextureVertex), (void*)offsetof(TextureVertex, vert));
        glVertexAttribPointer(_var_TexCoordIn, 2, GL_FLOAT, GL_FALSE,
            sizeof(TextureVertex), (void*)offsetof(TextureVertex, tex));

        glBindTexture(GL_TEXTURE_2D, _frame);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, _width, _height,
                     0, GL_RGBA, GL_UNSIGNED_BYTE, _data.data());
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, _frame);
        glUniform1i(_var_Texture, 0);

        glDrawArrays(GL_TRIANGLES, 0, 6);
    }

    void start(void)
    {
        task = std::async(std::launch::async, &OSXEmulator::task_fn, this);
    }

    void handle_event(InputKey key, bool pressed)
    {
        InputMap *map = machine()->input();
        if (pressed)
            map->depress(key);
        else
            map->release(key);
    }

    void end(void)
    {

    }

    void make_texture(RasterScreen *screen)
    {
        _data = screen->data;
    }


private:

    short _width;
    short _height;
    std::vector<RGBColor> _data;

    void task_fn(void)
    {
        Emulator::start();
    }

    std::future<void> task;
    GLuint _frame;

    ShaderProgram _program;
    GLuint _screen_buffer;
    GLuint _var_Position;
    GLuint _var_TexCoordIn;
    GLuint _var_Texture;
    GLuint _var_Projection;
    GLuint _var_Modelview;
};

struct emu_state
{
    OSXEmulator * emulator;
};

struct emu_state *
emu_load(const struct emu_config *config)
{
    struct emu_state *state = NULL;
    try {
        Options opts;
        if (config->driver != NULL)
            opts.driver = config->driver;
        if (config->rom != NULL)
            opts.rom = config->rom;

        state = new emu_state;

        state->emulator = new OSXEmulator(opts);

    } catch (...) {
        if (state != NULL) {
            delete state;
            state = NULL;
        }
        /* XXX: logging */
    }
    return state;
}

int
emu_start(struct emu_state *emu)
{
    emu->emulator->start();

    return 0;
}

int
emu_end(struct emu_state *emu)
{
    emu->emulator->stop();

    delete emu;

    return 0;
}

int
emu_render(struct emu_state *emu, float *modelview)
{
    emu->emulator->render(modelview);

    return 0;
}

int
emu_send_event(struct emu_state *emu, const struct emu_event *event)
{
    switch (event->key) {
    case EMU_KEY_JOY1UP:
        emu->emulator->handle_event(InputKey::Joy1Up, event->pressed);
        break;
    case EMU_KEY_JOY1DOWN:
        emu->emulator->handle_event(InputKey::Joy1Down, event->pressed);
        break;
    case EMU_KEY_JOY1LEFT:
        emu->emulator->handle_event(InputKey::Joy1Left, event->pressed);
        break;
    case EMU_KEY_JOY1RIGHT:
        emu->emulator->handle_event(InputKey::Joy1Right, event->pressed);
        break;
    case EMU_KEY_JOY1BTN1:
        emu->emulator->handle_event(InputKey::Joy1Btn1, event->pressed);
        break;
    case EMU_KEY_JOY1BTN2:
        emu->emulator->handle_event(InputKey::Joy1Btn2, event->pressed);
        break;
    case EMU_KEY_JOY1SELECT:
        emu->emulator->handle_event(InputKey::Select1, event->pressed);
        break;
    case EMU_KEY_JOY1START:
        emu->emulator->handle_event(InputKey::Start1, event->pressed);
        break;
    default:
        break;
    }

    return 0;
}
