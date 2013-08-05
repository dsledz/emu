
#include "emu/emu.h"
#include "driver/opengl.h"

Shader::Shader(GLenum type, const std::string &source):
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

Shader::~Shader(void)
{
    if (_shader != 0 && _program != 0)
        glDetachShader(_program, _shader);
}

void
Shader::attach(GLuint program)
{
    glAttachShader(program, _shader);
    _program = program;
}

ShaderProgram::ShaderProgram(void):
    _program(glCreateProgram())
{
}

ShaderProgram::~ShaderProgram(void)
{
    /* XXX: delete program */
}

void
ShaderProgram::build(
    const std::string &frag_source,
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


/*  _____                  ____  _               _
 * |  ___| __ __ _  __ _  / ___|| |__   __ _  __| | ___ _ __
 * | |_ | '__/ _` |/ _` | \___ \| '_ \ / _` |/ _` |/ _ \ '__|
 * |  _|| | | (_| | (_| |  ___) | | | | (_| | (_| |  __/ |
 * |_|  |_|  \__,_|\__, | |____/|_| |_|\__,_|\__,_|\___|_|
 *                 |___/
 */

#if TARGET_OS_IPHONE || TARGET_IPHONE_SIMULATOR
static std::string frag_source = R"(
varying lowp vec4 colorVarying;
varying lowp vec2 TexCoordOut;
uniform sampler2D Texture;

void main(void)
{
    gl_FragColor = texture2D(Texture, TexCoordOut);
})";
#else
static std::string frag_source = R"(
#version 120
varying vec2 TexCoordOut;
uniform sampler2D Texture;

void main(void)
{
    gl_FragColor = texture2D(Texture, TexCoordOut);
})";
#endif


/* __     __        _              ____  _               _
 * \ \   / /__ _ __| |_ _____  __ / ___|| |__   __ _  __| | ___ _ __
 *  \ \ / / _ \ '__| __/ _ \ \/ / \___ \| '_ \ / _` |/ _` |/ _ \ '__|
 *   \ V /  __/ |  | ||  __/>  <   ___) | | | | (_| | (_| |  __/ |
 *    \_/ \___|_|   \__\___/_/\_\ |____/|_| |_|\__,_|\__,_|\___|_|
 */

#if TARGET_OS_IPHONE || TARGET_IPHONE_SIMULATOR
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
#else
static std::string vert_source = R"(
#version 120
attribute vec3 Position;
attribute vec2 TexCoordIn;

uniform mat4 Projection;
uniform mat4 Modelview;

varying vec2 TexCoordOut;

void main()
{
    gl_Position = Projection * Modelview * vec4(Position, 1.0);
    TexCoordOut = TexCoordIn;
})";
#endif

class GfxTransformNone: public GfxTransform
{
public:
    GfxTransformNone(void) { }
    virtual ~GfxTransformNone(void) { }

    virtual void resize(short width, short height) {
        init_fb(width, height);
    }

    virtual void render(RasterScreen *screen) {
        const byte_t *src = screen->fb();
        byte_t *dest = reinterpret_cast<byte_t *>(fb());
        const short dest_pitch = pitch();

        for (int y = 0; y < screen->height(); y++)
            memcpy(&dest[y * dest_pitch],
                   &src[y * screen->pitch()],
                   screen->pitch());
    }
};

class GfxTransformScanline2x: public GfxTransform
{
public:
    GfxTransformScanline2x(void) { }
    virtual ~GfxTransformScanline2x(void) { }

    virtual void resize(short width, short height) {
        init_fb(width * 2, height * 2);
    }

    virtual void render(RasterScreen *screen) {
        const byte_t *src = screen->fb();
        byte_t *dest = reinterpret_cast<byte_t *>(fb());
        const short dest_pitch = pitch();

        for (int y = 0; y < screen->height(); y++) {
            unsigned *d0 = reinterpret_cast<unsigned *>(dest);
            dest += dest_pitch;
            unsigned *d1 = reinterpret_cast<unsigned *>(dest);
            dest += dest_pitch;
            const unsigned * s = reinterpret_cast<const unsigned *>(src);
            for (int x = 1; x < screen->width() - 1; x++) {
                d0[x*2] = s[x];
                d0[x*2+1] = s[x];
                d1[x*2] = d1[x*2+1] = 0xff000000;
            }
            src += screen->pitch();
        }
    }
};

class GfxTransform2x: public GfxTransform
{
public:
    GfxTransform2x(void) { }
    virtual ~GfxTransform2x(void) { }

    virtual void resize(short width, short height) {
        init_fb(width * 2, height * 2);
    }

    virtual void render(RasterScreen *screen) {
        const byte_t *src = screen->fb();
        byte_t *dest = reinterpret_cast<byte_t *>(fb());
        const short dest_pitch = pitch();

        for (int y = 0; y < screen->height(); y++) {
            unsigned *d0 = reinterpret_cast<unsigned *>(dest);
            dest += dest_pitch;
            unsigned *d1 = reinterpret_cast<unsigned *>(dest);
            dest += dest_pitch;
            const unsigned * s = reinterpret_cast<const unsigned *>(src);
            for (int x = 1; x < screen->width() - 1; x++) {
                d0[x*2] = s[x];
                d0[x*2+1] = s[x];
                d1[x*2] = s[x];
                d1[x*2+1] = s[x];
            }
            src += screen->pitch();
        }

    }
};

class GfxTransformScale2x: public GfxTransform
{
public:
    GfxTransformScale2x(void) { }
    virtual ~GfxTransformScale2x(void) { }

    virtual void resize(short width, short height) {
        init_fb(width * 2, height * 2);
    }

    virtual void render(RasterScreen *screen) {
        const byte_t *src = screen->fb();
        byte_t *dest = reinterpret_cast<byte_t *>(fb());
        const short dest_pitch = pitch();

        const unsigned *sl = reinterpret_cast<const unsigned *>(src);
        const unsigned *s, *sh;

        for (int y = 0; y < screen->height(); y++) {
            int x = 0;
            unsigned *d0 = reinterpret_cast<unsigned *>(dest);
            dest += dest_pitch;
            unsigned *d1 = reinterpret_cast<unsigned *>(dest);
            dest += dest_pitch;
            s = reinterpret_cast<const unsigned *>(src);
            if (y < screen->height() - 1)
                sh = reinterpret_cast<const unsigned *>(src + screen->pitch());
            d0[x*2] = s[x];
            d0[x*2+1] = s[x];
            d1[x*2] = s[x];
            d1[x*2+1] = s[x];
            for (x = 1; x < screen->width() - 1; x++) {
                d0[x*2] = s[x];
                d0[x*2+1] = s[x];
                d1[x*2] = s[x];
                d1[x*2+1] = s[x];
                if (s[x-1] == sl[x] && s[x-1] != sh[x] && sl[x] != s[x+1])
                    d0[x*2] = sl[x];
                if (sl[x] == s[x+1] && sl[x] != s[x-1] && s[x+1] != sh[x])
                    d0[x*2+1] = s[x+1];
                if (sh[x] == s[x-1] && s[x+1] != sl[x] && s[x-1] != sl[x])
                    d1[x*2] = s[x-1];
                if (s[x+1] == sh[x] && s[x+1] != sl[x] && sh[x] != s[x-1])
                    d1[x*2+1] = sh[x];
            }
            d0[x*2] = s[x];
            d0[x*2+1] = s[x];
            d1[x*2] = s[x];
            d1[x*2+1] = s[x];
            sl = s;
            src += screen->pitch();
        }
    }
};

gfx_transform_ptr
get_transform(GfxScale scale)
{
    gfx_transform_ptr transform;

    switch (scale) {
    case GfxScale::None:
        transform = gfx_transform_ptr(new GfxTransformNone());
        break;
    case GfxScale::Scaneline2x:
        transform = gfx_transform_ptr(new GfxTransformScanline2x());
        break;
    case GfxScale::Nearest2x:
        transform = gfx_transform_ptr(new GfxTransform2x());
        break;
    case GfxScale::Scale2x:
        transform = gfx_transform_ptr(new GfxTransformScale2x());
        break;
    }

    return transform;
}

OGLRasterScreen::OGLRasterScreen(void):
    RasterScreen(),
    _scale(GfxScale::None)
{
    _transform = get_transform(_scale);
}

OGLRasterScreen::~OGLRasterScreen(void)
{

}

void
OGLRasterScreen::resize(short width, short height)
{
    do_resize(width, height);
    _transform->resize(width, height);
}

void
OGLRasterScreen::init(void)
{
    /* XXX: Our screen should be dynamic based on the game's screen. */
    TextureVertex gScreenData[] = {
        { Vec3(0.5f,  0.5f, 1.0f), Vec2(1.0f, 0.0f) },
        { Vec3(-0.5f,  0.5f, 1.0f), Vec2(0.0f, 0.0f) },
        { Vec3(0.5f, -0.5f, 1.0f), Vec2(1.0f, 1.0f) },
        { Vec3(0.5f, -0.5f, 1.0f), Vec2(1.0f, 1.0f) },
        { Vec3(-0.5f,  0.5f, 1.0f), Vec2(0.0f, 0.0f) },
        { Vec3(-0.5f, -0.5f, 1.0f), Vec2(0.0f, 1.0f) },
    };

    glEnable(GL_BLEND);
    glEnable(GL_TEXTURE_2D);
    glEnable(GL_DEPTH_TEST);

    glClearColor(0.0,0.0,0.0,0.0);

    glGenBuffers(1, &_screen_buffer);
    glBindBuffer(GL_ARRAY_BUFFER, _screen_buffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(gScreenData), gScreenData,
                 GL_STATIC_DRAW);

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

void
OGLRasterScreen::flip(void)
{
    _transform->render(this);
}

void
OGLRasterScreen::render(void)
{
    GLuint tmp;
    glGenTextures(1, &tmp);

    scoped_texture tex(tmp);

    glBindTexture(GL_TEXTURE_2D, tmp);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, _transform->width(),
                 _transform->height(), 0, GL_RGBA, GL_UNSIGNED_BYTE,
                 _transform->fb());

    float modelview[] = {
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, -4.0, 1.0,
    };

    float projection[] = {
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0 };

    glUseProgram(_program);

    glUniformMatrix4fv(_var_Modelview, 1, 0, modelview);
    glUniformMatrix4fv(_var_Projection, 1, 0, projection);

    glBindBuffer(GL_ARRAY_BUFFER, _screen_buffer);
    /* XXX: Horrible hack */
    glVertexAttribPointer(_var_Position, 3, GL_FLOAT, GL_FALSE,
                          sizeof(TextureVertex), (void*)0);
    glVertexAttribPointer(_var_TexCoordIn, 2, GL_FLOAT, GL_FALSE,
                          sizeof(TextureVertex), (void*)12);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, tex);
    glUniform1i(_var_Texture, 0);

    glDrawArrays(GL_TRIANGLES, 0, 6);
}

GLRasterScreen::GLRasterScreen(void):
    RasterScreen(),
    _scale(GfxScale::None)
{
    _transform = get_transform(_scale);
}

GLRasterScreen::~GLRasterScreen(void)
{
}

void
GLRasterScreen::resize(short width, short height)
{
    do_resize(width, height);
    _transform->resize(width, height);
}

void
GLRasterScreen::init(void)
{
    /* XXX: Use new pipeline */
    glClearColor(0.0,0.0,0.0,0.0);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0.0, _transform->width(), _transform->height(), 1.0, -1.0, 1.0);
    glEnable(GL_BLEND);
    glEnable(GL_TEXTURE_2D);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
}

void
GLRasterScreen::flip(void)
{
    _transform->render(this);
}

void
GLRasterScreen::render(void)
{
    /* XXX: Use new pipeline */
    GLuint tmp;
    glGenTextures(1, &tmp);

    scoped_texture tex(tmp);

    glBindTexture(GL_TEXTURE_2D, tmp);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    glBindTexture(GL_TEXTURE_2D, tmp);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, _transform->width(),
                 _transform->height(), 0, GL_RGBA, GL_UNSIGNED_BYTE,
                 _transform->fb());

    glClear(GL_COLOR_BUFFER_BIT);
    glBindTexture(GL_TEXTURE_2D, tmp);
    glBegin(GL_QUADS);
    glTexCoord2f(0, 0);
    glVertex2f(0, 0);
    glTexCoord2f(1, 0);
    glVertex2f(_transform->width() - 1, 0);
    glTexCoord2f(1, 1);
    glVertex2f(_transform->width() - 1, _transform->height()- 1);
    glTexCoord2f(0, 1);
    glVertex2f(0, _transform->height() - 1);
    glEnd();
    glFlush();

}


