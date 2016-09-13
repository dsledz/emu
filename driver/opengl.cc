
#include "driver/opengl.h"
#include "emu/emu.h"

Shader::Shader(GLenum type, const std::string &source)
    : _shader(glCreateShader(type)), _program(0) {
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
  if (status == 0) throw OpenGLError();
}

Shader::~Shader(void) {
  if (_shader != 0 && _program != 0) glDetachShader(_program, _shader);
}

void Shader::attach(GLuint program) {
  glAttachShader(program, _shader);
  _program = program;
}

ShaderProgram::ShaderProgram(void) : _program(glCreateProgram()) {}

ShaderProgram::~ShaderProgram(void) { /* XXX: delete program */ }

void ShaderProgram::build(
    const std::string &frag_source, const std::string &vert_source,
    const std::vector<std::pair<std::string, GLuint> > &attribs) {
  Shader frag(GL_FRAGMENT_SHADER, frag_source);
  Shader vert(GL_VERTEX_SHADER, vert_source);

  frag.attach(_program);
  vert.attach(_program);

  for (auto it = attribs.begin(); it != attribs.end(); it++)
    glBindAttribLocation(_program, it->second, it->first.c_str());

  glLinkProgram(_program);

  GLint status;
  glGetProgramiv(_program, GL_LINK_STATUS, &status);
  if (status == 0) throw OpenGLError();
}

/*  _____                  ____  _               _
 * |  ___| __ __ _  __ _  / ___|| |__   __ _  __| | ___ _ __
 * | |_ | '__/ _` |/ _` | \___ \| '_ \ / _` |/ _` |/ _ \ '__|
 * |  _|| | | (_| | (_| |  ___) | | | | (_| | (_| |  __/ |
 * |_|  |_|  \__,_|\__, | |____/|_| |_|\__,_|\__,_|\___|_|
 *                 |___/
 */

#ifdef BUILD_IOS

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
varying vec4 colorVarying;
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

#ifdef BUILD_IOS

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

    gl_Position = Modelview * Position;
    TexCoordOut = TexCoordIn;
})";

#else

static std::string vert_source = R"(
attribute vec4 Position;
attribute vec2 TexCoordIn;

uniform mat4 Projection;
uniform mat4 Modelview;

varying vec4 colorVarying;
varying vec2 TexCoordOut;

void main()
{
    colorVarying = vec4(1.0, 1.0, 1.0, 1.0);

    gl_Position = Modelview * Position;
    TexCoordOut = TexCoordIn;
})";

#endif

GLSLRender::GLSLRender(FrameBuffer *fb): m_fb(fb) {}

GLSLRender::~GLSLRender(void) {}

void GLSLRender::init(void) {
  /* XXX: Our screen should be dynamic based on the game's screen. */
  TextureVertex gScreenData[] = {
      {Vec3(0.5f, 0.5f, 1.0f), Vec2(1.0f, 0.0f)},
      {Vec3(-0.5f, 0.5f, 1.0f), Vec2(0.0f, 0.0f)},
      {Vec3(0.5f, -0.5f, 1.0f), Vec2(1.0f, 1.0f)},
      {Vec3(0.5f, -0.5f, 1.0f), Vec2(1.0f, 1.0f)},
      {Vec3(-0.5f, 0.5f, 1.0f), Vec2(0.0f, 0.0f)},
      {Vec3(-0.5f, -0.5f, 1.0f), Vec2(0.0f, 1.0f)},
  };

  glEnable(GL_BLEND);
  glEnable(GL_TEXTURE_2D);
  glEnable(GL_DEPTH_TEST);

  glClearColor(0.0, 0.0, 0.0, 0.0);

  glGenBuffers(1, &_screen_buffer);
  glBindBuffer(GL_ARRAY_BUFFER, _screen_buffer);
  glBufferData(GL_ARRAY_BUFFER, sizeof(gScreenData), gScreenData,
               GL_STATIC_DRAW);

  std::vector<std::pair<std::string, GLuint> > attribs;
  // attribs.push_back(std::make_pair("position", _var_Position));
  // attribs.push_back(std::make_pair("normal", _var_Normal));

  _program.build(frag_source, vert_source, attribs);

  _var_Position = glGetAttribLocation(_program, "Position");
  glEnableVertexAttribArray(_var_Position);

  _var_TexCoordIn = glGetAttribLocation(_program, "TexCoordIn");
  glEnableVertexAttribArray(_var_TexCoordIn);

  _var_Texture = glGetUniformLocation(_program, "Texture");
  _var_Modelview = glGetUniformLocation(_program, "Modelview");
  _var_Projection = glGetUniformLocation(_program, "Projection");
}

void GLSLRender::render(void) {
  GfxTransform *transform = m_fb->transform();
  GLuint tmp;
  glGenTextures(1, &tmp);

  scoped_texture tex(tmp);

  glBindTexture(GL_TEXTURE_2D, tmp);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, transform->width(),
               transform->height(), 0, GL_RGBA, GL_UNSIGNED_BYTE,
               transform->fb());

  float modelviewprojection[] = {2.25642, 0.0, 0.0,    0.0, 0.0,    1.56969,
                                 0.0,     0.0, 0.0,    0.0, -1.002, -1.0,
                                 0.0,     0.0, 2.3048, 2.5};

  float projection[] = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                        0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};

  glUseProgram(_program);

  glUniformMatrix4fv(_var_Modelview, 1, 0, modelviewprojection);
  glUniformMatrix4fv(_var_Projection, 1, 0, projection);

  glBindBuffer(GL_ARRAY_BUFFER, _screen_buffer);
  /* XXX: Horrible hack */
  glVertexAttribPointer(_var_Position, 3, GL_FLOAT, GL_FALSE,
                        sizeof(TextureVertex), (void *)0);
  glVertexAttribPointer(_var_TexCoordIn, 2, GL_FLOAT, GL_FALSE,
                        sizeof(TextureVertex), (void *)12);

  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, tex);
  glUniform1i(_var_Texture, 0);

  glDrawArrays(GL_TRIANGLES, 0, 6);
  glEnd();
  glFlush();
}

#if OPENGL_LEGACY
GLRender::GLRender(FrameBuffer *fb): m_fb(fb) {}

GLRender::~GLRender(void) {}

void GLRender::init(void) {
  GfxTransform *transform = m_fb->transform();
  /* XXX: Use new pipeline */
  glClearColor(0.0, 0.0, 0.0, 0.0);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0.0, transform->width(), transform->height(), 1.0, -1.0, 1.0);
  glEnable(GL_BLEND);
  glEnable(GL_TEXTURE_2D);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

void GLRender::render(void) {
  GfxTransform *transform = m_fb->transform();
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
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, transform->width(),
               transform->height(), 0, GL_RGBA, GL_UNSIGNED_BYTE,
               transform->fb());

  glClear(GL_COLOR_BUFFER_BIT);
  glBindTexture(GL_TEXTURE_2D, tmp);
  glBegin(GL_QUADS);
  glTexCoord2f(0, 0);
  glVertex2f(0, 0);
  glTexCoord2f(1, 0);
  glVertex2f(transform->width() - 1, 0);
  glTexCoord2f(1, 1);
  glVertex2f(transform->width() - 1, transform->height() - 1);
  glTexCoord2f(0, 1);
  glVertex2f(0, transform->height() - 1);
  glEnd();
  glFlush();
}
#endif
