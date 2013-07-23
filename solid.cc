#include "solid.hh"
#include "oopengl.hh"
#include "shaders.hh"

static Program *solidProg;
static VertexArrayObject *solid;

void initSolids()
{
  solidProg = new Program();
  solidProg->vertexShader(solidVertexShaderSource);
  solidProg->fragmentShader(solidFragmentShaderSource);
  glBindAttribLocation(*solidProg, 0, "inPosition");
  glBindAttribLocation(*solidProg, 1, "inColor");
  glBindFragDataLocation(*solidProg, 0, "fragColor");
  solidProg->link();

  GetGLError();

  // Solid objects
  solid = new VertexArrayObject();
  solid->bind();

  // Positions
  GLfloat solid_data[] = {
    // X axis
    0.0, 0.0, 0.0, 0.6400, 0.3300, 0.2126,
    1.0, 0.0, 0.0, 0.6400, 0.3300, 0.2126,

    // Y axis
    0.0, 0.0, 0.0, 0.3000, 0.6000, 0.3290,
    0.0, 1.0, 0.0, 0.3000, 0.6000, 0.3290,

    // Z axis
    0.0, 0.0, 0.0, 0.1500, 0.0600, 0.0721,
    0.0, 0.0, 1.0, 0.1500, 0.0600, 0.0721
  };
  solid->buffer(GL_ARRAY_BUFFER, solid_data, sizeof(solid_data));

  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(GLfloat), 0);
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(GLfloat),
                        (void *)(3 * sizeof(GLfloat)));

  GetGLError();
}

void drawSolids(const Matrix<4,4> &mvpm, int width, int height,
                GLuint solidFBO)
{
  solidProg->use();
  solidProg->uniform<Matrix<4,4> >("modelViewProjMatrix") = mvpm;
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, solidFBO);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glEnable(GL_DEPTH_TEST);
  glDisable(GL_BLEND);
  glLineWidth(2);
  solid->bind();
  glViewport(0, 0, width, height);
  glDrawArrays(GL_LINES, 0, 6);

  GetGLError();
}
