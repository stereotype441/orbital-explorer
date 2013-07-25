/*
 * This file is part of the Electron Orbital Explorer. The Electron
 * Orbital Explorer is distributed under the Simplified BSD License
 * (also called the "BSD 2-Clause License"), in hopes that these
 * rendering techniques might be used by other programmers in
 * applications such as scientific visualization, video gaming, and so
 * on. If you find value in this software and use its technologies for
 * another purpose, I would love to hear back from you at bjthinks (at)
 * gmail (dot) com. If you improve this software and agree to release
 * your modifications under the below license, I encourage you to fork
 * the development tree on github and push your modifications. The
 * Electron Orbital Explorer's development URL is:
 * https://github.com/bjthinks/orbital-explorer
 * (This paragraph is not part of the software license and may be
 * removed.)
 *
 * Copyright (c) 2013, Brian W. Johnson
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * + Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * + Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in
 *   the documentation and/or other materials provided with the
 *   distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdexcept>
#include <vector>
#include <complex>
#include <cstdlib>
#include <cmath>
#include <cstdio>

#include "array.hh"
#include "genericops.hh"
#include "vector.hh"
#include "matrix.hh"
#include "oopengl.hh"

void GetGLError_(const char *file, int line)
{
  GLenum err;
  if ((err = glGetError()) != GL_NO_ERROR) {
    fprintf(stderr, "Detected OpenGL error at file %s line %d\n", file, line);
    do
      fprintf(stderr, "Error code %d (0x%x)\n", err, err);
    while ((err = glGetError()) != GL_NO_ERROR);
    exit(1);
  }
}

void Shader::compileSource(const char *source)
{
  GLint status;
  glShaderSource(id, 1, (const GLchar **)&source, NULL);
  glCompileShader(id);
  glGetShaderiv(id, GL_COMPILE_STATUS, &status);
  if (!status) {
    fprintf(stderr, "Fatal error: shader failed to compile\n");
    GLint len;
    glGetShaderiv(id, GL_INFO_LOG_LENGTH, &len);
    GLchar *msg = new GLchar[len];
    glGetShaderInfoLog(id, len, NULL, msg);
    fprintf(stderr, "Error message is:\n%s\n", msg);
    delete[] msg;
    exit(1);
  }
}

void Program::vertexShader(const char *source)
{
  VertexShader shader;
  shader.compileSource(source);
  attach(shader);
}

void Program::geometryShader(const char *source)
{
  GeometryShader shader;
  shader.compileSource(source);
  attach(shader);
}

void Program::fragmentShader(const char *source)
{
  FragmentShader shader;
  shader.compileSource(source);
  attach(shader);
}

void Program::link()
{
  GLint status;
  glLinkProgram(id);
  glGetProgramiv(id, GL_LINK_STATUS, &status);
  if (!status) {
    fprintf(stderr, "Fatal error: program failed to link\n");
    GLint len;
    glGetProgramiv(id, GL_INFO_LOG_LENGTH, &len);
    GLchar *msg = (GLchar *)malloc(len);
    glGetProgramInfoLog(id, len, NULL, msg);
    fprintf(stderr, "Error message is:\n%s\n", msg);
    free(msg);
    exit(1);
  }
}

void Program::use()
{
  glUseProgram(id);
  program_in_use = id;
}

GLuint Program::program_in_use = 0;

template <>
void Uniform<int>::operator=(const int &x)
{
  verify_used();
  glUniform1i(location, x);
}

template <>
void Uniform<float>::operator=(const float &x)
{
  verify_used();
  glUniform1f(location, x);
}

template <>
void Uniform<Vector<2> >::operator=(const Vector<2> &x)
{
  verify_used();
  glUniform2f(location, x[0], x[1]);
}

template <>
void Uniform<std::vector<Vector<4> > >::
operator=(const std::vector<Vector<4> > &x)
{
  verify_used();
  int n = x.size();
  GLfloat *data = new GLfloat[4*n];
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < 4; ++j)
      data[4*i+j] = x[i][j];
  glUniform4fv(location, n, data);
}

template <>
void Uniform<Matrix<3,2> >::operator=(const Matrix<3,2> &x)
{
  verify_used();
  Array<6,GLfloat> x_array(x);
  GLfloat *data_ptr = &(x_array[0]);
  glUniformMatrix2x3fv(location, 1, GL_TRUE, data_ptr);
}

template <>
void Uniform<Matrix<4,4> >::operator=(const Matrix<4,4> &x)
{
  verify_used();
  Array<16,GLfloat> x_array(x);
  GLfloat *data_ptr = &(x_array[0]);
  glUniformMatrix4fv(location, 1, GL_TRUE, data_ptr);
}

template <typename T>
void Uniform<T>::verify_used()
{
  if (!program.used()) {
    printf("Wrong program in use\n");
    exit(0);
  }
}

VertexArrayObject::VertexArrayObject() :
  arrayBuffer(NULL),
  elementArrayBuffer(NULL)
{
  glGenVertexArrays(1, &id);
}

void VertexArrayObject::buffer(GLenum target, const void *data, size_t size)
{
  bind();
  Buffer *b = new Buffer();
  b->bind(target);
  glBufferData(target, size, data, GL_STATIC_DRAW);

  Buffer *old = NULL;
  switch (target) {
  case GL_ARRAY_BUFFER:
    old = arrayBuffer;
    arrayBuffer = b;
    break;
  case GL_ELEMENT_ARRAY_BUFFER:
    old = elementArrayBuffer;
    elementArrayBuffer = b;
    break;
  default:
    fprintf(stderr, "Please add another case to the switch statement in "
            "VertexArrayObject::buffer()\n");
    exit(1);
  }

  if (old != NULL)
    delete old;
}
