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

#ifndef OOPENGL_HH
#define OOPENGL_HH

#include <stdexcept>
#include <vector>
#include <GL/glew.h>

class Uncopyable
{
public:
  Uncopyable() {}

private:
  Uncopyable(const Uncopyable &); // DO NOT DEFINE
  Uncopyable &operator=(const Uncopyable &); // DO NOT DEFINE
};

class Timer : public Uncopyable
{
public:
  Timer() {
    glGenQueries(2, id);
    glQueryCounter(id[0], GL_TIMESTAMP);
    glQueryCounter(id[1], GL_TIMESTAMP);
  }
  ~Timer() { glDeleteQueries(2, id); }
  void start() {
    glQueryCounter(id[0], GL_TIMESTAMP);
  }
  void stop() {
    glQueryCounter(id[1], GL_TIMESTAMP);
  }
  long read() {
    GLint timer_good;

    glGetQueryObjectiv(id[0], GL_QUERY_RESULT_AVAILABLE, &timer_good);
    if (!timer_good)
      goto bailout;

    glGetQueryObjectiv(id[1], GL_QUERY_RESULT_AVAILABLE, &timer_good);
    if (!timer_good)
      goto bailout;

    GLuint64 t0, t1;
    glGetQueryObjectui64v(id[0], GL_QUERY_RESULT, &t0);
    glGetQueryObjectui64v(id[1], GL_QUERY_RESULT, &t1);
    return t1 - t0;

  bailout:
    throw std::logic_error("GPU timer not ready");
  }

private:
  GLuint id[2];
};

class Shader : public Uncopyable
{
public:
  Shader(GLenum type) : id(glCreateShader(type)) {}
  void compileSource(const char *);
  ~Shader() { glDeleteShader(id); }
  operator GLuint() { return id; }

private:
  GLuint id;
};

class VertexShader : public Shader
{
public:
  VertexShader() : Shader(GL_VERTEX_SHADER) {}
};

class GeometryShader : public Shader
{
public:
  GeometryShader() : Shader(GL_GEOMETRY_SHADER) {}
};

class FragmentShader : public Shader
{
public:
  FragmentShader() : Shader(GL_FRAGMENT_SHADER) {}
};

template <typename T> class Uniform;

class Program : public Uncopyable
{
public:
  Program() : id(glCreateProgram()) {}
  operator GLuint() { return id; }
  void attach(Shader &shader) { glAttachShader(id, shader); }
  void detach(Shader &shader) { glDetachShader(id, shader); }
  void link();
  void use();
  bool used() { return id == program_in_use; }
  ~Program() { glDeleteProgram(id); }

  // Uniforms
  template <typename T> Uniform<T> uniform(const char *name);

  // Convenience functions to create, compile, and attach a shader
  void vertexShader(const char *source);
  void geometryShader(const char *source);
  void fragmentShader(const char *source);

private:
  GLuint id;
  static GLuint program_in_use;
};

template <typename T>
class Uniform // This class IS copyable
{
public:
  Uniform(Program &program_, const char *name) :
    program(program_),
    location(glGetUniformLocation(program, name))
  {}
  operator GLuint() { return location; }
  void operator=(const T &);

private:
  Program &program;
  GLuint location;

  void verify_used();
};

template <typename T>
inline Uniform<T> Program::uniform(const char *name)
{
  return Uniform<T>(*this, name);
}

class Buffer;
class VertexArrayObject : public Uncopyable
{
public:
  VertexArrayObject();
  void bind()          { glBindVertexArray(id); }
  ~VertexArrayObject() { glDeleteVertexArrays(1, &id); }

  void buffer(GLenum target, const void *data, size_t size);

  template <typename T> void buffer(GLenum target, const std::vector<T> &vec)
  { buffer(target, &vec[0], sizeof(T) * vec.size()); }

private:
  GLuint id;
  Buffer *arrayBuffer, *elementArrayBuffer;
};

class Buffer : public Uncopyable
{
public:
  Buffer()                 { glGenBuffers(1, &id); }
  operator GLuint()        { return id; }
  void bind(GLenum target) { glBindBuffer(target, id); }
  ~Buffer()                { glDeleteBuffers(1, &id); }

private:
  GLuint id;
};

class Texture : public Uncopyable
{
public:
  Texture()                { glGenTextures(1, &id); }
  operator GLuint()        { return id; }
  void bind(GLenum target) { glBindTexture(target, id); }
  ~Texture()               { glDeleteTextures(1, &id); }

private:
  GLuint id;
};

#endif
