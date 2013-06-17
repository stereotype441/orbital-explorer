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

#version 150

uniform mat4 modelViewProjMatrix;
uniform float brightness;

in vec4 position;
in vec3 uvY;
out vec4 inverted_position;
out vec3 integrand;

// Note sure what this transform is called, but since doing it twice
// gives back the original vector, we'll call it a "vector inverse"
// for now.
// (x, y, z, w) --> (x/w, y/w, z/w, 1/w)
vec4 vector_inverse(vec4 a)
{
  float w = a.w;
  a.w = 1.0;
  a /= w;
  return a;
}

// Calculate the position and inverted position of the vertex, setting
// the output varyings "gl_Position" and "inverted_position".
void calculate_position()
{
  vec4 pos = modelViewProjMatrix * position;
  gl_Position = pos;
  inverted_position = vector_inverse(pos);
}

// Apply brightness adjustment and color rotation to the input color,
// setting the output varying "integrand" to transformed color
// coordinates that are sensible to integrate.
void calculate_color()
{
  vec2 white = vec2(0.19784, 0.46832);
  vec2 uv = uvY.xy - white;
  float Y = brightness * uvY.z;

  integrand = Y * vec3(uv, 1.0);
}

void main(void)
{
  calculate_position();
  calculate_color();
}
