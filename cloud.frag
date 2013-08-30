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

noperspective in float one_over_w_front;
noperspective in float one_over_w_back;
noperspective in vec3 integrand_over_w_front;
noperspective in vec3 integrand_over_w_back;
noperspective in vec2 texPosition;
out vec3 integratedValue;
uniform sampler2D solidDepth;
uniform vec2 nearfar;
uniform bool depth_obscuration;

// Extract the w value (which is the pre-projection z value) of the
// solid object from the depth buffer that was used in the solid
// object rendering pass.
float compute_solid_w()
{
  // The depth buffer from the solid object rendering pass stores
  // something called z_w.
  float z_w = texture(solidDepth, texPosition).x;

  // We also need the "near" and "far" values from the frustum matrix,
  // which are passed in as uniforms.
  float n = nearfar[0];
  float f = nearfar[1];

  // Also, z_d = z_c / w_c
  // From the frustum matrix,
  // z_c = -(f + n) / (f - n) * z - 2 * f * n / (f - n) * w
  // w_c = -z
  // Substituting,
  // z_d = z_c / w_c = (f + n) / (f - n) + 2 * f * n / (f - n) * w / z
  // But pre-frustum w = 1, so
  // z_d = (f + n) / (f - n) + 2 * f * n / (f - n) / z
  // z_d * (f - n) = f + n + 2 * f * n / z
  // z_d * (f - n) - f - n = 2 * f * n / z
  // z = 2 * f * n / (z_d * (f - n) - f - n)
  // w = -z = -2 * f * n / (z_d * (f - n) - f - n)
  // Substitute z_d = 2 * z_w - 1
  // w = -2 * f * n / ((2 * z_w - 1) * (f - n) - f - n)
  // Simplify RHS
  // w = -2 * f * n / (2 * z_w * (f - n) - 2 * f)
  // w = f * n / (f - z_w * (f - n))
  return f * n / (f - z_w * (f - n));
}

void main(void)
{
  float w_front = 1.0 / one_over_w_front;
  float w_back = 1.0 / one_over_w_back;

  vec3 integrand_front = w_front * integrand_over_w_front;
  vec3 integrand_back = w_back * integrand_over_w_back;

  float solid_w = compute_solid_w();

  if (solid_w <= w_front)
    discard;

  if (solid_w < w_back) {
    float t = (solid_w - w_front) / (w_back - w_front);
    integrand_back = (1-t) * integrand_front + t * integrand_back;
    w_back = solid_w;
  }

  float depth = w_back - w_front;
  vec3 integrand_middle = (integrand_front + integrand_back) / 2.0;
  if (!depth_obscuration) {
    integratedValue = depth * integrand_middle;
  } else {
    // Assume the z-component of the integrand represents intensity,
    // and calculate the falloff of all components based on its
    // integral along the line of sight.
    // TODO
  }
}
