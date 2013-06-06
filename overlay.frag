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

in vec2 coord;
out vec3 RGB;
uniform sampler2D solidData;
uniform sampler2D cloudData;

vec3 srgb_gamma(vec3 c)
{
  return mix(c * 12.92,
             1.055 * pow(c, vec3(1.0 / 2.4)) - vec3(0.055),
             greaterThan(c, vec3(0.0031308)));
}

void main(void)
{
  // sRGB code here. We start by extracting xyY coordinates.

  // The input is integrated (u * Y, v * Y, Y) from rendering multiple
  // tetrahedra with additive blending.
  vec3 integrated_uvY = texture(cloudData, coord).xyz;

  // Extract u, v, and Y from the input.
  // Integral of intensity (Y) along line of sight.
  float integrated_Y = integrated_uvY[2];
  // Integral of intensity-scaled chromaticity (u * Y and v * Y), divided
  // by total intensity (Y), gives intensity-weighted chromaticity.
  vec2 cloud_uv = integrated_uvY.xy / integrated_Y;

  // Convert CIE (u,v) color coordinates (as per CIELUV) to (x,y)
  vec2 cloud_xy = vec2(9.0, 4.0) * cloud_uv;
  cloud_xy /= dot(vec3(6.0, -16.0, 12.0), vec3(cloud_uv, 1.0));

  // Exponential fall-off of intensity. Has no effect on chromaticity.
  // This takes into account that nearby "particles" of cloud or fog
  // will invariably block some fraction of farther-away "particles".
  float cloud_Y = 1 - exp(-integrated_Y);

  // Solid object blending
  vec3 solid_xyY = texture(solidData, coord).xyz;
  // Diminish solid illuminance by the amount of cloud attenuation in
  // front of it.
  float solid_Y = solid_xyY[2];
  solid_Y *= 1 - cloud_Y;
  // Compute final Y value
  float Y = solid_Y + cloud_Y;
  // Take weighted average of xy values
  vec3 xyz;
  xyz.xy = (solid_xyY.xy * solid_Y + cloud_xy * cloud_Y) / Y;
  xyz.z = 1 - xyz.x - xyz.y;

  // Convert xyz to XYZ
  vec3 XYZ = (Y / xyz.y) * xyz;

  // Convert XYZ to linear (i.e. pre-gamma) RGB values
  mat3 XYZ_to_linear_RGB = mat3(+3.2406, -0.9689,  0.0557,
                                -1.5372,  1.8758, -0.2040,
                                -0.4986,  0.0415,  1.0570);
  vec3 linear_RGB = XYZ_to_linear_RGB * XYZ;

  // Gamut clamping - keeping hue constant, desaturate towards an equally
  // intense grey until the RGB values are in [0,1]. This operation
  // effectively happens to the CIE XYZ values, and we are using the fact
  // that XYZ and pre-gamma RGB are related by a linear transformation to
  // streamline the calculation.
  vec3 grey_RGB = vec3(Y);
  vec3 RGB_overshoot = max(linear_RGB - vec3(1.0), vec3(0.0));
  vec3 blet = RGB_overshoot / (linear_RGB - grey_RGB); // NaN problem here?
  float t = max(blet.r, max(blet.g, blet.b));
  linear_RGB = mix(linear_RGB, grey_RGB, t);

  // Gamma correction
  RGB = srgb_gamma(linear_RGB);
}
