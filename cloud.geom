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

layout(lines_adjacency) in;
layout(triangle_strip,max_vertices=21) out;

struct vertex_data {
  vec4 p;
  vec4 n;
  vec3 d;
};

uniform vec2 nearfar;

in vec4 inverted_position[4];
in vec3 integrand[4];

noperspective out float one_over_w_front;
noperspective out float one_over_w_back;
noperspective out vec3 integrand_over_w_front;
noperspective out vec3 integrand_over_w_back;
noperspective out vec2 texPosition;

#if 0
// Swap two things
// Passing elements of an array as inout parameters appears to be
// broken with version 325 of Nvidia drivers.
void swap(inout vertex_data a, inout vertex_data b)
{
  vertex_data t = a;
  a = b;
  b = t;
}
#endif

vec4 vector_inverse(vec4 a)
{
  float w = a.w;
  a.w = 1.0;
  a /= w;
  return a;
}

// Return +/- 1, depending on the orientation of triangle a -> b -> c
// in the xy plane (ignoring z and w)
float triangle_orientation(vec4 a, vec4 b, vec4 c)
{
  vec2 ab = b.xy - a.xy;
  vec2 ac = c.xy - a.xy;
  return sign(ab.x * ac.y - ab.y * ac.x);
}

void outputVertex(vertex_data front, vertex_data back)
{
  one_over_w_front = front.n.w;
  one_over_w_back  = back.n.w;
  integrand_over_w_front = front.d * front.n.w;
  integrand_over_w_back = back.d * back.n.w;
  gl_Position = vec4(front.n.xy, 0.0, 1.0);
  texPosition = (front.n.xy + 1.0) / 2.0;
  EmitVertex();
}

void outputSimpleVertex(vertex_data vert)
{
  one_over_w_front = one_over_w_back  = vert.n.w;
  integrand_over_w_front = integrand_over_w_back = vert.d * vert.n.w;
  gl_Position = vec4(vert.n.xy, 0.0, 1.0);
  texPosition = (vert.n.xy + 1.0) / 2.0;
  EmitVertex();
}

vertex_data intersect_nearclip(vertex_data x, vertex_data y)
{
  vertex_data r;
  float near = nearfar[0];

  float t = (near - y.p.w) / (x.p.w - y.p.w);
  r.p = t * x.p + (1-t) * y.p;
  r.n = vector_inverse(r.p);
  r.d = t * x.d + (1-t) * y.d;

  return r;
}

void handle_tetrahedron(vertex_data x[4]);

void main(void)
{
  vertex_data x[4];

  // Number of vertices clipped by the near plane
  float near = nearfar[0];
  int num_clipped = 0;

  for (int i = 0; i < 4; ++i) {
    // Positions
    x[i].p = gl_in[i].gl_Position;

    // Count how many vertices are clipped
    if (x[i].p.w < near)
      ++num_clipped;

    // (x, y, z, w) --> (x, y, z, 1) / w
    x[i].n = inverted_position[i];

    // Densities
    x[i].d = integrand[i];
  }

  // Handle the near clipping plane.
  // Move clipped vertices to the front of the array using a "sorting network"
  if (x[0].p.w >= near && x[2].p.w < near) {
    //swap(x[0], x[2]);
    vertex_data temp = x[0];
    x[0] = x[2];
    x[2] = temp;
  }
  if (x[1].p.w >= near && x[3].p.w < near) {
    //swap(x[1], x[3]);
    vertex_data temp = x[1];
    x[1] = x[3];
    x[3] = temp;
  }
  if (x[0].p.w >= near && x[1].p.w < near) {
    //swap(x[0], x[1]);
    vertex_data temp = x[0];
    x[0] = x[1];
    x[1] = temp;
  }
  if (x[2].p.w >= near && x[3].p.w < near) {
    //swap(x[2], x[3]);
    vertex_data temp = x[2];
    x[2] = x[3];
    x[3] = temp;
  }
  if (x[1].p.w >= near && x[2].p.w < near) {
    //swap(x[1], x[2]);
    vertex_data temp = x[1];
    x[1] = x[2];
    x[2] = temp;
  }

  switch (num_clipped) {

  case 0:

    handle_tetrahedron(x);

    return;

  case 1:

    {
      // One vertex clipped:
      // Must determine the intersections of the x[0]-to-x[i] segments
      // with the near clipping plane, and construct three new tetrahedra
      // to render.
      vertex_data intersection01 = intersect_nearclip(x[0], x[1]);
      vertex_data intersection02 = intersect_nearclip(x[0], x[2]);
      vertex_data intersection03 = intersect_nearclip(x[0], x[3]);

      vertex_data subtetrahedron[4];
      subtetrahedron[0] = x[1];
      subtetrahedron[1] = intersection01;
      subtetrahedron[2] = intersection02;
      subtetrahedron[3] = intersection03;
      handle_tetrahedron(subtetrahedron);

      subtetrahedron[0] = x[1];
      subtetrahedron[1] = x[2];
      subtetrahedron[2] = intersection02;
      subtetrahedron[3] = intersection03;
      handle_tetrahedron(subtetrahedron);

      subtetrahedron[0] = x[1];
      subtetrahedron[1] = x[2];
      subtetrahedron[2] = x[3];
      subtetrahedron[3] = intersection03;
      handle_tetrahedron(subtetrahedron);
    }

    return;

  case 2:

    {
      // Two vertices clipped:
      // Must determine the intersections of the x[0,1]-to-x[2,3] segments
      // with the near clipping plane, and construct three new tetrahedra
      // to render.
      vertex_data intersection02 = intersect_nearclip(x[0], x[2]);
      vertex_data intersection03 = intersect_nearclip(x[0], x[3]);
      vertex_data intersection12 = intersect_nearclip(x[1], x[2]);
      vertex_data intersection13 = intersect_nearclip(x[1], x[3]);

      vertex_data subtetrahedron[4];
      subtetrahedron[0] = x[2];
      subtetrahedron[1] = x[3];
      subtetrahedron[2] = intersection02;
      subtetrahedron[3] = intersection12;
      handle_tetrahedron(subtetrahedron);

      subtetrahedron[0] = x[3];
      subtetrahedron[1] = intersection02;
      subtetrahedron[2] = intersection12;
      subtetrahedron[3] = intersection03;
      handle_tetrahedron(subtetrahedron);

      subtetrahedron[0] = x[3];
      subtetrahedron[1] = intersection12;
      subtetrahedron[2] = intersection03;
      subtetrahedron[3] = intersection13;
      handle_tetrahedron(subtetrahedron);
    }

    return;

  case 3:

    // Three vertices clipped:
    // Must replace x[0], x[1], and x[2] with their projections toward
    // x[3] at the point they intersect the near clipping plane.
    x[0] = intersect_nearclip(x[0], x[3]);
    x[1] = intersect_nearclip(x[1], x[3]);
    x[2] = intersect_nearclip(x[2], x[3]);
    handle_tetrahedron(x);

    return;

  case 4:

    return;

  }
}

void handle_tetrahedron(vertex_data x[4])
{
  // For each face of the tetrahedron, an orientation on that face
  // is induced by the orientation of the input tetrahedron.  Compute
  // whether those orientations are equal to or opposite from the
  // orientation in screen coordinates.
  // The orientation will be the same for front faces and reversed
  // for back faces (or vice versa, depending on whether the
  // tetrahedron is left or right handed).
  // Surprisingly, we DON'T need to know the handedness of the input
  // tetrahedron for our rendering calculations!
  // The induced orientations are:
  // Face 0: 3 -> 2 -> 1
  float orient0 = triangle_orientation(x[3].n, x[2].n, x[1].n);
  // Face 1: 0 -> 2 -> 3
  float orient1 = triangle_orientation(x[0].n, x[2].n, x[3].n);
  // Face 2: 3 -> 1 -> 0
  float orient2 = triangle_orientation(x[3].n, x[1].n, x[0].n);
  // Face 3: 0 -> 1 -> 2
  float orient3 = triangle_orientation(x[0].n, x[1].n, x[2].n);

  float orient01 = orient0 * orient1;
  float orient23 = orient2 * orient3;
  float orient = orient01 * orient23;

  // For now, discard tetrahedra where a face is seen exactly edge-on.
  // The sign() function should return 0.0 in that case, but exercise
  // an abundance of caution with floating point math.
  if (orient > -0.5 && orient < 0.5)
    return;

  // We call a tetrahedron good if its projection onto the canvas
  // has a triangular convex hull with one vertex in the interior.
  // It is conversely bad if the convex hull is a quadrilateral.
  // A tetrahedron is good iff an odd number of faces have flipped
  // orientation, i.e. there are either three front and one rear
  // faces or vice versa.
  bool good = orient < 0.;

  if (good) {
    // The vertex in the center is the one whose orientN value is
    // of a different sign.  Figure out the majority sign and which
    // vertex is in the middle.
    float majority;
    int middle;
    if (orient01 > 0.) {
      // 0 and 1 have the same sign; either 2 or 3 is opposite
      majority = orient0;
      if (orient2 * majority < 0.)
        middle = 2;
      else
        middle = 3;
    } else {
      // 2 and 3 have the same sign; either 0 or 1 is opposite
      majority = orient2;
      if (orient0 * majority < 0.)
        middle = 0;
      else
        middle = 1;
    }

    // Reorder so that the middle vertex is at index 0
    {
      //swap(x[0], x[middle]);
      vertex_data temp = x[0];
      x[0] = x[middle];
      x[middle] = temp;
    }

    // We need the point on the big face that intersects the middle point
    // in screen cooordinates.  To do this, we need to solve the following
    // system of equations in the xy plane:
    // x[0] = x[1] + t (x[2] - x[1]) + u (x[3] - x[1])
    // Rearranging, we get:
    // (x[2] - x[1]) t + (x[3] - x[1]) u = x[0] - x[1]
    // Which we write abstractly as:
    // A * X = B, where
    mat2 A = mat2((x[2].n - x[1].n).xy, (x[3].n - x[1].n).xy);
    vec2 B = (x[0].n - x[1].n).xy;
    vec2 X = inverse(A) * B;
    float t = X[0];
    float u = X[1];
    vertex_data y0 = x[0];
    vertex_data y1;
    vec4 weighted_x1 = (1 - t - u) * x[1].n;
    vec4 weighted_x2 = t * x[2].n;
    vec4 weighted_x3 = u * x[3].n;
    y1.n = weighted_x1 + weighted_x2 + weighted_x3;
    y1.d = weighted_x1.w * x[1].d + weighted_x2.w * x[2].d +
      weighted_x3.w * x[3].d;
    y1.d /= y1.n.w;
    if (y0.n.z > y1.n.z) {
      //swap(y0, y1);
      vertex_data temp = y0;
      y0 = y1;
      y1 = temp;
    }

    // Draw three triangles.
    // x[1] - x[2] - y
    outputSimpleVertex(x[1]);
    outputSimpleVertex(x[2]);
    outputVertex(y0, y1);

    // x[2] - y - x[3]
    outputSimpleVertex(x[3]);

    // y - x[3] - x[1]
    outputSimpleVertex(x[1]);
    EndPrimitive();
  } else {
    float orient02 = orient0 * orient2;

    // We assume here that two of the orientations are positive, and
    // two are negative.  The other possibility, that all four have
    // the same sign, would indicate an error; it's unclear if that
    // can happen at all, but it's probably worth checking for.
    if (orient01 > 0. && orient23 > 0. && orient02 > 0.)
         return;

    // Reorder so that vertices 0 and 1 have the same sign
    if (orient01 > 0.)
      // 0 and 1 already match, do nothng
      ;
    else if (orient02 > 0.) {
      // 0 and 2 match, so swap 1 with 2
      //swap(x[1], x[2]);
      vertex_data temp = x[1];
      x[1] = x[2];
      x[2] = temp;
    } else {
      // 0 and 3 match, so swap 1 with 3
      //swap(x[1], x[3]);
      vertex_data temp = x[1];
      x[1] = x[3];
      x[3] = temp;
    }

    // Compute the intersection point of segment 01 with segment 23
    // We need to solve this system of equations in the xy plane:
    // x[0] + t (x[1] - x[0]) = x[2] + u (x[3] - x[2])
    // which, by algebra, is:
    // (x[1] - x[0]) t + (x[2] - x[3]) u = x[2] - x[0]
    // Which we will write abstractly as:
    // A * X = B
    // Note that doing this calculation with x[].n instead of x[].v
    // will give us correct x, y, z values regardless of w.
    mat2 A = mat2((x[1].n-x[0].n).xy, (x[2].n-x[3].n).xy);
    vec2 B = (x[2].n - x[0].n).xy;
    vec2 X = inverse(A) * B; // NOTE: need to handle ill-conditioned case
    float t = X[0];
    float u = X[1];

    // We get two answers; one is the point on the front edge which
    // appears to intersect the back edge in screen coordinates, and
    // the other is the point on the back edge which appears to inter-
    // sect the front edge in screen coordinates.  We don't know which
    // is which, but we can tell the difference by looking at the z
    // values.
    // The two intersection points are:
    vertex_data y0, y1;
    vec4 weighted_x0 = (1-t) * x[0].n;
    vec4 weighted_x1 = t * x[1].n;
    y0.n = weighted_x0 + weighted_x1;
    y0.d = weighted_x0.w * x[0].d + weighted_x1.w * x[1].d;
    y0.d /= y0.n.w;

    vec4 weighted_x2 = (1-u) * x[2].n;
    vec4 weighted_x3 = u * x[3].n;
    y1.n = weighted_x2 + weighted_x3;
    y1.d = weighted_x2.w * x[2].d + weighted_x3.w * x[3].d;
    y1.d /= y1.n.w;

    // Make sure y0 is front and y1 is back
    if (y0.n.z > y1.n.z) {
      //swap(y0, y1);
      vertex_data temp = y0;
      y0 = y1;
      y1 = temp;
    }

    // Draw four triangles.
    // x[0] - x[2] - y
    outputSimpleVertex(x[0]);
    outputSimpleVertex(x[2]);
    outputVertex(y0, y1);

    // x[2] - y - x[1]
    outputSimpleVertex(x[1]);

    // A null triangle: y - x[1] - y
    outputVertex(y0, y1);

    // x[1] - y - x[3]
    outputSimpleVertex(x[3]);

    // y - x[3] - x[0]
    outputSimpleVertex(x[0]);
    EndPrimitive();
  }
}
