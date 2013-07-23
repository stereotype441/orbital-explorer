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

#ifndef TETRAHEDRALIZE_HH
#define TETRAHEDRALIZE_HH

#include "function.hh"
#include "delaunay.hh"

// Given a complex-valued function on three dimensional space, an initial
// radius, and number of vertices, subdivide an initial, large tetrahedron
// into a union of tetrahedra such that:
// 1. Linear approximation on each tetrahedron, using its four vertices
//    as anchors for the function value, approximates the given function
//    "reasonably well", and
// 2. The number of vertices in the tetrahedral mesh equals the requested
//    number, OR the linear approximation appears to be perfect on each
//    tatrahedron.
// 3. The computation takes place in a secondary thread, and can be polled
//    for whether or not it has finished.
// This computation may be restarted, hence the need for a class to hold
// both the tetrahedral subdivision, and the internal data relevant to the
// subdivision algorithm.

// This gets dangerously close to the "create a class to represent a
// computation" anti-pattern.  :-(

// A helper struct for storing stuff in a maximum priority heap.
struct TetraHeapItem
{
  TetraHeapItem(double error_, unsigned tetra_, Vector<3> point_) :
    error(error_),
    tetra(tetra_),
    point(point_)
  {}
  double error;
  unsigned tetra;
  Vector<3> point;
  bool operator<(const struct TetraHeapItem &rhs) const
  {
    return error < rhs.error;
  }
};

class TetrahedralSubdivision;
struct WorkerThreadData
{
  TetrahedralSubdivision *self;
  unsigned vertices;
};

class TetrahedralSubdivision
{
public:
  TetrahedralSubdivision(const Function<3,std::complex<double> > &f_,
                         double radius);
  void runUntil(unsigned vertices);
  bool isRunning();
  bool isFinished();
  void kill();
  int numVertices();
  std::vector<float> vertexPositions();
  std::vector<unsigned> tetrahedronVertexIndices();

  // Thread interface only, not for class-external use
  void work(unsigned vertices);

private:
  double simplexVolume(unsigned tetra) const;
  std::pair<Vector<3>,double> find_worst_point(unsigned tetra);
  bool isBoundary(unsigned tetra);
  void handleNewTetrahedron(unsigned tetra);
  const Function<3,std::complex<double> > &f;
  bool running, finished, die;
  Delaunay<3> subdivision;
  std::vector<TetraHeapItem> heap_of_tetrahedra;
  unsigned examined;
  pthread_t worker;
  WorkerThreadData worker_data;
  pthread_mutex_t mutex;
};

#endif
