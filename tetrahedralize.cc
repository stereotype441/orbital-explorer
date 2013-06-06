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
#include <set>
#include <map>
#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdio>

#include "util.hh"
#include "array.hh"
#include "genericops.hh"
#include "vector.hh"
#include "matrix.hh"
#include "function.hh"
#include "delaunay.hh"
#include "tetrahedralize.hh"

using namespace std;

double TetrahedralSubdivision::simplexVolume(unsigned tetra) const
{
  const Simplex<3> &simplex = subdivision.getSimplex(tetra);

  const Vector<3> &w = subdivision.getPoint(simplex.formingPoint(0));
  const Vector<3> &x = subdivision.getPoint(simplex.formingPoint(1));
  const Vector<3> &y = subdivision.getPoint(simplex.formingPoint(2));
  const Vector<3> &z = subdivision.getPoint(simplex.formingPoint(3));
  return fabs(dot_product(x - w, cross_product(y - w, z - w)));
}

pair<Vector<3>,double>
TetrahedralSubdivision::find_worst_point(unsigned tetra)
{
  const Simplex<3> &simplex = subdivision.getSimplex(tetra);

  // Worst point so far, and its absolute error
  Vector<3> worst_point;
  double worst_point_absolute_error = 0.0;

  // Values of the function at the simplex's vertices
  vector<complex<double> > vertex_value(4);
  for (unsigned i = 0; i < 4; ++i) {
    const Vector<3> &v = subdivision.getPoint(simplex.formingPoint(i));
    vertex_value[i] = f(v);
  }

  // Test points have barycentric coordinates that are multiples of 1/n
  const unsigned n = 11;
  unsigned bary[4];
  Vector<3> p0, p1, test_point(0.0);
  for (bary[0] = 0; bary[0] < n; ++bary[0]) {
    p0 = (double(bary[0]) / double(n))
      * subdivision.getPoint(simplex.formingPoint(0));
    for (bary[1] = 0; bary[1] <= n - bary[0]; ++bary[1]) {
      if (bary[1] == n)
        continue;
      p1 = p0 + (double(bary[1]) / double(n))
        * subdivision.getPoint(simplex.formingPoint(1));
      for (bary[2] = 0; bary[2] <= n - bary[0] - bary[1];
           ++bary[2]) {
        if (bary[2] == n)
          continue;

        bary[3] = n - bary[0] - bary[1] - bary[2];
        if (bary[3] == n)
          continue;

        // Calculate the location of this test point
        test_point = p1;
        for (unsigned i = 2; i < 4; ++i) {
          double c = double(bary[i]) / double(n);
          const Vector<3> &v = subdivision.getPoint(simplex.formingPoint(i));
          test_point += c * v;
        }

        // Is it worse than the worst so far?
        complex<double> actual_value = f(test_point);
        complex<double> interpolated_value = 0.0;
        for (unsigned i = 0; i < 4; ++i) {
          double c = double(bary[i]) / double(n);
          interpolated_value += c * vertex_value[i];
        }
        double test_point_absolute_error = abs(actual_value - interpolated_value);
        if (test_point_absolute_error > worst_point_absolute_error) {
          worst_point_absolute_error = test_point_absolute_error;
          worst_point = test_point;
        }
      }
    }
  }

  return make_pair(worst_point, worst_point_absolute_error);
}

bool TetrahedralSubdivision::isBoundary(unsigned tetra)
{
  const Simplex<3> &simplex = subdivision.getSimplex(tetra);

  unsigned i;
  for (i = 0; i < 4; ++i)
    if (simplex.formingPoint(i) < 4)
      break;

  return i < 4;
}

void TetrahedralSubdivision::handleNewTetrahedron(unsigned tetra)
{
  // It might have already been subdivided
  if (!subdivision.hasSimplex(tetra))
    return;

  // Ignore tetrahedra that go way out to the giant radius
  if (isBoundary(tetra))
    return;

  // Find the worst point in this tetrahedron
  pair<Vector<3>,double> worst = find_worst_point(tetra);
  Vector<3> worst_point = worst.first;
  double worst_point_absolute_error = worst.second;

  double volume = simplexVolume(tetra);

  double error = pow(worst_point_absolute_error, 2.0) * volume;

  heap_of_tetrahedra.push_back(TetraHeapItem(error, tetra, worst_point));
  push_heap(heap_of_tetrahedra.begin(), heap_of_tetrahedra.end());
}

TetrahedralSubdivision::
TetrahedralSubdivision(const Function<3,complex<double> > &f_, double radius) :
  f(f_), running(false), finished(false), die(false)
{
  // Set up an initial bounding tetrahedron of a large size
  Array<4,Vector<3> > bounding_tetrahedron;
  double big = 6.0 * radius;
  bounding_tetrahedron[0] = Vector3( big,  big,  big);
  bounding_tetrahedron[1] = Vector3( big, -big, -big);
  bounding_tetrahedron[2] = Vector3(-big,  big, -big);
  bounding_tetrahedron[3] = Vector3(-big, -big,  big);

  // Create a Delaunay triangulation
  subdivision = Delaunay<3>(bounding_tetrahedron);

  // Add some vertices to bound the radius of significance
  Vector<3> point;
  for (int i=-1; i<=1; i+=2)
    for (int j=-1; j<=1; j+=2)
      for (int k=-1; k<=1; k+=2) {
        point[0] = i * radius;
        point[1] = j * radius;
        point[2] = k * radius;
        subdivision.inefficientAddPoint(point);
      }

  // Add tetrahedra to a heap sorted by worst error
  for (examined = 1; examined <= subdivision.maxSimplex(); ++examined)
    handleNewTetrahedron(examined);

  pthread_mutex_init(&mutex, NULL);
}

bool TetrahedralSubdivision::isRunning()
{
  pthread_mutex_lock(&mutex);
  bool blet = running;
  pthread_mutex_unlock(&mutex);
  return blet;
}

bool TetrahedralSubdivision::isFinished()
{
  pthread_mutex_lock(&mutex);
  bool blet = finished;
  finished = false;
  pthread_mutex_unlock(&mutex);
  return blet;
}

void TetrahedralSubdivision::work(unsigned vertices)
{
  while (heap_of_tetrahedra.size() > 0 && subdivision.numPoints() < vertices) {
    TetraHeapItem next_tetrahedron = heap_of_tetrahedra[0];
    pop_heap(heap_of_tetrahedra.begin(), heap_of_tetrahedra.end());
    heap_of_tetrahedra.pop_back();

    if (!subdivision.hasSimplex(next_tetrahedron.tetra))
      continue;

    pthread_mutex_lock(&mutex);
    if (die) {
      running = false;
      finished = true;
      pthread_mutex_unlock(&mutex);
      return;
    }
    subdivision.addPoint(next_tetrahedron.point,
                         next_tetrahedron.tetra);
    pthread_mutex_unlock(&mutex);

    // Add any new tetrahedra to the heap
    for (; examined <= subdivision.maxSimplex(); ++examined)
      handleNewTetrahedron(examined);
  }

  pthread_mutex_lock(&mutex);
  running = false;
  finished = true;
  pthread_mutex_unlock(&mutex);
}

static void *start_worker(void *arg)
{
  WorkerThreadData *worker_data = reinterpret_cast<WorkerThreadData *>(arg);
  worker_data->self->work(worker_data->vertices);
  return NULL;
}

void TetrahedralSubdivision::runUntil(unsigned vertices)
{
  running = true;
  worker_data.self = this;
  worker_data.vertices = vertices;
  pthread_create(&worker, NULL, start_worker, &worker_data);
}

void TetrahedralSubdivision::kill()
{
  pthread_mutex_lock(&mutex);
  if (running) {
    die = true;
    // Spin until the other thread exits
    while (running) {
      pthread_mutex_unlock(&mutex);
      pthread_mutex_lock(&mutex);
    }
    die = false;
  }
  pthread_mutex_unlock(&mutex);
}

int TetrahedralSubdivision::numVertices()
{
  pthread_mutex_lock(&mutex);
  int n = subdivision.numPoints();
  pthread_mutex_unlock(&mutex);
  return n;
}

vector<float> TetrahedralSubdivision::vertexPositions()
{
  pthread_mutex_lock(&mutex);
  unsigned num_points = subdivision.numPoints();
  vector<float> vp(num_points * 3);
  for (unsigned p = 0; p < num_points; ++p) {
    Vector<3> point = subdivision.getPoint(p);
    vp[3 * p + 0] = point[0];
    vp[3 * p + 1] = point[1];
    vp[3 * p + 2] = point[2];
  }
  pthread_mutex_unlock(&mutex);
  return vp;
}

vector<unsigned> TetrahedralSubdivision::tetrahedronVertexIndices()
{
  vector<unsigned> vi;
  pthread_mutex_lock(&mutex);
  for (unsigned simplex_index = 0;
       simplex_index <= subdivision.maxSimplex();
       ++simplex_index) {
    if (!subdivision.hasSimplex(simplex_index))
      continue;
    const Simplex<3> &simplex = subdivision.getSimplex(simplex_index);
    unsigned i;
    for (i = 0; i < 4; ++i)
      if (simplex.formingPoint(i) < 4)
        break;
    if (i != 4) continue;
    for (i = 0; i < 4; ++i)
      vi.push_back(simplex.formingPoint(i));
  }
  pthread_mutex_unlock(&mutex);
  return vi;
}
