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

template <unsigned n>
Vector<n> find_circumcenter(const Array<n+1,Vector<n> > &xs)
{
  Matrix<n,n> a;
  for (unsigned i=0; i<n; ++i)
    for (unsigned j=0; j<n; ++j)
      a(i,j) = xs[i][j] - xs[n][j];
  Vector<n> b;
  for (unsigned i=0; i<n; ++i)
    b[i] = (norm_squared(xs[i]) - norm_squared(xs[n])) / 2.0;
  return inverse(a) * b;
}

template<unsigned n>
double average_distance_squared(const Vector<n> &x,
                                const Array<n+1,Vector<n> > &ys)
{
  double sum = 0.0;

  for (unsigned i=0; i<n+1; ++i)
    sum += norm_squared(x - ys[i]);

  return sum / double(n+1);
}

template <unsigned n>
struct Face
{
  Face() {} // needed for unit tests
  explicit Face(const Array<n,unsigned> &ps) : points(ps) {}
  Array<n,unsigned> points;
  bool operator==(const Face &rhs) const
  {
    for (unsigned i=0; i<n; ++i)
      if (points[i] != rhs.points[i])
        return false;
    return true;
  }
  bool operator<(const Face &rhs) const
  {
    for (unsigned i=0; i<n; ++i)
      if (points[i] < rhs.points[i])
        return true;
      else if (points[i] > rhs.points[i])
        return false;
    return false; // a < a -> false
  }
};

template <unsigned n>
class Simplex
{
public:
  Simplex() {} // needed for unit tests
  Simplex(const std::vector<Vector<n> > &, const Array<n+1,unsigned> &);

  // unclear which of the following getters are needed
  unsigned formingPoint(unsigned i) const { return fp[i]; }
  unsigned &adjacency(unsigned i) { return adj[i]; }
  unsigned adjacency(unsigned i) const { return adj[i]; }
  const Vector<n> &circumcenter() const { return c; }
  double radiusSquared() const { return r2; }

  bool isInsideCircumsphere(const Vector<n> &) const;
  Array<n+1,Face<n> > faces() const;
  unsigned connectAcrossFace(const Face<n> &, unsigned);

private:
  Array<n+1,unsigned> fp;
  Array<n+1,unsigned> adj;
  Vector<n> c;
  double r2;
};

template <unsigned n>
inline Simplex<n>::Simplex(const std::vector<Vector<n> > &points,
                           const Array<n+1,unsigned> &fp_)
  : fp(fp_),
    adj(0)
{
  Array<n+1, Vector<n> > point_locations;
  for (unsigned i=0; i<=n; ++i)
    point_locations[i] = points.at(fp[i]);
  c = find_circumcenter(point_locations);
  r2 = average_distance_squared(c, point_locations);
}

template <>
inline Simplex<3>::Simplex(const std::vector<Vector<3> > &points,
                           const Array<4,unsigned> &fp_)
  : fp(fp_),
    adj(0)
{
  Array<4, Vector<3> > point_locations;
  for (unsigned i = 0; i <= 3; ++i)
    point_locations[i] = points.at(fp[i]);
  c = find_circumcenter(point_locations);
  r2 = average_distance_squared(c, point_locations);

  // Sanity test: volume must be positive
  Array<3, Vector<3> > side_vectors;
  for (unsigned i = 0; i < 3; ++i)
    side_vectors[i] = point_locations[i] - point_locations[3];
  double volume = dot_product(cross_product(side_vectors[0], side_vectors[1]),
                              side_vectors[2]);
  if (fabs(volume) < 1e-12) {
    for (unsigned i = 0; i < 4; ++i)
      printf("Forming point %d: %f %f %f\n", i, point_locations[i][0],
             point_locations[i][1], point_locations[i][2]);
    throw std::logic_error("Tetrahedron with no volume\n");
  }

  // Sanity test: detect possible numerical error
  double shortest_side_length = 1e30;
  for (unsigned i = 0; i < 4; ++i)
    for (unsigned j = i + 1; j < 4; ++j) {
      double this_side_length = norm(point_locations[i] - point_locations[j]);
      if (this_side_length < shortest_side_length)
        shortest_side_length = this_side_length;
    }
  if (shortest_side_length < 1e-6) {
    printf("Shortest side: %f\n", shortest_side_length);
    throw std::logic_error("Simplex got too small\n");
  }
}

template <unsigned n>
bool Simplex<n>::isInsideCircumsphere(const Vector<n> &v) const
{
  return norm_squared(v-c) < r2 * (1. + 1e-9);
}

template <unsigned n>
Array<n+1,Face<n> > Simplex<n>::faces() const
{
  Array<n+1,Face<n> > fs;

  // fs[0] = [ fp[1] fp[2] fp[3] ]
  // fs[1] = [ fp[0] fp[2] fp[3] ]
  // fs[2] = [ fp[0] fp[1] fp[3] ]
  // fs[3] = [ fp[0] fp[1] fp[2] ]
  for (unsigned i=0; i<n+1; ++i)
    for (unsigned j=0; j<n; ++j)
      fs[i].points[j] = (j >= i ? fp[j+1] : fp[j]);

  return fs;
}

template <unsigned n>
unsigned Simplex<n>::connectAcrossFace(const Face<n> &face,
                                       unsigned other_simplex_index)
{
  // Find the position of the forming point which is not on the face
  unsigned forming_point_pos;
  for (forming_point_pos = 0; forming_point_pos < n; ++forming_point_pos)
    if (formingPoint(forming_point_pos) != face.points[forming_point_pos])
      break;

  unsigned old_connection = adjacency(forming_point_pos);
  adjacency(forming_point_pos) = other_simplex_index;

  return old_connection;
}

template <unsigned n>
class Delaunay
{
public:
  Delaunay() {} // needed for unit tests
  Delaunay(const Array<n+1,Vector<n> > &);

  unsigned numPoints() const;
  const Vector<n> &getPoint(unsigned) const;
  unsigned maxSimplex() const;
  bool hasSimplex(unsigned) const;
  const Simplex<n> &getSimplex(unsigned) const;

  void inefficientAddPoint(const Vector<n> &);
  void addPoint(const Vector<n> &, unsigned);

private:
  unsigned findOneDeletedSimplex(const Vector<n> &) const;
  std::set<unsigned> findDeletedSimplices(const Vector<n> &, unsigned) const;
  typedef std::map<Face<n>,unsigned> Hole;
  Hole deleteSimplices(const std::set<unsigned> &);
  void deleteSimplex(Hole &, unsigned);
  void addSimplices(unsigned, Hole &);
  void addSimplex(Hole &, Simplex<n> &);

  std::vector<Vector<n> > points;
  unsigned max_simplex;
  std::map<unsigned, Simplex<n> > simplex_map;
};

template <unsigned n>
Delaunay<n>::Delaunay(const Array<n+1,Vector<n> > &vs)
  : points(vs.toVector()),
    max_simplex(1),
    simplex_map()
{
  Array<n+1,unsigned> ind;
  for (unsigned i=0; i<n+1; ++i)
    ind[i] = i;
  simplex_map = singleton<unsigned,Simplex<n> >(1, Simplex<n>(points, ind));
}

template <unsigned n>
unsigned Delaunay<n>::numPoints() const
{
  return points.size();
}

template <unsigned n>
const Vector<n> &Delaunay<n>::getPoint(unsigned i) const
{
  if (i >= points.size())
    throw std::range_error("Tried to get an out-of-range point");
  return points[i];
}

template <unsigned n>
unsigned Delaunay<n>::maxSimplex() const
{
  return max_simplex;
}

template <unsigned n>
bool Delaunay<n>::hasSimplex(unsigned i) const
{
  return simplex_map.find(i) != simplex_map.end();
}

template <unsigned n>
const Simplex<n> &Delaunay<n>::getSimplex(unsigned i) const
{
  typename std::map<unsigned,Simplex<n> >::const_iterator s;
  s = simplex_map.find(i);
  if (s == simplex_map.end())
    throw std::logic_error("getSimplex() called on nonexistent simplex");
  return s->second;
}

template <unsigned n>
void Delaunay<n>::inefficientAddPoint(const Vector<n> &v)
{
  addPoint(v, findOneDeletedSimplex(v));
}

template <unsigned n>
unsigned Delaunay<n>::findOneDeletedSimplex(const Vector<n> &v) const
{
  for (unsigned i=0; i<=max_simplex; ++i)
    if (hasSimplex(i) && getSimplex(i).isInsideCircumsphere(v))
      return i;

  throw std::logic_error("Couldn\'t find a simplex circumscribing the point");
}

template <unsigned n>
void Delaunay<n>::addPoint(const Vector<n> &new_point, unsigned in_simplex)
{
  unsigned new_point_index = points.size();
  points.push_back(new_point);

  // Find all simplices whose circumspheres enclose the new point
  std::set<unsigned> deleted_set = findDeletedSimplices(new_point, in_simplex);

  // Delete them and keep track of the hole
  Hole hole = deleteSimplices(deleted_set);

  // Add new simplices to fill in the hole
  addSimplices(new_point_index, hole);
}

template <unsigned n>
std::set<unsigned>
Delaunay<n>::findDeletedSimplices(const Vector<n> &v, unsigned start) const
{
  std::set<unsigned> deleted_set;
  std::vector<unsigned> frontier;
  frontier.push_back(start);
  while (!frontier.empty()) {
    unsigned f = frontier[frontier.size()-1];
    frontier.pop_back();
    if (deleted_set.find(f) != deleted_set.end())
      continue;
    const Simplex<n> &s = getSimplex(f);
    if (!s.isInsideCircumsphere(v))
      continue;
    deleted_set.insert(f);
    for (unsigned i=0; i<n+1; ++i) {
      unsigned a = s.adjacency(i);
      if (a != 0)
        frontier.push_back(a);
    }
  }

  if (deleted_set.empty())
    throw std::logic_error("addPoint: simplex does not circumscribe point");

  return deleted_set;
}

template <unsigned n>
typename Delaunay<n>::Hole
Delaunay<n>::deleteSimplices(const std::set<unsigned> &deleted_set)
{
  Hole hole;
  for (std::set<unsigned>::iterator i = deleted_set.begin();
       i != deleted_set.end(); ++i)
    deleteSimplex(hole, *i);
  return hole;
}

template <unsigned n>
void Delaunay<n>::deleteSimplex(Hole &hole, unsigned delete_me_index)
{
  const Simplex<n> &delete_me = getSimplex(delete_me_index);
  Array<n+1,Face<n> > faces = delete_me.faces();
  for (unsigned j=0; j<n+1; ++j)
    if (hole.find(faces[j]) == hole.end())
      hole[faces[j]] = delete_me.adjacency(j);
    else
      hole.erase(faces[j]);

  simplex_map.erase(delete_me_index);
}

template <unsigned n>
void Delaunay<n>::addSimplices(unsigned new_point_index, Hole &hole)
{
  std::vector<Simplex<n> > simplex_list;
  bool success = false;
  do {
    simplex_list.clear();
    typename Hole::const_iterator i;
    try {
      for (i = hole.begin(); i != hole.end(); ++i) {
        Array<n+1,unsigned> forming_points;
        for (unsigned j=0; j<n; ++j)
          forming_points[j] = i->first.points[j];
        forming_points[n] = new_point_index;
        // The following line can throw an exception if the new simplex that
        // we add is degenerate
        Simplex<n> new_simplex(points, forming_points);
        simplex_list.push_back(new_simplex);
      }

      success = true;
    } catch (std::exception &e) {
      deleteSimplex(hole, i->second);
    }
  } while (!success);

  for (unsigned j = 0; j < simplex_list.size(); ++j)
    addSimplex(hole, simplex_list[j]);

  if (hole.size() != 0)
    throw std::logic_error("Combinatorial error in adding new simplices");
}

template <unsigned n>
void Delaunay<n>::addSimplex(Hole &hole,
                             Simplex<n> &new_simplex)
{
  unsigned new_simplex_index = ++max_simplex;
  Array<n+1,Face<n> > faces = new_simplex.faces();
  for (unsigned j=0; j<n+1; ++j) {
    typename Hole::const_iterator find_face = hole.find(faces[j]);
    if (find_face == hole.end()) {
      hole[faces[j]] = new_simplex_index;
      new_simplex.adjacency(j) = 0;
      continue;
    }
    unsigned adjacent_simplex_index = find_face->second;
    hole.erase(faces[j]);
    new_simplex.adjacency(j) = adjacent_simplex_index;
    if (!adjacent_simplex_index)
      continue;
    if (!hasSimplex(adjacent_simplex_index))
      throw std::logic_error("Tried to connect to a nonexistent simplex");
    unsigned old_connection = simplex_map[adjacent_simplex_index].
      connectAcrossFace(faces[j], new_simplex_index);
    // Sanity check: old connection should be to a nonexistent simplex
    if (hasSimplex(old_connection))
      throw std::logic_error("Tried to connect something already connected");
  }
  simplex_map[new_simplex_index] = new_simplex;
}
