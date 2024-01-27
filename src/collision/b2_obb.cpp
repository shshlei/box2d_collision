#include "box2d_collision/b2_math.h"

b2OBB::b2OBB(const b2Mat22 & axis, const b2Vec2 & center, const b2Vec2 & extent): axis_(axis), center_(center), extent_(extent)
{
}

const b2Mat22 & b2OBB::axis() const
{
  return axis_;
}

const b2Vec2 & b2OBB::center() const
{
  return center_;
}

const b2Vec2 & b2OBB::extent() const
{
  return extent_;
}

b2Scalar b2OBB::width() const
{
  return 2.0 * extent_[0];
}

b2Scalar b2OBB::height() const
{
  return 2.0 * extent_[1];
}

b2Scalar b2OBB::volume() const
{
  return width() * height();
}

b2Scalar b2OBB::size() const
{
  return extent_.squaredNorm();
}

bool b2OBB::contain(const b2Vec2 & p) const
{
  b2Vec2 local_p = p - center_;
  b2Scalar proj = local_p.dot(axis_.col(0));
  if ((proj > extent_[0]) || (proj < -extent_[0]))
    return false;

  proj = local_p.dot(axis_.col(1));
  if ((proj > extent_[1]) || (proj < -extent_[1]))
    return false;

  return true;
}

bool b2OBB::overlap(const b2OBB & other) const
{
  /// compute the relative transform
  b2Vec2 t = other.center() - center_;
  b2Vec2 T(axis_.col(0).dot(t), axis_.col(1).dot(t));
  b2Mat22 R = axis_.transpose() * other.axis();
  return !obbDisjoint(R, T, extent_, other.extent());
}

void computeVertices(const b2OBB & b, b2Vec2 vertices[4])
{
  b2Vec2 extaxis_0 = b.axis().col(0) * b.extent()[0];
  b2Vec2 extaxis_1 = b.axis().col(1) * b.extent()[1];
  vertices[0] = b.center() - extaxis_0 - extaxis_1;
  vertices[1] = b.center() + extaxis_0 - extaxis_1;
  vertices[2] = b.center() + extaxis_0 + extaxis_1;
  vertices[3] = b.center() - extaxis_0 + extaxis_1;
}

bool obbDisjoint(const b2Mat22 & B, const b2Vec2 & T, const b2Vec2 & a, const b2Vec2 & b)
{
  b2Scalar t, s;
  const b2Scalar reps = 1e-6;

  b2Mat22 Bf = B.cwiseAbs();
  Bf.array() += reps;

  // if any of these tests are one-sided, then the polyhedra are disjoint

  // A1 x A2 = A0
  t = ((T[0] < 0.0) ? -T[0] : T[0]);
  if (t > (a[0] + Bf.row(0).dot(b)))
    return true;

  // B1 x B2 = B0
  s = B.col(0).dot(T);
  t = ((s < 0.0) ? -s : s);
  if (t > (b[0] + Bf.col(0).dot(a)))
    return true;

  // A2 x A0 = A1
  t = ((T[1] < 0.0) ? -T[1] : T[1]);
  if (t > (a[1] + Bf.row(1).dot(b)))
    return true;

  // B2 x B0 = B1
  s = B.col(1).dot(T);
  t = ((s < 0.0) ? -s : s);
  if (t > (b[1] + Bf.col(1).dot(a)))
    return true;

  return false;
}
