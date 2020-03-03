#pragma once
#include <goldo/geometry/geometry.hpp>

namespace goldo {

struct TrajectoryPoint {
  Vector2D position;
  Vector2D tangent;
  float speed;
  float curvature;
};

class Trajectory {
 public:
  Trajectory();

  void clear();
  void setPoints(Vector2D* points, float* knots, unsigned num_points);
  void setPoints(Vector2D* points, unsigned num_points);

  float minParameter() const;
  float maxParameter() const;

  //! \brief compute position of point on current segment
  TrajectoryPoint computePoint(float parameter) const;

 private:
  Vector2D mControlPoints[32];
  float mKnots[36];
  unsigned mNumPoints;
};
}  // namespace goldobot
