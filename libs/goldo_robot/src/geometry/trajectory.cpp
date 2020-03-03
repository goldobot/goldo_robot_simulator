#include "goldo/geometry/trajectory.hpp"

#include <algorithm>
#include <cmath>

using namespace goldo;

Trajectory::Trajectory() : mNumPoints(0) {}

void Trajectory::clear() { mNumPoints = 0; }

void Trajectory::setPoints(Vector2D* points, float* knots,
                           unsigned num_points) {
  for (unsigned i = 0; i < num_points; i++) {
    mControlPoints[i] = points[i];
    mKnots[i + 2] = knots[i];
  }

  for (unsigned i = 0; i < 2; i++) {
    mKnots[i] = knots[0];
    mKnots[num_points + 2 + i] = knots[num_points - 1];
  }

  mNumPoints = num_points;
}

void Trajectory::setPoints(Vector2D* points, unsigned num_points) {
  Vector2D points_alt[4];
  if (num_points == 2) {
    points_alt[0] = points[0];
    points_alt[1] = Vector2D{points[0].x * (1.0f-0.33f) + points[1].x * 0.33f, points[0].y * (1.0f-0.33f) + points[1].y * 0.33f};
    points_alt[2] = Vector2D{points[0].x * 0.33f + points[1].x * (1.0f-0.33f), points[0].y * 0.33f + points[1].y * (1.0f-0.33f)};
    points_alt[3] = points[1];
    points = points_alt;
    num_points = 4;
  }
  if (num_points == 3) {
    points = points_alt;
    num_points = 4;
  }

  float knots[16];
  float parameters[16];
  float dist = 0.0f;
  Vector2D p1 = points[0];
  parameters[0] = dist;

  for (unsigned i = 1; i < num_points; i++) {
    Vector2D p2 = points[i];
    float dx = p2.x - p1.x;
    float dy = p2.y - p1.y;
    dist += sqrtf(dx * dx + dy * dy);
    parameters[i] = dist;
    p1 = p2;
  };
  knots[0] = parameters[0];
  knots[1] = parameters[0];
  knots[num_points - 2] = parameters[num_points - 1];
  knots[num_points - 1] = parameters[num_points - 1];

  for (unsigned i = 0; i < num_points - 4; i++) {
    knots[i + 2] =
        (parameters[i + 1] + parameters[i + 2] + parameters[i + 3]) / 3.0f;
  }

  setPoints(points, knots, num_points);
  return;
}

float Trajectory::minParameter() const {
  return mNumPoints > 0 ? mKnots[0] : 0;
}

float Trajectory::maxParameter() const {
  return mNumPoints > 0 ? mKnots[mNumPoints + 2] : 0;
}

TrajectoryPoint Trajectory::computePoint(float x) const {
  if (x < minParameter()) {
    x = minParameter();
  }

  if (x > maxParameter()) {
    x = maxParameter();
  }

  // Find knot interval containing output
  int k = 3;
  while (mKnots[k + 1] < x) {
    k++;
  }

  Vector2D d0[4];
  Vector2D d1[4];
  Vector2D d2[4];

  for (int j = 0; j < 4; j++) {
    d0[j] = mControlPoints[j + k - 3];
    d1[j] = Vector2D{0, 0};
    d2[j] = Vector2D{0, 0};
  }

  // Recursively compute position
  for (int r = 1; r < 4; r++) {
    for (int j = 3; j >= r; j--) {
      int k1 = k + j - 3;
      int k2 = k1 + 4 - r;
      float alpha = 0;
      float d_alpha = 0;
      if (mKnots[k2] - mKnots[k1] > 1e-12) {
        d_alpha = 1.0f / (mKnots[k2] - mKnots[k1]);
        alpha = (x - mKnots[k1]) / (mKnots[k2] - mKnots[k1]);
      }

      d1[j].x = (1.0f - alpha) * d1[j - 1].x - d_alpha * d0[j - 1].x +
                alpha * d1[j].x + d_alpha * d0[j].x;
      d1[j].y = (1.0f - alpha) * d1[j - 1].y - d_alpha * d0[j - 1].y +
                alpha * d1[j].y + d_alpha * d0[j].y;

      d0[j].x = (1.0f - alpha) * d0[j - 1].x + alpha * d0[j].x;
      d0[j].y = (1.0f - alpha) * d0[j - 1].y + alpha * d0[j].y;
    }
  }

  float speed = sqrtf(d1[3].x * d1[3].x + d1[3].y * d1[3].y);
  float curvature = sqrtf(d2[3].x * d2[3].x + d2[3].y * d2[3].y);
  float t_c = 1.0f / speed;
  return TrajectoryPoint{d0[3], d1[3].x * t_c, d1[3].y * t_c, speed,
                         curvature * t_c * t_c};
}
