#pragma once

#include "eigen.hh"

#include <vector>

namespace viewer {

using Vec2 = Eigen::Vector2d;
using Vec4 = Eigen::Vector4d;

struct Line {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Vec2 start;
  Vec2 end;
  Vec4 color = Vec4::Ones();
};

struct Ray {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Vec2 origin;
  Vec2 direction;
  Vec4 color = Vec4::Ones();
};

struct Circle {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Vec2   center;
  double radius;
  Vec4   color = Vec4::Ones();
};

struct Points {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::vector<Vec2> points;
  Vec4              color = Vec4::Ones();
};

struct Renderables {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::vector<Line>   lines;
  std::vector<Ray>    rays;
  std::vector<Circle> circles;
  std::vector<Points> points;

  void clear() {
    lines.clear();
    rays.clear();
    circles.clear();
    points.clear();
  }
};
}
