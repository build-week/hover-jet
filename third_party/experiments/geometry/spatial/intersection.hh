#pragma once

namespace geometry {
namespace spatial {

struct Intersection {
  double distance = -1.0;
  bool   intersected = false;

  static Intersection no_intersection() {
    Intersection intersect;
    intersect.intersected = false;
    return intersect;
  }
};
}
}
