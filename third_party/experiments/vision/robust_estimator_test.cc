#include "robust_estimator.hh"

#include "testing/gtest.hh"

namespace slam {

TEST(RobustEstimator, est) {
  // TODO: real test

  TukeyCost hest(1.0);
  for (double z = -1.5; z < 1.5; z += 0.01) {
    std::cerr << hest(z * z).weight << ", ";
  }
}
}