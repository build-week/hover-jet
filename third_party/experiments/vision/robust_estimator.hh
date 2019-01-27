#pragma once

#include <cmath>

namespace slam {

struct CostWeight {
  double cost;
  double weight;
};

class RobustEstimator {
 public:
  virtual ~RobustEstimator() = default;

  virtual CostWeight cost_weight(const double x2) const = 0;

  CostWeight operator()(const double x2) const {
    return cost_weight(x2);
  }
};

class L2Cost final : public RobustEstimator {
 public:
  CostWeight cost_weight(const double x2) const override {
    return {x2, 1.0};
  }
};

//
// https://hal.inria.fr/inria-00074015/document
class HuberCost final : public RobustEstimator {
 public:
  HuberCost(const double k) {
    k_  = k;
    kk_ = k * k;
  }

  CostWeight cost_weight(const double x2) const override {
    if (x2 < kk_) {
      return {x2 * 0.5, 1.0};
    } else {
      const double x = std::sqrt(x2);
      return {(k_ * x) - (0.5 * kk_), k_ / x};
    }
  }

 private:
  double k_  = 0.0;
  double kk_ = 0.0;
};

// https://hal.inria.fr/inria-00074015/document
class TukeyCost final : public RobustEstimator {
 public:
  TukeyCost(const double c) {
    cc_ = c * c;
    c_  = c;
  }

  CostWeight cost_weight(const double x2) const override {
    constexpr double six_inv = 1.0 / 6.0;
    if (x2 <= cc_) {
      // See journal for poly expansion, maybe not faster
      const double one_minus_term  = (1.0 - (x2 / cc_));
      const double one_minus_term2 = one_minus_term * one_minus_term;
      const double one_minus_term3 = one_minus_term2 * one_minus_term;

      const double cost   = (cc_ * six_inv) * (1.0 - one_minus_term3);
      const double weight = one_minus_term2;
      return {cost, weight};

    } else {
      const double cost = cc_ * six_inv;
      return {cost, 0.0};
    }
  }

 private:
  double cc_ = 0.0;
  double c_  = 0.0;
};
}