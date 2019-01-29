#pragma once

//%deps(opengl)

#include "viewer/primitives/primitive.hh"

#include "eigen.hh"

#include <vector>

namespace viewer {

struct Surface {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  Eigen::MatrixXd surface;
  double scale;
};

struct HistogramConfig {
  int num_bins;

  // If false, will automatically compute min/max
  bool force_min_max = false;
  // Max/min if force is enabled
  double min;
  double max;
};

class Histogram {
public:
  Histogram(const std::vector<double> &values, const HistogramConfig &config, const Eigen::Vector4d &color);

  void draw() const;

private:
  HistogramConfig config_;
  std::vector<int> count_;
  double bin_size_;
  int max_count_;
  Eigen::Vector4d color_;
};

class Plot final : public Primitive {
public:
  void draw() const override;

  void add_surface(const Surface &surface) {
    surfaces_.push_back(surface);
  }

  void add_histogram(const Histogram &histogram) {
    histograms_.push_back(histogram);
  }

private:
  // Will alignment fuck us?
  std::vector<Surface> surfaces_;
  std::vector<Histogram> histograms_;
};
}
