#include "viewer/primitives/plot.hh"

#include <GL/glew.h>
#include "viewer/gl_aliases.hh"

// TODO
#include <iostream>
#include <limits>

namespace viewer {
using Vec2 = Eigen::Vector2d;
using Vec4 = Eigen::Vector4d;

Histogram::Histogram(const std::vector<double> &values, const HistogramConfig &config, const Vec4 &color) {
  config_ = config;
  color_ = color;
  double min = std::numeric_limits<double>::max();
  double max = std::numeric_limits<double>::lowest();

  if (config_.force_min_max) {
    min = config_.min;
    max = config_.max;
  } else {
    for (size_t k = 0; k < values.size(); ++k) {
      min = std::min(values[k], min);
      max = std::max(values[k], max);
    }
  }

  const double range = max - min;
  const double bin_scale = config_.num_bins / range;

  std::vector<int> count(config_.num_bins, 0);
  for (size_t k = 0; k < values.size(); ++k) {
    const int bin_id = static_cast<int>((values[k] - min) * bin_scale);
    if (bin_id < config_.num_bins && bin_id > 0) {
      ++count[bin_id];
    }
  }

  max_count_ = 0;
  for (int k = 0; k < static_cast<int>(count.size()); ++k) {
    max_count_ = std::max(count[k], max_count_);
  }

  count_ = std::move(count);
  bin_size_ = 1.0 / bin_scale;
}

void Histogram::draw() const {
  const double y_normalizer = 1.0 / max_count_;
  const double x_normalizer = 1.0 / static_cast<double>(count_.size());

  glColor4f(1.0, 1.0, 1.0, 0.8);
  glBegin(GL_LINE_STRIP);
  for (size_t k = 0; k < count_.size(); ++k) {
    const double x = k * x_normalizer;
    const double y = count_[k] * y_normalizer;
    const Vec2 line_bottom(x, 0.0);
    const Vec2 line_top(x, y);
    const Vec2 offset(x_normalizer, 0.0);
    glVertex(line_bottom);
    glVertex(line_top);
    glVertex(Vec2(line_top + offset));
    glVertex(Vec2(line_bottom + offset));
  }
  glEnd();

  glColor(color_);
  glBegin(GL_QUADS);
  for (size_t k = 0; k < count_.size(); ++k) {
    const double x = k * x_normalizer;
    const double y = count_[k] * y_normalizer;
    const Vec2 line_bottom(x, 0.0);
    const Vec2 line_top(x, y);
    const Vec2 offset(x_normalizer, 0.0);
    glVertex(line_bottom);
    glVertex(line_top);
    glVertex(Vec2(line_top + offset));
    glVertex(Vec2(line_bottom + offset));
  }
  glEnd();
}

namespace {

void draw_surface(const Surface &surface) {
  glBegin(GL_QUADS);

  const std::vector<std::array<int, 2>> offsets = {{0, -1}, {-1, -1}, {-1, 0}};
  const double max = surface.surface.maxCoeff();
  const double min = surface.surface.minCoeff();
  const double range = max - min;

  constexpr double EPS = 1e-6;
  const double normalizer = range < EPS ? 1.0 : 1.0 / range;

  for (int row = 1; row < surface.surface.rows(); ++row) {
    for (int col = 1; col < surface.surface.cols(); ++col) {
      const double value = surface.surface(row, col);
      glVertex3d(row * surface.scale, col * surface.scale, value);

      glColor4f(normalizer * value, (normalizer * value * 0.5) + 0.2, (normalizer * value * 0.25) + 0.2, 0.8);

      for (const auto &r : offsets) {
        const int &x = r[0];
        const int &y = r[1];

        const int row_plus_x = row + x;
        const int col_plus_y = col + y;

        const double value = surface.surface(row_plus_x, col_plus_y);

        glVertex3d(row_plus_x * surface.scale, col_plus_y * surface.scale, value);
      }
    }
  }

  glEnd();
}
} // namespace

void Plot::draw() const {
  for (const auto &surface : surfaces_) {
    draw_surface(surface);
  }
  for (const auto &histogram : histograms_) {
    histogram.draw();
  }
}
} // namespace viewer
