#include "fluids/fields_2d.hh"

#include "eigen.hh"

namespace jcc {
namespace fluids {
namespace plane {

std::array<Eigen::MatrixXd, 2> mul(const std::array<Eigen::MatrixXd, 2>& a,
                                   const double b) {
  std::array<Eigen::MatrixXd, 2> c;
  constexpr int N = 2;
  for (int k = 0; k < N; ++k) {
    c[k] = a[k] * b;
  }
  return c;
}

std::array<Eigen::MatrixXd, 2> add(const std::array<Eigen::MatrixXd, 2>& a,
                                   const std::array<Eigen::MatrixXd, 2>& b) {
  std::array<Eigen::MatrixXd, 2> c;
  constexpr int N = 2;
  for (int k = 0; k < N; ++k) {
    c[k] = a[k] + b[k];
  }
  return c;
}

// Vector field operations
// ∇f(x, y)
std::array<Eigen::MatrixXd, 2> compute_gradient(const Eigen::MatrixXd& potential_field,
                                                const double dx) {
  std::array<Eigen::MatrixXd, 2> out;

  const int rows = potential_field.rows();
  const int cols = potential_field.cols();

  out[0] = Eigen::MatrixXd::Zero(rows, cols);
  out[1] = Eigen::MatrixXd::Zero(rows, cols);

  const double inv_dx = 1.0 / (2.0 * dx);

  out[0].middleRows(1, rows - 1) =
      (potential_field.bottomRows(rows - 1) - potential_field.topRows(rows - 1)) * inv_dx;

  out[1].middleCols(1, cols - 1) =
      (potential_field.rightCols(cols - 1) - potential_field.leftCols(cols - 1)) * inv_dx;

  return out;
}

// Scalar field operations
// ∇·<x, y>
Eigen::MatrixXd compute_divergence(const std::array<Eigen::MatrixXd, 2>& vector_field,
                                   const double dx) {
  const auto& u = vector_field[0];
  const auto& v = vector_field[1];

  const int rows = u.rows();
  const int cols = u.cols();

  Eigen::MatrixXd result = Eigen::MatrixXd::Zero(rows, cols);

  const double inv_dx = 1.0 / (2.0 * dx);
  result.middleRows(1, rows - 1) +=
      (u.bottomRows(rows - 1) - u.topRows(rows - 1)) * inv_dx;

  result.middleCols(1, cols - 1) +=
      (v.rightCols(cols - 1) - v.leftCols(cols - 1)) * inv_dx;

  return result;
}

// Δf(x, y)
Eigen::MatrixXd compute_laplacian(const Eigen::MatrixXd& field, const double dx) {
  const int rows = field.rows();
  const int cols = field.cols();

  const double inv_dx = (1.0 / (dx * dx));

  Eigen::MatrixXd result = Eigen::MatrixXd::Zero(rows, cols);
  result.topRows(rows - 1) += field.bottomRows(rows - 1);
  result -= 2.0 * field;
  result.bottomRows(rows - 1) += field.topRows(rows - 1);

  result.leftCols(cols - 1) += field.rightCols(cols - 1);
  result -= 2.0 * field;
  result.rightCols(cols - 1) += field.leftCols(cols - 1);
  result *= inv_dx;

  return result;
}

}  // namespace  plane.
}  // namespace fluids.
}  // namespace jcc.