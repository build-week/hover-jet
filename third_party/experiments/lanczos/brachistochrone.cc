#include "numerics/numdiff.hh"
#include "numerics/optimize.hh"

#include "viewer/primitives/simple_geometry.hh"
#include "viewer/window_3d.hh"

namespace lanczos {

Eigen::VectorXd sigmoid_blend(const Eigen::VectorXd &x,
                              const double y0,
                              const double y1) {
  constexpr double scale = 25.0;
  const Eigen::VectorXd x_exp = (scale * x).array().exp();
  const Eigen::VectorXd t = x_exp.array() / (x_exp.array() + 1.0);

  const Eigen::VectorXd convex_blend =
      ((1.0 - t.array()) * y0) + (t.array() * y1);

  return convex_blend;
}

struct BrachistochroneParameters {
  double y0 = 2.0;
  double y1 = 0.0;
  double g = 9.0;
  double dx = 0.001;
};

Eigen::VectorXd add_boundary_conditions(
    const Eigen::VectorXd &y, const BrachistochroneParameters &params) {
  Eigen::VectorXd y_augmented(y.rows() + 2);
  y_augmented(0) = params.y0;
  y_augmented.segment(1, y.rows()) = y;
  y_augmented(y.rows() + 1) = params.y1;
  return y_augmented;
}

void visualize_curve(const Eigen::VectorXd &y,
                     const Eigen::VectorXd &gradient,
                     const BrachistochroneParameters &params) {
  std::cout << "Grad: " << std::endl;
  std::cout << gradient.transpose() << std::endl;

  const auto view = viewer::get_window3d("Curve Visualization");
  view->clear();
  const auto geo = view->add_primitive<viewer::SimpleGeometry>();

  const Eigen::VectorXd y_augmented = add_boundary_conditions(y, params);

  for (int k = 1; k < y_augmented.rows(); ++k) {
    const double x0 = (k - 1) * params.dx;
    const double x1 = k * params.dx;
    const Eigen::Vector3d start(x0, 0.0, y_augmented(k - 1));
    const Eigen::Vector3d end(x1, 0.0, y_augmented(k));
    geo->add_line({start, end});

    constexpr bool VIS_GRAD = true;
    if (VIS_GRAD && k < gradient.rows()) {
      const double grad = gradient(k);
      const double scaled_grad = grad * 0.5;
      const Eigen::Vector3d grad_end(x1, 0.0, y_augmented(k) + scaled_grad);
      geo->add_line({end, grad_end, Eigen::Vector4d(1.0, 0.0, 0.5, 0.5)});
    }
  }
  view->spin_until_step();
}

double cost_function(const Eigen::VectorXd &y,
                     Eigen::VectorXd *const gradient,
                     Eigen::MatrixXd *const hessian) {
  const BrachistochroneParameters params = {};

  const auto true_cost = [params](const Eigen::VectorXd &x,
                                  bool debug_output = false) {
    const Eigen::VectorXd heights = add_boundary_conditions(x, params);

    Eigen::VectorXd arclength(x.rows() + 1);
    for (int k = 1; k < heights.rows(); ++k) {
      const double height_diff = heights(k) - heights(k - 1);
      arclength(k - 1) = std::hypot(height_diff, params.dx);
    }

    const double sqrt_two_g = std::sqrt(2.0 * params.g);
    Eigen::VectorXd velocity(x.rows() + 1);
    for (int k = 1; k < heights.rows(); ++k) {
      const double v_prev = sqrt_two_g * std::sqrt(params.y0 - heights(k - 1));
      const double v_this = sqrt_two_g * std::sqrt(params.y0 - heights(k));
      velocity(k - 1) = 0.5 * (v_prev + v_this);
    }

    const Eigen::VectorXd times = arclength.array() / velocity.array();
    const double total_time = times.sum();
    if (debug_output) {
      std::cout << "arclength:\n" << arclength.transpose() << std::endl;
      std::cout << "velocity:\n" << velocity.transpose() << std::endl;
      std::cout << "times:\n" << times.transpose() << std::endl;
      std::cout << "heights:\n" << heights.transpose() << std::endl;
    }
    return total_time;
  };

  if (gradient) {
    *gradient = numerics::dynamic_numerical_gradient(y, true_cost);

    // Debug Nonsense
    constexpr bool PRINT_DEBUG = false;
    true_cost(y, PRINT_DEBUG);
    static int debug_output_increment = 0;
    if (debug_output_increment % 2 == 0) {
      visualize_curve(y, *gradient, params);
    }
    ++debug_output_increment;
  }

  return true_cost(y);
}

void solve() {
  const auto view = viewer::get_window3d("Curve Visualization");
  view->set_target_from_world(
      SE3(SO3::exp(Eigen::Vector3d(-3.1415 * 0.5, 0.0, 0.0)),
          Eigen::Vector3d::Zero()));

  numerics::OptimizationProblem problem;
  problem.objective = cost_function;

  numerics::OptimizationState state;
  state.x = 1.0 * Eigen::VectorXd::Ones(100).array();

  const auto result =
      numerics::optimize<numerics::ObjectiveMethod::kGradientDescent,
                         numerics::ConstraintMethod::kAugLag>(state, problem);

  view->spin_until_step();
  std::cout << result.x.transpose() << std::endl;
}

}  // namespace lanczos

int main() {
  lanczos::solve();
}