#include "geometry/visualization/visualize_nullspace.hh"

#include "geometry/perp.hh"

#include "numerics/optimize.hh"
#include "numerics/wrap_optimizer.hh"

// todo
#include "viewer/window_3d.hh"

namespace geometry {
namespace visualization {

namespace {
using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;

using ImplicitSurfaceFnc = const std::function<double(const Eigen::VectorXd&)>;

class NullspaceSearch {
 public:
  NullspaceSearch(const std::function<double(const Eigen::VectorXd&)>& fnc) {
    wrapped_fnc = numerics::wrap_numerical_grad(fnc);
    const auto fnc_sqr = [this](const Eigen::VectorXd& x,
                                Eigen::VectorXd* const gradient,
                                Eigen::MatrixXd* const hessian) {
      const double val = this->wrapped_fnc(x, gradient, hessian);

      // Gradient for f^2
      if (gradient) {
        (*gradient) *= 2.0 * val;
      }
      return val * val;
    };

    problem_ = {fnc_sqr, {}, {}};
  }

  Eigen::VectorXd project(const Eigen::VectorXd& x) const {
    const numerics::OptimizationState initialization{x, {}};
    const auto result =
        numerics::optimize<numerics::ObjectiveMethod::kGradientDescent,
                           numerics::ConstraintMethod::kAugLag>(initialization, problem_);
    return result.x;
  }

  Eigen::VectorXd gradient(const Eigen::VectorXd& x) const {
    Eigen::VectorXd g;
    wrapped_fnc(x, &g, nullptr);
    return g;
  }

 private:
  numerics::CostFunction wrapped_fnc;
  numerics::OptimizationProblem problem_;
};

Vec3 augment(const Eigen::VectorXd& x) {
  Vec3 point = Vec3::Zero();
  if (x.size() > 3) {
    point = x.head(3);
  } else {
    point.head(x.size()) = x;
  }

  return point;
}

}  // namespace

void visualize_nullspace(viewer::SimpleGeometry& geo,
                         const ImplicitSurfaceFnc& fnc,
                         const Eigen::VectorXd& x0) {
  const NullspaceSearch nss(fnc);

  const double step_size = 0.25;

  // viewer::Polygon poly;
  // poly.color = Eigen::Vector4d(0.0, 0.0, 0.0, 0.0);
  // poly.outline = true;

  viewer::Points points;

  Eigen::VectorXd x = nss.project(x0);
  Eigen::VectorXd prev_dir = perp(nss.gradient(x));
  for (int k = 0; k < 120; ++k) {
    const Eigen::VectorXd projected = nss.project(x);

    const Eigen::VectorXd df_dx = nss.gradient(projected);

    Eigen::VectorXd direction = perp(df_dx);

    if (direction.dot(prev_dir) < 0.0) {
      direction *= -1.0;
    }
    prev_dir = direction;

    const Eigen::VectorXd next_x = projected + (direction * step_size);

    const auto point = augment(projected);

    constexpr bool DEBUG = false;
    if (DEBUG) {
      const auto view = viewer::get_window3d("Mr. Demo");
      geo.add_ray({point, augment(direction), step_size, Vec4(1.0, 0.0, 0.0, 0.7)});
      geo.add_point({augment(next_x), Vec4(1.0, 1.0, 0.0, 0.9), 20.0});
      geo.flush();
      view->spin_until_step();
    }
    // poly.points.push_back(point);
    points.points.push_back(point);

    x = next_x;
  }

  // geo.add_polygon(poly);
  geo.add_points(points);
}
}  // namespace visualization
}  // namespace geometry
