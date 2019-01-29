#include "vision/bootstrapping/bootstrap_pose.hh"

#include <iomanip>

#include "eigen.hh"

#include "geometry/ray_ray_approx_intersect.hh"
#include "numerics/numdiff.hh"
#include "vision/robust_estimator.hh"

namespace slam {
using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;

using Vec5 = VecNd<5>;
using Mat5 = MatNd<5, 5>;
using Vec6 = VecNd<6>;

BootstrapResult compute_nonmetric_pose(const std::vector<Vec2>& image_points_a,
                                       const std::vector<Vec2>& image_points_b,
                                       const CameraModel&       camera_model,
                                       const SE3&               initial_view_b_from_a) {
  // Primer /////////////////////////////////////////////////////////////////////////////////////
  // To do this, we must assume a scene scale, and then simultaneously estimate *all* of the structure parameters, and
  // *all* of the pose parameters
  //
  // - Each observation constraints two degrees of freedom: x, y image-frame translation
  // - There are only five observable degrees of freedom: absolute rotation and direction of motion on S2.
  //
  //
  ///////////////////////////////////////////////////////////////////////////////////////////////
  // Given a camera pose and a correspondence, we can estimate a "best 3d point" by triangulation
  //
  // --> We can do this by computing "pose" as:
  // SE3(SO3::exp(r_params), SO3::exp(0.0, *t_params) * Vec3::UnitX())
  //

  const L2Cost    cost_fcn;
  BootstrapResult result;

  result.weights.resize(image_points_a.size());

  double residual    = 0.0;
  Vec5   innovation  = Vec5::Zero();
  Mat5   information = Mat5::Zero();

  for (size_t k = 0; k < image_points_a.size(); ++k) {
    std::cout << "--- " << k << std::endl;

    const Vec2& pt_a = image_points_a[k];
    const Vec2& pt_b = image_points_b[k];

    const geometry::Ray ray_a_frame_a = camera_model.unproject(pt_a);
    const geometry::Ray ray_b_frame_a = initial_view_b_from_a.inverse() * camera_model.unproject(pt_b);
    const auto          triangulation = geometry::ray_ray_approx_intersect(ray_a_frame_a, ray_b_frame_a);
    if (!triangulation.valid) {
      std::cout << "Invalid triangulation: " << k << std::endl;
      continue;
    }

    const Vec3   triangulated_point_a = ray_a_frame_a(triangulation.along_a);
    const Vec3   triangulated_point_b = ray_b_frame_a(triangulation.along_b);
    const double triangulation_error  = (triangulated_point_a - triangulated_point_b).norm();
    if (triangulation_error > 1.0) {
      std::cout << "LARGE ERROR: " << triangulation_error << std::endl;
    }
    std::cout << "triangulation error: " << triangulation_error << std::endl;

    // If the triangulation was valid:

    const auto observation_fcn =
        [&camera_model, &initial_view_b_from_a, &ray_a_frame_a, &pt_b](const Vec5& parameters) -> Vec2 {

      const Vec3 translation = SO3::exp(Vec3(0.0, parameters(3), parameters(4))) * Vec3::UnitX();

      // const SE3 delta = SE3::exp(tangent);
      const SO3 rotation = SO3::exp(parameters.head<3>());
      const SE3 delta    = SE3(rotation, translation);

      const SE3 guess_b_from_a = (delta * initial_view_b_from_a);
      // const geometry::Ray ray_b_frame_b  = camera_model.unproject(pt_b);

      // const geometry::Ray ray_a_frame_b         = guess_b_from_a * ray_a_frame_a;
      // const auto          triangulation_frame_b = geometry::ray_ray_approx_intersect(ray_a_frame_b, ray_b_frame_b);
      // const Vec3          triangulated_point_a_frame_b = ray_a_frame_b(triangulation_frame_b.along_a);
      // const Vec3          triangulated_point_b_frame_b = ray_b_frame_b(triangulation_frame_b.along_b);

      // if (!triangulation_frame_b.valid) {
      //  std::cout << "invalid triangulation" << std::endl;
      //}

      std::cout << "delta: " << delta.log().transpose() << std::endl;
      std::cout << "Guess:" << guess_b_from_a.log().transpose() << std::endl;

      const geometry::Ray ray_b_frame_a = guess_b_from_a.inverse() * camera_model.unproject(pt_b);
      const auto          triangulation = geometry::ray_ray_approx_intersect(ray_a_frame_a, ray_b_frame_a);

      const Vec3 triangulated_point_a = ray_a_frame_a(triangulation.along_a);
      const Vec3 triangulated_point_b = ray_b_frame_a(triangulation.along_b);

      std::cout << "err: " << std::setprecision(16) << (triangulated_point_a - triangulated_point_b).norm()
                << std::endl;
      return camera_model.project(guess_b_from_a * triangulated_point_a);
    };

    // z - h(x)
    const Vec2 v_k = pt_b - observation_fcn(Vec5::Zero());
    const MatNd<2, 5> J_k = numerics::numerical_jacobian<2, 5>(Vec5::Zero(), observation_fcn, 1e-6);

    const CostWeight cost_weight_x = cost_fcn(v_k(0) * v_k(0));
    const CostWeight cost_weight_y = cost_fcn(v_k(1) * v_k(1));

    const Eigen::Matrix2d weight = Vec2(cost_weight_x.weight, cost_weight_y.weight).asDiagonal();
    information += J_k.transpose() * weight * J_k;
    innovation += J_k.transpose() * weight * v_k;

    std::cout << "innovation: " << innovation.transpose() << std::endl;
    std::cout << "v_k: " << v_k.transpose() << std::endl;

    residual += cost_weight_x.cost;
    residual += cost_weight_y.cost;

    result.weights[k] = cost_weight_x.weight * cost_weight_y.weight;
  }

  const Eigen::LDLT<Mat5> ldlt(information);
  // result.info_at_convergence = information;

  constexpr double MAX_RCOND = 0.08;
  if (ldlt.rcond() > MAX_RCOND) {
    std::cout << "BAD RESULT PROBABLY" << std::endl;
  }

  if (ldlt.info() != Eigen::Success) {
    result.success = false;
    return result;
  }

  const Vec5 delta               = ldlt.solve(innovation);
  const Vec3 translation         = SO3::exp(Vec3(0.0, delta(3), delta(4))) * Vec3::UnitX();
  const SO3  rotation            = SO3::exp(delta.head<3>());
  const SE3  new_from_previous   = SE3(rotation, translation);
  result.view_b_from_view_a_step = new_from_previous;

  result.rms_residual = std::sqrt(residual / static_cast<double>(image_points_a.size()));
  result.rcond        = ldlt.rcond();
  result.success      = true;

  std::cout << "r: " << rotation.log().transpose() << std::endl;
  std::cout << "t: " << translation.transpose() << std::endl;

  // TODO: Set estimated structure

  return result;
}
}