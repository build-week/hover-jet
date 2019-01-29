#include "image_align.hh"

#include "numerics/numdiff.hh"

#include "eigen.hh"

namespace slam {

ImageAlignmentResult ImageAligner::perspective_npoint(const CameraModel&         cam_model,
                                                      const SE3&                 initial_camera_from_object,
                                                      const std::vector<Vec2>&   observed,
                                                      const std::vector<Vec3>&   object,
                                                      const RobustEstimator&     estimator,
                                                      Out<ImageAlignmentContext> ctx) const {
  //
  // Compute the log_camera_from_object of the camera in object frame
  //

  ImageAlignmentResult result;
  result.weights.resize(observed.size());

  using Scalar = double;
  using SE3    = Sophus::SE3<Scalar>;
  using Mat6   = Eigen::Matrix<double, 6, 6>;

  Mat6   information = Mat6::Zero();
  Vec6   innovation  = Vec6::Zero();
  double residual    = 0.0;
  for (size_t k = 0; k < observed.size(); ++k) {
    const Vec3 obj_pt   = object[k];
    const Vec2 image_pt = observed[k];

    // TODO: do this analytically (annoying homogeneous projection, tho)
    // Change in log_camera_from_object -> change in reprojection error
    const auto observation_fcn =
        [&cam_model, &obj_pt, &initial_camera_from_object](const Vec6& log_camera_from_object) -> Vec2 {
      return cam_model.project(SE3::exp(log_camera_from_object) * initial_camera_from_object * obj_pt);
    };
    const MatNd<2, 6> J_k = numerics::numerical_jacobian<2, 6>(Vec6::Zero(), observation_fcn, 1e-6);

    const Vec2 v_k = (image_pt - cam_model.project(initial_camera_from_object * obj_pt));

    /*
    const CostWeight cost_weight = estimator(v_k.dot(v_k));
    residual += cost_weight.cost;
    result.weights[k] = cost_weight.weight;
    constexpr double LEVENBERG_LAMBDA = 0.0;
    information += cost_weight.weight * J_k.transpose() * J_k + (LEVENBERG_LAMBDA * Mat6::Identity());
    innovation += cost_weight.weight * J_k.transpose() * v_k;
    */

    const CostWeight cost_weight_x = estimator(v_k(0) * v_k(0));
    const CostWeight cost_weight_y = estimator(v_k(1) * v_k(1));

    const Eigen::Matrix2d weight =
        (Eigen::Matrix2d() << cost_weight_x.weight, 0.0, 0.0, cost_weight_y.weight).finished();

    information += J_k.transpose() * weight * J_k;
    innovation += J_k.transpose() * weight * v_k;

    residual += cost_weight_x.cost;
    residual += cost_weight_y.cost;
    result.weights[k] = cost_weight_x.weight * cost_weight_y.weight;
  }

  const Eigen::LDLT<Mat6> ldlt(information);
  result.info_at_convergence = information;

  constexpr double MAX_RCOND = 0.08;
  if (ldlt.rcond() > MAX_RCOND) {
    std::cout << "BAD RESULT PROBABLY" << std::endl;
  }

  if (ldlt.info() != Eigen::Success) {
    result.success = false;
    return result;
  }

  const Vec6 delta    = ldlt.solve(innovation);
  result.delta        = SE3::exp(delta);
  result.rms_residual = std::sqrt(residual / static_cast<double>(observed.size()));
  result.rcond        = ldlt.rcond();
  result.success      = true;
  return result;
}

ImageAlignmentResult ImageAligner::standard_align(const CameraModel&       cam_model,
                                                  const SE3&               initial_camera_from_object,
                                                  const std::vector<Vec2>& observed,
                                                  const std::vector<Vec3>& object) const {
  // std::vector<Vec2> observed;
  // std::vector<Vec3> object;

  // for (size_t k = 0; k < correspondences.size(); ++k) {
  //   const bool matched = correspondences[k].hm_distance < 800;
  //   if (correspondences[k].origin_index != -1 && correspondences[k].destination_index != -1 && matched) {
  //     observed.push_back(in_observed.at(correspondences[k].origin_index));
  //     object.push_back(in_object.at(correspondences[k].destination_index));
  //   }
  // }

  // if (observed.size() < 3u) {
  //   return {};
  // }

  constexpr int         HBR_ITERS            = 20;
  constexpr int         TKY_ITERS            = 2;
  SE3                   aligned_from_initial = initial_camera_from_object;
  ImageAlignmentContext context;

  for (int k = 0; k < HBR_ITERS; ++k) {
    // const TukeyCost estimator(100.0);
    const HuberCost estimator(1000.0);
    // const L2Cost estimator;

    const auto result = perspective_npoint(cam_model, aligned_from_initial, observed, object, estimator, out(context));

    if (!result.success) {
      std::cout << "FAILED, DROPPING OUT" << std::endl;
      return result;
    }

    aligned_from_initial = result.delta * aligned_from_initial;
    std::cout << "k --- HUBER :: " << (result.success ? "success" : "failure") << std::endl;
    std::cout << "\tResidual : " << result.rms_residual << std::endl;
    std::cout << "\tRCOND    : " << result.rcond << std::endl;
    std::cout << "\tdelta    : " << result.delta.log().transpose() << std::endl;
    std::cout << "\ttotal    : " << aligned_from_initial.log().transpose() << std::endl;
  }

  ImageAlignmentResult last_result;
  last_result.delta   = aligned_from_initial;
  last_result.success = true;
  for (int k = 0; k < TKY_ITERS; ++k) {
    const TukeyCost estimator(100.0);
    const auto result = perspective_npoint(cam_model, aligned_from_initial, observed, object, estimator, out(context));
    if (!result.success) {
      return result;
    }

    aligned_from_initial = result.delta * aligned_from_initial;
    std::cout << "Residual:" << result.rms_residual << std::endl;
    std::cout << "REFINE (" << result.success << ")" << k << " :: " << result.delta.log().transpose() << std::endl;

    last_result       = result;
    last_result.delta = aligned_from_initial;
  }

  // std::cout << last_result.delta << std::endl;
  return last_result;
}
}  // namespace slam
