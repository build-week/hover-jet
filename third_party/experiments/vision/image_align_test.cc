#include "image_align.hh"
#include "robust_estimator.hh"

#include "testing/gtest.hh"

#include "sophus/se3.hpp"

namespace slam {

using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;

using Scalar = double;
using SE3    = Sophus::SE3<Scalar>;
using SO3    = Sophus::SO3<Scalar>;

TEST(ImageAlign, do_nothing_if_aligned) {
  //
  // Setup
  //

  constexpr double  FX = 10.0;
  constexpr double  FY = 10.0;
  constexpr double  CX = 0.0;
  constexpr double  CY = 0.0;
  const CameraModel model(FX, FY, CX, CY);

  const SE3 camera_from_object(SO3::exp(Vec3(0.0, 0.0, 0.0)), Vec3(0.0, 0.0, 0.0));

  constexpr int     NUM_PTS = 50;
  std::vector<Vec2> projected(NUM_PTS);
  std::vector<Vec3> object(NUM_PTS);
  for (int k = 0; k < NUM_PTS; ++k) {
    object[k]    = Vec3::Random() + Vec3(0.0, 0.0, 2.0);
    projected[k] = model.project(camera_from_object * object[k]);
  }

  //
  // Action
  //

  const ImageAlignmentConfig config{};
  ImageAlignmentContext      context;

  ImageAligner aligner(config);
  const L2Cost estimator;
  aligner.perspective_npoint(model, SE3(), projected, object, estimator, out(context));
}

TEST(ImageAlign, do_something_if_unaligned) {
  //
  // Setup
  //

  constexpr double  FX = 10.0;
  constexpr double  FY = 10.0;
  constexpr double  CX = 0.0;
  constexpr double  CY = 0.0;
  const CameraModel model(FX, FY, CX, CY);

  const SE3 camera_from_object(SO3::exp(Vec3(0.3, 0.0, 0.1)), Vec3(1.0, 0.0, 0.1));

  constexpr int     NUM_PTS = 50;
  std::vector<Vec2> projected(NUM_PTS);
  std::vector<Vec3> object(NUM_PTS);
  for (int k = 0; k < NUM_PTS; ++k) {
    object[k]    = Vec3::Random() + Vec3(0.0, 0.0, 2.0);
    projected[k] = model.project(camera_from_object * object[k]);
  }

  //
  // Action
  //

  const ImageAlignmentConfig config{};
  ImageAlignmentContext      context;
  ImageAligner               aligner(config);

  constexpr int ITERS = 5;
  SE3           aligned_from_initial;

  for (int k = 0; k < ITERS; ++k) {
    const L2Cost estimator;
    const auto   result =
        aligner.perspective_npoint(model, aligned_from_initial, projected, object, estimator, out(context));
    aligned_from_initial = result.delta * aligned_from_initial;
  }
  constexpr double EPS = 1e-6;
  EXPECT_LT((aligned_from_initial * camera_from_object.inverse()).log().norm(), EPS);
}

TEST(ImageAlign, align_with_outliers) {
  //
  // Setup
  //

  constexpr double  FX = 10.0;
  constexpr double  FY = 10.0;
  constexpr double  CX = 0.0;
  constexpr double  CY = 0.0;
  const CameraModel model(FX, FY, CX, CY);

  const SE3 camera_from_object(SO3::exp(Vec3(0.3, 0.0, 0.1)), Vec3(1.0, 0.0, 0.1));

  constexpr int     NUM_PTS = 500;
  std::vector<Vec2> projected(NUM_PTS);
  std::vector<Vec3> object(NUM_PTS);
  for (int k = 0; k < NUM_PTS; ++k) {
    object[k]     = Vec3::Random() + Vec3(0.0, 0.0, 2.0);
    object[k].z() = 2.0;
    projected[k]  = model.project(camera_from_object * object[k]) + (Vec2::Random() * 0.01);
  }

  object.push_back(Vec3(7.0, 2.0, 0.0) + Vec3(0.0, 0.0, 2.0));
  projected.push_back(Vec2::Random());

  //
  // Action
  //

  const ImageAlignmentConfig config{};
  ImageAlignmentContext      context;
  ImageAligner               aligner(config);

  constexpr int HBR_ITERS = 3;
  constexpr int TKY_ITERS = 3;
  SE3           aligned_from_initial;

  for (int k = 0; k < HBR_ITERS; ++k) {
    const HuberCost estimator(1.0);
    const auto      result =
        aligner.perspective_npoint(model, aligned_from_initial, projected, object, estimator, out(context));
    aligned_from_initial = result.delta * aligned_from_initial;
    std::cout << "Residual:" << result.rms_residual << std::endl;
    std::cout << "HUBER (" << result.success << ")" << k << " :: " << result.delta.log().transpose() << std::endl;
  }

  for (int k = 0; k < TKY_ITERS; ++k) {
    const TukeyCost estimator(1.0);
    const auto      result =
        aligner.perspective_npoint(model, aligned_from_initial, projected, object, estimator, out(context));
    aligned_from_initial = result.delta * aligned_from_initial;

    std::cout << "Residual:" << result.rms_residual << std::endl;
    std::cout << "REFINE (" << result.success << ")" << k << " :: " << result.delta.log().transpose() << std::endl;
  }

  constexpr double EPS = 1e-3;
  EXPECT_LT((aligned_from_initial * camera_from_object.inverse()).log().norm(), EPS);
}
}