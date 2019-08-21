#pragma once
#include <cassert>
#include <vector>

//%deps(opencv)
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

#include "camera/camera_manager.hh"
#include "third_party/experiments/eigen.hh"
#include "third_party/experiments/sophus.hh"

#include "infrastructure/comms/serialization/serialization_macros.hh"

namespace jet {

struct MarkerDetection {
  SE3 marker_center_from_camera;
  int id;
};

struct MarkerInWorld {
  SE3 world_from_marker;
  int id;
};

struct BoardIdsAndCorners {
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
};

template <int ROWS, int COLS>
struct MatWrapper {
  using Mat = MatNd<ROWS, COLS>;

  MatWrapper() {
  }
  MatWrapper(const MatWrapper&) = default;
  // MatWrapper(const MatWrapper&&) = default;

  explicit MatWrapper(const Mat& mtx) {
    for (int r = 0; r < ROWS; r++) {
      for (int c = 0; c < COLS; c++) {
        vals[c + (r * COLS)] = mtx(r, c);
      }
    }
  }

  MatWrapper& operator=(const Mat& m) {
    *this = MatWrapper(m);
    return *this;
  }

  operator Mat() const {
    const Eigen::Map<const Mat> mtx_mmap(vals.data());
    Mat true_mtx(mtx_mmap);
    return true_mtx;
  }

  std::array<double, ROWS * COLS> vals;

  SERIALIZABLE_STRUCTURE(MatWrapper, vals);
};

template <int N>
using VecWrapper = MatWrapper<N, 1>;

struct BoardPointImagePointAssociation {
  VecWrapper<2> point_board_space;
  VecWrapper<2> point_image_space;

  SERIALIZABLE_STRUCTURE(BoardPointImagePointAssociation, point_board_space, point_image_space);
};

BoardIdsAndCorners get_ids_and_corners(const cv::Mat& input_image);

std::vector<MarkerInWorld> get_world_from_marker_centers(const cv::Mat& camera_image, const SE3& world_from_camera);

std::optional<SE3> estimate_board_bottom_left_from_camera(const BoardIdsAndCorners& ids_corners,
                                                          const Calibration& calibration);

std::vector<BoardPointImagePointAssociation> obj_points_img_points_from_image(const BoardIdsAndCorners& ids_corners);

constexpr float FIDUCIAL_WIDTH_METERS = 99.0 / 1000;
constexpr float FIDUCIAL_GAP_WIDTH_METERS = 50.0 / 1000;

inline cv::Ptr<cv::aruco::Dictionary> get_aruco_dictionary() {
  return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
}

inline cv::Ptr<cv::aruco::GridBoard> get_aruco_board() {
  return cv::aruco::GridBoard::create(4, 4, FIDUCIAL_WIDTH_METERS, FIDUCIAL_GAP_WIDTH_METERS, get_aruco_dictionary());
}

}  // namespace jet
