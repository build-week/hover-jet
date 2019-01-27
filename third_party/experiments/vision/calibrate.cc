#include "block_transform.hh"
#include "features.hh"
#include "histogram_equalize.hh"
#include "image_align.hh"
#include "images.hh"
#include "io.hh"
#include "pyramid.hh"

#include "out.hh"
#include "util/clamp.hh"

#include "viewer/primitives/frame.hh"
#include "viewer/primitives/image.hh"
#include "viewer/primitives/image_frame.hh"
#include "viewer/primitives/simple_geometry.hh"
#include "viewer/window_3d.hh"
#include "viewer/window_manager.hh"

#include "eigen.hh"
#include <opencv2/opencv.hpp>

#include <iostream>
#include <limits>
#include <ostream>

using Vec2   = Eigen::Vector2d;
using Vec3   = Eigen::Vector3d;
using Vec32i = Eigen::Matrix<uint8_t, 32, 1>;

namespace slam {

std::vector<Vec32i> eigenize_descriptors(const cv::Mat& descriptors) {
  std::vector<Vec32i> eigen_descriptors(descriptors.rows);
  for (int k = 0; k < descriptors.rows; ++k) {
    for (int j = 0; j < descriptors.cols; ++j) {
      eigen_descriptors[k](j) = descriptors.at<uint8_t>(k, j);
    }
  }
  return eigen_descriptors;
}

int compute_l1_distance(const Vec32i& a, const Vec32i& b, int dummy) {
  int distance = 0;
  for (int k = 0; k < a.size(); ++k) {
    const int l_dist = std::abs(a(k) - b(k));
    distance += l_dist;
  }
  return distance;
}

int compute_hamming_distance(const Vec32i& a, const Vec32i& b, const int max_error) {
  int distance = 0;
  for (int k = 0; k < a.size(); ++k) {
    const int l_dist = std::abs(a(k) - b(k));
    if (l_dist > max_error) {
      distance += 1;
    }
  }
  return distance;
}

template <typename Callable>
std::vector<Correspondence> match_by_brute_force_hamming(const std::vector<Vec32i>& set_a,
                                                         const std::vector<Vec32i>& set_b,
                                                         const int                  diff_thresh,
                                                         const Callable&            dist_function) {
  std::vector<Correspondence> correspondences(set_a.size());
  for (size_t a_index = 0; a_index < set_a.size(); ++a_index) {
    int best_index = -1;
    int best_dist  = std::numeric_limits<int>::max();

    for (size_t b_index = 0; b_index < set_b.size(); ++b_index) {
      const int dist = dist_function(set_a[a_index], set_b[b_index], diff_thresh);
      if (dist < best_dist) {
        best_index = b_index;
        best_dist  = dist;
      }
    }
    correspondences[a_index] = {static_cast<int>(a_index), best_index, best_dist};
  }
  return correspondences;
}

std::vector<Vec32i> compute_orb_features(const cv::Mat& image, Out<std::vector<cv::KeyPoint>> key_points) {
  constexpr int   N_FEATURES     = 500;
  constexpr float SCALE_FACTOR   = 1.2f;
  constexpr int   N_LEVELS       = 8;
  constexpr int   EDGE_THRESHOLD = 12;
  constexpr int   FIRST_LEVEL    = 0;
  const cv::ORB   orb_detector(N_FEATURES, SCALE_FACTOR, N_LEVELS, EDGE_THRESHOLD, FIRST_LEVEL);

  cv::Mat descriptors;
  orb_detector(image, cv::Mat(), is_out(*key_points), is_out(descriptors));
  return eigenize_descriptors(descriptors);
}

void try_align(const std::vector<cv::KeyPoint>&   observed_pts,
               const std::vector<cv::KeyPoint>&   calibration_pts,
               const std::vector<Correspondence>& correspondences) {
  constexpr double FX = 1.0;
  constexpr double FY = 1.0;
  constexpr double CX = -1600 * 0.5;
  constexpr double CY = -1067 * 0.5;
  // constexpr double CX = 0.0;
  // constexpr double CY = 0.0;

  const CameraModel model(FX, FY, CX, CY);

  if (observed_pts.size() < 5u) {
    return;
  }

  std::vector<Vec2> observed;
  std::vector<Vec3> object;
  std::vector<Vec3> object_pts_real;
  std::vector<Vec2> observed_effective;

  for (const auto& observed_pt : observed_pts) {
    // observed.emplace_back(observed_pt.pt.x * 0.01, observed_pt.pt.y * 0.01);
    // obserd
  }

  const SE3 distortion_from_object(SO3::exp(Vec3(0.0, 0.0, 0.0)), Vec3(12.0, 15.0, 0.0));

  // const SE3 distortion_from_object;
  std::vector<Correspondence> fake_correspondences;
  for (const auto& calibration_pt : calibration_pts) {
    fake_correspondences.push_back({static_cast<int>(object.size()), static_cast<int>(object.size()), 10});

    object.emplace_back(calibration_pt.pt.x, calibration_pt.pt.y, 10.0);
    observed.emplace_back(model.project(distortion_from_object * object.back()));
    object_pts_real.emplace_back(calibration_pt.pt.x, calibration_pt.pt.y, 10.0);
    observed_effective.emplace_back(model.project(distortion_from_object * object.back()) - Vec2(CX, CY));
  }

  ImageAligner               aligner;
  static Sophus::SE3<double> initial;
  const auto                 result = aligner.standard_align(model, initial, observed, object, fake_correspondences);
  // const auto                 result = aligner.standard_align(model, initial, observed, object, correspondences);

  auto win  = viewer::get_window3d("Window A");
  auto geom = std::make_shared<viewer::SimpleGeometry>();

  static bool first = true;
  if (first) {
    first = false;
    geom->add_axes({SE3(), 10.0});
    auto contained_geom = std::make_shared<viewer::SimpleGeometry>();
    auto image_frame    = std::make_shared<viewer::ImageFrame>(SE3(SO3(), Vec3(0.0, 0.0, 0.0)), 1067, 1600, 0.01);

    contained_geom->add_points({object_pts_real, Eigen::Vector4d(0.0, 0.8, 0.1, 0.7), 25.0});
    contained_geom->add_points2d({observed_effective, Eigen::Vector4d(0.7, 0.2, 0.1, 0.7), 25.0, FX});
    image_frame->add_primitive(contained_geom);
    win->add_primitive(image_frame);
  }

  const SE3 camera_from_object = result.delta.inverse();

  if (result.success) {
    geom->add_axes({camera_from_object});

    for (const Vec2& pt : observed) {
      /*      for (const auto corresp : correspondences) {
            const bool matched = corresp.hm_distance < 800;

            if (corresp.origin_index != -1 && corresp.destination_index != -1 && matched) {
            const Vec2 pt = observed[corresp.origin_index];
      */
      const Vec3 ray_direction          = model.unproject(pt);
      const Vec3 ray_direction_adjusted = Vec3(-0.01 * ray_direction.x(), -0.01 * ray_direction.y(), ray_direction.z());

      // + Vec3(1067 * 0.01, 1600 * 0.01, 0.0)
      geom->add_ray({camera_from_object.translation(), camera_from_object.so3() * ray_direction_adjusted, 150.0,
                     Eigen::Vector4d(1.0, 0.0, 0.0, 0.6)});
      // }
    }

    win->add_primitive(geom);
    win->spin_until_step();
    initial = result.delta;
  }

  std::cout << result.success << std::endl;
  std::cout << "LOG: " << result.delta.log().transpose() << std::endl;
  std::cout << "RES: " << result.rms_residual << std::endl;
}
}

int main() {
  const std::string video_filename = "/home/jacob/repos/slam/data/calibration/iphone_goat.mov";
  const std::string calibration_image_filename =
      "/home/jacob/repos/slam/data/calibration/domestic_goat_kid_in_capeweed.jpg";

  // const std::string video_filename             = "/home/jacob/repos/slam/data/calibration/godzilla_poster.mov";
  // const std::string calibration_image_filename = "/home/jacob/repos/slam/data/calibration/godzilla.jpg";

  const cv::Mat calibration_image_color = cv::imread(calibration_image_filename);
  std::cout << calibration_image_color.size() << std::endl;
  cv::Mat calibration_image;
  cv::cvtColor(calibration_image_color, is_out(calibration_image), CV_BGR2GRAY);

  auto win = viewer::get_window3d("Window A");

  {
    auto calibration_img_primitive       = std::make_shared<viewer::Image>(calibration_image_color, 0.01);
    auto calibration_img_primitive_frame = std::make_shared<viewer::Frame>(SE3(SO3(), Vec3(0.0, 0.0, 50.1)));
    calibration_img_primitive_frame->add_primitive(calibration_img_primitive);
    win->add_primitive(calibration_img_primitive_frame);
  }

  {
    auto calibration_img_primitive = std::make_shared<viewer::Image>(calibration_image_color, 0.0001, 0.3);

    auto calibration_img_primitive_frame = std::make_shared<viewer::Frame>(SE3(SO3(), Vec3(0.0, 0.0, 1.0)));
    calibration_img_primitive_frame->add_primitive(calibration_img_primitive);
    win->add_primitive(calibration_img_primitive_frame);
  }

  std::vector<cv::KeyPoint> calibration_key_points;
  const auto calibration_descriptors = slam::compute_orb_features(calibration_image, out(calibration_key_points));

  /*  for (const auto& pt : calibration_key_points) {
      cv::circle(calibration_image, pt.pt, pt.size, cv::Scalar(255, 0, 0));
    }

    cv::imshow("calimg", calibration_image);
    cv::waitKey(0);
  */
  cv::VideoCapture capture(video_filename);

  if (!capture.isOpened()) {
    std::cout << "Failure to open stream" << std::endl;
  }

  cv::Mat gray_frame;
  cv::Mat frame;
  while (true) {
    capture >> frame;
    if (frame.empty()) {
      break;
    };
    cv::cvtColor(frame, is_out(gray_frame), CV_BGR2GRAY);

    std::vector<cv::KeyPoint> key_points;
    const auto                descriptors = slam::compute_orb_features(gray_frame, out(key_points));

    // const auto matches =
    //     slam::match_by_brute_force_hamming(descriptors, calibration_descriptors, 50, slam::compute_hamming_distance);
    const auto matches =
        slam::match_by_brute_force_hamming(descriptors, calibration_descriptors, 50, slam::compute_l1_distance);

    try_align(key_points, calibration_key_points, matches);

    {
      cv::Mat draw_frame;
      cv::Mat draw_calibration_image;

      frame.copyTo(draw_frame);
      calibration_image_color.copyTo(draw_calibration_image);
      for (size_t k = 0; k < key_points.size(); ++k) {
        const bool       matched = matches[k].hm_distance < 800;
        const cv::Scalar color(0, matched ? 255 : 0, matched ? 0 : 255);
        const auto       pt = key_points[k];

        if (matched) {
          cv::circle(is_out(draw_frame), pt.pt, pt.size, color);

          const auto cal_key_pt = calibration_key_points[matches[k].destination_index];

          cv::circle(is_out(draw_calibration_image), cal_key_pt.pt, cal_key_pt.size, color);
          std::ostringstream text;
          text << "" << k;
          constexpr double SCALE     = 4.0;
          constexpr int    FONT      = 1;
          constexpr int    THICKNESS = 3;

          cv::putText(
              is_out(draw_calibration_image), text.str(), cal_key_pt.pt, FONT, SCALE, cv::Scalar(255, 0, 0), THICKNESS);
          cv::putText(is_out(draw_frame), text.str(), pt.pt, FONT, SCALE, cv::Scalar(255, 0, 0), THICKNESS);
        }
      }

      cv::Mat draw_frame_down;
      cv::Mat draw_calibration_image_down;

      pyrDown(draw_frame, draw_frame_down, cv::Size(draw_frame.cols / 2, draw_frame.rows / 2));
      pyrDown(draw_calibration_image,
              draw_calibration_image_down,
              cv::Size(draw_calibration_image.cols / 2, draw_calibration_image.rows / 2));

      // cv::imshow("feature_image", draw_frame_down);
      // cv::imshow("calibration_image", draw_calibration_image_down);

      // cv::imshow("feature_image", draw_frame);
      // cv::imshow("calibration_image", draw_calibration_image);

      // const int key = cv::waitKey(10);
      // if (key == 113 || key == 1048689) {
      // break;
      // }
    }
  }
}