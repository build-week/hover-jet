//%bin(slam_balsaq_main)
//%deps(balsa_queue)

#include "vision/slam_balsaq.hh"
#include <iostream>
#include "camera/camera_image_message.hh"
#include "config/fiducial_map/read_fiducial_map.hh"
#include "infrastructure/balsa_queue/bq_main_macro.hh"
#include "infrastructure/comms/mqtt_comms_factory.hh"
#include "infrastructure/time/duration.hh"

namespace jet {

void SlamBq::init(const Config& config) {
  subscriber_ = make_subscriber("camera_image_channel");
  publisher_ = make_publisher("slam_channel");
}

struct Landmark {
  cv::Mat descriptor;
  cv::KeyPoint keypoint;
};

class ImageWithLandmarks {
 public:
  ImageWithLandmarks(cv::Mat);
  std::vector<Landmark> landmarks;
  void draw();

 private:
  cv::Mat grayscale_scene_image;
  // TODO eliminate this member
  std::vector<cv::KeyPoint> keypoints;
};

float pixelwise_squared_difference(cv::Mat a, cv::Mat b) {
  // TODO check that a and b are same size, same format
  float result = 0.0;
  for (int i = 0; i < a.size().height; i++) {
    for (int j = 0; j < a.size().width; j++) {
      auto pixel1 = (float)(a.at<char>)(i, j) / 10.0;
      auto pixel2 = (float)(b.at<char>)(i, j) / 10.0;
      result += std::pow(pixel1 - pixel2, 2.0);
    }
  }
  return result;
}

ImageWithLandmarks::ImageWithLandmarks(cv::Mat _grayscale_scene_image) {
  grayscale_scene_image = _grayscale_scene_image;
  int fast_threshold = 80;
  std::vector<cv::KeyPoint> all_keypoints;

  // TODO choose a better feature type
  cv::FAST(grayscale_scene_image, all_keypoints, fast_threshold, true);
  for (auto it = all_keypoints.begin(); it != all_keypoints.end(); ++it) {
    auto x = (int)it->pt.x;
    auto y = (int)it->pt.y;
    auto WINDOW_RADIUS = 10;
    // std::cout << x << " " << y << std::endl;
    // std::cout << grayscale_scene_image.size().height << std::endl;
    // std::cout << grayscale_scene_image.size().width << std::endl;

    if (x - WINDOW_RADIUS >= 0 && y - WINDOW_RADIUS >= 0 &&
        x + WINDOW_RADIUS * 2 < grayscale_scene_image.size().width &&
        x + WINDOW_RADIUS * 2 < grayscale_scene_image.size().height) {
      cv::Rect roi(cv::Point(std::max(0, y - WINDOW_RADIUS), std::max(0, x - WINDOW_RADIUS)),
                   cv::Size(WINDOW_RADIUS * 2, WINDOW_RADIUS * 2));
      cv::Mat croppedImage = grayscale_scene_image(roi);
      Landmark landmark;
      landmark.descriptor = croppedImage;
      landmark.keypoint = *it;
      landmarks.push_back(landmark);
      keypoints.push_back(*it);
    }
    std::cout << "n landmarks " << landmarks.size() << std::endl;
  }
  // // Note that this doesn't copy the data
}

void ImageWithLandmarks::draw() {
  auto canvas = grayscale_scene_image.clone();
  cv::drawKeypoints(grayscale_scene_image, keypoints, canvas);
  cv::imshow("camera image", canvas);
  cv::waitKey(1);
  if (landmarks.size() > 0) {
    cv::imshow("patch", landmarks.at(0).descriptor);
    cv::waitKey(1);
  }
}

std::vector<std::tuple<cv::KeyPoint, cv::KeyPoint, float>> match(ImageWithLandmarks a, ImageWithLandmarks b) {
  std::vector<std::tuple<cv::KeyPoint, cv::KeyPoint, float>> result;
  if (a.landmarks.size() == 0 || b.landmarks.size() == 0) {
    return result;
  }
  for (auto ita = a.landmarks.begin(); ita != a.landmarks.end(); ++ita) {
    auto best_match = std::make_tuple(ita->keypoint, b.landmarks.at(0).keypoint, 10000000);
    for (auto itb = b.landmarks.begin(); itb != b.landmarks.end(); ++itb) {
      float distance = pixelwise_squared_difference(ita->descriptor, itb->descriptor);
      // std::cout << distance << std::endl;

      if (distance < std::get<2>(best_match)) {
        // std::cout << "here" << std::endl;
        best_match = std::make_tuple(ita->keypoint, itb->keypoint, distance);
      }
    }
    result.push_back(best_match);
  }

  sort(result.begin(), result.end(),
       [](const auto& lhs, const auto& rhs) { return std::get<2>(lhs) < std::get<2>(rhs); });
  auto max_n_matches = (int)std::min(a.landmarks.size(), b.landmarks.size());
  result.resize(std::min(max_n_matches, 3));
  for (auto it = result.begin(); it != result.end(); ++it) {
    std::cout << "element in result " << std::get<2>(*it) << std::get<0>(*it).pt << std::get<1>(*it).pt << std::endl;
  }
  return result;
}

void SlamBq::loop() {
  CameraImageMessage image_message;

  // Wait until we have the latest image_message
  bool got_msg = false;
  while (subscriber_->read(image_message, 1)) {
    got_msg = true;
  }
  if (got_msg) {
    cv::Mat camera_frame_1;
    const cv::Mat camera_frame_rgb_1 = get_image_mat(image_message).clone();
    cv::cvtColor(camera_frame_rgb_1, camera_frame_1, cv::COLOR_RGB2GRAY);
    if (!old_image.empty()) {
      // ImageWithLandmarks image_w_landmarks_1(camera_frame_1);
      // ImageWithLandmarks image_w_landmarks_2(old_image);
      // auto matches = match(image_w_landmarks_1, image_w_landmarks_2);
      // auto canvas = camera_frame_1.clone();
      // for (auto it = matches.begin(); it != matches.end(); ++it) {
      //   cv::line(canvas, std::get<0>(*it).pt, std::get<1>(*it).pt, CV_RGB(250,250,0));
      //   auto circle_radius = std::max(4 + (int)std::get<2>(*it), 4);
      //   cv::circle(canvas, std::get<0>(*it).pt, circle_radius, CV_RGB(250,250,0));
      //   cv::circle(canvas, std::get<1>(*it).pt, circle_radius, CV_RGB(250,250,0));
      // }
      // cv::imshow("camera image", canvas);
      // cv::waitKey(1);

      auto img_1 = old_image;
      auto img_2 = camera_frame_1;

      auto detector = cv::ORB::create();

      std::vector<cv::KeyPoint> keypoints_1, keypoints_2;

      detector->detect(img_1, keypoints_1);
      detector->detect(img_2, keypoints_2);
      auto canvas = camera_frame_1.clone();
      if (keypoints_1.size() > 0 && keypoints_2.size() > 0) {
        // auto extractor = cv::BriefDescriptorExtractor::create();
        cv::Mat descriptors_1, descriptors_2;
        detector->compute(img_1, keypoints_1, descriptors_1);
        detector->compute(img_2, keypoints_2, descriptors_2);
        std::vector<cv::DMatch> matches;
        auto bf = cv::BFMatcher(cv::NORM_HAMMING);
        bf.match(descriptors_1, descriptors_2, matches);

        // drawMatches( canvas, keypoints_1, img_2, keypoints_2,
        //          good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
        //          vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

        for (auto it = matches.begin(); it != matches.end(); ++it) {
          std::cout << it->distance << std::endl;
          auto kp1 = keypoints_1.at(it->queryIdx);
          auto kp2 = keypoints_2.at(it->trainIdx);
          if (it->distance < 10) {
            cv::line(canvas, kp1.pt, kp2.pt, it->distance);
            auto circle_radius = it->distance;  // std::max(4 + (int)std::get<2>(*it), 4);
            cv::circle(canvas, kp1.pt, circle_radius, CV_RGB(250, 250, 0));
            cv::circle(canvas, kp2.pt, circle_radius, CV_RGB(250, 250, 0));
          }
        }

        last_msg_recvd_timestamp_ = get_current_time();
        if (last_msg_recvd_timestamp_ < get_current_time() - Duration::from_seconds(1)) {
          gonogo().nogo("More than 1 second since last image message");
        } else {
          gonogo().go("slam go");
        }
      }
      cv::imshow("camera image", canvas);
      cv::waitKey(1);
    }
    old_image = camera_frame_1.clone();
  }
}

void SlamBq::shutdown() {
  std::cout << "slam detection BQ shutting down." << std::endl;
}

}  // namespace jet
BALSA_QUEUE_MAIN_FUNCTION(jet::SlamBq)

// std::cout << "new_frame" << std::endl;
// std::cout << "got a frame with " << keypoints.size() << " FAST features" << std::endl;
// int pixel_skip = 3;
// for (int i = 0; i < camera_frame.size().height; i += pixel_skip) {
//   for (int j = 0; j < camera_frame.size().width; j += pixel_skip) {
//     std::cout << "point " << (camera_frame.size().width - j) / 100.0 << " "
//               << (camera_frame.size().height - i) / 100.0 <<

//         " 0 " << (int)(camera_frame.ptr(i, j)[2]) << " " << (int)(camera_frame.ptr(i, j)[1]) << " "
//               << (int)(camera_frame.ptr(i, j)[0]) << " " << 1 << std::endl;
//   }
// }
// for (auto it = keypoints.begin(); it != keypoints.end(); ++it) {
//   std::cout << "point " << (it->pt.x) / 100.0 << " " << (it->pt.y) / 100.0 << " "
//             << ".1 orange 2" << std::endl;
// }
