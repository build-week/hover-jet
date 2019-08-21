//%deps(yaml-cpp)
#include "config/fiducial_map/read_fiducial_map.hh"

namespace jet {

FiducialDescription get_fiducial_pose() {
  YAML::Node config = YAML::LoadFile("/jet/config/fiducial_map/fiducial_map.yaml");
  auto const node = config["fiducial_5x5"];

  FiducialDescription result;
  const auto log_translation_tag_from_world = node["log_translation_tag_from_world"].as<std::vector<double>>();
  const auto log_rotation_tag_from_world = node["log_rotation_tag_from_world"].as<std::vector<double>>();

  result.tag_from_world =
      SE3(SO3::exp(jcc::Vec3(
              log_rotation_tag_from_world[0], log_rotation_tag_from_world[1], log_rotation_tag_from_world[2])),
          jcc::Vec3(
              log_translation_tag_from_world[0], log_translation_tag_from_world[1], log_translation_tag_from_world[2]));

  result.tag_size_squares = node["artag_squares"].as<int>();
  result.arcode_width_mm = node["artag_width_mm"].as<double>();
  result.arcode_gap_mm = node["artag_gap_mm"].as<double>();
  return result;
}

CameraExtrinsics get_camera_extrinsics() {
  YAML::Node node = YAML::LoadFile("/jet/config/camera_extrinsics.yaml");

  CameraExtrinsics result;
  const auto log_translation_camera_from_frame = node["log_translation_camera_from_frame"].as<std::vector<double>>();
  const auto log_rotation_camera_from_frame = node["log_rotation_camera_from_frame"].as<std::vector<double>>();
  result.camera_from_frame =
      SE3(SO3::exp(jcc::Vec3(
              log_rotation_camera_from_frame[0], log_rotation_camera_from_frame[1], log_rotation_camera_from_frame[2])),
          jcc::Vec3(log_translation_camera_from_frame[0],
                    log_translation_camera_from_frame[1],
                    log_translation_camera_from_frame[2]));
  return result;
}

}  //  namespace jet
