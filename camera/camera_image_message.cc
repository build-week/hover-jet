#include "camera/camera_image_message.hh"

namespace jet {

cv::Mat get_image_mat(const CameraImageMessage& message) {
  cv::Mat result = cv::Mat(message.height, message.width, CV_8UC3);
  const std::size_t n_elements = message.height * message.width * 3u;
  constexpr std::size_t SIZE_OF_UCHAR = sizeof(uint8_t);
  std::memcpy(result.data, message.image_data.data(), SIZE_OF_UCHAR * n_elements);
  return result;
}

CameraImageMessage create_camera_image_message(const cv::Mat& camera_frame,
                                               const std::string& serial_number,
                                               const Timestamp cap_time_vehicle) {
  CameraImageMessage message;
  const std::size_t n_elements = camera_frame.rows * camera_frame.cols * 3u;
  message.image_data.resize(n_elements);
  constexpr std::size_t SIZE_OF_UCHAR = sizeof(uint8_t);
  if (camera_frame.isContinuous()) {
    std::memcpy(message.image_data.data(), camera_frame.data, SIZE_OF_UCHAR * n_elements);
  }
  message.timestamp = cap_time_vehicle;
  message.height = camera_frame.size().height;
  message.width = camera_frame.size().width;
  message.camera_serial_number = serial_number;
  return message;
}
}  // namespace jet