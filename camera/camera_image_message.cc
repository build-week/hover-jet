#include "camera/camera_image_message.hh"

namespace jet {

cv::Mat getImageMat(CameraImageMessage message) {
  cv::Mat result = cv::Mat(message.height, message.width, CV_8UC3);
  const std::size_t n_elements = message.height * message.width * 3u;
  constexpr std::size_t SIZE_OF_UCHAR = sizeof(uint8_t);
  std::memcpy(result.data, message.image_data.data(),
              SIZE_OF_UCHAR * n_elements);
  return result;
}

}  // namespace jet