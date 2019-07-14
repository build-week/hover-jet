//%bin(pub_test_main)

#include "infrastructure/comms/shared_struct/shared_struct_publisher.hh"
#include "camera/camera_image_message.hh"

int main() {
  auto struct_server = std::make_unique<jet::SharedStructPublisher>("test_object");

  while (1)
  {
    jet::CameraImageMessage image;
    struct_server->publish(image);
    std::cout << "Image sequence is: " << image.header.sequence_number << " published at: " << image.header.timestamp_ns << std::endl;
    usleep(100000);
  }
}
