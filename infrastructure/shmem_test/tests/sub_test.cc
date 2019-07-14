//%bin(sub_test_main)

#include "infrastructure/comms/shared_struct/shared_struct_subscriber.hh"
#include "camera/camera_image_message.hh"
#include "infrastructure/time/time_utils.hh"

int main() {
  auto struct_client = std::make_unique<jet::SharedStructSubscriber>("test_object");
  jet::Timestamp last_timestamp = 0;

  while (1)
  {
    jet::CameraImageMessage image;
    struct_client->read(image);
    if (last_timestamp != image.header.timestamp_ns)
    {
      last_timestamp = image.header.timestamp_ns;
      std::cout << "Image sequence is: " << image.header.sequence_number << " published at: " << image.header.timestamp_ns << std::endl;
      std::cout << "Time diff: " << jet::time::get_current_time() - image.header.timestamp_ns << "ns" << std::endl;
    }
  }
}
