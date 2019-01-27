#include <librealsense/rs.hpp>

#include <opencv2/opencv.hpp>

#include <cstdio>
#include <iostream>

int main() try {
  // Turn on logging. We can separately enable logging to console or to file, and use different severity filters for
  // each.
  std::cout << "hello" << std::endl;
  rs::log_to_console(rs::log_severity::warn);
  // rs::log_to_file(rs::log_severity::debug, "librealsense.log");

  // Create a context object. This object owns the handles to all connected realsense devices.
  rs::context ctx;
  printf("There are %d connected RealSense devices.\n", ctx.get_device_count());
  if (ctx.get_device_count() == 0) return EXIT_FAILURE;

  // This tutorial will access only a single device, but it is trivial to extend to multiple devices
  rs::device *dev = ctx.get_device(0);
  printf("\nUsing device 0, an %s\n", dev->get_name());
  printf("    Serial number: %s\n", dev->get_serial());
  printf("    Firmware version: %s\n", dev->get_firmware_version());

  // Configure depth and color to run with the device's preferred settings
  dev->enable_stream(rs::stream::depth, rs::preset::best_quality);
  dev->enable_stream(rs::stream::color, rs::preset::best_quality);
  dev->start();

  while (true) {
    // Wait for new frame data
    dev->wait_for_frames();

    // Retrieve our images
    const uint16_t *depth_image = (const uint16_t *)dev->get_frame_data(rs::stream::depth);
    const uint8_t * color_image = (const uint8_t *)dev->get_frame_data(rs::stream::color);

    // Retrieve camera parameters for mapping between depth and color
    rs::intrinsics depth_intrin   = dev->get_stream_intrinsics(rs::stream::depth);
    rs::extrinsics depth_to_color = dev->get_extrinsics(rs::stream::depth, rs::stream::color);
    rs::intrinsics color_intrin   = dev->get_stream_intrinsics(rs::stream::color);
    float          scale          = dev->get_depth_scale();

    // cv::Mat color_image_mat(cv::Size(color_intrin.width, color_intrin.height), CV_8UC3);
    // for (int dy = 0; dy < color_intrin.height; ++dy) {
    //   for (int dx = 0; dx < color_intrin.width; ++dx) {
    //     color_image_mat.at<cv::Vec3b>(dy, dx) = color_image[3 * (dy * color_intrin.width + dx)];
    //   }
    // }

    cv::Mat rgb(color_intrin.height, color_intrin.width, CV_8UC3, (uchar *)color_image);

    cv::Mat bgr;
    cv::cvtColor(rgb, bgr, CV_BGR2RGB);

    cv::imshow("image", bgr);
    cv::waitKey(1);

    for (int dy = 0; dy < depth_intrin.height; ++dy) {
      for (int dx = 0; dx < depth_intrin.width; ++dx) {
        // Retrieve the 16-bit depth value and map it into a depth in meters
        uint16_t depth_value     = depth_image[dy * depth_intrin.width + dx];
        float    depth_in_meters = depth_value * scale;

        // Skip over pixels with a depth value of zero, which is used to indicate no data
        if (depth_value == 0) continue;

        // Map from pixel coordinates in the depth image to pixel coordinates in the color image
        rs::float2 depth_pixel = {(float)dx, (float)dy};
        rs::float3 depth_point = depth_intrin.deproject(depth_pixel, depth_in_meters);
        rs::float3 color_point = depth_to_color.transform(depth_point);
        rs::float2 color_pixel = color_intrin.project(color_point);

        // Use the color from the nearest color pixel, or pure white if this point falls outside the color image
        const int cx = (int)std::round(color_pixel.x), cy = (int)std::round(color_pixel.y);
        if (cx < 0 || cy < 0 || cx >= color_intrin.width || cy >= color_intrin.height) {
        } else {
        }

        // Emit a vertex at the 3D location of this depth pixel
      }
    }
  }

  return EXIT_SUCCESS;
} catch (const rs::error &e) {
  // Method calls against librealsense objects may throw exceptions of type rs::error
  printf("rs::error was thrown when calling %s(%s):\n", e.get_failed_function().c_str(), e.get_failed_args().c_str());
  printf("    %s\n", e.what());
  return EXIT_FAILURE;
}