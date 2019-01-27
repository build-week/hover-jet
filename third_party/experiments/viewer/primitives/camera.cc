#include "viewer/primitives/camera.hh"

#include <GL/glew.h>

#include "viewer/gl_aliases.hh"

namespace viewer {

cv::Mat Camera::capture_framebuffer() const {
  constexpr int ORIGIN_X = 0;
  constexpr int ORIGIN_Y = 0;
  constexpr int FORMAT = GL_BGR;

  constexpr int WIDTH = 640;
  constexpr int HEIGHT = 640;

  constexpr int TYPE = GL_UNSIGNED_BYTE;

  uint8_t* out_data = new uint8_t[WIDTH * HEIGHT * 3];
  glReadPixels(ORIGIN_X, ORIGIN_Y, WIDTH, HEIGHT, FORMAT, TYPE, out_data);
  constexpr int IM_CV_TYPE = CV_8UC3;
  const cv::Mat image(HEIGHT, WIDTH, IM_CV_TYPE, out_data);
  cv::Mat flipped;
  cv::flip(image, flipped, 0);
  delete out_data;
  return flipped;
}

void Camera::prepare_view() const {
  glViewport(0, 0, size_.width, size_.height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  const double aspect_ratio =
      (static_cast<double>(size_.width) / static_cast<double>(size_.height));
  constexpr double NEAR_CLIP = 0.001;
  constexpr double FAR_CLIP = 1000.0;
  constexpr double FOV = 90.0;
  gluPerspective(FOV, aspect_ratio, NEAR_CLIP, FAR_CLIP);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glTransform(world_from_camera_.inverse());
}

void Camera::draw() {
  const std::lock_guard<std::mutex> lk(draw_mutex_);
  image_ = capture_framebuffer();

  proj_ = Projection::get_from_current();
  have_image_ = true;
}

}  // namespace viewer