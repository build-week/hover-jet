#include "viewer/primitives/image.hh"

#include <GL/glew.h>

#include <cassert>

namespace viewer {

Image::Image(const cv::Mat& image, const double scale, double alpha) {
  to_update_ = true;
  alpha_ = alpha;
  width_m_ = scale;
  update_image(image);
}

Image::Image(const Eigen::MatrixXd& image, double scale, double alpha) {
  to_update_ = true;
  alpha_ = alpha;
  width_m_ = scale;
  update_image(image);
}

void Image::update_image(const Eigen::MatrixXd& image) {
  const std::lock_guard<std::mutex> lk(draw_mutex_);

  const Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> image_as_uchar =
      (image * 255.0).cast<uint8_t>();
  cv::Mat new_image;
  cv::eigen2cv(image_as_uchar, new_image);
  cv::cvtColor(new_image, image_, cv::COLOR_GRAY2BGR);
  to_update_ = true;
}

void Image::update_image(const cv::Mat& image) {
  const std::lock_guard<std::mutex> lk(draw_mutex_);
  if (image.channels() == 1) {
    cv::Mat not_yet_flipped;
    cv::cvtColor(image, not_yet_flipped, cv::COLOR_GRAY2BGR);
    cv::flip(not_yet_flipped, image_, 0);
  } else if (image.channels() == 3) {
    cv::flip(image, image_, 0);
  } else {
    assert(false);
  }
  to_update_ = true;
}

void Image::update_gl() const {
  if (!allocated_texture_) {
    glGenTextures(1, &texture_id_);
    allocated_texture_ = true;
  }

  glBindTexture(GL_TEXTURE_2D, texture_id_);

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image_.cols, image_.rows, 0, GL_BGR,
               GL_UNSIGNED_BYTE, image_.data);
}

void Image::draw() const {
  const std::lock_guard<std::mutex> lk(draw_mutex_);

  if (to_update_) {
    update_gl();
    to_update_ = false;
  }

  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, texture_id_);

  glColor4d(1.0, 1.0, 1.0, alpha_);
  glBegin(GL_QUADS);

  const double aspect_ratio = image_.cols / static_cast<double>(image_.rows);

  const double height = aspect_ratio * width_m_;

  glTexCoord2d(1, 0);
  glVertex3d(height - (height * 0.5), 0 - (width_m_ * 0.5), 0);
  glTexCoord2d(1, 1);
  glVertex3d(height - (height * 0.5), width_m_ - (width_m_ * 0.5), 0);
  glTexCoord2d(0, 1);
  glVertex3d(0 - (height * 0.5), width_m_ - (width_m_ * 0.5), 0);
  glTexCoord2d(0, 0);
  glVertex3d(0 - (height * 0.5), 0 - (width_m_ * 0.5), 0);

  glEnd();

  glDisable(GL_TEXTURE_2D);
}
}  // namespace viewer
