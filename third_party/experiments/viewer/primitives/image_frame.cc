
#include "viewer/primitives/image_frame.hh"

#include <GL/glew.h>
#include "viewer/gl_aliases.hh"

namespace viewer {

ImageFrame::ImageFrame(const SE3& frame_from_parent, int rows, int cols, double scale) : Frame(frame_from_parent) {
  update_transform(frame_from_parent);
  rows_  = rows;
  cols_  = cols;
  scale_ = scale;
}

void ImageFrame::draw() const {
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glTransform(frame_from_parent_);

  Eigen::Matrix4d adjust;
  adjust.row(0) = Eigen::Vector4d(scale_, 0.0, 0.0, 0.0);
  adjust.row(1) = Eigen::Vector4d(0.0, scale_, 0.0, 0.0);
  adjust.row(2) = Eigen::Vector4d(0.0, 0.0, 1.0, 0.0);
  adjust.row(3) = Eigen::Vector4d(0.0, 0.0, 0.0, 1.0);

  glApply(adjust);

  // glTranslated(-cols_, -rows_, 0.0);
  // glTranslated(cols_, rows_, 0.0);
  for (const auto& primitive : primitives_) {
    primitive->draw();
  }
  glPopMatrix();
}
}