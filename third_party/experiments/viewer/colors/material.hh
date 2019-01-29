#pragma once

#include "eigen.hh"

namespace viewer {
namespace colors {

struct Material {
  jcc::Vec3 ambient;
  jcc::Vec3 diffuse;
  jcc::Vec3 specular;
  double shine;
};

inline Material get_plastic(const jcc::Vec4& color) {
  const Eigen::Vector3d color3 = color.head<3>();
  return Material{color3, jcc::Vec3(0.55, 0.55, 0.55), jcc::Vec3(0.70, 0.70, 0.70), 0.25};
}

// Material get_plastic(const jcc::Vec3& color);
//
// void gl_material(const Material& material);

inline Eigen::Vector4f vecken(const jcc::Vec3& v) {
  const Eigen::Vector4f vv = (Eigen::Vector4f() << v.x(), v.y(), v.z(), 1.0f).finished();
  return vv;
}

inline void gl_material(const Material& material) {
  const Eigen::Vector4f ambient = vecken(material.ambient);
  glMaterialfv(GL_FRONT, GL_AMBIENT, ambient.data());
  const Eigen::Vector4f diffuse = vecken(material.diffuse);
  glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuse.data());

  const Eigen::Vector4f specular = vecken(material.specular);
  glMaterialfv(GL_FRONT, GL_SPECULAR, specular.data());
  glMaterialf(GL_FRONT, GL_SHININESS, material.shine * 128.0f);
}

}  // namespace colors
}  // namespace viewer
