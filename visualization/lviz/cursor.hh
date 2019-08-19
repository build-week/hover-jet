#pragma once
#include "visualization/lviz/graphics_helpers.hh"
#include <Eigen/Dense>

constexpr double MOUSE_SENSITIVITY = .005;

class Cursor {
public:
  Eigen::Vector2d last_mouse_loc;
  Eigen::Vector2d current_mouse_loc;
  Eigen::Vector3f cursor_loc;
  Eigen::Vector3f camera_translation_from_cursor_normalized;

  Cursor() {

    last_mouse_loc << 0, 0;
    current_mouse_loc << 0, 0;
    camera_translation_from_cursor_normalized << 1, 2, 3;
    cursor_loc << 0, 0, 0;
  }
  void update_cursor_state(Eigen::Vector4f, bool, bool, double);
  Eigen::Matrix4f camera_from_world(float);
  void set_camera_translation_from_cursor_normalized(Eigen::Vector3f new_val) {
    camera_translation_from_cursor_normalized = new_val;
  }
};
