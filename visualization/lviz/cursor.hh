#pragma once
#include "visualization/lviz/graphics_helpers.hh"
#include <Eigen/Dense>

constexpr double MOUSE_SENSITIVITY = .005;

class Cursor {
public:
  bool right_mouse_button_down = false;
  bool left_mouse_button_down = false;
  double camera_distance_from_cursor = 5;
  bool perspective_mode = true;

  Eigen::Vector2d last_mouse_loc;
  Eigen::Vector2d current_mouse_loc;
  Eigen::Vector3f cursor_loc;
  Eigen::Vector3f camera_translation_from_cursor_normalized;
  Cursor();

  void update_cursor_state(Eigen::Vector4f, bool, bool, double);
  Eigen::Matrix4f camera_from_world(float);
  void set_camera_translation_from_cursor_normalized(Eigen::Vector3f new_val) {
    camera_translation_from_cursor_normalized = new_val;
  }
};
