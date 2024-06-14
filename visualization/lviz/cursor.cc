#include "visualization/lviz/cursor.hh"

  Cursor::Cursor() {

    last_mouse_loc << 0, 0;
    current_mouse_loc << 0, 0;
    camera_translation_from_cursor_normalized = Eigen::Vector3f(1, 2, 3).normalized();
    cursor_loc << 0, 0, 0;
  }

Eigen::Matrix4f Cursor::camera_from_world(float camera_distance_from_cursor) {
  return get_camera_from_world(camera_translation_from_cursor_normalized *
                                       camera_distance_from_cursor +
                                   cursor_loc,
                               cursor_loc);
}

void Cursor::update_cursor_state(Eigen::Vector4f mouse_move_image_space,
                                 bool right_mouse_button_down,
                                 bool left_mouse_button_down,
                                 double camera_distance_from_cursor) {
  const auto mouse_move_world_space =
      camera_from_world(camera_distance_from_cursor).inverse() * mouse_move_image_space;

  // translating cursor
  if (right_mouse_button_down)
    cursor_loc += mouse_move_world_space.head<3>() * MOUSE_SENSITIVITY *
                  camera_distance_from_cursor * .5;

  // rotating cursor
  if (left_mouse_button_down)
    camera_translation_from_cursor_normalized +=
        mouse_move_world_space.head<3>() * MOUSE_SENSITIVITY;
  if (camera_translation_from_cursor_normalized.y() > 1) {
    camera_translation_from_cursor_normalized /=
        camera_translation_from_cursor_normalized.y();
  }
  camera_translation_from_cursor_normalized.normalize();
}

