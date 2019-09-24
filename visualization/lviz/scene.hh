#pragma once
#include "visualization/lviz/graphics_helpers.hh"
#include <GL/glew.h>

struct ColoredPoint {
  GLfloat x;
  GLfloat y;
  GLfloat z;
  GLfloat r;
  GLfloat g;
  GLfloat b;
  GLfloat scale;
};

struct Frame {
  std::vector<ColoredPoint> points;
  std::vector<ColoredPoint> lines;
  float display_duration;
};

class Scene {
private:
  std::vector<Frame> frames;
  GLuint point_vertex_buffer_id;
  GLuint line_vertex_buffer_id;
  int current_frame_index;
  bool is_playing;

public:
  Scene() {
    add_frame();
    is_playing = false;
  }

  bool get_is_playing() { return is_playing; }

  void toggle_is_playing() { is_playing = !is_playing; }

  void setup_buffers() {
    glGenBuffers(1, &point_vertex_buffer_id);
    glGenBuffers(1, &line_vertex_buffer_id);
  }

  void add_to_frame_index(int delta) { current_frame_index += delta; }

  Frame &get_current_frame() {
    return frames[current_frame_index % frames.size()];
  }

  void add_frame() {
    Frame new_frame;
    new_frame.display_duration = .1;
    frames.push_back(new_frame);
  }

  void add_point(ColoredPoint pt) { frames.back().points.push_back(pt); }

  void set_frame_duration(float new_duration) { frames.back().display_duration = new_duration; }

  void add_line(std::pair<ColoredPoint, ColoredPoint> points) {
    frames.back().lines.push_back(points.first);
    frames.back().lines.push_back(points.second);
  }

  void draw() {

    enter_vertex_buffer_state(
        point_vertex_buffer_id, (GLfloat *)&get_current_frame().points[0],
        sizeof(ColoredPoint) * get_current_frame().points.size());

    auto n_point_elements = get_current_frame().points.size() * 7;
    glDrawArrays(GL_POINTS, 0, n_point_elements);

    enter_vertex_buffer_state(
        line_vertex_buffer_id, (GLfloat *)&get_current_frame().lines[0],
        sizeof(ColoredPoint) * get_current_frame().lines.size());

    auto n_line_elements = get_current_frame().lines.size() * 7;

    // TODO enable optional dashed lines
    // enter_stipple_state();
    glLineWidth(1.5);
    glDrawArrays(GL_LINES, 0, n_line_elements);
    // exit_stipple_state();

    exit_buffer_state();
  }
};