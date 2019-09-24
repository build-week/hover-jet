#pragma once
//%deps(opengl, glfw, pthread, glew)

#include <Eigen/Dense>

#include <GL/glew.h>

#include <GLFW/glfw3.h>

#include <math.h>
#include "visualization/lviz/shaders.hh"

GLuint LoadShaders();

GLFWwindow* get_window();

Eigen::Matrix4f get_perspective_mat(double fov, double aspect, double near, double far);

Eigen::Matrix4f get_orthographic_mat(double width, double height, double near, double far);

Eigen::Matrix4f get_image_from_view(float aspect_ratio, float near_distance, float far_distance, float camera_distance_from_cursor, bool orthogonal);

Eigen::Matrix4f get_camera_from_world(Eigen::Vector3f camera_loc, Eigen::Vector3f target);

void enter_vertex_buffer_state(GLuint vertex_buffer_id, const GLfloat vertex_buffer_data[], size_t size);

void exit_buffer_state();

void enter_stipple_state();

void exit_stipple_state();