#include "visualization/lviz/graphics_helpers.hh"

#include <iostream>

GLuint LoadShaders() {
  // Create the shaders
  GLuint VertexShaderID = glCreateShader(GL_VERTEX_SHADER);
  GLuint FragmentShaderID = glCreateShader(GL_FRAGMENT_SHADER);

  std::string VertexShaderCode = vertex_shader_text;

  std::string FragmentShaderCode = fragment_shader_text;

  GLint Result = GL_FALSE;
  int InfoLogLength;

  // Compile Vertex Shader
  char const *VertexSourcePointer = VertexShaderCode.c_str();
  glShaderSource(VertexShaderID, 1, &VertexSourcePointer, NULL);
  glCompileShader(VertexShaderID);

  // Check Vertex Shader
  glGetShaderiv(VertexShaderID, GL_COMPILE_STATUS, &Result);
  glGetShaderiv(VertexShaderID, GL_INFO_LOG_LENGTH, &InfoLogLength);
  if (InfoLogLength > 0) {
    std::vector<char> VertexShaderErrorMessage(InfoLogLength + 1);
    glGetShaderInfoLog(VertexShaderID, InfoLogLength, NULL, &VertexShaderErrorMessage[0]);
    printf("%s\n", &VertexShaderErrorMessage[0]);
  }

  // Compile Fragment Shader
  char const *FragmentSourcePointer = FragmentShaderCode.c_str();
  glShaderSource(FragmentShaderID, 1, &FragmentSourcePointer, NULL);
  glCompileShader(FragmentShaderID);

  // Check Fragment Shader
  glGetShaderiv(FragmentShaderID, GL_COMPILE_STATUS, &Result);
  glGetShaderiv(FragmentShaderID, GL_INFO_LOG_LENGTH, &InfoLogLength);
  if (InfoLogLength > 0) {
    std::vector<char> FragmentShaderErrorMessage(InfoLogLength + 1);
    glGetShaderInfoLog(FragmentShaderID, InfoLogLength, NULL, &FragmentShaderErrorMessage[0]);
    printf("%s\n", &FragmentShaderErrorMessage[0]);
  }

  // Link the program
  printf("Linking program\n");
  GLuint ProgramID = glCreateProgram();
  glAttachShader(ProgramID, VertexShaderID);
  glAttachShader(ProgramID, FragmentShaderID);
  glLinkProgram(ProgramID);

  // Check the program
  glGetProgramiv(ProgramID, GL_LINK_STATUS, &Result);
  glGetProgramiv(ProgramID, GL_INFO_LOG_LENGTH, &InfoLogLength);
  if (InfoLogLength > 0) {
    std::vector<char> ProgramErrorMessage(InfoLogLength + 1);
    glGetProgramInfoLog(ProgramID, InfoLogLength, NULL, &ProgramErrorMessage[0]);
    printf("%s\n", &ProgramErrorMessage[0]);
  }

  return ProgramID;
}

GLFWwindow *get_window() {
  if (!glfwInit()) {
    // Initialization failed
    fprintf(stderr, "Failed to initialize GLFW\n");
  }
  glfwWindowHint(GLFW_SAMPLES, 4);

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
  GLFWwindow *window = glfwCreateWindow(640, 480, "3D Viz", NULL, NULL);
  if (!window) {
    // Window or OpenGL context creation failed
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  glewInit();

  glEnable(GL_MULTISAMPLE);
  glLineWidth(.5);
  glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_ALWAYS);
  glDepthFunc(GL_GREATER);  // TODO perhaps some issue in projection is making
                            // this necessary

  return window;
}

// adapted from
// https://stackoverflow.com/questions/1638355/function-for-perspective-projection-of-a-matrix-in-c
Eigen::Matrix4f get_perspective_mat(double fov, double aspect, double near, double far) {
  double yScale = 1.0 / tan(fov);
  double xScale = yScale / aspect;
  Eigen::Matrix4f mat;
  mat << xScale, 0, 0, 0, 0, yScale, 0, 0, 0, 0, -(far + near) / (near - far), -2 * far * near / (near - far), 0, 0, -1,
      0;

  return mat;
}

Eigen::Matrix4f get_orthographic_mat(double width, double height, double near, double far) {
  double xScale = 1.0 / width;
  double yScale = 1.0 / height;
  Eigen::Matrix4f mat;
  mat << xScale, 0, 0, 0, 0, yScale, 0, 0, 0, 0, 2 / (far - near), -(far + near) / (near - far), 0, 0, 0, 10;
  return mat;
}

Eigen::Matrix4f get_image_from_view(float aspect_ratio, float near_distance, float far_distance, float camera_distance_from_cursor, bool orthogonal) {
  if (orthogonal) {
    constexpr float orthograpic_scale_constant = .052;
    return get_orthographic_mat(camera_distance_from_cursor * aspect_ratio * orthograpic_scale_constant,
                                           camera_distance_from_cursor * orthograpic_scale_constant,
                                           near_distance,
                                           far_distance);
  } else {
    return get_perspective_mat(.6, aspect_ratio, near_distance, far_distance);
  }
}

Eigen::Matrix4f get_camera_from_world(Eigen::Vector3f camera_loc, Eigen::Vector3f target) {
  const Eigen::Vector3f look_direction = (camera_loc - target).normalized();
  const Eigen::Vector3f global_up(0, 0, 1);
  const Eigen::Vector3f right = global_up.cross(look_direction).normalized();
  const Eigen::Vector3f up = look_direction.cross(right).normalized();

  // JAKE: better/ cleaner way to write this?
  Eigen::Matrix4f camera_from_world_rotation;
  camera_from_world_rotation << right.x(), right.y(), right.z(), 0, up.x(), up.y(), up.z(), 0, look_direction.x(),
      look_direction.y(), look_direction.z(), 0, 0, 0, 0, 1;

  Eigen::Matrix4f camera_from_world_translation;
  camera_from_world_translation << 1, 0, 0, -camera_loc.x(), 0, 1, 0, -camera_loc.y(), 0, 0, 1, -camera_loc.z(), 0, 0,
      0, 1;
  return camera_from_world_rotation * camera_from_world_translation;
}

void enter_vertex_buffer_state(GLuint vertex_buffer_id, const GLfloat vertex_buffer_data[], size_t size) {
  glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_id);
  glBufferData(GL_ARRAY_BUFFER, size, vertex_buffer_data, GL_STATIC_DRAW);

  // position
  glVertexAttribPointer(0,                  // attribute 0. No particular reason for 0, but
                                            // must match the layout in the shader.
                        3,                  // size
                        GL_FLOAT,           // type
                        GL_FALSE,           // normalized?
                        7 * sizeof(float),  // stride
                        (void *)0           // array buffer offset
  );
  glEnableVertexAttribArray(0);

  // color
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 7 * sizeof(float), (void *)(3 * sizeof(float)));
  glEnableVertexAttribArray(1);

  // scale
  glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, 7 * sizeof(float), (void *)(6 * sizeof(float)));
  glEnableVertexAttribArray(2);
}

void exit_buffer_state() {
  glDisableVertexAttribArray(0);
  glDisableVertexAttribArray(1);
  glDisableVertexAttribArray(2);
}

void enter_stipple_state() {
  glPushAttrib(GL_ENABLE_BIT);
  glLineStipple(1, 0x0F0F);
  glEnable(GL_LINE_STIPPLE);
}

void exit_stipple_state() {
  glDisable(GL_LINE_STIPPLE);
  glPopAttrib();
}
