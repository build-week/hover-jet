//%deps(opengl, glfw, pthread, glew)


#include "visualization/lviz/cursor.hh"
#include "visualization/lviz/graphics_helpers.hh"
#include "visualization/lviz/scene.hh"

#include <any>
#include <chrono>
#include <cmath>
#include <iostream>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <math.h>
#include <variant>

#include <sophus/se3.hpp>
#include <Eigen/Dense>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <regex> 
// #include <lars/parser/generator.h>

struct rgb_color {
  int r;
  int g;
  int b;
};

rgb_color color_from_rgb_tuple(std::tuple<int, int, int> rgb) {
  return rgb_color{std::get<0>(rgb), std::get<1>(rgb), std::get<2>(rgb)};
}

struct NewScene {};


// TODO stop using this everywhere
using TreeNode = std::variant<int, float, ColoredPoint, rgb_color, std::string,
                              std::pair<ColoredPoint, ColoredPoint>,
                              std::tuple<float, float, float>,
                              std::tuple<int, int, int>, NewScene>;

// lars::ParserGenerator<TreeNode> get_parser() {
//   using namespace std;

//   lars::ParserGenerator<TreeNode> calculator;

//   auto &g = calculator;
//   g.setSeparator(g["Whitespace"] << "[\t ]");

//   g["Float"] << "'-'? [0-9]+ ('.' [0-9]+)?" >>
//       [](auto e) { return stof(e.string()); };
//   g["Vec3"] << "Float Float Float" >> [](auto e) {
//     return std::make_tuple(std::get<float>(e[0].evaluate()),
//                            std::get<float>(e[1].evaluate()),
//                            std::get<float>(e[2].evaluate()));
//   };

//   // TODO check 0..255
//   g["Byte"] << "[0-9]+" >> [](auto e) { return stoi(e.string()); };
//   g["ByteTriple"] << "Byte Byte Byte" >> [](auto e) {
//     return std::make_tuple(std::get<int>(e[0].evaluate()),
//                            std::get<int>(e[1].evaluate()),
//                            std::get<int>(e[2].evaluate()));
//   };

//   g["RgbId"] << "'rgb'";
//   g["NamedRgb"] << "RgbId ByteTriple" >> [](auto e) {
//     return color_from_rgb_tuple(
//         std::get<std::tuple<int, int, int>>(e[1].evaluate()));
//   };
//   g["NakedRgb"] << "ByteTriple" >> [](auto e) {
//     return color_from_rgb_tuple(
//         std::get<std::tuple<int, int, int>>(e[0].evaluate()));
//   };

//   g["HsvId"] << "'hsv'";
//   g["HsvColor"] << "HsvId ByteTriple" >> [](auto e) {
//     return color_from_rgb_tuple(
//         std::get<std::tuple<int, int, int>>(e[1].evaluate()));
//   };

//   g["Color"] << "NamedRgb | NakedRgb";

//   g["Point"] << "'point' Float Float Float Color Float" >> [](auto e) {
//     auto position = std::make_tuple(stof(e[0].string()), stof(e[1].string()),
//                                     stof(e[2].string()));
//     auto color = std::get<rgb_color>(e[3].evaluate());
//     auto scale = std::get<float>(e[4].evaluate());
//     return ColoredPoint{std::get<0>(position),
//                         std::get<1>(position),
//                         std::get<2>(position),
//                         (float)color.r / 255.0f,
//                         (float)color.g / 255.0f,
//                         (float)color.b / 255.0f,
//                         scale};
//   };

//   g["Line"] << "'line'  Vec3 Color Vec3 Color" >> [](auto e) {
//     auto position1 = std::get<std::tuple<float, float, float>>(e[0].evaluate());
//     auto position2 = std::get<std::tuple<float, float, float>>(e[2].evaluate());
//     auto color1 = std::get<rgb_color>(e[1].evaluate());
//     auto color2 = std::get<rgb_color>(e[3].evaluate());
//     float scale = 1.0;
//     return std::make_pair(
//         ColoredPoint{std::get<0>(position1), std::get<1>(position1),
//                      std::get<2>(position1), (float)color1.r / 255.0f,
//                      (float)color1.g / 255.0f, (float)color1.b / 255.0f, scale},
//         ColoredPoint{std::get<0>(position2), std::get<1>(position2),
//                      std::get<2>(position2), (float)color2.r / 255.0f,
//                      (float)color2.g / 255.0f, (float)color2.b / 255.0f, scale});
//   };
//   // g["DashedLine"] << "'dashed' Line";

//   g["NewSceneId"] << "'new_frame'" >> [](auto e) { return NewScene{}; };

//   g["SceneElement"] << "Point | Line | NewSceneId";

//   g.setStart(g["SceneElement"]);
//   return calculator;
// }

Eigen::Vector2d get_mouse_loc(auto window) {
  double xpos, ypos;
  glfwGetCursorPos(window, &xpos, &ypos);
  return Eigen::Vector2d(xpos, ypos);
}

std::pair<int, int> get_window_dimensions(auto window) {
  int window_width, window_height;
  glfwGetWindowSize(window, &window_width, &window_height);
  return std::make_pair(window_width, window_height);
}

void reset_viewport_size(auto window) {
  auto window_dimensions = get_window_dimensions(window);
  glViewport(0, 0, window_dimensions.first, window_dimensions.second);
}

bool right_mouse_button_down = false;
bool left_mouse_button_down = false;
double camera_distance_from_cursor = 5;
Scene scene;
Cursor cursor;
std::mutex scene_modification_mutex;
std::mutex buffer_modification_mutex;

void mouse_button_callback(GLFWwindow *window, int button, int action,
                           int mods) {
  if (button == GLFW_MOUSE_BUTTON_RIGHT)
    right_mouse_button_down = action == GLFW_PRESS;
  if (button == GLFW_MOUSE_BUTTON_LEFT)
    left_mouse_button_down = action == GLFW_PRESS;
}

static void key_callback(GLFWwindow *window, int key, int scancode, int action,
                         int mods) {
  if (action == GLFW_PRESS) {
    if (key == GLFW_KEY_ESCAPE)
      glfwSetWindowShouldClose(window, GLFW_TRUE);
    if (key == GLFW_KEY_RIGHT)
      scene.add_to_frame_index(1);
    if (key == GLFW_KEY_LEFT)
      scene.add_to_frame_index(-1);
    if (key == GLFW_KEY_SPACE)
      scene.toggle_is_playing();
    if (key == GLFW_KEY_X)
      cursor.set_camera_translation_from_cursor_normalized(
          Eigen::Vector3f(1, 0, 0).normalized());
    if (key == GLFW_KEY_Y)
      cursor.set_camera_translation_from_cursor_normalized(
          Eigen::Vector3f(0.00001, 1, 0).normalized());
    if (key == GLFW_KEY_Z)
      cursor.set_camera_translation_from_cursor_normalized(
          Eigen::Vector3f(0, 0, 1).normalized());
  }
}

void scroll_callback(GLFWwindow *window, double xoffset, double yoffset) {
  camera_distance_from_cursor += yoffset;
  camera_distance_from_cursor =
      std::clamp(camera_distance_from_cursor, .01, 10000.0);
}

std::vector<TreeNode> scene_element_buffer;

// inline TreeNode
// scene_element_from_line(std::string line,
//                         lars::ParserGenerator<TreeNode> &calculator) {
//   try {
//     return calculator.run(line);
//   } catch (lars::SyntaxError &error) {
//     return line;
//   }
// }

// void run_stream_parser(Scene &scene) {
  // lars::ParserGenerator<TreeNode> calculator = get_parser();

  // // vastly increases stream reading speed
  // std::ios_base::sync_with_stdio(false);
  // while (true) {
  //   std::string line;
  //   std::getline(std::cin, line);
  //   if (line.length() > 0) {
  //     TreeNode result = scene_element_from_line(line, calculator);
  //     if (std::holds_alternative<std::string>(result)) {
  //       std::cout << std::get<std::string>(result) << std::endl;
  //       continue;
  //     }
  //     buffer_modification_mutex.lock();
  //     scene_element_buffer.push_back(result);
  //     buffer_modification_mutex.unlock();
  //   }
  // }
// }

void run_stream_parser(Scene &scene) {
  // vastly increases stream reading speed
  std::ios_base::sync_with_stdio(false);
  std::regex("(point)\\s+4");
  while (true) {
    std::string line;
    std::getline(std::cin, line);
    if (line.length() > 0) {

      buffer_modification_mutex.lock();
      scene_element_buffer.push_back(ColoredPoint{1,1,1,1,1,1,10});
      buffer_modification_mutex.unlock();
    }
  }
}

void apply_scene_changes(Scene &scene) {
  buffer_modification_mutex.lock();

  for (auto element : scene_element_buffer) {
    if (std::holds_alternative<ColoredPoint>(element)) {
      scene.add_point(std::get<ColoredPoint>(element));
    }
    if (std::holds_alternative<std::pair<ColoredPoint, ColoredPoint>>(
            element)) {
      scene.add_line(std::get<std::pair<ColoredPoint, ColoredPoint>>(element));
    }
    if (std::holds_alternative<NewScene>(element)) {
      scene.add_frame();
    }
  }
  // std::cout << scene_element_buffer.size() << std::endl;
  scene_element_buffer.clear();
  buffer_modification_mutex.unlock();
}

int main() {

  using namespace std;
  auto window = get_window();
  glfwSetMouseButtonCallback(window, mouse_button_callback);
  glfwSetScrollCallback(window, scroll_callback);
  glfwSetKeyCallback(window, key_callback);
  GLuint shader_program_id = LoadShaders();
  scene.setup_buffers();
  glUseProgram(shader_program_id);

  // GLuint vertex_buffer_id;
  // Generate 1 buffer, put the resulting identifier in vertex_buffer_id
  // glGenBuffers(1, &vertex_buffer_id);
  // The following commands will talk about our 'vertex_buffer_id' buffer


  int frames_so_far = 0;
  cursor.last_mouse_loc = get_mouse_loc(window);
  cursor.current_mouse_loc = get_mouse_loc(window);

  std::thread parsing_thread(run_stream_parser, std::ref(scene));

  auto frame_duration = std::chrono::milliseconds(500);
  auto frame_time_remaining = frame_duration;

  while (!glfwWindowShouldClose(window)) {
    auto render_start_time = std::chrono::high_resolution_clock::now();

    // update mouse history
    auto mouse_delta = cursor.current_mouse_loc - cursor.last_mouse_loc;
    cursor.last_mouse_loc = cursor.current_mouse_loc;
    cursor.current_mouse_loc = get_mouse_loc(window);

    frames_so_far++;

    // calculate mouse movements
    const Eigen::Vector4f mouse_move_image_space(-mouse_delta.x(),
                                                 mouse_delta.y(), 0.0, 0.0);
    cursor.update_cursor_state(mouse_move_image_space, right_mouse_button_down,
                               left_mouse_button_down,
                               camera_distance_from_cursor);

    auto window_dimensions = get_window_dimensions(window);
    auto aspect_ratio =
        1.0 * window_dimensions.first / window_dimensions.second;
    Eigen::Matrix4f image_from_world =
        get_perspective_mat(.6, aspect_ratio, .1, 10000) *
        cursor.camera_from_world(camera_distance_from_cursor);

    GLuint image_from_world_idx =
        glGetUniformLocation(shader_program_id, "image_from_world");
    glUniformMatrix4fv(image_from_world_idx, 1, GL_FALSE,
                       image_from_world.data());

    // get width and height
    reset_viewport_size(window);

    // Keep running
    glfwSwapBuffers(window);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearDepth(0.0f);
    glClearColor(0, 43.0f / 255, 54.0f / 255, 1.0f); //solarized base03

    glfwPollEvents();
    scene.draw();
    apply_scene_changes(scene);
    auto render_end_time = std::chrono::high_resolution_clock::now();
    auto loop_time_so_far =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            render_end_time - render_start_time);
    frame_time_remaining = frame_time_remaining - loop_time_so_far;
    if ((frame_time_remaining.count() < 0) && scene.get_is_playing()){
      scene.add_to_frame_index(1);
      frame_time_remaining = std::chrono::milliseconds((int)(scene.get_current_frame().display_duration * 1000));
    }
  }
  glfwDestroyWindow(window);
  glfwTerminate();

  return 0;
}
