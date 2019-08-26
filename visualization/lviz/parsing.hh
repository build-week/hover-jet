#include "third_party/tinycolormap/tinycolormap.hh"

#include <regex>
#include <iostream>
#include <variant>


constexpr bool VERBOSE_PARSING = false;

struct rgb_color {
  float r;
  float g;
  float b;
};

struct NewScene {};

struct SetFrameTime {
  float frame_duration;
};

using SceneUpdate = std::variant<ColoredPoint, std::pair<ColoredPoint, ColoredPoint>, NewScene, SetFrameTime>;

std::string FLOAT_STR = ("[+-]?(?:\\d+(?:[.]\\d*)?(?:e[+-]?\\d+)?|[.]\\d+(?:e[+-]?\\d+)?)");
std::string WHITESPACE_STR = "\\s+";
std::string ALL_COLOR_NAMES_STR = "(?:(?:red)|(?:green)|(?:blue)|(?:orange)|(?:magenta)|(?:violet)|(?:cyan))";
std::string VEC3_STR =
    "(" + FLOAT_STR + ")" + WHITESPACE_STR + "(" + FLOAT_STR + ")" + WHITESPACE_STR + "(" + FLOAT_STR + ")";
std::string RGB_COLOR_STR = "(?:" + FLOAT_STR + WHITESPACE_STR + FLOAT_STR + WHITESPACE_STR + FLOAT_STR + ")";
std::string VIRIDIS_COLOR_STR = "(?:viridis" + WHITESPACE_STR + FLOAT_STR + ")";
std::string COLOR = "(" + RGB_COLOR_STR + "|" + ALL_COLOR_NAMES_STR + "|" + VIRIDIS_COLOR_STR + ")";

std::regex RGB_COLOR_REGEX("(" + FLOAT_STR + ")" + WHITESPACE_STR + "(" + FLOAT_STR + ")" + WHITESPACE_STR + "(" +
                           FLOAT_STR + ")");

std::regex VIRIDIS_COLOR_REGEX("(?:viridis" + WHITESPACE_STR + "(" + FLOAT_STR + "))");

std::regex ALL_COLOR_NAMES_REGEX(ALL_COLOR_NAMES_STR);

std::regex POINT_REGEX("^point" + WHITESPACE_STR + VEC3_STR + WHITESPACE_STR + COLOR + WHITESPACE_STR + "(" + FLOAT_STR +
                       ")$");

std::regex LINE_REGEX("^line" + WHITESPACE_STR + VEC3_STR + WHITESPACE_STR + COLOR + WHITESPACE_STR + VEC3_STR +
                      WHITESPACE_STR + COLOR);

std::regex NEW_FRAME_REGEX("^new_frame$");
std::regex SET_FRAME_TIME_REGEX("^set_frame_time" + WHITESPACE_STR + "(" + FLOAT_STR + ")$");

std::regex WHITESPACE_REGEX(WHITESPACE_STR);



Eigen::Vector3f vec3_from_strings(const std::tuple<std::string, std::string, std::string> &strings) {
  return Eigen::Vector3f(std::stof(std::get<0>(strings)), std::stof(std::get<1>(strings)),
                         std::stof(std::get<2>(strings)));
}

rgb_color parse_color(const std::string &color_string) {
  std::smatch matches;
  if (std::regex_search(color_string, matches, RGB_COLOR_REGEX)) {
    return rgb_color{stof(matches[1]) / 255.0, stof(matches[2]) / 255.0, stof(matches[3]) / 255.0};
  }
  else if (std::regex_search(color_string, matches, ALL_COLOR_NAMES_REGEX)) {
    if (matches[0] == "red") {
      return rgb_color{220.0 / 255.0, 50.0 / 255.0, 47.0 / 255.0};
    } else if (matches[0] == "green") {
      return rgb_color{133.0 / 255.0, 153.0 / 255.0, 0.0 / 255.0};
    } else if (matches[0] == "blue") {
      return rgb_color{38.0 / 255.0, 139.0 / 255.0, 210.0 / 255.0};
    } else if (matches[0] == "orange") {
      return rgb_color{203.0 / 255.0, 75.0 / 255.0, 22.0 / 255.0};
    } else if (matches[0] == "magenta") {
      return rgb_color{211.0 / 255.0, 54.0 / 255.0, 130.0 / 255.0};
    } else if (matches[0] == "violet") {
      return rgb_color{108.0 / 255.0, 113.0 / 255.0, 196.0 / 255.0};
    } else if (matches[0] == "cyan") {
      return rgb_color{42.0 / 255.0, 161.0 / 255.0, 152.0 / 255.0};
    }
  }
  else if (std::regex_search(color_string, matches, VIRIDIS_COLOR_REGEX)) {
    auto value = stof(matches[1]);
    value = std::clamp(value, 0.0f, 1.0f);
    const auto color = tinycolormap::GetColor(value, tinycolormap::ColormapType::Viridis);
    return rgb_color{(float)color.r(), (float)color.g(), (float)color.b()};


  }


  std::cout << "ERROR: could not parse color " << matches[0] << " . Substituting white.";
  return rgb_color{1,1,1};
}

std::optional<SceneUpdate> scene_element_from_line(const std::string &line) {
  std::smatch matches;

  if (std::regex_search(line, matches, POINT_REGEX)) {
    if (VERBOSE_PARSING){
        for (size_t i = 0; i < matches.size(); ++i) {
          std::cout << i << ": '" << matches[i].str() << "'\n";
        }
    }
    Eigen::Vector3f location;
    location << stof(matches[1]), stof(matches[2]), stof(matches[3]);
    auto color = parse_color(matches[4]);
    float scale = std::stof(matches[5]);

    return std::make_optional(ColoredPoint{location.x(), location.z(), location.y(), color.r, color.g, color.b, scale});
  } else if (std::regex_search(line, matches, LINE_REGEX)) {
    
    if (VERBOSE_PARSING){
        for (size_t i = 0; i < matches.size(); ++i) {
        std::cout << i << ": '" << matches[i].str() << "'\n";
        }
    }
    
    Eigen::Vector3f location1;
    location1 << stof(matches[1]), stof(matches[2]), stof(matches[3]);
    auto color1 = parse_color(matches[4]);

    Eigen::Vector3f location2;
    location2 << stof(matches[5]), stof(matches[6]), stof(matches[7]);
    auto color2 = parse_color(matches[8]);
    return std::make_optional(
        std::make_pair(ColoredPoint{location1.x(), location1.z(), location1.y(), color1.r, color1.g, color1.b, 1},
                       ColoredPoint{location2.x(), location2.z(), location2.y(), color2.r, color2.g, color2.b, 1}));
  } else if (std::regex_search(line, matches, NEW_FRAME_REGEX)) {
    return std::make_optional(NewScene{});
  } else if (std::regex_search(line, matches, SET_FRAME_TIME_REGEX)) {
    auto frame_duration = std::stof(matches[1]);
    return std::make_optional(SetFrameTime{frame_duration});
  } else {
    std::cout << line << std::endl;
  }
  return std::nullopt;
}