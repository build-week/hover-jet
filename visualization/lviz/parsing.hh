#include "third_party/tinycolormap/tinycolormap.hh"
#include "visualization/lviz/scene.hh"
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

// TODO instead have a master list of all color names and generate these regexes
std::string ALL_COLOR_NAMES_STR = "(?:(?:red)|(?:green)|(?:blue)|(?:orange)|(?:magenta)|(?:violet)|(?:cyan))";
std::string VEC3_STR =
    "(" + FLOAT_STR + ")" + WHITESPACE_STR + "(" + FLOAT_STR + ")" + WHITESPACE_STR + "(" + FLOAT_STR + ")";
std::string RGB_COLOR_STR = "(?:" + FLOAT_STR + WHITESPACE_STR + FLOAT_STR + WHITESPACE_STR + FLOAT_STR + ")";
std::string COLORMAP_COLOR_STR = "(?:(?:(?:viridis)|(?:inferno))" + WHITESPACE_STR + FLOAT_STR + ")";
std::string COLOR = "(" + RGB_COLOR_STR + "|" + ALL_COLOR_NAMES_STR + "|" + COLORMAP_COLOR_STR + ")";

std::regex RGB_COLOR_REGEX("(" + FLOAT_STR + ")" + WHITESPACE_STR + "(" + FLOAT_STR + ")" + WHITESPACE_STR + "(" +
                           FLOAT_STR + ")");

// TODO instead have a master list of all colormap names and generate these regexes
std::regex COLORMAP_COLOR_REGEX("(?:((?:viridis)|(?:inferno))" + WHITESPACE_STR + "(" + FLOAT_STR + "))");

std::regex ALL_COLOR_NAMES_REGEX(ALL_COLOR_NAMES_STR);

std::regex POINT_REGEX("point" + WHITESPACE_STR + VEC3_STR + WHITESPACE_STR + COLOR + WHITESPACE_STR + "(" + FLOAT_STR +
                       ")");

std::regex LINE_REGEX("^line" + WHITESPACE_STR + VEC3_STR + WHITESPACE_STR + COLOR + WHITESPACE_STR + VEC3_STR +
                      WHITESPACE_STR + COLOR + "$");

std::regex NEW_FRAME_REGEX("new_frame");
std::regex SET_FRAME_TIME_REGEX("set_frame_time" + WHITESPACE_STR + "(" + FLOAT_STR + ")");

std::regex WHITESPACE_REGEX(WHITESPACE_STR);



Eigen::Vector3f vec3_from_strings(const std::tuple<std::string, std::string, std::string> &strings) {
  return Eigen::Vector3f(std::stof(std::get<0>(strings)), std::stof(std::get<1>(strings)),
                         std::stof(std::get<2>(strings)));
}

rgb_color parse_color(const std::string &color_string) {
  std::smatch matches;
  if (std::regex_match(color_string, matches, RGB_COLOR_REGEX)) {
    return rgb_color{stof(matches[1]) / 255.0, stof(matches[2]) / 255.0, stof(matches[3]) / 255.0};
  }

  else if (std::regex_match(color_string, matches, COLORMAP_COLOR_REGEX)) {

    auto value = stof(matches[2]);
    value = std::clamp(value, 0.0f, 1.0f);


    tinycolormap::ColormapType colormap;
    const auto colormap_name = matches[1];
    if (colormap_name == "viridis"){
        colormap = tinycolormap::ColormapType::Viridis;
    }
    else if (colormap_name == "inferno"){
        colormap = tinycolormap::ColormapType::Inferno;
    }
    else{
        std::cerr << "ERROR: unrecognized colormap " << colormap_name << " . Substituting viridis." << std::endl;
        colormap = tinycolormap::ColormapType::Viridis;
    }
    const auto color = tinycolormap::GetColor(value, colormap);
    return rgb_color{(float)color.r(), (float)color.g(), (float)color.b()};
  }
    else if (std::regex_match(color_string, matches, ALL_COLOR_NAMES_REGEX)) {
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
  std::cerr << "ERROR: could not parse color " << matches[0] << " . Substituting white." << std::endl;
  return rgb_color{1,1,1};
}

std::optional<SceneUpdate> scene_element_from_line(const std::string &line) {
  std::smatch matches;

  if (std::regex_match(line, matches, POINT_REGEX)) {
    if (VERBOSE_PARSING){
        for (size_t i = 0; i < matches.size(); ++i) {
          std::cout << i << ": '" << matches[i].str() << "'\n";
        }
    }
    Eigen::Vector3f location;
    location << stof(matches[1]), stof(matches[2]), stof(matches[3]);
    const auto color = parse_color(matches[4]);
    const float scale = std::stof(matches[5]);

    return std::make_optional(ColoredPoint{location.x(), location.z(), location.y(), color.r, color.g, color.b, scale});
  } else if (std::regex_match(line, matches, LINE_REGEX)) {
    
    if (VERBOSE_PARSING){
        for (size_t i = 0; i < matches.size(); ++i) {
        std::cout << i << ": '" << matches[i].str() << "'\n";
        }
    }
    
    Eigen::Vector3f line_endpoint_1;
    line_endpoint_1 << stof(matches[1]), stof(matches[2]), stof(matches[3]);
    auto color_1 = parse_color(matches[4]);

    Eigen::Vector3f line_endpoint_2;
    line_endpoint_2 << stof(matches[5]), stof(matches[6]), stof(matches[7]);
    auto color_2 = parse_color(matches[8]);
    return std::make_optional(
        std::make_pair(ColoredPoint{line_endpoint_1.x(), line_endpoint_1.z(), line_endpoint_1.y(), color_1.r, color_1.g, color_1.b, 1},
                       ColoredPoint{line_endpoint_2.x(), line_endpoint_2.z(), line_endpoint_2.y(), color_2.r, color_2.g, color_2.b, 1}));
  } else if (std::regex_match(line, matches, NEW_FRAME_REGEX)) {
    return std::make_optional(NewScene{});
  } else if (std::regex_match(line, matches, SET_FRAME_TIME_REGEX)) {
    auto frame_duration = std::stof(matches[1]);
    return std::make_optional(SetFrameTime{frame_duration});
  } else {
    std::cout << line << std::endl;
  }
  return std::nullopt;
}