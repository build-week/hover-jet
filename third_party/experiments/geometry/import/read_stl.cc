#include "geometry/import/read_stl.hh"

#include <fstream>
#include <iostream>

namespace geometry {
namespace import {

namespace {
using Vec3 = Eigen::Vector3d;

// Read something from a file
template <typename T>
T read_from_file(std::ifstream &stream) {
  T data;
  stream.read(reinterpret_cast<char *>(&data), sizeof(data));
  return data;
}

Eigen::Vector3f read_vec3(std::ifstream &stream) {
  Eigen::Vector3f vec;
  for (int k = 0; k < 3; ++k) {
    float f = read_from_file<float>(stream);
    stream.seekg(0, std::ios::cur);
    vec(k) = f;
  }
  return vec;
}
}  // namespace

// Read an stl from a file
// https://en.wikipedia.org/wiki/STL_(file_format)#Binary_STL
jcc::Optional<TriMesh> read_stl(const std::string &file_path) {
  std::ifstream stl_file(file_path, std::ios::binary);
  if (!stl_file.is_open()) {
    return {};
  }
  // Seek 80 chars
  constexpr int HEADER_SIZE = 80;
  stl_file.seekg(HEADER_SIZE, std::ios::beg);
  const uint32_t triangle_count = read_from_file<uint32_t>(stl_file);

  constexpr size_t BYTES_PER_FLOAT = 4;
  constexpr size_t FLOATS_PER_VERT = 3;
  constexpr size_t VERTS_PER_TRI = 4;
  constexpr size_t BYTES_ATTRIB_BYTE_CT = 2;
  constexpr size_t BYTES_PER_TRI =
      FLOATS_PER_VERT * BYTES_PER_FLOAT * VERTS_PER_TRI + BYTES_ATTRIB_BYTE_CT;

  TriMesh tri_mesh;
  for (size_t i = 0; i < triangle_count; ++i) {
    stl_file.seekg(HEADER_SIZE + 4 + (i * BYTES_PER_TRI), std::ios::beg);

    Triangle tri;
    tri.normal = read_vec3(stl_file).cast<double>();
    for (int k = 0; k < 3; ++k) {
      tri.vertices[k] = read_vec3(stl_file).cast<double>();
    }
    tri_mesh.triangles.push_back(tri);
  }

  return {tri_mesh};
}
}  // namespace import
}  // namespace geometry
