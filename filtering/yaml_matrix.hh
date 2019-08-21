#include "third_party/experiments/eigen.hh"

//%deps(yaml-cpp)
#include <yaml-cpp/yaml.h>
#include <cassert>

namespace jet {

template <int rows, int cols>
void set_matrix(YAML::Node& node, const std::string& id, const MatNd<rows, cols>& mat) {
  for (int j = 0; j < cols; ++j) {
    for (int i = 0; i < rows; ++i) {
      node[id].push_back(mat(i, j));
    }
  }
}

template <int rows, int cols>
MatNd<rows, cols> read_matrix(const YAML::Node& node) {
  using Mat = const MatNd<rows, cols>;
  assert(node);
  const std::vector<double> vv = node.as<std::vector<double>>();
  const Eigen::Map<Mat> mmap_mat(vv.data());
  // Copy the map
  const Mat mat = mmap_mat;
  return mat;
}

template <int rows, int cols>
void read_matrix(const YAML::Node& node, MatNd<rows, cols>& mat) {
  using Mat = const MatNd<rows, cols>;
  assert(node);
  const std::vector<double> vv = node.as<std::vector<double>>();
  const Eigen::Map<Mat> mmap_mat(vv.data());
  // Copy the map
  mat = mmap_mat;
}

}  // namespace jet