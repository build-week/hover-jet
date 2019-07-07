//%deps(${GTEST_LIBRARIES}, pthread)
//%deps(yaml-cpp)
#include "filtering/yaml_matrix.hh"
// #include "third_party/experiments/testing/gtest.hh"

#include <iostream>

namespace jet {

// Can't make this work, todo: figure out
// TEST(YamlMatrix, yaml_matrix) {
//   const MatNd<3, 5> xx = MatNd<3, 5>::Random();
//
//   YAML::Node node;
//   set_matrix(node, "my_matrix", xx);
//
//   const MatNd<3, 5> result = read_matrix<3, 5>(node, "my_matrix");
// }

void go() {
  const MatNd<3, 5> xx = MatNd<3, 5>::Random();

  YAML::Node node;
  set_matrix(node, "my_matrix", xx);

  const MatNd<3, 5> result = read_matrix<3, 5>(node["my_matrix"]);

  assert((xx - result).norm() < 1e-5);
}

}  // namespace jet

int main() {
  jet::go();
}