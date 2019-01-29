#include "geometry/import/read_stl.hh"
#include "testing/gtest.hh"

namespace geometry {
namespace import {

TEST(ReadStlTest, read_stl) {
  const std::string file_path = "/home/jacob/repos/experiments/data/cube_shape.stl";
  const auto result = read_stl(file_path);

  EXPECT_TRUE(static_cast<bool>(result));
}
TEST(ReadStlTest, read_stl_fails) {
  const std::string file_path =
      "/home/jacob/repos/experiments/data/A_NON_EXISTENT_FILE.stl";
  const auto result = read_stl(file_path);
  EXPECT_FALSE(static_cast<bool>(result));
}

}  // namespace import
}  // namespace geometry
