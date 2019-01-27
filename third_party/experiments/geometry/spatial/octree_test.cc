#include "testing/gtest.hh"

#include "geometry/import/read_stl.hh"

namespace geometry {
namespace spatial {

TEST(OctreeTest, test_octree) {
  const std::string file_path = "/home/jacob/repos/experiments/data/test_stuff.stl";
  const auto tri = *geometry::import::read_stl(file_path);
}
}
}
