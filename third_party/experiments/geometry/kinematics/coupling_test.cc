#include "geometry/kinematics/coupling.hh"

#include "eigen_helpers.hh"
#include "testing/gtest.hh"

namespace geometry {
namespace kinematics {

TEST(Coupling, test_coupling_at_identity) {
  const jcc::Vec3 v(1.0, 0.0, 0.0);
  const jcc::Vec3 w(0.0, 0.0, 0.0);
  const jcc::Vec6 eps = jcc::vstack(v, w);

  const SE3 vehicle_from_sensor = SE3();
  const SE3 world_from_vehicle = SE3();

  const SE3 world_2_from_world_1 = SE3::exp(eps);
  const SE3 world_from_vehicle_2 = world_2_from_world_1 * world_from_vehicle;

  std::cout << world_from_vehicle_2.translation().transpose() << std::endl;

  std::cout << "OP: " << observed_posedot(world_from_vehicle, eps).transpose()
            << std::endl;
}

}  // namespace kinematics
}  // namespace geometry
