#include "geometry/kinematics/coupling.hh"

namespace geometry {
namespace kinematics {

jcc::Vec6 observed_posedot(const SE3& mover_from_observer, const jcc::Vec6& posedot) {
  return mover_from_observer.Adj() * posedot;
}

}  // namespace kinematics
}  // namespace geometry
