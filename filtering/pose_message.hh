#pragma once

#include "third_party/experiments/eigen.hh"
#include "third_party/experiments/sophus.hh"

#include "infrastructure/comms/schemas/message.hh"
#include "infrastructure/comms/serialization/serialization_macros.hh"
#include "infrastructure/time/timestamp.hh"

namespace jet {

struct Pose {
  SE3 world_from_jet;
  jcc::Vec3 v_world_frame;
  jcc::Vec3 w_world_frame;
  Timestamp timestamp;
};

// Intentionally uninitialized, for detectability via valgrind
struct PoseMessage : Message {
  // TODO(jake): Implement a serializable matrix type
  std::array<double, 6> log_world_from_jet;
  std::array<double, 3> v_world_frame;
  std::array<double, 3> w_world_frame;

  // Timestamp that the observation was *generated*
  Timestamp timestamp;

  MESSAGE(PoseMessage, log_world_from_jet, v_world_frame, w_world_frame, timestamp);

  Pose to_pose() const;
  static PoseMessage from_pose(const Pose& pose);
};

}  // namespace jet
