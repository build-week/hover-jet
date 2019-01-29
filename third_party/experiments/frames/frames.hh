#pragma once

#include "sophus.hh"

// TODO:
//  - Add cmake support for adding frames

namespace frames {
enum class FrameId {
  CAMERA,   //
  BODY,     //
  VEHICLE,  //
  PILOT,    //
  LOCAL,    //
  WORLD     //
};

// A templated frame class, enforcing transforms via the type system
// The transform encodes a transform from source to destination
//
template <FrameId destination, FrameId source, typename Group = SE3>
class Transform {
 public:
  Transform(const Group& g) : destination_from_source_(g) {
  }

  template <FrameId other_source>
  Transform<destination, other_source, Group> operator*(const Transform<source, other_source, Group>& other) const {
    return destination_from_source_ * other.destination_from_source();
  }

  Transform<source, destination, Group> inverse() const {
    return {destination_from_source_.inverse()};
  }

  const Group& destination_from_source() const {
    return destination_from_source_;
  }

 private:
  Group destination_from_source_;
};
}