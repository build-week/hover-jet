#pragma once

#include "estimation/filter.hh"
#include "estimation/jet/jet_rk4.hh"
#include "estimation/observation_model.hh"

// TODO
#include "estimation/jet/fiducial_measurement.hh"

namespace estimation {
namespace jet_filter {

using JetEkf = Ekf<State>;
using ImuModel = ObservationModel<State, AccelMeasurement>;

}  // namespace jet_filter
}  // namespace estimation