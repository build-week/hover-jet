#include "eigen.hh"

#include "fluids/navier_stokes_2d.hh"

#include "fluids/fields_2d.hh"
// Rendering
#include "viewer/primitives/image.hh"
#include "viewer/primitives/simple_geometry.hh"
#include "viewer/window_3d.hh"

namespace jcc {
namespace fluids {

plane::SimulationState enforce_boundary_conditions(const plane::SimulationState& state,
                                                   const plane::SimulationConfig& sim_config) {
  auto new_state = state;
  // new_state.pressure_field

  // Top/Bottom boundaries
  // new_state.velocity_field[0]. =
  return new_state;
}

void simulate() {
  constexpr int SIZE = 200;
  plane::SimulationState state;
  state.pressure_field = Eigen::MatrixXd::Zero(SIZE, SIZE);
  // state.pressure_field.block(40, 40, 20, 20).array() = 0.001;

  state.velocity_field[0] = Eigen::MatrixXd::Zero(SIZE, SIZE);
  state.velocity_field[1] = Eigen::MatrixXd::Zero(SIZE, SIZE);

  auto viewer = viewer::get_window3d("America's Favorite Visualization Tool");
  constexpr double SCALE = 100.0;
  auto image = std::make_shared<viewer::Image>(state.pressure_field, SCALE);
  viewer->add_primitive(image);

  const plane::SimulationConfig cfg;
  for (int k = 0; k < 1000; ++k) {
    std::cout << "k: " << k << std::endl;
    std::cout << "velocity_field-x " << state.velocity_field[0].minCoeff() << " , "
              << state.velocity_field[0].maxCoeff() << std::endl;
    std::cout << "velocity_field-y " << state.velocity_field[1].minCoeff() << " , "
              << state.velocity_field[1].maxCoeff() << std::endl;
    std::cout << "pressure_field " << state.pressure_field.minCoeff() << ", " << state.pressure_field.maxCoeff()
              << std::endl;

    Eigen::MatrixXd force_x = Eigen::MatrixXd::Zero(SIZE, SIZE);
    if (k < 3) {
      force_x.block(3, 20, 100, 80).array() = 0.1;
    }

    // image->update_image(state.pressure_field);
    // image->update_image((state.pressure_field / state.pressure_field.maxCoeff()));
    state.pressure_field = (state.pressure_field.array() > 0.0).cast<double>() * state.pressure_field.array();
    // image->update_image(state.pressure_field / std::max(1e-8, state.pressure_field.maxCoeff()));
    image->update_image(state.pressure_field / 1e-8);

    const auto advection = plane::compute_advection(state.velocity_field, cfg);
    const auto pressure = plane::compute_pressure(state.pressure_field, cfg);
    const auto diffusion = plane::compute_diffusion(state.velocity_field, cfg);

    auto divergent_dvel = plane::add(plane::add(advection, pressure), diffusion);
    divergent_dvel[0] += force_x;

    const auto divergent_vel = plane::add(plane::mul(divergent_dvel, cfg.dt), state.velocity_field);

    state.velocity_field = divergent_vel;

    state = plane::compute_projection(state, cfg);
    state = enforce_boundary_conditions(state, cfg);
    viewer->spin_until_step();
  }
}

}  // namespace fluids.
}  // namespace jcc.

int main() {
  jcc::fluids::simulate();
}