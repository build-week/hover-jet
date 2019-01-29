#include "planning/jet/jet_planner.hh"

#include "planning/differentiation.hh"
#include "planning/generic_planner.hh"
#include "planning/jet/jet_xlqr.hh"

namespace planning {
namespace jet {

double huber(double x, double k) {
  constexpr double half = 0.5;
  const double abs_x = std::abs(x);
  if (abs_x < k) {
    return (x * x) * half;
  } else {
    return k * (abs_x - (k * half));
  }
}

const double square(double x) {
  return x * x;
}

double quad_hinge(double x, double k) {
  return square(std::max(x - k, 0.0));
}

auto make_jet_cost(const Desires& desires) {
  return [desires](const State& state, const VecNd<U_DIM>& u, int t) {
    const auto control = from_vector(u);
    double cost = 0.0;

    {
      cost += 9.0 * control.q.squaredNorm();
      cost += 0.03 * control.throttle_dot * control.throttle_dot;
    }
    {
      const jcc::Vec3 attitude = state.R_world_from_body * jcc::Vec3::UnitZ();
      const double sin_error = attitude.cross(jcc::Vec3::UnitZ()).squaredNorm();
      cost += 14.0 * sin_error;

      if (t >= HORIZON - 1) {
        const jcc::Vec3 error = state.x - desires.target;

        cost += 100.0 * huber(error.norm(), 5.0);
        cost += 200.0 * huber(error.z(), 5.0);

        cost += 150.0 * state.v.squaredNorm();
      }

      const jcc::Vec3 velocity_weights(25.0, 25.0, 25.0);
      cost += state.v.dot(velocity_weights.asDiagonal() * state.v);
      cost += desires.supp_v_weight * state.v.squaredNorm();

      cost += 20.0 * state.w.squaredNorm();
      // const jcc::Vec3 target_w(0.01, 0.0, 0.05);
      // cost += 35.0 * (target_w - state.w).squaredNorm();

      cost += 25.0 * quad_hinge(state.throttle_pct, 8.0);
      cost += 100.0 * quad_hinge(-state.throttle_pct, 0.0);
    }
    return cost;
  };
}

Parameters get_parameters() {
  Parameters params;
  params.mass = 100.0;
  params.unit_z = jcc::Vec3::UnitZ();
  params.external_force = -jcc::Vec3::UnitZ() * 5.0;
  return params;
}

State dynamics(const State& state, const VecNd<U_DIM>& u, const double dt) {
  const Parameters params = get_parameters();
  return rk4_integrate(state, from_vector(u), params, dt);
}

auto make_generate_derivatives(const Problem<JetDim, State>& problem) {
  return [problem](const State& state,
                   const VecNd<U_DIM>& u,
                   int t) -> GenericDerivatives<JetDim> {
    const Differentiator diff(problem);

    GenericDerivatives<JetDim> derivatives;
    derivatives.A = diff.state_jacobian(state, u);
    derivatives.B = diff.control_jacobian(state, u);

    derivatives.Q = diff.state_hessian(state, u, t);
    derivatives.R = diff.control_hessian(state, u, t);

    derivatives.g_x = diff.state_gradient(state, u, t);
    derivatives.g_u = diff.control_gradient(state, u, t);

    return derivatives;
  };
}

std::vector<StateControl> plan(const State& x0,
                               const Desires& desires,
                               const std::vector<Controls>& initialization) {
  const JetProblem prob(
      make_jet_cost(desires), dynamics, compute_delta, apply_delta, HORIZON, DT);
  const JetXlqr planner(prob, make_generate_derivatives(prob));

  const auto pre = planner.solve(x0);

  std::vector<StateControl> result;
  for (int k = 0; k < HORIZON; ++k) {
    result.push_back({pre.x[k], from_vector(pre.u[k])});
  }

  return result;
}

}  // namespace jet
}  // namespace planning