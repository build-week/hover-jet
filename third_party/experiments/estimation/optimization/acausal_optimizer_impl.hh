#pragma once

#include "estimation/optimization/acausal_optimizer.hh"
#include "numerics/group_diff.hh"

#include <limits>

namespace estimation {
namespace optimization {

template <typename Prob>
VecXd AcausalOptimizer<Prob>::add_observation_residual(const State& x,
                                                       const Measurement& z,
                                                       const Parameters& p,
                                                       int x_ind,
                                                       int residual_ind,
                                                       int param_ind,
                                                       Out<BlockSparseMatrix> bsm) const {
  // y = (z[t] - h(x[t]; p))
  // J[obs_ind,  state_ind] = dy/dx[t] = H
  // J[obs_ind, params_ind] = dy/dp    = C

  const auto& model = models_.at(z.type);
  const auto y_of_x = [&model, &z, &p](const State& x) {
    // Hold p
    return model.error(x, z.observation, p);
  };
  const auto y_of_p = [&model, &z, &x](const Parameters& p) {
    // Hold x
    return model.error(x, z.observation, p);
  };

  const MatXd dy_dx = -numerics::dynamic_group_jacobian<State>(x, y_of_x);
  const MatXd dy_dp = -numerics::dynamic_group_jacobian<Parameters>(p, y_of_p);
  bsm->set(residual_ind, x_ind, dy_dx);
  bsm->set(residual_ind, param_ind, dy_dp);

  return model.error(x, z.observation, p);
}

template <typename Prob>
VecXd AcausalOptimizer<Prob>::add_dynamics_residual(const State& x_0,
                                                    const State& x_1,
                                                    const Parameters& p,
                                                    double dt,
                                                    int x_ind,
                                                    int residual_ind,
                                                    int param_ind,
                                                    Out<BlockSparseMatrix> bsm) const {
  // y = (x[t+1] - f(x[t]; p)
  // J[obs_ind,          t] = dy/dx[t]   = A
  // J[obs_ind, params_ind] = dy/dp      = G
  // J[obs_ind,        t+1] = dy/dx[t+1] = -I

  const auto& model = dynamics_;
  const auto y_of_x = [dt, &model, &x_1, &p](const State& x) {
    // Hold p, x_1
    return State::compute_delta(x_1, model(x, p, dt));
  };

  const auto y_of_p = [dt, &model, &x_0, &x_1](const Parameters& p) {
    // Hold x_0, x_1
    return State::compute_delta(x_1, model(x_0, p, dt));
  };

  const MatXd dy_dx = -numerics::dynamic_group_jacobian<State>(x_0, y_of_x);
  const MatXd dy_dp = -numerics::dynamic_group_jacobian<Parameters>(p, y_of_p);

  using StateToStateJac = MatNd<State::DIM, State::DIM>;
  const StateToStateJac dy_d_x1 = -StateToStateJac::Identity();
  // const StateToStateJac dy_d_x1 = StateToStateJac::Zero();

  bsm->set(residual_ind, x_ind, dy_dx);
  bsm->set(residual_ind, x_ind + 1, dy_d_x1);
  bsm->set(residual_ind, param_ind, dy_dp);

  return State::compute_delta(x_1, model(x_0, p, dt));
}

template <typename Prob>
LinearSystem AcausalOptimizer<Prob>::populate(const Solution& soln) const {
  assert(soln.x.size() == heap_.size());
  LinearSystem system;

  constexpr int n_params = 1;
  const int n_measurements = static_cast<int>(heap_.size());
  const int n_states = static_cast<int>(soln.x.size());
  const int n_residuals = (n_states - 1) + n_measurements;

  BlockSparseMatrix J(n_residuals, n_states + n_params);
  BlockSparseMatrix R_inv(n_residuals, n_residuals);

  std::vector<VecXd> v;
  v.resize(n_residuals);

  // Immediately after the states
  const int p_ind = n_states;

  // covariances_[model_id]
  const auto& p = soln.p;
  const auto& measurements = heap_.to_sorted_vector();
  for (int t = 0; t < static_cast<int>(measurements.size()); ++t) {
    const auto& z_t = measurements.at(t);

    // TODO: Do something special if dt == 0
    // Options:
    // -> Share a measurement on a state
    // -> [Infinitely] weight state-matching observation
    // The two options are equivalent, but the first is well-conditioned

    // TODO: Divide information by dt
    const int x_ind = t;
    const int z_ind = 2 * t;
    const State& x_t = soln.x.at(t);
    const VecXd y_obs =
        add_observation_residual(x_t, z_t, p, x_ind, z_ind, p_ind, out(J));
    v[z_ind] = y_obs;

    const MatXd& cov = covariances_.at(z_t.type);
    R_inv.set(z_ind, z_ind, cov.inverse());

    if (t < static_cast<int>(soln.x.size()) - 1) {
      const double dt =
          to_seconds(measurements.at(t + 1).time_of_validity - z_t.time_of_validity);

      assert(dt > 0.0);
      // assert(dt < 0.3);  // Arbitrary constant that is surprising enough to indicate a
      // bug

      const State& x_t1 = soln.x.at(t + 1);

      const int z_obs_ind = z_ind + 1;
      const VecXd y_dyn =
          add_dynamics_residual(x_t, x_t1, p, dt, x_ind, z_obs_ind, p_ind, out(J));
      v[z_obs_ind] = y_dyn;

      R_inv.set(z_obs_ind, z_obs_ind, (dyn_cov_ * dt).inverse());
    }
  }

  system.v = v;
  system.J = J;
  system.R_inv = R_inv;

  return system;
}

template <typename Prob>
typename AcausalOptimizer<Prob>::Solution AcausalOptimizer<Prob>::update_solution(
    const Solution& prev_soln, const VecXd& delta) const {
  constexpr int n_params = 1;
  const int n_states = static_cast<int>(prev_soln.x.size());
  Solution updated_soln;
  updated_soln.x.resize(n_states);

  assert(delta.rows() == (n_states * State::DIM) + (n_params * Parameters::DIM));
  for (int t = 0; t < static_cast<int>(prev_soln.x.size()); ++t) {
    const VecNd<State::DIM> sub_delta = delta.segment(t * State::DIM, State::DIM);
    updated_soln.x[t] = State::apply_delta(prev_soln.x.at(t), sub_delta);
  }
  const VecNd<Parameters::DIM> p_delta =
      delta.segment(n_states * State::DIM, Parameters::DIM);

  updated_soln.p = Parameters::apply_delta(prev_soln.p, p_delta);
  return updated_soln;
}

template <typename Prob>
double AcausalOptimizer<Prob>::cost(const LinearSystem& system) const {
  double cost = 0.0;
  for (int k = 0; k < static_cast<int>(system.v.size()); ++k) {
    const VecXd& sub_v = system.v.at(k);

    cost += sub_v.dot(system.R_inv.get(k, k) * sub_v);
  }

  assert(cost > 0.0);

  return cost;
}

template <typename Prob>
typename AcausalOptimizer<Prob>::Solution AcausalOptimizer<Prob>::solve(
    const Solution& initialization, const Visitor& visitor) const {
  assert(initialization.x.size() == heap_.size());
  Solution soln = initialization;

  // Generate our jacobian

  constexpr double LAMBDA_UP_FACTOR = 10.0;
  constexpr double LAMBDA_DOWN_FACTOR = 0.5;
  constexpr double LAMBDA_DOWN_LITE_FACTOR = 0.75;
  constexpr double LAMBDA_INITIAL = 5.0;
  constexpr double DECREASE_RATIO_FOR_DAMPING_DOWN = 0.7;
  constexpr double MAX_LAMBDA = 1e3;
  constexpr double MIN_LAMBDA = 1e-25;

  LinearSystem current_system = populate(soln);
  double prev_cost = cost(current_system);
  double lambda = LAMBDA_INITIAL;

  for (int k = 0; k < 1000; ++k) {
    if (visitor) {
      visitor(soln);
    }
    if (lambda > MAX_LAMBDA) {
      std::cout << "\tConverging (Lambda)" << std::endl;
      break;
    }

    std::cout << "\n-------" << std::endl;
    std::vector<VecXd> v;
    current_system = populate(soln);

    std::cout << "Cost at [" << k << "]: " << cost(current_system) << std::endl;

    const VecXd delta =
        current_system.J.solve_lst_sq(current_system.v, current_system.R_inv, lambda);
    const auto new_soln = update_solution(soln, delta);

    const auto new_system = populate(new_soln);
    const double new_cost = cost(new_system);

    std::cout << "\tPossible cost: " << new_cost << std::endl;
    if (new_cost > prev_cost) {
      lambda *= LAMBDA_UP_FACTOR;
      std::cout << "\tIncreasing damping to " << lambda << std::endl;
      continue;
    } else if (new_cost < DECREASE_RATIO_FOR_DAMPING_DOWN * prev_cost) {
      lambda *= LAMBDA_DOWN_FACTOR;
      lambda = std::max(lambda, MIN_LAMBDA);
      std::cout << "\tDecreasing damping to " << lambda << std::endl;
    } else {
      lambda *= LAMBDA_DOWN_LITE_FACTOR;
      lambda = std::max(lambda, MIN_LAMBDA);
      std::cout << "\tDecreasing damping to " << lambda << std::endl;
    }

    prev_cost = new_cost;
    current_system = new_system;
    soln = new_soln;
  }
  {
    std::vector<VecXd> v_final;
    const LinearSystem system_final = populate(soln);
    std::cout << "Cost Final: " << cost(system_final) << std::endl;
    if (visitor) {
      visitor(soln);
    }
  }

  return soln;
}

}  // namespace optimization
}  // namespace estimation
