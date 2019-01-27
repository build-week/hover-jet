#pragma once

#include "numerics/numdiff.hh"

#include <functional>

namespace numerics {

template <typename X>
using ApplyDelta = std::function<X(const X&, const VecNd<X::DIM>&)>;

template <typename X>
using ComputeDelta = std::function<VecNd<X::DIM>(const X&, const X&)>;

template <typename X, typename Y>
using GroupFnc = std::function<Y(const X&)>;

template <typename X, int ROWS>
using GroupToDelta = std::function<VecNd<ROWS>(const X&)>;

template <typename X, typename Y>
MatNd<Y::DIM, X::DIM> group_jacobian(const X& x,
                                     const GroupFnc<X, Y>& fnc,
                                     const ComputeDelta<Y>& compute_delta,
                                     const ApplyDelta<X>& apply_delta) {
  const auto f_x = [&apply_delta, &compute_delta, &fnc, &x](const VecNd<X::DIM>& dx) {
    const X xplus_dx = apply_delta(x, dx);
    return compute_delta(fnc(xplus_dx), fnc(x));
  };
  return numerical_jacobian<Y::DIM>(VecNd<X::DIM>::Zero().eval(), f_x);
}

template <typename X, typename Y>
MatNd<Y::DIM, X::DIM> group_jacobian(const X& x, const GroupFnc<X, Y>& fnc) {
  const auto f_x = [&fnc, &x](const VecNd<X::DIM>& dx) {
    const X xplus_dx = X::apply_delta(x, dx);
    return Y::compute_delta(fnc(xplus_dx), fnc(x));
  };
  return numerical_jacobian<Y::DIM>(VecNd<X::DIM>::Zero().eval(), f_x);
}

template <int N_ROWS, typename X>
MatNd<N_ROWS, X::DIM> group_jacobian(const X& x, const GroupToDelta<X, N_ROWS>& fnc) {
  const auto f_x = [&fnc, &x](const VecNd<X::DIM>& dx) -> VecNd<N_ROWS> {
    const X xplus_dx = X::apply_delta(x, dx);
    return fnc(xplus_dx) - fnc(x);
  };
  return numerical_jacobian<N_ROWS>(VecNd<X::DIM>::Zero().eval(), f_x);
}

template <typename X>
Eigen::MatrixXd dynamic_group_jacobian(const X& x,
                                       const GroupToDelta<X, Eigen::Dynamic>& fnc) {
  const auto f_x = [&fnc, &x](const VecNd<X::DIM>& dx) -> VecXd {
    const X xplus_dx = X::apply_delta(x, dx);
    return fnc(xplus_dx) - fnc(x);
  };
  return dynamic_numerical_jacobian(VecNd<X::DIM>::Zero().eval(), f_x);
}

}  // namespace numerics