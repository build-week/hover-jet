#pragma once

#include "numerics/cost_function.hh"
#include "numerics/numdiff.hh"

namespace numerics {

template <typename Callable>
CostFunction wrap_numerical_grad(const Callable& fnc) {
  return [fnc](const detail::Vecx& x,
               detail::Vecx* const gradient,
               detail::Matx* const hessian) {
    if (gradient) {
      *gradient = dynamic_numerical_gradient(x, fnc);
    }
    return fnc(x);
  };
}

}  // namespace numerics
