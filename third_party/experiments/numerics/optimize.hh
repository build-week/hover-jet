#pragma once

#include "eigen.hh"

#include "numerics/cost_function.hh"
#include "numerics/optimization_problem.hh"

#include <functional>
#include <vector>

namespace numerics {

enum class ObjectiveMethod : uint8_t {
  kNewton = 0,
  kGaussNewton = 1,
  kGradientDescent = 2,
};

enum class ConstraintMethod : uint8_t {
  kAugLag = 0,
  kPenalty = 1,
};

// TODO: Return a value indicating success or failure
template <ObjectiveMethod OBJECTIVE_METHOD, ConstraintMethod CONSTRAINT_METHOD>
OptimizationState optimize(const OptimizationState& initialization,
                           const OptimizationProblem& problem);
}  // namespace numerics
