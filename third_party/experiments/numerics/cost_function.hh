#pragma once

#include "eigen.hh"

namespace numerics {

namespace detail {
using Vecx = Eigen::VectorXd;
using Matx = Eigen::MatrixXd;
}  // namespace detail

using CostFunction =                                    //
    std::function<double(const detail::Vecx& x,         // The "state"
                         detail::Vecx* const gradient,  // Optional
                         detail::Matx* const hessian    // Optional
                         )>;
//
}  // namespace numerics