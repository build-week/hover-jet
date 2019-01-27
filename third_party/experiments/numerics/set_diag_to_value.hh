#include "eigen.hh"

namespace numerics {

// Edits in place
template <int dim, int row, int mat_size>
void set_diag_to_value(MatNd<mat_size, mat_size>& mat, double value) {
  mat.template block<dim, dim>(row, row) = (MatNd<dim, dim>::Identity() * value);
}

}  // namespace numerics
