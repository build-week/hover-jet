#include "estimation/optimization/block_sparse_matrix.hh"

#include "out.hh"

// TODO
#include <iostream>

namespace estimation {
namespace optimization {

namespace {
/*
VecXd to_dynamic(const std::vector<VecXd> v) {
int numel = 0;
for (const auto& sub_v : v) {
  numel += sub_v.rows();
}

VecXd vec(numel);
int elements_so_far = 0;
for (const auto& sub_v : v) {
  for (int sub_row = 0; sub_row < sub_v.rows(); ++sub_row) {
    // triplets.emplace_back(elements_so_far + sub_row, sub_v[sub_row]);
    vec[elements_so_far + sub_row] = sub_v[sub_row];
  }
  elements_so_far += sub_v.rows();
}
return vec;
}
*/

void insert_into_triplets(const Eigen::MatrixXd& mat,
                          int offset_row,
                          int offset_col,
                          Out<std::vector<Eigen::Triplet<double>>> triplets) {
  // triplets.reserve(mat.rows() * mat.cols());

  for (int row = 0; row < mat.rows(); ++row) {
    for (int col = 0; col < mat.cols(); ++col) {
      const double& val = mat(row, col);
      if (std::abs(val) > 1e-12) {
        triplets->emplace_back(row + offset_row, col + offset_col, val);
      }
    }
  }
}

Eigen::SparseVector<double> to_sparse(const std::vector<VecXd> v) {
  int numel = 0;
  for (const auto& sub_v : v) {
    numel += sub_v.rows();
  }

  int count_so_far = 0;
  Eigen::SparseVector<double> sp_vec(numel);
  for (const auto& sub_v : v) {
    for (int sub_row = 0; sub_row < sub_v.rows(); ++sub_row) {
      sp_vec.coeffRef(count_so_far + sub_row) = sub_v[sub_row];
    }
    count_so_far += sub_v.rows();
  }

  return sp_vec;
}

}  // namespace

BlockSparseMatrix::BlockSparseMatrix(const int n_block_rows, const int n_block_cols) {
  rows_.resize(n_block_rows);
  n_cols_.resize(n_block_cols, -1);
  valid_ = true;
}

void BlockSparseMatrix::set(int row, int col, const Eigen::MatrixXd& mat) {
  assert(valid_);
  //
  // Assert in col/row bounds
  //

  assert(row >= 0);
  assert(col >= 0);
  assert(row < static_cast<int>(rows_.size()));
  assert(col < static_cast<int>(n_cols_.size()));

  //
  // Update (or assert) row
  //

  if (rows_[row].cols.empty()) {
    rows_[row].n_rows = mat.rows();
  } else {
    assert(mat.rows() == rows_[row].n_rows);
  }

  //
  // Update (or assert) n_cols
  //

  if (n_cols_[col] == -1) {
    n_cols_[col] = mat.cols();
  } else {
    assert(n_cols_[col] == mat.cols());
  }

  //
  // Actually update
  //

  rows_[row].cols[col] = {mat};
}

const Eigen::MatrixXd& BlockSparseMatrix::get(int row, int col) const {
  assert(valid_);
  //
  // Enforce row, col bounds
  //
  assert(row >= 0);
  assert(col >= 0);
  assert(row < static_cast<int>(rows_.size()));
  assert(col < static_cast<int>(n_cols_.size()));

  // This is (implicitly) zero, but you probably didn't mean to do this
  const auto& block_row = rows_.at(row);
  assert(block_row.cols.count(col) != 0);
  return block_row.cols.at(col).block;
}

int BlockSparseMatrix::real_rows() const {
  assert(valid_);
  int total = 0;
  for (const auto& row : rows_) {
    assert(row.n_rows != -1);
    total += row.n_rows;
  }

  return total;
}

int BlockSparseMatrix::real_cols() const {
  assert(valid_);
  int total = 0;
  for (const auto& n : n_cols_) {
    total += n;
  }
  return total;
}

int BlockSparseMatrix::real_rows_above_block(int block_row) const {
  assert(valid_);
  assert(block_row >= 0);
  assert(block_row < static_cast<int>(rows_.size()));

  int total = 0;
  for (int i = 0; i < block_row; ++i) {
    const auto& row = rows_[i];
    assert(row.n_rows != -1);
    total += row.n_rows;
  }

  return total;
}

int BlockSparseMatrix::real_cols_left_of_block(int block_col) const {
  assert(valid_);
  assert(block_col >= 0);
  assert(block_col < static_cast<int>(n_cols_.size()));

  int total = 0;
  for (int i = 0; i < block_col; ++i) {
    total += n_cols_[i];
  }
  return total;
}

BlockSparseMatrix::SpMat BlockSparseMatrix::to_eigen_sparse() const {
  assert(valid_);
  const int rows = real_rows();
  const int cols = real_cols();
  SpMat mat(rows, cols);

  std::vector<Eigen::Triplet<double>> triplets;

  // (This is knowable)
  // Eigen::VectorXi nnz_cols(cols);

  const int num_rows = static_cast<int>(rows_.size());
  for (int block_row_ind = 0; block_row_ind < num_rows; ++block_row_ind) {
    const int true_row_num = real_rows_above_block(block_row_ind);
    const auto& row = rows_.at(block_row_ind);

    for (const auto& col_block : row.cols) {
      const int block_col_ind = col_block.first;
      const auto& col = col_block.second;
      const int true_col_num = real_cols_left_of_block(block_col_ind);

      const MatXd& submat = col.block;
      insert_into_triplets(submat, true_row_num, true_col_num, out(triplets));
    }
  }

  mat.setFromTriplets(triplets.begin(), triplets.end());

  return mat;
}

VecXd BlockSparseMatrix::solve_lst_sq(const std::vector<VecXd>& residuals,
                                      const BlockSparseMatrix& R_inv,
                                      const double lambda) const {
  assert(valid_);
  const SpMat J = to_eigen_sparse();

  SpMat identity(J.cols(), J.cols());
  identity.setIdentity();

  const SpMat JtRi = J.transpose() * R_inv.to_eigen_sparse();

  // const SpMat JtJ = (J.transpose() * J) + (lambda * identity);

  const SpMat JtRiJ = (JtRi * J);
  // Levenberg
  // const SpMat damped_JtRiJ = (JtRi * J) + (lambda * identity);

  // Levenberg-Marquardt
  const SpMat damped_JtRiJ = JtRiJ + (lambda * SpMat(JtRiJ.diagonal().asDiagonal()));

  const Eigen::SimplicialCholesky<SpMat> chol(damped_JtRiJ);

  const SpVec v = to_sparse(residuals);

  // Jtv
  // const SpMat Jtv = J.transpose() * v;
  // const VecXd delta = chol.solve(Jtv);

  const SpMat JtRiv = JtRi * v;
  const VecXd delta = chol.solve(JtRiv);

  return delta;
}

}  // namespace optimization
}  // namespace estimation