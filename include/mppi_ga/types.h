#ifndef PARAMPC_UTILS_H
#define PARAMPC_UTILS_H

#include <Eigen/Core>

namespace mppi_ga
{

using Eigen::Index;
using Float = float;

using Mat = Eigen::Matrix<Float, Eigen::Dynamic, Eigen::Dynamic, Eigen::StorageOptions::RowMajor>;
using MatBlock = Eigen::Block<Mat>;
using Vec = Eigen::Matrix<Float, Eigen::Dynamic, 1>;
using VecRow = Eigen::Matrix<Float, 1, Eigen::Dynamic>;
using VecBlock = Eigen::Block<Vec, Eigen::Dynamic, 1>;
using VecBlockConst = Eigen::Block<const Vec, Eigen::Dynamic, 1>;

inline void resizeNullify(Mat &M, size_t rows, size_t cols)
{
  M.resize(rows, cols);
  M.setZero();
}
inline void resizeNullify(Vec &v, size_t rows)
{
  v.resize(rows);
  v.setZero();
}

// when working with another library
template <class Matrix, class Derived>
void toEigenM(const Matrix &M, Eigen::MatrixBase<Derived> &Me)
{
  for(size_t row = 0; row < Me.rows(); ++row)
  {
    for(size_t col = 0; col < Me.cols(); ++col)
      Me(row,col) = M[row][col];
  }
}

template <class Vector, class Derived>
void toEigenV(const Vector &V, Eigen::MatrixBase<Derived> &Ve)
{
  Ve.resize(V.size());
  for(size_t row = 0; row < Ve.rows(); ++row)
    Ve(row) = V[row];
}

template <class Matrix>
void fromEigen(const Mat &Me, Matrix &M)
{
  for(size_t row = 0; row < Me.rows(); ++row)
  {
    for(size_t col = 0; col < Me.cols(); ++col)
       M[row][col] = Me(row,col);
  }
}

template <class Vector>
void fromEigen(const Vec &Ve, Vector &V)
{
  for(size_t row = 0; row < Ve.rows(); ++row)
    V[row] = Ve(row);
}



}

#endif // PARAMPC_UTILS_H
