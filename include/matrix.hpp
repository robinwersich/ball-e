#include <cmath>

#include "matrix.h"

template <typename T, unsigned Rows, unsigned Cols>
Matrix<T, Rows, Cols>::Matrix() : data{} {}

template <typename T, unsigned Rows, unsigned Cols>
Matrix<T, Rows, Cols>::Matrix(const std::array<T, Rows * Cols>& data) : data{data} {}

template <typename T, unsigned Rows, unsigned Cols>
Matrix<T, Rows, Cols> Matrix<T, Rows, Cols>::identity()
  requires(Rows == Cols)
{
  Matrix<T, Rows, Cols> matrix;
  for (size_t i = 0; i < Rows; ++i) {
    for (size_t j = 0; j < Cols; ++j) { matrix(i, j) = i == j ? 1 : 0; }
  }
  return matrix;
}

template <typename T, unsigned Rows, unsigned Cols>
Matrix<T, Rows, Cols> Matrix<T, Rows, Cols>::rotate(float degrees)
  requires(Rows == 2 && Cols == 2)
{
  double radians = degrees * M_PI / 180;
  return {std::cos(radians), -std::sin(radians), std::sin(radians), std::cos(radians)};
}

template <typename T, unsigned Rows, unsigned Cols>
T& Matrix<T, Rows, Cols>::operator()(unsigned row, unsigned col) {
  return data[row * Cols + col];
}

template <typename T, unsigned Rows, unsigned Cols>
const T& Matrix<T, Rows, Cols>::operator()(unsigned row, unsigned col) const {
  return data[row * Cols + col];
}

template <typename T, unsigned Rows, unsigned Cols>
Matrix<T, Rows, Cols> Matrix<T, Rows, Cols>::operator+(const Matrix<T, Rows, Cols>& other) const {
  Matrix<T, Rows, Cols> result;
  std::transform(data.begin(), data.end(), other.data.begin(), result.data.begin(), std::plus<T>());
  return result;
}

template <typename T, unsigned Rows, unsigned Cols>
template <unsigned OtherCols>
Matrix<T, Rows, OtherCols> Matrix<T, Rows, Cols>::operator*(const Matrix<T, Cols, OtherCols>& other
) const {
  Matrix<T, Rows, OtherCols> result;
  for (unsigned i = 0; i < Rows; ++i) {
    for (unsigned j = 0; j < OtherCols; ++j) {
      for (unsigned k = 0; k < Cols; ++k) { result(i, j) += (*this)(i, k) * other(k, j); }
    }
  }
  return result;
}
