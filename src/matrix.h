#pragma once

#include <array>

template <typename T, unsigned Rows, unsigned Cols>
class Matrix {
 public:
  Matrix();
  Matrix(const std::array<T, Rows * Cols>& data);

  static Matrix<T, Rows, Cols> identity()
    requires(Rows == Cols);
  static Matrix<T, Rows, Cols> rotate(float degrees)
    requires(Rows == 2 && Cols == 2);

  T& operator()(unsigned row, unsigned col);
  const T& operator()(unsigned row, unsigned col) const;

  Matrix<T, Rows, Cols> operator+(const Matrix<T, Rows, Cols>& other) const;

  template <unsigned OtherCols>
  Matrix<T, Rows, OtherCols> operator*(const Matrix<T, Cols, OtherCols>& other) const;

  std::array<T, Rows * Cols> data;
};

#include "matrix.hpp"
