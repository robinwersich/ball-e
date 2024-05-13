#pragma once

#include <array>

template <typename T, unsigned Rows, unsigned Cols>
class Matrix {
 public:
  Matrix() : data{} {}
  Matrix(const std::array<T, Rows * Cols>& data) : data{data} {}
  Matrix(std::initializer_list<T> list) {
    std::copy(list.begin(), list.end(), data.begin());
  }

  static Matrix<T, Rows, Cols> identity()
    requires(Rows == Cols);
  static Matrix<T, Rows, Cols> rotate(float degrees)
    requires(Rows == 2 && Cols == 2);

  T& operator()(unsigned row, unsigned col) {return data[row * Cols + col];}
  const T& operator()(unsigned row, unsigned col) const {return data[row * Cols + col];}

  T& operator[](unsigned index) { return data[index]; }
  const T& operator[](unsigned index) const { data[index]; }

  T& x() requires(Cols == 1 && Rows >= 1) { return data[0]; }
  T& y() requires(Cols == 1 && Rows >= 2) { return data[1]; }
  T& z() requires(Cols == 1 && Rows >= 3) { return data[2]; }
  const T& x() const requires(Cols == 1 && Rows >= 1) { return data[0]; }
  const T& y() const requires(Cols == 1 && Rows >= 2) { return data[1]; }
  const T& z() const requires(Cols == 1 && Rows >= 3) { return data[2]; }

  Matrix<T, Rows, Cols> operator+(const Matrix<T, Rows, Cols>& other) const;

  Matrix<T, Rows, Cols> operator*(T scalar) const;

  template <unsigned OtherCols>
  Matrix<T, Rows, OtherCols> operator*(const Matrix<T, Cols, OtherCols>& other) const;

  std::array<T, Rows * Cols> data;
};

template <typename T>
using Vector2D = Matrix<T, 2, 1>;
template <typename T>
using Vector3D = Matrix<T, 3, 1>;

#include "matrix.hpp"
