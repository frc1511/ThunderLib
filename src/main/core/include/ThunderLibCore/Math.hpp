#pragma once

#include <concepts>
#include <cmath>

namespace thunder {
namespace core {

template <typename T>
concept Lerpable = requires(T a, T b) {
  { a + b } -> std::convertible_to<T>;
  { a - b } -> std::convertible_to<T>;
  { a * 1.0 } -> std::convertible_to<T>;
};

// std::lerp doesn't work with all types, this one does.
template <Lerpable T>
constexpr T Lerp(T lower, T upper, double t) {
  return lower + (upper - lower) * t;
}

constexpr bool FloatEquals(float a, float b, float epsilon = 1e-6f) {
  return std::abs(a - b) < epsilon;
}

constexpr bool DoubleEquals(double a, double b, double epsilon = 1e-6) {
  return std::abs(a - b) < epsilon;
}

}  // namespace core
}  // namespace thunder
