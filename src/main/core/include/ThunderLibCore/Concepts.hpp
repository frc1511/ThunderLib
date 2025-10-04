#pragma once

#include <concepts>
#include <exception>

namespace thunder::core {

template <typename T>
concept Arithmetic = requires(T a, T b) {
  { a + b } -> std::convertible_to<T>;
  { a - b } -> std::convertible_to<T>;
  { a * b } -> std::convertible_to<T>;
  { a / b } -> std::convertible_to<T>;
};

template <typename T>
concept StandardException = std::derived_from<T, std::exception>;

}  // namespace thunder::core
