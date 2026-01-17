#pragma once

#include <frc/geometry/Pose2d.h>
#include <concepts>
#include <cmath>

// Random utility math functions.

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

inline bool FloatEquals(float a, float b, float epsilon = 1e-6f) {
  return std::abs(a - b) < epsilon;
}

inline bool DoubleEquals(double a, double b, double epsilon = 1e-6) {
  return std::abs(a - b) < epsilon;
}

inline bool Pose2dEquals(const frc::Pose2d& a, const frc::Pose2d& b, units::meter_t positionEpsilon = 0.01_m, units::degree_t rotationEpsilon = 0.01_deg) {
  bool xEquals = DoubleEquals(a.X().value(), b.X().value(), positionEpsilon.value());
  bool yEquals = DoubleEquals(a.Y().value(), b.Y().value(), positionEpsilon.value());
  bool rotationEquals = DoubleEquals(a.Rotation().Degrees().value(), b.Rotation().Degrees().value(), rotationEpsilon.value());
  return xEquals && yEquals && rotationEquals;
}

inline bool Pose2dEquals(const std::optional<frc::Pose2d>& a, const std::optional<frc::Pose2d>& b, units::meter_t positionEpsilon = 0.01_m, units::degree_t rotationEpsilon = 0.01_deg) {
  if (a.has_value() != b.has_value()) {
    return false;
  }
  if (!a.has_value() && !b.has_value()) {
    return true;
  }
  return Pose2dEquals(a.value(), b.value(), positionEpsilon, rotationEpsilon);
}

}  // namespace core
}  // namespace thunder
