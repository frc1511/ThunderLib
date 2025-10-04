#pragma once

#include <frc/geometry/Rotation2d.h>
#include <units/angle.h>
#include <cassert>

namespace thunder {
namespace types {

/**
 * Represents an angle in the range of (-π, π].
 * This class is similar to frc::Rotation2d, but is designed to represent angles
 * in a more mathematically consistent way, avoiding issues with wrapping and
 * discontinuities.
 */
class CanonicalAngle {
  double m_cos = 1.0;
  double m_sin = 0.0;

 public:
  /**
   * Constructs a CanonicalAngle with a default value of 0 radians.
   */
  CanonicalAngle() = default;

  /**
   * Constructs a CanonicalAngle from a Rotation2d.
   * @param rotation The Rotation2d to convert to a CanonicalAngle.
   */
  /*implicit*/ CanonicalAngle(const frc::Rotation2d& rotation)
      : m_cos(rotation.Cos()), m_sin(rotation.Sin()) {}

  /**
   * Constructs a CanonicalAngle with the specified angle value.
   * @param value The angle value.
   */
  /*implicit*/ CanonicalAngle(units::angle_unit auto value)  // NOLINT
      : m_cos{gcem::cos(value.template convert<units::radian>().value())},
        m_sin{gcem::sin(value.template convert<units::radian>().value())} {}

  /**
   * Constructs a CanonicalAngle with the specified x and y components.
   * @param x The x component or cosine of the angle.
   * @param y The y component or sine of the angle.
   */
  CanonicalAngle(double x, double y) {
    double magnitude = gcem::hypot(x, y);
    assert((magnitude > 1e-6) && "Cannot create CanonicalAngle with zero magnitude");
    m_cos = x / magnitude;
    m_sin = y / magnitude;
  }

  double cos() const { return m_cos; }
  double sin() const { return m_sin; }
  double tan() const { return m_sin / m_cos; }

  units::radian_t radians() const { return units::radian_t{gcem::atan2(m_sin, m_cos)}; }

  units::degree_t degrees() const { return radians().convert<units::degree>(); }

  /*implicit*/ operator frc::Rotation2d() const { return frc::Rotation2d{m_cos, m_sin}; }

  /*implicit*/ operator units::angle_unit auto() const { return radians(); }

  bool operator==(const CanonicalAngle& other) const {
    return gcem::hypot(cos() - other.cos(), sin() - other.sin()) < 1E-9;
  }

  bool operator!=(const CanonicalAngle& other) const { return !(*this == other); }

  /**
   * Takes the inverse (negation) of the current angle.
   * @return The inverse of the current angle.
   */
  CanonicalAngle operator-() const { return CanonicalAngle{-radians()}; }

  CanonicalAngle operator+(const CanonicalAngle& other) const { return rotateBy(other); }

  CanonicalAngle operator-(const CanonicalAngle& other) const { return rotateBy(-other); }

  CanonicalAngle operator*(double scalar) const { return CanonicalAngle(radians() * scalar); }

  CanonicalAngle operator/(double scalar) const { return CanonicalAngle(radians() / scalar); }

  /**
   * Rotates the current angle by another angle.
   * @param other The angle to rotate by.
   * @return The rotated angle.
   */
  CanonicalAngle rotateBy(const CanonicalAngle& other) const {
    return {cos() * other.cos() - sin() * other.sin(), cos() * other.sin() + sin() * other.cos()};
  }

  /**
   * Calculates the supplementary angle.
   * @return The supplementary angle.
   */
  CanonicalAngle supplementary() const { return CanonicalAngle{-m_cos, -m_sin}; }

  /**
   * Checks if this angle is supplementary to another angle.
   * @param other The angle to check against.
   * @return True if the angles are supplementary, false otherwise.
   */
  bool isSupplementaryTo(const CanonicalAngle& other) const { return supplementary() == other; }
};

}  // namespace types

using types::CanonicalAngle;

}  // namespace thunder

