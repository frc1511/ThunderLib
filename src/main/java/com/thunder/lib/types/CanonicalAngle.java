package com.thunder.lib.types;

import static edu.wpi.first.units.Units.Radians;

import java.util.Objects;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;

/**
 * Represents an angle in the range of (-π, π].
 * This class is similar to Rotation2d, but is designed to represent angles in a
 * more mathematically consistent way, avoiding issues with wrapping and
 * discontinuities.
 */
public class CanonicalAngle {
  private double m_cos;
  private double m_sin;

  /**
   * Constructs a CanonicalAngle with a default value of 0 radians.
   */
  public CanonicalAngle() {
    m_cos = 1.0;
    m_sin = 0.0;
  }

  /**
   * Constructs a CanonicalAngle from a Rotation2d.
   * @param rotation The Rotation2d to convert to a CanonicalAngle.
   */
  public CanonicalAngle(Rotation2d rotation) {
    m_cos = rotation.getCos();
    m_sin = rotation.getSin();
  }

  /**
   * Constructs a CanonicalAngle with the specified radian value.
   * @param radians The value in radians.
   */
  public CanonicalAngle(double radians) {
    m_cos = Math.cos(radians);
    m_sin = Math.sin(radians);
  }

  /**
   * Constructs a CanonicalAngle with the specified Angle value.
   * @param angle The Angle value.
   */
  public CanonicalAngle(Angle angle) {
    this(angle.in(Radians));
  }

  /**
   * Constructs and returns a CanonicalAngle with the specified radian value.
   * @param radians The value in radians.
   * @return The angle object.
   */
  public static CanonicalAngle fromRadians(double radians) {
    return new CanonicalAngle(radians);
  }

  /**
   * Constructs and returns a CanonicalAngle with the specified degree value.
   * @param degrees The value in degrees.
   * @return The angle object.
   */
  public static CanonicalAngle fromDegrees(double degrees) {
    return new CanonicalAngle(Math.toRadians(degrees));
  }

  /**
   * Constructs a CanonicalAngle with the specified x and y components.
   * @param x The x component or cosine of the angle.
   * @param y The y component or sine of the angle.
   */
  public CanonicalAngle(double x, double y) {
    double magnitude = Math.hypot(x, y);
    if (magnitude < 1e-6) {
      MathSharedStore.reportError(
          "x and y components of Rotation2d are zero\n", Thread.currentThread().getStackTrace());
    }

    m_cos = x / magnitude;
    m_sin = y / magnitude;
  }

  /**
   * Returns the cosine of the angle.
   * @return Cosine of the angle.
   */
  public double getCos() {
    return m_cos;
  }

  /**
   * Returns the sine of the angle.
   * @return Sine of the angle.
   */
  public double getSin() {
    return m_sin;
  }

  /**
   * Returns the tangent of the angle.
   * @return Tangent of the angle.
   */
  public double getTan() {
    return m_sin / m_cos;
  }

  /**
   * Returns the angle in radians.
   * @return Angle in radians.
   */
  public double getRadians() {
    return Math.atan2(m_sin, m_cos);
  }

  /**
   * Returns the angle in degrees.
   * @return Angle in degrees.
   */
  public double getDegrees() {
    return Math.toDegrees(getRadians());
  }

  /**
   * Converts this CanonicalAngle to a Rotation2d.
   * @return The corresponding Rotation2d.
   */
  public Rotation2d toRotation2d() {
    return new Rotation2d(m_cos, m_sin);
  }

  /**
   * Converts this CanonicalAngle to an Angle measure.
   * @return The corresponding Angle measure.
   */
  public Angle toAngleMeasure() {
    return Radians.of(getRadians());
  }

  /**
   * Adds two angles together.
   * @param other The angles to add.
   * @return The sum of the two angles.
   */
  public CanonicalAngle plus(CanonicalAngle other) {
    return rotateBy(other);
  }

  /**
   * Subtracts the new angle from the current angle.
   * @param other The angle to subtract.
   * @return The difference between the two angles.
   */
  public CanonicalAngle minus(CanonicalAngle other) {
    return rotateBy(other.unaryMinus());
  }

  /**
   * Takes the inverse (negation) of the current angle.
   * @return The inverse of the current angle.
   */
  public CanonicalAngle unaryMinus() {
    return new CanonicalAngle(-getRadians());
  }

  /**
   * Rotates the current angle by another angle.
   * @param other The angle to rotate by.
   * @return The rotated angle.
   */
  public CanonicalAngle rotateBy(CanonicalAngle other) {
    double newCos = m_cos * other.m_cos - m_sin * other.m_sin;
    double newSin = m_cos * other.m_sin + m_sin * other.m_cos;
    return new CanonicalAngle(newCos, newSin);
  }

  /**
   * Calculates the supplementary angle.
   * @return The supplementary angle.
   */
  public CanonicalAngle supplementary() {
    return new CanonicalAngle(-m_cos, -m_sin);
  }

  /**
   * Checks if this angle is supplementary to another angle.
   * @param other The angle to check against.
   * @return True if the angles are supplementary, false otherwise.
   */
  public boolean isSupplementaryTo(CanonicalAngle other) {
    return supplementary().equals(other);
  }

  @Override
  public String toString() {
    return String.format("CanonicalAngle(Rads: %.2f, Deg: %.2f)", getRadians(), getDegrees());
  }

  @Override
  public boolean equals(Object obj) {
    return obj instanceof CanonicalAngle other
        && Math.hypot(m_cos - other.m_cos, m_sin - other.m_sin) < 1E-9;
  }

  @Override
  public int hashCode() {
    return Objects.hash(m_cos, m_sin);
  }
}
