package com.thunder.lib.tests.types;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import com.thunder.lib.tests.BaseTestSuite;
import com.thunder.lib.types.CanonicalAngle;

public class CanonicalAngleTestSuite extends BaseTestSuite {
  @Test
  void testDefaultConstructor() {
    CanonicalAngle angle = new CanonicalAngle();
    assertEquals(0.0, angle.getRadians(), DOUBLE_TOLERANCE);
  }

  @Test
  void testConstructorFromRotation2d() {
    Rotation2d rotation = new Rotation2d(1.0);
    CanonicalAngle angle = new CanonicalAngle(rotation);
    assertEquals(1.0, angle.getRadians(), DOUBLE_TOLERANCE);
  }

  @Test
  void testConstructorFromRadians() {
    CanonicalAngle angle = new CanonicalAngle(Math.PI / 4);
    assertEquals(Math.PI / 4, angle.getRadians(), DOUBLE_TOLERANCE);
  }

  @Test
  void testConstructorFromAngle() {
    CanonicalAngle angle = new CanonicalAngle(Degrees.of(45.0));
    assertEquals(45.0, angle.getDegrees(), DOUBLE_TOLERANCE);
  }

  @Test
  void testFromRadians() {
    CanonicalAngle angle = CanonicalAngle.fromRadians(Math.PI / 4);
    assertEquals(Math.PI / 4, angle.getRadians(), DOUBLE_TOLERANCE);
  }

  @Test
  void testFromDegrees() {
    CanonicalAngle angle = CanonicalAngle.fromDegrees(45.0);
    assertEquals(45.0, angle.getDegrees(), DOUBLE_TOLERANCE);
  }

  @Test
  void testConstructorFromComponents() {
    double angleRad = 1.0;
    CanonicalAngle angle = new CanonicalAngle(Math.cos(angleRad), Math.sin(angleRad));
    assertEquals(angleRad, angle.getRadians(), DOUBLE_TOLERANCE);
  }

  @Test
  void testCosSinTan() {
    double angleRad = Math.PI / 4;
    CanonicalAngle angle = new CanonicalAngle(angleRad);
    assertEquals(Math.cos(angleRad), angle.getCos(), DOUBLE_TOLERANCE);
    assertEquals(Math.sin(angleRad), angle.getSin(), DOUBLE_TOLERANCE);
    assertEquals(Math.tan(angleRad), angle.getTan(), DOUBLE_TOLERANCE);
  }

  @Test
  void testRadiansDegreesConversion() {
    double angleRad = Math.PI / 4;
    double angleDeg = 45.0;
    CanonicalAngle angle = new CanonicalAngle(angleRad);
    assertEquals(angleDeg, angle.getDegrees(), DOUBLE_TOLERANCE);
    assertEquals(angleRad, angle.getRadians(), DOUBLE_TOLERANCE);
  }

  @Test
  void testToRotation2d() {
    CanonicalAngle angle = new CanonicalAngle(Math.PI / 4);
    Rotation2d rotation = angle.toRotation2d();
    assertEquals(Math.PI / 4, rotation.getRadians(), DOUBLE_TOLERANCE);
  }

  @Test
  void testConversionToAngleMeasure() {
    CanonicalAngle angle = new CanonicalAngle(Math.PI / 4);
    Angle angleMeasurement = angle.toAngleMeasure();
    assertEquals(Math.PI / 4, angleMeasurement.in(Radians), DOUBLE_TOLERANCE);
  }

  @Test
  void testEquals() {
    CanonicalAngle angle1 = CanonicalAngle.fromDegrees(+180.0);
    CanonicalAngle angle2 = CanonicalAngle.fromDegrees(-180.0);
    assertEquals(angle1, angle2);
  }

  @Test
  void testUnaryMinus() {
    CanonicalAngle angle = CanonicalAngle.fromDegrees(90.0);
    CanonicalAngle negatedAngle = angle.unaryMinus();
    assertEquals(-90.0, negatedAngle.getDegrees(), DOUBLE_TOLERANCE);
  }

  @Test
  void testPlus() {
    CanonicalAngle angle1 = CanonicalAngle.fromDegrees(30.0);
    CanonicalAngle angle2 = CanonicalAngle.fromDegrees(60.0);
    CanonicalAngle result = angle1.plus(angle2);
    assertEquals(90.0, result.getDegrees(), DOUBLE_TOLERANCE);
  }

  @Test
  void testPlusWrap() {
    CanonicalAngle angle1 = CanonicalAngle.fromDegrees(90.0);
    CanonicalAngle angle2 = CanonicalAngle.fromDegrees(180.0);
    CanonicalAngle result = angle1.plus(angle2);
    assertEquals(-90.0, result.getDegrees(), DOUBLE_TOLERANCE);
  }

  @Test
  void testMinus() {
    CanonicalAngle angle1 = CanonicalAngle.fromDegrees(90.0);
    CanonicalAngle angle2 = CanonicalAngle.fromDegrees(30.0);
    CanonicalAngle result = angle1.minus(angle2);
    assertEquals(60.0, result.getDegrees(), DOUBLE_TOLERANCE);
  }

  @Test
  void testRotateBy() {
    CanonicalAngle angle1 = CanonicalAngle.fromDegrees(30.0);
    CanonicalAngle angle2 = CanonicalAngle.fromDegrees(60.0);
    CanonicalAngle result = angle1.rotateBy(angle2);
    assertEquals(90.0, result.getDegrees(), DOUBLE_TOLERANCE);
  }

  @Test
  void testSupplementaryAngle() {
    CanonicalAngle angle = CanonicalAngle.fromDegrees(30.0);
    CanonicalAngle supplementary = angle.supplementary();
    assertEquals(-150.0, supplementary.getDegrees(), DOUBLE_TOLERANCE);
  }

  @Test
  void testIsSupplementaryTo() {
    CanonicalAngle angle1 = CanonicalAngle.fromDegrees(30.0);
    CanonicalAngle angle2 = CanonicalAngle.fromDegrees(210.0);
    assertTrue(angle1.isSupplementaryTo(angle2));
  }
}