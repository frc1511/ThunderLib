#include <gtest/gtest.h>

#include <ThunderLibCore/Types.hpp>
#include <numbers>

using namespace thunder::core;

TEST(CanonicalAngleTests, DefaultConstructor) {
  CanonicalAngle angle;
  EXPECT_DOUBLE_EQ(angle.radians().value(), 0.0);
}

TEST(CanonicalAngleTests, ConstructorFromRotation2d) {
  frc::Rotation2d rotation(1.0_rad);
  CanonicalAngle angle(rotation);
  EXPECT_DOUBLE_EQ(angle.radians().value(), 1.0);
}

TEST(CanonicalAngleTests, ConstructorFromAngle) {
  CanonicalAngle angle(1.0_rad);
  EXPECT_DOUBLE_EQ(angle.radians().value(), 1.0);
}

TEST(CanonicalAngleTests, ConstructorFromComponents) {
  double angleRad = units::radian_t(30_deg).value();
  CanonicalAngle angle(gcem::cos(angleRad), gcem::sin(angleRad));
  EXPECT_DOUBLE_EQ(angle.radians().value(), angleRad);
}

TEST(CanonicalAngleTests, CosSinTanMethods) {
  CanonicalAngle angle(1.0_rad);
  EXPECT_DOUBLE_EQ(angle.cos(), gcem::cos(1.0));
  EXPECT_DOUBLE_EQ(angle.sin(), gcem::sin(1.0));
  EXPECT_DOUBLE_EQ(angle.tan(), gcem::tan(1.0));
}

TEST(CanonicalAngleTests, RadiansDegreesConversion) {
  double angleRad = 1.0;
  double angleDeg = units::degree_t(1.0_rad).value();
  CanonicalAngle angle(1.0_rad);
  EXPECT_DOUBLE_EQ(angle.radians().value(), angleRad);
  EXPECT_DOUBLE_EQ(angle.degrees().value(), angleDeg);
}

TEST(CanonicalAngleTests, ImplicitConversionToRotation2d) {
  CanonicalAngle angle(1.0_rad);
  frc::Rotation2d rotation = angle;
  EXPECT_DOUBLE_EQ(rotation.Radians().value(), 1.0);
}

TEST(CanonicalAngleTests, ImplicitConversionToAngleUnit) {
  CanonicalAngle angle(1.0_rad);
  units::radian_t radian = angle;
  EXPECT_DOUBLE_EQ(radian.value(), 1.0);
}

TEST(CanonicalAngleTests, Equalit) {
  CanonicalAngle angle1(1.0_rad);
  CanonicalAngle angle2(1.0_rad);
  CanonicalAngle angle3(2.0_rad);

  EXPECT_TRUE(angle1 == angle2);
  EXPECT_FALSE(angle1 == angle3);
  EXPECT_FALSE(angle1 != angle2);
  EXPECT_TRUE(angle1 != angle3);
}

TEST(CanonicalAngleTests, Negation) {
  CanonicalAngle angle(1.0_rad);
  CanonicalAngle negated = -angle;
  EXPECT_DOUBLE_EQ(negated.radians().value(), -1.0);
}

TEST(CanonicalAngleTests, Addition) {
  CanonicalAngle angle1(1.0_rad);
  CanonicalAngle angle2(0.5_rad);
  CanonicalAngle sum = angle1 + angle2;
  EXPECT_DOUBLE_EQ(sum.radians().value(), (1.0 + 0.5));
}

TEST(CanonicalAngleTests, AdditionWrap) {
  CanonicalAngle angle1(90_deg);
  CanonicalAngle angle2(180_deg);
  CanonicalAngle sum = angle1 + angle2;
  EXPECT_DOUBLE_EQ(sum.degrees().value(), -90.0);
}

TEST(CanonicalAngleTests, Subtraction) {
  CanonicalAngle angle1(1.0_rad);
  CanonicalAngle angle2(0.5_rad);
  CanonicalAngle difference = angle1 - angle2;
  EXPECT_DOUBLE_EQ(difference.radians().value(), (1.0 - 0.5));
}

TEST(CanonicalAngleTests, SubtractionWrap) {
  CanonicalAngle angle1(45_deg);
  CanonicalAngle angle2(-45_deg);
  CanonicalAngle difference = angle1 - angle2;
  EXPECT_DOUBLE_EQ(difference.degrees().value(), 90.0);
}

TEST(CanonicalAngleTests, RotateBy) {
  CanonicalAngle angle1(1.0_rad);
  CanonicalAngle angle2(0.5_rad);
  CanonicalAngle rotated = angle1.rotateBy(angle2);
  EXPECT_DOUBLE_EQ(rotated.radians().value(), (1.0 + 0.5));
}

TEST(CanonicalAngleTests, SupplementaryAngle) {
  CanonicalAngle angle(1.0_rad);
  CanonicalAngle supplementary = angle.supplementary();
  EXPECT_DOUBLE_EQ(supplementary.radians().value(), 1.0 - std::numbers::pi);
}

TEST(CanonicalAngleTests, IsSupplementaryTo) {
  CanonicalAngle angle(1.0_rad);
  CanonicalAngle supplementary = angle.supplementary();
  EXPECT_TRUE(angle.isSupplementaryTo(supplementary));
  EXPECT_FALSE(angle.isSupplementaryTo(CanonicalAngle(2.0_rad)));
}

