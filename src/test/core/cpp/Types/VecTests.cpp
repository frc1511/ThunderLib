#include <gtest/gtest.h>

#include <ThunderLibCore/Types.hpp>
#include <units/length.h>

using namespace thunder::core;

//
// Vec2
//

TEST(Vec2Tests, DefaultConstructor) {
  Vec2 vec;
  EXPECT_DOUBLE_EQ(vec.x, 0.0);
  EXPECT_DOUBLE_EQ(vec.y, 0.0);
}

TEST(Vec2Tests, ParameterizedConstructor) {
  Vec2 vec(3.0, 4.0);
  EXPECT_DOUBLE_EQ(vec.x, 3.0);
  EXPECT_DOUBLE_EQ(vec.y, 4.0);
}

TEST(Vec2Tests, CopyConstructor) {
  Vec2 original(5.0, 6.0);
  Vec2 copy(original);
  EXPECT_DOUBLE_EQ(copy.x, 5.0);
  EXPECT_DOUBLE_EQ(copy.y, 6.0);

  // Make sure references aren't copied.
  original.x = 6.0;
  original.y = 7.0;
  EXPECT_DOUBLE_EQ(copy.x, 5.0);
  EXPECT_DOUBLE_EQ(copy.y, 6.0);
}

TEST(Vec2Tests, Size) {
  Vec2 vec;
  EXPECT_EQ(vec.size, 2);
}

TEST(Vec2Tests, ArrayAccess) {
  Vec2 vec(1.0, 2.0);
  EXPECT_DOUBLE_EQ(vec[0], 1.0);
  EXPECT_DOUBLE_EQ(vec[1], 2.0);
}

TEST(Vec2Tests, Assignment) {
  Vec2 vec1(1.0, 2.0);
  Vec2 vec2;
  vec2 = vec1;
  EXPECT_DOUBLE_EQ(vec2.x, 1.0);
  EXPECT_DOUBLE_EQ(vec2.y, 2.0);
}

TEST(Vec2Tests, Equal) {
  Vec2 vec1(1.0, 2.0);
  Vec2 vec2(1.0, 2.0);
  Vec2 vec3(3.0, 4.0);

  EXPECT_TRUE(vec1 == vec2);
  EXPECT_FALSE(vec1 == vec3);
}

TEST(Vec2Tests, NotEqual) {
  Vec2 vec1(1.0, 2.0);
  Vec2 vec2(1.0, 2.0);
  Vec2 vec3(3.0, 4.0);

  EXPECT_FALSE(vec1 != vec2);
  EXPECT_TRUE(vec1 != vec3);
}

TEST(Vec2Tests, LessThan) {
  Vec2 vec1(1.0, 2.0);
  Vec2 vec2(3.0, 4.0);

  EXPECT_TRUE(vec1 < vec2);
  EXPECT_FALSE(vec2 < vec1);
}

TEST(Vec2Tests, LessThanOrEqual) {
  Vec2 vec1(1.0, 2.0);
  Vec2 vec2(1.0, 2.0);
  Vec2 vec3(3.0, 4.0);

  EXPECT_TRUE(vec1 <= vec2);
  EXPECT_TRUE(vec1 <= vec3);
  EXPECT_FALSE(vec3 <= vec1);
}

TEST(Vec2Tests, GreaterThan) {
  Vec2 vec1(1.0, 2.0);
  Vec2 vec2(3.0, 4.0);

  EXPECT_FALSE(vec1 > vec2);
  EXPECT_TRUE(vec2 > vec1);
}

TEST(Vec2Tests, GreaterThanOrEqual) {
  Vec2 vec1(1.0, 2.0);
  Vec2 vec2(1.0, 2.0);
  Vec2 vec3(3.0, 4.0);

  EXPECT_TRUE(vec1 >= vec2);
  EXPECT_FALSE(vec1 >= vec3);
  EXPECT_TRUE(vec3 >= vec1);
}

TEST(Vec2Tests, Add) {
  Vec2 vec1(1.0, 2.0);
  Vec2 vec2(3.0, 4.0);
  Vec2 result = vec1 + vec2;

  EXPECT_DOUBLE_EQ(result.x, 4.0);
  EXPECT_DOUBLE_EQ(result.y, 6.0);
}

TEST(Vec2Tests, ScalarAdd) {
  Vec2 vec(1.0, 2.0);
  double scalar = 3.0;
  Vec2 result = vec + scalar;

  EXPECT_DOUBLE_EQ(result.x, 4.0);
  EXPECT_DOUBLE_EQ(result.y, 5.0);
}

TEST(Vec2Tests, Subtract) {
  Vec2 vec1(5.0, 6.0);
  Vec2 vec2(3.0, 4.0);
  Vec2 result = vec1 - vec2;

  EXPECT_DOUBLE_EQ(result.x, 2.0);
  EXPECT_DOUBLE_EQ(result.y, 2.0);
}

TEST(Vec2Tests, ScalarSubtract) {
  Vec2 vec(5.0, 6.0);
  double scalar = 2.0;
  Vec2 result = vec - scalar;

  EXPECT_DOUBLE_EQ(result.x, 3.0);
  EXPECT_DOUBLE_EQ(result.y, 4.0);
}

TEST(Vec2Tests, ScalarMultiply) {
  Vec2 vec(2.0, 3.0);
  double scalar = 2.0;
  Vec2 result = vec * scalar;

  EXPECT_DOUBLE_EQ(result.x, 4.0);
  EXPECT_DOUBLE_EQ(result.y, 6.0);
}

TEST(Vec2Tests, ScalarDivide) {
  Vec2 vec(4.0, 6.0);
  double scalar = 2.0;
  Vec2 result = vec / scalar;

  EXPECT_DOUBLE_EQ(result.x, 2.0);
  EXPECT_DOUBLE_EQ(result.y, 3.0);
}

TEST(Vec2Tests, AddAssign) {
  Vec2 vec1(1.0, 2.0);
  Vec2 vec2(3.0, 4.0);
  vec1 += vec2;

  EXPECT_DOUBLE_EQ(vec1.x, 4.0);
  EXPECT_DOUBLE_EQ(vec1.y, 6.0);
}

TEST(Vec2Tests, SubtractAssign) {
  Vec2 vec1(5.0, 6.0);
  Vec2 vec2(3.0, 4.0);
  vec1 -= vec2;

  EXPECT_DOUBLE_EQ(vec1.x, 2.0);
  EXPECT_DOUBLE_EQ(vec1.y, 2.0);
}

TEST(Vec2Tests, ScalarAddAssign) {
  Vec2 vec(1.0, 2.0);
  double scalar = 3.0;
  vec += scalar;

  EXPECT_DOUBLE_EQ(vec.x, 4.0);
  EXPECT_DOUBLE_EQ(vec.y, 5.0);
}

TEST(Vec2Tests, ScalarSubtractAssign) {
  Vec2 vec(5.0, 6.0);
  double scalar = 2.0;
  vec -= scalar;

  EXPECT_DOUBLE_EQ(vec.x, 3.0);
  EXPECT_DOUBLE_EQ(vec.y, 4.0);
}

TEST(Vec2Tests, ScalarMultiplyAssign) {
  Vec2 vec(2.0, 3.0);
  double scalar = 2.0;
  vec *= scalar;

  EXPECT_DOUBLE_EQ(vec.x, 4.0);
  EXPECT_DOUBLE_EQ(vec.y, 6.0);
}

TEST(Vec2Tests, ScalarDivideAssign) {
  Vec2 vec(4.0, 6.0);
  double scalar = 2.0;
  vec /= scalar;

  EXPECT_DOUBLE_EQ(vec.x, 2.0);
  EXPECT_DOUBLE_EQ(vec.y, 3.0);
}

TEST(Vec2Tests, WithUnits) {
  Vec<2, units::meter_t> vec(1.0_m, 2.0_m);

  vec += 3.0_m;
  EXPECT_DOUBLE_EQ(vec.x.value(), 4.0);
  EXPECT_DOUBLE_EQ(vec.y.value(), 5.0);
}

//
// Vec3
//

TEST(Vec3Tests, SanityCheck) {
  Vec3 vec(1.0, 2.0, 3.0);
  EXPECT_DOUBLE_EQ(vec.x, 1.0);
  EXPECT_DOUBLE_EQ(vec.y, 2.0);
  EXPECT_DOUBLE_EQ(vec.z, 3.0);
}

//
// Vec4
//

TEST(Vec4Tests, SanityCheck) {
  Vec4 vec(1.0, 2.0, 3.0, 4.0);
  EXPECT_DOUBLE_EQ(vec.x, 1.0);
  EXPECT_DOUBLE_EQ(vec.y, 2.0);
  EXPECT_DOUBLE_EQ(vec.z, 3.0);
  EXPECT_DOUBLE_EQ(vec.w, 4.0);
}

//
// Displacement2d
//

TEST(Displacement2dTests, SanityCheck) {
  Displacement2d disp(3.0_m, 4.0_m);
  EXPECT_DOUBLE_EQ(disp.x.value(), 3.0);
  EXPECT_DOUBLE_EQ(disp.y.value(), 4.0);
}

TEST(Displacement2dTests, Distance) {
  Displacement2d disp(3.0_m, 4.0_m);
  EXPECT_DOUBLE_EQ(disp.distance().value(), 5.0);
}

TEST(Displacement2dTests, Angle) {
  Displacement2d disp(3.0_m, 4.0_m);
  EXPECT_DOUBLE_EQ(disp.angle().radians().value(), std::atan2(4.0, 3.0));
}

//
// Point2d
//

TEST(Point2dTests, SanityCheck) {
  Point2d point(5.0_m, 6.0_m);
  EXPECT_DOUBLE_EQ(point.x.value(), 5.0);
  EXPECT_DOUBLE_EQ(point.y.value(), 6.0);
}

TEST(Point2dTests, DistanceTo) {
  Point2d point1(1.0_m, 2.0_m);
  Point2d point2(4.0_m, 6.0_m);
  EXPECT_DOUBLE_EQ(point1.distanceTo(point2).value(), 5.0);
}

TEST(Point2dTests, AngleTo) {
  Point2d point1(1.0_m, 2.0_m);
  Point2d point2(units::meter_t(1.0 + gcem::sqrt(3.0)), 3.0_m);
  EXPECT_DOUBLE_EQ(point1.angleTo(point2).degrees().value(), 30.0);
}

TEST(Point2dTests, ExtendAtAngle) {
  Point2d point(1.0_m, 2.0_m);
  Displacement2d disp(3.0_m, 4.0_m);
  Point2d extended = point.extendAtAngle(disp.angle(), disp.distance());
  EXPECT_DOUBLE_EQ(extended.x.value(), 1.0 + 3.0);
  EXPECT_DOUBLE_EQ(extended.y.value(), 2.0 + 4.0);
}

