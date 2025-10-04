#include <ThunderLibCore/Types.hpp>

namespace thunder::core {

void to_json(wpi::json& json, const Vec2& vec) noexcept {
  json = wpi::json{
      {"x", vec.x},
      {"y", vec.y},
  };
}

void from_json(const wpi::json& json, Vec2& vec) {
  json.at("x").get_to(vec.x);
  json.at("y").get_to(vec.y);
}

void to_json(wpi::json& json, const Vec3& vec) noexcept {
  json = wpi::json{
      {"x", vec.x},
      {"y", vec.y},
      {"z", vec.z},
  };
}

void from_json(const wpi::json& json, Vec3& vec) {
  json.at("x").get_to(vec.x);
  json.at("y").get_to(vec.y);
  json.at("z").get_to(vec.z);
}

void to_json(wpi::json& json, const Vec4& vec) noexcept {
  json = wpi::json{
      {"x", vec.x},
      {"y", vec.y},
      {"z", vec.z},
      {"w", vec.w},
  };
}

void from_json(const wpi::json& json, Vec4& vec) {
  json.at("x").get_to(vec.x);
  json.at("y").get_to(vec.y);
  json.at("z").get_to(vec.z);
  json.at("w").get_to(vec.w);
}

void to_json(wpi::json& json, const Vec2i& vec) noexcept {
  json = wpi::json{
      {"x", vec.x},
      {"y", vec.y},
  };
}

void from_json(const wpi::json& json, Vec2i& vec) {
  json.at("x").get_to(vec.x);
  json.at("y").get_to(vec.y);
}

void to_json(wpi::json& json, const Vec3i& vec) noexcept {
  json = wpi::json{
      {"x", vec.x},
      {"y", vec.y},
      {"z", vec.z},
  };
}

void from_json(const wpi::json& json, Vec3i& vec) {
  json.at("x").get_to(vec.x);
  json.at("y").get_to(vec.y);
  json.at("z").get_to(vec.z);
}

void to_json(wpi::json& json, const Vec4i& vec) noexcept {
  json = wpi::json{
      {"x", vec.x},
      {"y", vec.y},
      {"z", vec.z},
      {"w", vec.w},
  };
}

void from_json(const wpi::json& json, Vec4i& vec) {
  json.at("x").get_to(vec.x);
  json.at("y").get_to(vec.y);
  json.at("z").get_to(vec.z);
  json.at("w").get_to(vec.w);
}

void to_json(wpi::json& json, const Vec2u& vec) noexcept {
  json = wpi::json{
      {"x", vec.x},
      {"y", vec.y},
  };
}

void from_json(const wpi::json& json, Vec2u& vec) {
  json.at("x").get_to(vec.x);
  json.at("y").get_to(vec.y);
}

void to_json(wpi::json& json, const Vec3u& vec) noexcept {
  json = wpi::json{
      {"x", vec.x},
      {"y", vec.y},
      {"z", vec.z},
  };
}

void from_json(const wpi::json& json, Vec3u& vec) {
  json.at("x").get_to(vec.x);
  json.at("y").get_to(vec.y);
  json.at("z").get_to(vec.z);
}

void to_json(wpi::json& json, const Vec4u& vec) noexcept {
  json = wpi::json{
      {"x", vec.x},
      {"y", vec.y},
      {"z", vec.z},
      {"w", vec.w},
  };
}

void from_json(const wpi::json& json, Vec4u& vec) {
  json.at("x").get_to(vec.x);
  json.at("y").get_to(vec.y);
  json.at("z").get_to(vec.z);
  json.at("w").get_to(vec.w);
}

void to_json(wpi::json& json, const CanonicalAngle& angle) noexcept {
  json = angle.radians().value();
}

void from_json(const wpi::json& json, CanonicalAngle& angle) {
  double radians;
  json.get_to(radians);
  angle = CanonicalAngle(units::radian_t(radians));
}

void to_json(wpi::json& json, const Displacement2d& displacement) noexcept {
  json = wpi::json{
      {"x", displacement.x.value()},
      {"y", displacement.y.value()},
  };
}

void from_json(const wpi::json& json, Displacement2d& displacement) {
  double x, y;
  json.at("x").get_to(x);
  json.at("y").get_to(y);
  displacement = Displacement2d(units::meter_t(x), units::meter_t(y));
}

void to_json(wpi::json& json, const Point2d& point) noexcept {
  json = wpi::json{
      {"x", point.x.value()},
      {"y", point.y.value()},
  };
}

void from_json(const wpi::json& json, Point2d& point) {
  double x, y;
  json.at("x").get_to(x);
  json.at("y").get_to(y);
  point = Point2d(units::meter_t(x), units::meter_t(y));
}

void to_json(wpi::json& json, const Measurement2d& measurement) noexcept {
  json = wpi::json{
      {"width", measurement.x.value()},
      {"length", measurement.y.value()},
  };
}

void from_json(const wpi::json& json, Measurement2d& measurement) {
  double width, length;
  json.at("width").get_to(width);
  json.at("length").get_to(length);
  measurement = Measurement2d(units::meter_t(width), units::meter_t(length));
}

void to_json(wpi::json& json, const Rect& rect) noexcept {
  json = wpi::json{
      {"min", rect.min},
      {"max", rect.max},
  };
}

void from_json(const wpi::json& json, Rect& rect) {
  json.at("min").get_to(rect.min);
  json.at("max").get_to(rect.max);
}

}  // namespace thunder::core
