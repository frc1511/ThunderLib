#pragma once

#include <ThunderLibCore/Concepts.hpp>
#include <ThunderLibCore/Error.hpp>

#include <frc/geometry/Rotation2d.h>
#include <units/angle.h>
#include <units/length.h>
#include <gcem.hpp>
#include <wpi/json.h>

#include <algorithm>
#include <array>

namespace thunder::core {

template <int N, Arithmetic T>
struct VecBase {
  static constexpr int size = N;
  std::array<T, N> data;

 public:
  // Constructors

  VecBase() : data{} {}

  template <typename... Args>
  VecBase(Args... args) : data{static_cast<T>(args)...} {
    static_assert(sizeof...(args) == N, "Incorrect number of arguments");
  }

  // Default copy - don't let calls get caught by the variadic constructor.
  VecBase(const VecBase& other) = default;

  virtual ~VecBase() = default;

  // Data Access

  T& operator[](int index) {
    if (index >= N || index < 0) {
      throw OutOfRangeError::Construct("Index out of range");
    }
    index = std::clamp(index, 0, N - 1);
    return this->data[index];
  }

  // Assignment

  VecBase& operator=(const VecBase& other) = default;

  // Comparison

  bool operator==(const VecBase& other) const { return (this->data == other.data); }

  bool operator!=(const VecBase& other) const { return !(*this == other); }

  bool operator<(const VecBase& other) const {
    bool result = true;
    for (int i = 0; i < N; ++i) {
      result &= (this->data[i] < other.data[i]);
    }
    return result;
  }

  bool operator<=(const VecBase& other) const { return (*this < other) || (*this == other); }

  bool operator>(const VecBase& other) const { return !(*this <= other); }

  bool operator>=(const VecBase& other) const { return !(*this < other); }

  // Arithmetic

  VecBase operator-() const {
    VecBase result;
    for (int i = 0; i < N; ++i) {
      result.data[i] = -this->data[i];
    }
    return result;
  }

  VecBase operator+(const VecBase& other) const {
    VecBase result = *this;
    return result += other;
  }

  VecBase operator-(const VecBase& other) const {
    VecBase result = *this;
    return result -= other;
  }

  VecBase operator+(T scalar) const {
    VecBase result = *this;
    return result += scalar;
  }

  VecBase operator-(T scalar) const {
    VecBase result = *this;
    return result -= scalar;
  }

  VecBase operator*(T scalar) const {
    VecBase result = *this;
    return result *= scalar;
  }

  VecBase operator/(T scalar) const {
    if (scalar == 0) {
      throw InvalidArgumentError::Construct("Division by zero");
    }
    VecBase result = *this;
    return result /= scalar;
  }

  // Arithmetic Assignment

  VecBase& operator+=(const VecBase& other) {
    for (int i = 0; i < N; ++i) {
      this->data[i] += other.data[i];
    }
    return *this;
  }

  VecBase& operator-=(const VecBase& other) {
    for (int i = 0; i < N; ++i) {
      this->data[i] -= other.data[i];
    }
    return *this;
  }

  VecBase& operator+=(T scalar) {
    for (int i = 0; i < N; ++i) {
      this->data[i] += scalar;
    }
    return *this;
  }

  VecBase& operator-=(T scalar) {
    for (int i = 0; i < N; ++i) {
      this->data[i] -= scalar;
    }
    return *this;
  }

  VecBase& operator*=(T scalar) {
    for (int i = 0; i < N; ++i) {
      this->data[i] *= scalar;
    }
    return *this;
  }

  VecBase& operator/=(T scalar) {
    if (scalar == 0) {
      throw InvalidArgumentError::Construct("Division by zero");
    }
    for (int i = 0; i < N; ++i) {
      this->data[i] /= scalar;
    }
    return *this;
  }
};

// Declare some necessary attributes of Vec classes derived from VecBase.
#define VECBASE_TYPE(Name, Base)                                                       \
 public:                                                                               \
  using Base::Base;       /* Inherit constructors */                                   \
  Name(const Name& other) /* Copy data */                                              \
      : Base(static_cast<const Base&>(other)) {}                                       \
  /*implicit*/ Name(const Base& other) /* Convert from base (so inherited ops work) */ \
      : Base(other) {}                                                                 \
  Name& operator=(const Name& other) { /* Inherit assignment */                        \
    Base::operator=(other);                                                            \
    return *this;                                                                      \
  }

template <int N, Arithmetic T>
struct Vec;

template <Arithmetic T>
struct Vec<2, T> : public VecBase<2, T> {
  using value_type = T;
  using base_type = VecBase<2, T>;
  using type = Vec<2, T>;

  VECBASE_TYPE(Vec, base_type)

  T& x{this->data[0]};
  T& y{this->data[1]};
};

template <Arithmetic T>
struct Vec<3, T> : public VecBase<3, T> {
  using value_type = T;
  using base_type = VecBase<3, T>;
  using type = Vec<3, T>;

  VECBASE_TYPE(Vec, base_type)

  T& x{this->data[0]};
  T& y{this->data[1]};
  T& z{this->data[2]};
};

template <Arithmetic T>
struct Vec<4, T> : public VecBase<4, T> {
  using value_type = T;
  using base_type = VecBase<4, T>;
  using type = Vec<4, T>;

  VECBASE_TYPE(Vec, base_type)

  T& x{this->data[0]};
  T& y{this->data[1]};
  T& z{this->data[2]};
  T& w{this->data[3]};
};

using Vec2 = Vec<2, double>;
using Vec3 = Vec<3, double>;
using Vec4 = Vec<4, double>;

void to_json(wpi::json& json, const Vec2& vec) noexcept;
void from_json(const wpi::json& json, Vec2& vec);
void to_json(wpi::json& json, const Vec3& vec) noexcept;
void from_json(const wpi::json& json, Vec3& vec);
void to_json(wpi::json& json, const Vec4& vec) noexcept;
void from_json(const wpi::json& json, Vec4& vec);

using Vec2i = Vec<2, int>;
using Vec3i = Vec<3, int>;
using Vec4i = Vec<4, int>;

void to_json(wpi::json& json, const Vec2i& vec) noexcept;
void from_json(const wpi::json& json, Vec2i& vec);
void to_json(wpi::json& json, const Vec3i& vec) noexcept;
void from_json(const wpi::json& json, Vec3i& vec);
void to_json(wpi::json& json, const Vec4i& vec) noexcept;
void from_json(const wpi::json& json, Vec4i& vec);

using Vec2u = Vec<2, unsigned>;
using Vec3u = Vec<3, unsigned>;
using Vec4u = Vec<4, unsigned>;

void to_json(wpi::json& json, const Vec2u& vec) noexcept;
void from_json(const wpi::json& json, Vec2u& vec);
void to_json(wpi::json& json, const Vec3u& vec) noexcept;
void from_json(const wpi::json& json, Vec3u& vec);
void to_json(wpi::json& json, const Vec4u& vec) noexcept;
void from_json(const wpi::json& json, Vec4u& vec);

class CanonicalAngle {
  double m_cos = 1.0;
  double m_sin = 0.0;

 public:
  CanonicalAngle() = default;

  /*implicit*/ CanonicalAngle(const frc::Rotation2d& rotation)
      : m_cos(rotation.Cos()), m_sin(rotation.Sin()) {}

  /*implicit*/ CanonicalAngle(units::angle_unit auto value)  // NOLINT
      : m_cos{gcem::cos(value.template convert<units::radian>().value())},
        m_sin{gcem::sin(value.template convert<units::radian>().value())} {}

  CanonicalAngle(double x, double y) {
    double magnitude = gcem::hypot(x, y);
    ThunderLibCoreAssert(magnitude > 1e-6, "Cannot create CanonicalAngle with zero magnitude");
    m_cos = x / magnitude;
    m_sin = y / magnitude;
  }

  explicit CanonicalAngle(const Vec2& vec) : CanonicalAngle(vec.x, vec.y) {}

  double cos() const { return m_cos; }
  double sin() const { return m_sin; }
  double tan() const { return m_sin / m_cos; }

  units::radian_t radians() const { return units::radian_t{gcem::atan2(m_sin, m_cos)}; }

  units::degree_t degrees() const { return radians().convert<units::degree>(); }

  /*implicit*/ operator frc::Rotation2d() const { return frc::Rotation2d{m_cos, m_sin}; }

  /*implicit*/ operator units::angle_unit auto() const { return radians(); }

  bool operator==(const CanonicalAngle& other) const {
    return gcem::hypot(cos() - other.cos(), sin() - other.sin()) < 1E-6;
  }

  bool operator!=(const CanonicalAngle& other) const { return !(*this == other); }

  CanonicalAngle operator-() const { return CanonicalAngle{-radians()}; }

  CanonicalAngle operator+(const CanonicalAngle& other) const { return rotateBy(other); }

  CanonicalAngle operator-(const CanonicalAngle& other) const { return rotateBy(-other); }

  CanonicalAngle operator*(double scalar) const { return CanonicalAngle(radians() * scalar); }

  CanonicalAngle operator/(double scalar) const {
    if (scalar == 0.0) {
      throw InvalidArgumentError::Construct("Division by zero");
    }
    return CanonicalAngle(radians() / scalar);
  }

  CanonicalAngle rotateBy(const CanonicalAngle& other) const {
    return {cos() * other.cos() - sin() * other.sin(), cos() * other.sin() + sin() * other.cos()};
  }

  CanonicalAngle supplementary() const { return CanonicalAngle{-m_cos, -m_sin}; }

  bool isSupplementaryTo(const CanonicalAngle& other) const { return supplementary() == other; }
};

void to_json(wpi::json& json, const CanonicalAngle& angle) noexcept;
void from_json(const wpi::json& json, CanonicalAngle& angle);

struct Displacement2d : public VecBase<2, units::meter_t> {
  using value_type = units::meter_t;
  using base_type = VecBase<2, units::meter_t>;
  using type = Displacement2d;

  VECBASE_TYPE(Displacement2d, base_type)

  value_type& x{this->data[0]};
  value_type& y{this->data[1]};

  Displacement2d operator*(double scalar) const { return Displacement2d{x * scalar, y * scalar}; }
  Displacement2d operator/(double scalar) const {
    if (scalar == 0.0) {
      throw InvalidArgumentError::Construct("Division by zero");
    }
    return Displacement2d{x / scalar, y / scalar};
  }

  units::meter_t distance() const { return units::meter_t(gcem::hypot(x.value(), y.value())); }

  CanonicalAngle angle() const { return CanonicalAngle{x.value(), y.value()}; }
};

void to_json(wpi::json& json, const Displacement2d& displacement) noexcept;
void from_json(const wpi::json& json, Displacement2d& displacement);

struct Point2d : public VecBase<2, units::meter_t> {
  using value_type = units::meter_t;
  using base_type = VecBase<2, units::meter_t>;
  using type = Point2d;

  VECBASE_TYPE(Point2d, base_type)

  value_type& x{this->data[0]};
  value_type& y{this->data[1]};

  units::meter_t distanceTo(const Point2d& other) const {
    Displacement2d displacement = other - *this;
    return displacement.distance();
  }

  CanonicalAngle angleTo(const Point2d& other) const {
    Displacement2d displacement = other - *this;
    return displacement.angle();
  }

  Point2d extendAtAngle(const CanonicalAngle& angle, units::meter_t distance) const {
    units::meter_t dx = gcem::cos(angle.radians().value()) * distance;
    units::meter_t dy = gcem::sin(angle.radians().value()) * distance;
    return Point2d{x + dx, y + dy};
  }

  Displacement2d operator-(const Point2d& other) const { return Displacement2d{x - other.x, y - other.y}; }
};

void to_json(wpi::json& json, const Point2d& point) noexcept;
void from_json(const wpi::json& json, Point2d& point);

struct Measurement2d : public VecBase<2, units::meter_t> {
  using value_type = units::meter_t;
  using base_type = VecBase<2, units::meter_t>;
  using type = Measurement2d;

  VECBASE_TYPE(Measurement2d, base_type)

  value_type& x{this->data[0]};
  value_type& y{this->data[1]};
  value_type& width{this->data[0]};
  value_type& length{this->data[1]};
};

void to_json(wpi::json& json, const Measurement2d& measurement) noexcept;
void from_json(const wpi::json& json, Measurement2d& measurement);

struct Rect {
  Vec2 min;
  Vec2 max;

  Rect() = default;
  Rect(const Vec2& min, const Vec2& max) : min(min), max(max) {}
  Rect(double x1, double y1, double x2, double y2) : min(x1, y1), max(x2, y2) {}

  bool operator==(const Rect& other) const noexcept { return min == other.min && max == other.max; }

  double width() const { return max.x - min.x; }
  double height() const { return max.y - min.y; }
};

void to_json(wpi::json& json, const Rect& rect) noexcept;
void from_json(const wpi::json& json, Rect& rect);

#undef VECBASE_TYPE

}  // namespace thunder::core
