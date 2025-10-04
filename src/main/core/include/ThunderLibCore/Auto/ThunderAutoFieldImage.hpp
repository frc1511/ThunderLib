#pragma once

#include <ThunderLibCore/Types.hpp>
#include <units/length.h>
#include <wpi/json.h>
#include <filesystem>
#include <variant>
#include <string_view>

namespace thunder::core {

enum class ThunderAutoFieldImageType {
  CUSTOM,
  BUILTIN,
};

enum class ThunderAutoBuiltinFieldImage : size_t {
  FIELD_2025,
  FIELD_2024,
  FIELD_2023,
  FIELD_2022,
  LATEST = FIELD_2025,
  _COUNT,
  INVALID = _COUNT,
};

const char* ThunderAutoBuiltinFieldImageToString(ThunderAutoBuiltinFieldImage builtin);

ThunderAutoBuiltinFieldImage StringToThunderAutoBuiltinFieldImage(std::string_view str);

class ThunderAutoFieldImage {
  std::variant<ThunderAutoBuiltinFieldImage, std::filesystem::path> m_image;
  // Image coordinates representing the corners of the field in the image.
  Rect m_imageFieldBounds;

  Measurement2d m_fieldSize;

 public:
  ThunderAutoFieldImage();

  /**
   * @brief Construct a ThunderAutoFieldImage from a built-in field image.
   * @param builtin The built-in field image to use.
   */
  explicit ThunderAutoFieldImage(ThunderAutoBuiltinFieldImage builtin);

  /**
   * @brief Construct a ThunderAutoFieldImage from a custom image file.
   * @param image_path The path to the custom image file.
   * @param rect Image coordinates representing the corners of the field in the
   *             image.
   * @param size The size of the field in meters.
   */
  ThunderAutoFieldImage(const std::filesystem::path& imagePath, Rect rect, Measurement2d size);

  ThunderAutoFieldImage(const std::filesystem::path& imagePath, Vec2 min, Vec2 max, Measurement2d size);

  ThunderAutoFieldImage(const wpi::json& json, const std::filesystem::path& projectDirectory);

  void fromJson(const wpi::json& json, const std::filesystem::path& projectDirectory);
  void toJson(const std::filesystem::path& projectDirectory, wpi::json& json) const;

  ThunderAutoFieldImageType type() const;

  void setCustomImagePath(const std::filesystem::path& path) { m_image = path; }
  const std::filesystem::path& customImagePath() const;

  void setBuiltinImage(ThunderAutoBuiltinFieldImage builtin) { m_image = builtin; }
  ThunderAutoBuiltinFieldImage builtinImage() const;

  void setImageFieldBoundMin(const Vec2& min) { m_imageFieldBounds.min = min; }
  void setImageFieldBoundMax(const Vec2& max) { m_imageFieldBounds.max = max; }
  const Rect& imageFieldBounds() const { return m_imageFieldBounds; }

  const Measurement2d& fieldSize() const { return m_fieldSize; }

  bool operator==(const ThunderAutoFieldImage& other) const noexcept = default;
};

}  // namespace thunder::core
