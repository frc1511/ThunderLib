#include <ThunderLibCore/Auto/ThunderAutoFieldImage.hpp>

#include <ThunderLibCore/Error.hpp>

namespace thunder::core {

static const Rect s_imageFieldBounds[]{
    // 2025
    Rect(0.0795935557, 0.1139555361, 1.0 - 0.0795935557, 1.0 - 0.1139555361),

    // 2024
    Rect(0.0800781250, 0.0786132813, 0.9199218750, 0.90625),

    // 2023
    Rect(0.0280000009, 0.0188442469, 0.9739999771, 0.9825640917),

    // 2022
    Rect(0.12, 0.16, 0.88, 0.84),

    // INVALID
    Rect(0.0, 0.0, 0.0, 0.0),
};

static const Measurement2d s_fieldSizes[]{
    // 2025
    Measurement2d(17.548249_m, 8.077200_m),

    // 2022-2024 (TODO: Measure these better!)
    Measurement2d(16.54175_m, 8.0137_m),
    Measurement2d(16.54175_m, 8.0137_m),
    Measurement2d(16.54175_m, 8.0137_m),

    // INVALID
    Measurement2d(0.0_m, 0.0_m),
};

static constexpr const char* kProjectDirPlaceholder = "%PROJECT_DIR%";

const char* ThunderAutoBuiltinFieldImageToString(ThunderAutoBuiltinFieldImage builtin) {
  switch (builtin) {
    using enum ThunderAutoBuiltinFieldImage;
    case FIELD_2022:
      return "2022";
    case FIELD_2023:
      return "2023";
    case FIELD_2024:
      return "2024";
    case FIELD_2025:
      return "2025";
    default:
      ThunderLibUnreachable("Invalid Field Image Type");
  }
}

ThunderAutoBuiltinFieldImage StringToThunderAutoBuiltinFieldImage(std::string_view str) {
  if (str == "2022") {
    return ThunderAutoBuiltinFieldImage::FIELD_2022;
  } else if (str == "2023") {
    return ThunderAutoBuiltinFieldImage::FIELD_2023;
  } else if (str == "2024") {
    return ThunderAutoBuiltinFieldImage::FIELD_2024;
  } else if (str == "2025") {
    return ThunderAutoBuiltinFieldImage::FIELD_2025;
  }
  return ThunderAutoBuiltinFieldImage::INVALID;
}

ThunderAutoFieldImage::ThunderAutoFieldImage()
    : ThunderAutoFieldImage(ThunderAutoBuiltinFieldImage::LATEST) {}

ThunderAutoFieldImage::ThunderAutoFieldImage(ThunderAutoBuiltinFieldImage builtinImage)
    : m_image(builtinImage),
      m_imageFieldBounds(s_imageFieldBounds[(size_t)builtinImage]),
      m_fieldSize(s_fieldSizes[(size_t)builtinImage]) {}

ThunderAutoFieldImage::ThunderAutoFieldImage(const std::filesystem::path& imagePath,
                                             Rect rect,
                                             Measurement2d size)
    : m_image(imagePath), m_imageFieldBounds(rect), m_fieldSize(size) {}

ThunderAutoFieldImage::ThunderAutoFieldImage(const std::filesystem::path& imagePath,
                                             Vec2 min,
                                             Vec2 max,
                                             Measurement2d size)
    : ThunderAutoFieldImage(imagePath, Rect{min, max}, size) {}

ThunderAutoFieldImage::ThunderAutoFieldImage(const wpi::json& json,
                                             const std::filesystem::path& projectDirectory)
    : ThunderAutoFieldImage() {
  fromJson(json, projectDirectory);
}

ThunderAutoFieldImageType ThunderAutoFieldImage::type() const {
  if (std::holds_alternative<ThunderAutoBuiltinFieldImage>(m_image)) {
    return ThunderAutoFieldImageType::BUILTIN;
  } else if (std::holds_alternative<std::filesystem::path>(m_image)) {
    return ThunderAutoFieldImageType::CUSTOM;
  }
  ThunderLibUnreachable("Invalid ThunderAutoFieldImage type");
}

const std::filesystem::path& ThunderAutoFieldImage::customImagePath() const {
  return std::get<std::filesystem::path>(m_image);
}

ThunderAutoBuiltinFieldImage ThunderAutoFieldImage::builtinImage() const {
  return std::get<ThunderAutoBuiltinFieldImage>(m_image);
}

void ThunderAutoFieldImage::fromJson(const wpi::json& json, const std::filesystem::path& projectDirectory) {
  using enum ThunderAutoFieldImageType;

  size_t typeInt = json.at("img_type").get<size_t>();
  if (typeInt != (size_t)BUILTIN && typeInt != (size_t)CUSTOM) {
    throw RuntimeError::Construct("Invalid field image type: {}", typeInt);
  }

  ThunderAutoFieldImageType type = static_cast<ThunderAutoFieldImageType>(typeInt);

  std::string imageString = json.at("img").get<std::string>();

  if (type == BUILTIN) {
    ThunderAutoBuiltinFieldImage builtinImage = StringToThunderAutoBuiltinFieldImage(imageString);
    if (builtinImage == ThunderAutoBuiltinFieldImage::INVALID) {
      throw RuntimeError::Construct("Invalid built-in field image name: '{}'", imageString);
    }
    *this = ThunderAutoFieldImage(builtinImage);

  } else if (type == CUSTOM) {
    Rect imageRect;
    imageRect.min.x = json.at("min_x").get<double>();
    imageRect.min.y = json.at("min_y").get<double>();
    imageRect.max.x = json.at("max_x").get<double>();
    imageRect.max.y = json.at("max_y").get<double>();

    Measurement2d fieldSize;
    fieldSize.x = units::meter_t(json.at("field_size_x").get<double>());
    fieldSize.y = units::meter_t(json.at("field_size_y").get<double>());

    if (imageString.starts_with(kProjectDirPlaceholder)) {
      imageString.replace(0, strlen(kProjectDirPlaceholder), projectDirectory.string());
    }

    if (!std::filesystem::exists(imageString)) {
      throw RuntimeError::Construct("Custom field image file does not exist: {}", imageString);
    }

    *this = ThunderAutoFieldImage(imageString, imageRect, fieldSize);

  } else {
    ThunderLibUnreachable("Invalid Field Image Type");
  }
}

void ThunderAutoFieldImage::toJson(const std::filesystem::path& projectDirectory, wpi::json& json) const {
  using enum ThunderAutoFieldImageType;

  std::string imageString;

  ThunderAutoFieldImageType imageType = type();
  switch (imageType) {
    case BUILTIN:
      imageString = ThunderAutoBuiltinFieldImageToString(builtinImage());
      break;
    case CUSTOM: {
      imageString = customImagePath();

      std::string projectDirectoryStr = projectDirectory.string();
      if (imageString.starts_with(projectDirectoryStr)) {
        imageString.replace(0, projectDirectoryStr.length(), kProjectDirPlaceholder);
      }
    } break;
    default:
      ThunderLibUnreachable("Invalid Field Image Type");
  }

  json = wpi::json{
      {"img_type", static_cast<size_t>(imageType)},
      {"img", imageString},
  };

  if (imageType == CUSTOM) {
    json["min_x"] = m_imageFieldBounds.min.x;
    json["min_y"] = m_imageFieldBounds.min.y;
    json["max_x"] = m_imageFieldBounds.max.x;
    json["max_y"] = m_imageFieldBounds.max.y;
    json["field_size_x"] = m_fieldSize.x.value();
    json["field_size_y"] = m_fieldSize.y.value();
  }
}

}  // namespace thunder::core
