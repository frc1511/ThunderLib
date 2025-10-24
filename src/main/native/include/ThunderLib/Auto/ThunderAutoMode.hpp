#pragma once

namespace thunder {

class ThunderAutoProject;

namespace driver {

class ThunderAutoMode;

}  // namespace driver

class ThunderAutoMode final {
 public:
  ~ThunderAutoMode();

  ThunderAutoMode(const ThunderAutoMode&) = delete;
  ThunderAutoMode& operator=(const ThunderAutoMode&) = delete;
  ThunderAutoMode(ThunderAutoMode&&) noexcept = delete;
  ThunderAutoMode& operator=(ThunderAutoMode&&) noexcept = delete;

  bool isValid() const noexcept;

  // TODO: Methods

  driver::ThunderAutoMode* getHandle() noexcept;

 private:
  friend class ThunderAutoProject;

  // Ownership is transferred to the constructed object.
  explicit ThunderAutoMode(driver::ThunderAutoMode* handle) noexcept;

 private:
  driver::ThunderAutoMode* m_handle = nullptr;
};

}  // namespace thunder
