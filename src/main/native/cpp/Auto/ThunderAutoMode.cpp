#include <ThunderLib/Auto/ThunderAutoMode.hpp>
#include <ThunderLibDriver/Auto/ThunderAutoMode.hpp>

namespace thunder {

ThunderAutoMode::ThunderAutoMode(driver::ThunderAutoMode* trajectory) noexcept : m_handle(trajectory) {}

ThunderAutoMode::~ThunderAutoMode() {
  if (m_handle) {
    delete m_handle;
  }
}

bool ThunderAutoMode::isValid() const noexcept {
  return m_handle != nullptr;
}

driver::ThunderAutoMode* ThunderAutoMode::getHandle() noexcept {
  return m_handle;
}

}  // namespace thunder
