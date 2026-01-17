#include <ThunderLib/Auto/ThunderAutoMode.hpp>
#include <ThunderLib/Auto/ThunderAutoProject.hpp>
#include <ThunderLibDriver/Auto/ThunderAutoMode.hpp>

namespace thunder {

ThunderAutoModeStep::ThunderAutoModeStep(driver::ThunderAutoModeStep* handle) noexcept : m_handle(handle) {}

ThunderAutoModeStep::~ThunderAutoModeStep() {
  if (m_handle) {
    delete m_handle;
  }
}

bool ThunderAutoModeStep::isValid() const noexcept {
  return m_handle != nullptr && m_handle->getType() != driver::ThunderAutoModeStepType::UNKNOWN;
}

ThunderAutoModeStepType ThunderAutoModeStep::type() const noexcept {
  if (!isValid())
    return ThunderAutoModeStepType::UNKNOWN;

  return static_cast<ThunderAutoModeStepType>(m_handle->getType());
}

std::string ThunderAutoModeStep::getActionName() const noexcept {
  if (!isValid())
    return "";

  return m_handle->getItemName();
}

std::string ThunderAutoModeStep::getTrajectoryName() const noexcept {
  if (!isValid())
    return "";

  return m_handle->getItemName();
}

std::string ThunderAutoModeStep::getConditionName() const noexcept {
  if (!isValid())
    return "";

  return m_handle->getItemName();
}

driver::ThunderAutoModeStep* ThunderAutoModeStep::getHandle() noexcept {
  return m_handle;
}

const driver::ThunderAutoModeStep* ThunderAutoModeStep::getHandle() const noexcept {
  return m_handle;
}

ThunderAutoMode::ThunderAutoMode(driver::ThunderAutoMode* trajectory) noexcept : m_handle(trajectory) {}

ThunderAutoMode::~ThunderAutoMode() {
  if (m_handle) {
    delete m_handle;
  }
}

bool ThunderAutoMode::isValid() const noexcept {
  return m_handle != nullptr;
}

std::shared_ptr<ThunderAutoModeStep> ThunderAutoMode::getFirstStep() {
  if (!m_handle)
    return nullptr;

  driver::ThunderAutoModeStep* driverFirstStep = m_handle->getFirstStep();
  if (!driverFirstStep)
    return nullptr;
  ThunderAutoModeStep* firstStep = new ThunderAutoModeStep(driverFirstStep);
  return std::shared_ptr<ThunderAutoModeStep>(firstStep);
}

std::shared_ptr<ThunderAutoModeStep> ThunderAutoMode::getNextStep(
    std::shared_ptr<ThunderAutoModeStep> previousStep) {
  if (!m_handle)
    return nullptr;

  driver::ThunderAutoModeStep* driverNextStep = m_handle->getNextStep(previousStep->getHandle());
  if (!driverNextStep)
    return nullptr;
  ThunderAutoModeStep* nextStep = new ThunderAutoModeStep(driverNextStep);
  return std::shared_ptr<ThunderAutoModeStep>(nextStep);
}

std::shared_ptr<ThunderAutoModeStep> ThunderAutoMode::getFirstStepOfBranch(
    std::shared_ptr<ThunderAutoModeStep> branchStep,
    bool booleanCondition) {
  if (!m_handle)
    return nullptr;

  driver::ThunderAutoModeStep* driverFirstStep =
      m_handle->getFirstStepOfBranch(branchStep->getHandle(), booleanCondition);
  if (!driverFirstStep)
    return nullptr;
  ThunderAutoModeStep* firstStep = new ThunderAutoModeStep(driverFirstStep);
  return std::shared_ptr<ThunderAutoModeStep>(firstStep);
}

std::shared_ptr<ThunderAutoModeStep> ThunderAutoMode::getFirstStepOfBranch(
    std::shared_ptr<ThunderAutoModeStep> branchStep,
    int switchCondition) {
  if (!m_handle)
    return nullptr;

  driver::ThunderAutoModeStep* driverFirstStep =
      m_handle->getFirstStepOfBranch(branchStep->getHandle(), switchCondition);
  if (!driverFirstStep)
    return nullptr;
  ThunderAutoModeStep* firstStep = new ThunderAutoModeStep(driverFirstStep);
  return std::shared_ptr<ThunderAutoModeStep>(firstStep);
}

bool ThunderAutoMode::isRunnable(const ThunderAutoProject& project) const noexcept {
  if (!m_handle || !project.getHandle())
    return false;

  return m_handle->isRunnable(*project.getHandle());
}

driver::ThunderAutoMode* ThunderAutoMode::getHandle() noexcept {
  return m_handle;
}

const driver::ThunderAutoMode* ThunderAutoMode::getHandle() const noexcept {
  return m_handle;
}

}  // namespace thunder
