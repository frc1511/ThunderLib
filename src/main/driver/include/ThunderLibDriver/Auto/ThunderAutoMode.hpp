#pragma once

#include <ThunderLibCore/Auto/ThunderAutoMode.hpp>
#include <string>

namespace thunder::driver {

class ThunderAutoProject;

enum class ThunderAutoModeStepType {
  UNKNOWN,
  ACTION,
  TRAJECTORY,
  BRANCH_BOOL,
  BRANCH_SWITCH,
};

class ThunderAutoModeStep final {
 public:
  ThunderAutoModeStep(ThunderAutoModeStepType type,
                      const std::string& itemName,
                      const core::ThunderAutoModeStepPath& path) noexcept;

  ThunderAutoModeStep(const core::ThunderAutoModeStep* step,
                      const core::ThunderAutoModeStepPath& path) noexcept;

  ThunderAutoModeStepType getType() const noexcept { return m_type; }
  const std::string& getItemName() const noexcept { return m_itemName; }
  const core::ThunderAutoModeStepPath& getPath() const noexcept { return m_path; }

 private:
  ThunderAutoModeStepType m_type = ThunderAutoModeStepType::UNKNOWN;
  std::string m_itemName;
  core::ThunderAutoModeStepPath m_path;
};

class ThunderAutoMode final {
 public:
  explicit ThunderAutoMode(std::shared_ptr<core::ThunderAutoMode> autoMode) noexcept;

  ThunderAutoModeStep* getFirstStep() const noexcept;
  ThunderAutoModeStep* getNextStep(ThunderAutoModeStep* previousStep) const noexcept;

  ThunderAutoModeStep* getFirstStepOfBranch(ThunderAutoModeStep* branchStep,
                                            bool booleanCondition) const noexcept;
  ThunderAutoModeStep* getFirstStepOfBranch(ThunderAutoModeStep* branchStep,
                                            int switchCondition) const noexcept;

  bool isRunnable(const ThunderAutoProject& project) const noexcept;

 private:
  std::shared_ptr<core::ThunderAutoMode> m_autoMode;
};

}  // namespace thunder::driver
