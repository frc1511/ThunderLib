#pragma once

#include <string>
#include <memory>

namespace thunder {

class ThunderAutoProject;

namespace driver {

class ThunderAutoMode;
class ThunderAutoModeStep;

}  // namespace driver

class ThunderAutoMode;

enum class ThunderAutoModeStepType {
  UNKNOWN,
  ACTION,
  TRAJECTORY,
  BRANCH_BOOL,
  BRANCH_SWITCH,
};

class ThunderAutoModeStep final {
 public:
  ~ThunderAutoModeStep();

  ThunderAutoModeStep(const ThunderAutoModeStep&) = delete;
  ThunderAutoModeStep& operator=(const ThunderAutoModeStep&) = delete;
  ThunderAutoModeStep(ThunderAutoModeStep&&) noexcept = delete;
  ThunderAutoModeStep& operator=(ThunderAutoModeStep&&) noexcept = delete;

  bool isValid() const noexcept;

  ThunderAutoModeStepType type() const noexcept;

  std::string getActionName() const noexcept;
  std::string getTrajectoryName() const noexcept;
  std::string getConditionName() const noexcept;

  driver::ThunderAutoModeStep* getHandle() noexcept;
  const driver::ThunderAutoModeStep* getHandle() const noexcept;

 private:
  friend class ThunderAutoMode;

  // Ownership is transferred to the constructed object.
  explicit ThunderAutoModeStep(driver::ThunderAutoModeStep* handle) noexcept;

 private:
  driver::ThunderAutoModeStep* m_handle;
};

class ThunderAutoMode final {
 public:
  ~ThunderAutoMode();

  ThunderAutoMode(const ThunderAutoMode&) = delete;
  ThunderAutoMode& operator=(const ThunderAutoMode&) = delete;
  ThunderAutoMode(ThunderAutoMode&&) noexcept = delete;
  ThunderAutoMode& operator=(ThunderAutoMode&&) noexcept = delete;

  bool isValid() const noexcept;

  std::shared_ptr<ThunderAutoModeStep> getFirstStep();
  std::shared_ptr<ThunderAutoModeStep> getNextStep(std::shared_ptr<ThunderAutoModeStep> previousStep);

  std::shared_ptr<ThunderAutoModeStep> getFirstStepOfBranch(std::shared_ptr<ThunderAutoModeStep> branchStep,
                                                            bool booleanCondition);
  std::shared_ptr<ThunderAutoModeStep> getFirstStepOfBranch(std::shared_ptr<ThunderAutoModeStep> branchStep,
                                                            int switchCondition);

  bool isRunnable(const ThunderAutoProject& project) const noexcept;

  driver::ThunderAutoMode* getHandle() noexcept;
  const driver::ThunderAutoMode* getHandle() const noexcept;

 private:
  friend class ThunderAutoProject;

  // Ownership is transferred to the constructed object.
  explicit ThunderAutoMode(driver::ThunderAutoMode* handle) noexcept;

 private:
  driver::ThunderAutoMode* m_handle = nullptr;
};

}  // namespace thunder
