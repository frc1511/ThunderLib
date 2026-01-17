#include <ThunderLibDriver/Auto/ThunderAutoMode.hpp>
#include <ThunderLibDriver/Auto/ThunderAutoProject.hpp>
#include <ThunderLibCore/Error.hpp>
#include <ThunderLibDriver/Logger.hpp>

namespace thunder::driver {

ThunderAutoModeStep::ThunderAutoModeStep(ThunderAutoModeStepType type,
                                         const std::string& itemName,
                                         const core::ThunderAutoModeStepPath& path) noexcept
    : m_type(type), m_itemName(itemName), m_path(path) {}

ThunderAutoModeStep::ThunderAutoModeStep(const core::ThunderAutoModeStep* step,
                                         const core::ThunderAutoModeStepPath& path) noexcept
    : m_path(path) {
  if (!step) {
    return;
  }

  core::ThunderAutoModeStepType type = step->type();
  switch (type) {
    case core::ThunderAutoModeStepType::ACTION: {
      m_type = ThunderAutoModeStepType::ACTION;
      const auto* actionStep = reinterpret_cast<const core::ThunderAutoModeActionStep*>(step);
      m_itemName = actionStep->actionName;
      break;
    }
    case core::ThunderAutoModeStepType::TRAJECTORY: {
      m_type = ThunderAutoModeStepType::TRAJECTORY;
      const auto* trajectoryStep = reinterpret_cast<const core::ThunderAutoModeTrajectoryStep*>(step);
      m_itemName = trajectoryStep->trajectoryName;
      break;
    }
    case core::ThunderAutoModeStepType::BRANCH_BOOL: {
      m_type = ThunderAutoModeStepType::BRANCH_BOOL;
      const auto* branchStep = reinterpret_cast<const core::ThunderAutoModeBoolBranchStep*>(step);
      m_itemName = branchStep->conditionName;
      break;
    }
    case core::ThunderAutoModeStepType::BRANCH_SWITCH: {
      m_type = ThunderAutoModeStepType::BRANCH_SWITCH;
      const auto* branchStep = reinterpret_cast<const core::ThunderAutoModeSwitchBranchStep*>(step);
      m_itemName = branchStep->conditionName;
      break;
    }
    default:
      break;
  }
}

ThunderAutoMode::ThunderAutoMode(std::shared_ptr<core::ThunderAutoMode> autoMode) noexcept
    : m_autoMode(autoMode) {}

ThunderAutoModeStep* ThunderAutoMode::getFirstStep() const noexcept {
  if (!m_autoMode || m_autoMode->steps.empty())
    return nullptr;

  const std::unique_ptr<core::ThunderAutoModeStep>& step = m_autoMode->steps.front();
  return new ThunderAutoModeStep(step.get(), core::ThunderAutoModeStepPath(0));
}

ThunderAutoModeStep* ThunderAutoMode::getNextStep(ThunderAutoModeStep* previousStep) const noexcept {
  if (!m_autoMode || !previousStep)
    return nullptr;

  const core::ThunderAutoModeStepPath& previousStepPath = previousStep->getPath();

  core::ThunderAutoModeStepDirectoryPath directoryPath = previousStepPath.directoryPath();
  const core::ThunderAutoMode::StepDirectory* directory;
  try {
    directory = &m_autoMode->findStepDirectoryAtPath(directoryPath);
  } catch (core::ThunderError& e) {  // Should never happen when funciton used correctly, but just in case.
    ThunderLibLogger::Error("Invalid previous step path ({}): {}",
                            core::ThunderAutoModeStepPathToString(previousStepPath), e.what());
    return nullptr;
  }
  size_t previousStepIndex = previousStepPath.stepIndex();

  while (previousStepIndex + 1 >= directory->size()) {
    // Hit the root directory, so this was the final step in the auto mode.
    if (directoryPath.depth() == 0) {
      return nullptr;
    }

    // Go up a directory.
    previousStepIndex = directoryPath.path().back().stepIndex;
    directoryPath = directoryPath.parentPath();
    directory = &m_autoMode->findStepDirectoryAtPath(directoryPath);
  }

  core::ThunderAutoModeStepPath nextStepPath = directoryPath.step(previousStepIndex + 1);
  auto nextStepIt = std::next(directory->begin(), previousStepIndex + 1);

  const core::ThunderAutoModeStep* nextStep = nextStepIt->get();
  return new ThunderAutoModeStep(nextStep, nextStepPath);
}

ThunderAutoModeStep* ThunderAutoMode::getFirstStepOfBranch(ThunderAutoModeStep* branchStep,
                                                           bool booleanCondition) const noexcept {
  if (!m_autoMode || !branchStep)
    return nullptr;

  if (branchStep->getType() != ThunderAutoModeStepType::BRANCH_BOOL)
    return nullptr;

  const core::ThunderAutoModeStepPath& branchStepPath = branchStep->getPath();

  core::ThunderAutoModeStepDirectoryPath branchDirectoryPath = branchStepPath.boolBranch(booleanCondition);
  const core::ThunderAutoMode::StepDirectory* branchDirectory;
  try {
    branchDirectory = &m_autoMode->findStepDirectoryAtPath(branchDirectoryPath);
  } catch (core::ThunderError& e) {  // Should never happen when funciton used correctly, but just in case.
    ThunderLibLogger::Error("Invalid branch step path ({}): {}",
                            core::ThunderAutoModeStepPathToString(branchStepPath), e.what());
    return nullptr;
  }

  core::ThunderAutoModeStepPath stepPath = branchDirectoryPath.step(0);
  if (branchDirectory->empty()) {
    return nullptr;
  }
  const core::ThunderAutoModeStep* step = branchDirectory->front().get();
  return new ThunderAutoModeStep(step, stepPath);
}

ThunderAutoModeStep* ThunderAutoMode::getFirstStepOfBranch(ThunderAutoModeStep* driverBranchStep,
                                                           int switchCondition) const noexcept {
  if (!m_autoMode || !driverBranchStep)
    return nullptr;

  if (driverBranchStep->getType() != ThunderAutoModeStepType::BRANCH_SWITCH)
    return nullptr;

  const core::ThunderAutoModeStepPath& branchStepPath = driverBranchStep->getPath();
  const core::ThunderAutoModeStep* branchStep;

  try {
    const auto& [branchStepDirectory, branchStepIt] = m_autoMode->findStepAtPath(branchStepPath);
    branchStep = branchStepIt->get();

  } catch (core::ThunderError& e) {  // Should never happen when funciton used correctly, but just in case.
    ThunderLibLogger::Error("Invalid branch step path ({}): {}",
                            core::ThunderAutoModeStepPathToString(branchStepPath), e.what());
    return nullptr;
  }

  if (branchStep->type() != core::ThunderAutoModeStepType::BRANCH_SWITCH)
    return nullptr;

  const auto* switchBranchStep = reinterpret_cast<const core::ThunderAutoModeSwitchBranchStep*>(branchStep);

  core::ThunderAutoModeStepPath stepPath(0);
  const core::ThunderAutoModeStep* step;

  auto caseBranchIt = switchBranchStep->caseBranches.find(switchCondition);
  if (caseBranchIt == switchBranchStep->caseBranches.end()) {  // Default
    const core::ThunderAutoMode::StepDirectory& defaultBranchDirectory = switchBranchStep->defaultBranch;
    if (defaultBranchDirectory.empty()) {
      return nullptr;
    }
    stepPath = branchStepPath.switchBranchDefault().step(0);
    step = defaultBranchDirectory.front().get();

  } else {  // Case
    const core::ThunderAutoMode::StepDirectory& caseBranchDirectory = caseBranchIt->second;
    if (caseBranchDirectory.empty()) {
      return nullptr;
    }
    stepPath = branchStepPath.switchBranchCase(switchCondition).step(0);
    step = caseBranchDirectory.front().get();
  }

  return new ThunderAutoModeStep(step, stepPath);
}

bool ThunderAutoMode::isRunnable(const ThunderAutoProject& project) const noexcept {
  if (!m_autoMode || !project.isLoaded())
    return false;

  core::ThunderAutoModeStepTrajectoryBehavior behavior =
      m_autoMode->getTrajectoryBehavior(project.getTrajectorySkeletons());

  bool hasError = (bool)behavior.errorInfo;
  return !hasError;
}

}  // namespace thunder::driver
