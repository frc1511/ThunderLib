#pragma once

#include <wpi/json.h>

#include <string>
#include <map>
#include <memory>
#include <list>

namespace thunder::core {

enum class ThunderAutoModeStepType {
  UNKNOWN,
  ACTION,
  TRAJECTORY,
  // PATHFIND,
  BRANCH_BOOL,
  BRANCH_SWITCH,
  // LOOP,
};

/**
 * Represents a step in a ThunderAuto mode.
 */
struct ThunderAutoModeStep {
  virtual ~ThunderAutoModeStep() = default;
  virtual ThunderAutoModeStepType type() const noexcept = 0;
};

bool operator==(const ThunderAutoModeStep& lhs, const ThunderAutoModeStep& rhs) noexcept;
bool operator==(const std::shared_ptr<ThunderAutoModeStep>& lhs,
                const std::shared_ptr<ThunderAutoModeStep>& rhs) noexcept;

/**
 * A step that executes an action.
 */
struct ThunderAutoModeActionStep final : public ThunderAutoModeStep {
  ThunderAutoModeStepType type() const noexcept override { return ThunderAutoModeStepType::ACTION; }

  std::string actionName;

  bool operator==(const ThunderAutoModeActionStep& other) const noexcept {
    bool actionNamesMatch = (actionName == other.actionName);
    return actionNamesMatch;
  }
};

/**
 * A step that runs a trajectory.
 */
struct ThunderAutoModeTrajectoryStep final : public ThunderAutoModeStep {
  ThunderAutoModeStepType type() const noexcept override { return ThunderAutoModeStepType::TRAJECTORY; }

  std::string trajectoryName;

  bool operator==(const ThunderAutoModeTrajectoryStep& other) const noexcept {
    bool trajectoryNamesMatch = (trajectoryName == other.trajectoryName);
    return trajectoryNamesMatch;
  }
};

/**
 * A step that branches to another step based on a boolean condition.
 */
struct ThunderAutoModeBoolBranchStep final : public ThunderAutoModeStep {
  ThunderAutoModeStepType type() const noexcept override { return ThunderAutoModeStepType::BRANCH_BOOL; }

  std::list<std::shared_ptr<ThunderAutoModeStep>> trueBranch;
  std::list<std::shared_ptr<ThunderAutoModeStep>> elseBranch;

  bool operator==(const ThunderAutoModeBoolBranchStep& other) const noexcept {
    bool trueStepBranchesMatch = (trueBranch == other.trueBranch);
    bool elseStepBranchesMatch = (elseBranch == other.elseBranch);

    return (trueStepBranchesMatch && elseStepBranchesMatch);
  }
};

/**
 * A step that branches to another step based on a switch case.
 */
struct ThunderAutoModeSwitchBranchStep final : public ThunderAutoModeStep {
  ThunderAutoModeStepType type() const noexcept override { return ThunderAutoModeStepType::BRANCH_SWITCH; }

  std::map<int, std::list<std::shared_ptr<ThunderAutoModeStep>>> caseBranches;
  std::list<std::shared_ptr<ThunderAutoModeStep>> defaultBranch;

  bool operator==(const ThunderAutoModeSwitchBranchStep& other) const noexcept {
    bool caseBranchesMatch = (caseBranches == other.caseBranches);
    bool defaultBranchesMatch = (defaultBranch == other.defaultBranch);

    return (caseBranchesMatch && defaultBranchesMatch);
  }
};

/**
 * Represents an auto mode, which is a sequence of steps that the robot
 * executes.
 */
struct ThunderAutoMode final {
  std::list<std::shared_ptr<ThunderAutoModeStep>> steps;

  bool operator==(const ThunderAutoMode& other) const noexcept = default;
};

void to_json(wpi::json& json, const ThunderAutoMode& mode);
void from_json(const wpi::json& json, ThunderAutoMode& mode);

}  // namespace thunder::core
