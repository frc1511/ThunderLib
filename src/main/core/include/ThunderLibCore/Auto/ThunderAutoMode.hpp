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

  virtual std::unique_ptr<ThunderAutoModeStep> clone() const = 0;
};

bool operator==(const ThunderAutoModeStep& lhs, const ThunderAutoModeStep& rhs) noexcept;
bool operator==(const std::unique_ptr<ThunderAutoModeStep>& lhs,
                const std::unique_ptr<ThunderAutoModeStep>& rhs) noexcept;

/**
 * A step that executes an action.
 */
struct ThunderAutoModeActionStep final : public ThunderAutoModeStep {
  ThunderAutoModeStepType type() const noexcept override { return ThunderAutoModeStepType::ACTION; }

  std::string actionName;

  ThunderAutoModeActionStep() = default;

  ThunderAutoModeActionStep(const ThunderAutoModeActionStep& other) = default;
  ThunderAutoModeActionStep& operator=(const ThunderAutoModeActionStep& other) noexcept = default;

  std::unique_ptr<ThunderAutoModeStep> clone() const override {
    return std::make_unique<ThunderAutoModeActionStep>(*this);
  }

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

  ThunderAutoModeTrajectoryStep() = default;

  ThunderAutoModeTrajectoryStep(const ThunderAutoModeTrajectoryStep& other) = default;
  ThunderAutoModeTrajectoryStep& operator=(const ThunderAutoModeTrajectoryStep& other) noexcept = default;

  std::unique_ptr<ThunderAutoModeStep> clone() const override {
    return std::make_unique<ThunderAutoModeTrajectoryStep>(*this);
  }

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

  std::list<std::unique_ptr<ThunderAutoModeStep>> trueBranch;
  std::list<std::unique_ptr<ThunderAutoModeStep>> elseBranch;
  std::string conditionName;

  ThunderAutoModeBoolBranchStep() = default;

  ThunderAutoModeBoolBranchStep(const ThunderAutoModeBoolBranchStep& other);
  ThunderAutoModeBoolBranchStep& operator=(const ThunderAutoModeBoolBranchStep& other) noexcept;

  std::unique_ptr<ThunderAutoModeStep> clone() const override {
    return std::make_unique<ThunderAutoModeBoolBranchStep>(*this);
  }

  bool operator==(const ThunderAutoModeBoolBranchStep& other) const noexcept {
    bool trueStepBranchesMatch = (trueBranch == other.trueBranch);
    bool elseStepBranchesMatch = (elseBranch == other.elseBranch);
    bool conditionNamesMatch = (conditionName == other.conditionName);

    return trueStepBranchesMatch && elseStepBranchesMatch && conditionNamesMatch;
  }
};

/**
 * A step that branches to another step based on a switch case.
 */
struct ThunderAutoModeSwitchBranchStep final : public ThunderAutoModeStep {
  ThunderAutoModeStepType type() const noexcept override { return ThunderAutoModeStepType::BRANCH_SWITCH; }

  std::map<int, std::list<std::unique_ptr<ThunderAutoModeStep>>> caseBranches;
  std::list<std::unique_ptr<ThunderAutoModeStep>> defaultBranch;
  std::string conditionName;

  ThunderAutoModeSwitchBranchStep() = default;

  ThunderAutoModeSwitchBranchStep(const ThunderAutoModeSwitchBranchStep& other);
  ThunderAutoModeSwitchBranchStep& operator=(const ThunderAutoModeSwitchBranchStep& other) noexcept;

  std::unique_ptr<ThunderAutoModeStep> clone() const override {
    return std::make_unique<ThunderAutoModeSwitchBranchStep>(*this);
  }

  bool operator==(const ThunderAutoModeSwitchBranchStep& other) const noexcept {
    bool caseBranchesMatch = (caseBranches == other.caseBranches);
    bool defaultBranchesMatch = (defaultBranch == other.defaultBranch);
    bool conditionNamesMatch = (conditionName == other.conditionName);

    return caseBranchesMatch && defaultBranchesMatch && conditionNamesMatch;
  }
};

/**
 * Represents an auto mode, which is a sequence of steps that the robot
 * executes.
 */
struct ThunderAutoMode final {
  std::list<std::unique_ptr<ThunderAutoModeStep>> steps;

  ThunderAutoMode() = default;

  ThunderAutoMode(const ThunderAutoMode& other);
  ThunderAutoMode& operator=(const ThunderAutoMode& other) noexcept;

  bool operator==(const ThunderAutoMode& other) const noexcept = default;
};

void to_json(wpi::json& json, const ThunderAutoMode& mode);
void from_json(const wpi::json& json, ThunderAutoMode& mode);

}  // namespace thunder::core
