#pragma once

#include <wpi/json.h>
#include <string>
#include <map>
#include <memory>
#include <list>
#include <utility>

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

const char* ThunderAutoModeStepTypeToString(ThunderAutoModeStepType type) noexcept;

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
 * Represents a path to a specific step within an auto mode.
 */
struct ThunderAutoModeStepPath {
  struct Node {
    enum class DirectoryType {
      ROOT,
      BOOL_TRUE,
      BOOL_ELSE,
      SWITCH_CASE,
      SWITCH_DEFAULT,
    } directoryType = DirectoryType::ROOT;

    int caseBranchValue = 0;

    size_t stepIndex = 0;

    Node() = default;

    explicit Node(DirectoryType dirType)
        : directoryType(dirType) {}

    bool operator==(const Node& other) const noexcept {
      if (directoryType != other.directoryType) {
        return false;
      }
      if (directoryType == DirectoryType::SWITCH_CASE && caseBranchValue != other.caseBranchValue) {
        return false;
      }
      return stepIndex == other.stepIndex;
    }

    bool isSameDirectoryAs(const Node& other) const noexcept {
      if (directoryType != other.directoryType) {
        return false;
      }
      if (directoryType == DirectoryType::SWITCH_CASE && caseBranchValue != other.caseBranchValue) {
        return false;
      }
      return true;
    }

    Node prev() const {
      Node prev = *this;
      --prev.stepIndex;
      return prev;
    }

    Node next() const {
      Node next = *this;
      ++next.stepIndex;
      return next;
    }
  };

  std::vector<Node> path;

  bool operator==(const ThunderAutoModeStepPath& other) const noexcept = default;

  ThunderAutoModeStepPath parentPath() const {
    ThunderAutoModeStepPath parent = *this;
    if (!parent.path.empty()) {
      parent.path.pop_back();
    }
    return parent;
  }

  bool hasParentPath(const ThunderAutoModeStepPath& other) const noexcept {
    if (other.path.size() >= path.size()) {
      return false;
    }

    for (size_t i = 0; i < other.path.size(); ++i) {
      if (path[i] != other.path[i]) {
        return false;
      }
    }

    return true;
  }

  bool isInSameDirectoryAs(const ThunderAutoModeStepPath& other) const noexcept {
    if (other.path.size() != path.size()) {
      return false;
    }

    for (size_t i = 0; i < path.size() - 1; ++i) {
      if (path[i] != other.path[i]) {
        return false;
      }
    }

    return path.back().isSameDirectoryAs(other.path.back());
  }

  ThunderAutoModeStepPath operator/(Node node) const {
    ThunderAutoModeStepPath newPath = *this;
    newPath.path.push_back(node);
    return newPath;
  }

  ThunderAutoModeStepPath& operator/=(Node node) {
    path.push_back(node);
    return *this;
  }

  Node& lastNode() { return path.back(); }

  const Node& lastNode() const { return path.back(); }
};

std::string ThunderAutoModeStepPathToString(const ThunderAutoModeStepPath& stepPath);

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

  using StepDirectory = std::list<std::unique_ptr<ThunderAutoModeStep>>;

  using StepPosition = std::pair<StepDirectory*, StepDirectory::iterator>;

  /**
   * Finds a step at the given path. Throws an exception if the path is invalid.
   *
   * @param stepPath The path to the step.
   *
   * @return A pair containing a pointer to the step's directory and an iterator to the step in that
   * directory.
   */
  StepPosition findStepAtPath(const ThunderAutoModeStepPath& stepPath);

  /**
   * Finds the step directory at the given path. Throws an exception if the path is invalid.
   *
   * @param stepPath The path to the step directory.
   *
   * @return A reference to the step directory.
   */
  StepDirectory& findStepDirectoryAtPath(const ThunderAutoModeStepPath& stepPath);
};

void to_json(wpi::json& json, const ThunderAutoMode& mode);
void from_json(const wpi::json& json, ThunderAutoMode& mode);

}  // namespace thunder::core
