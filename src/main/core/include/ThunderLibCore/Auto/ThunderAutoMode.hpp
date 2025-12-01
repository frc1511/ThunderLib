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

  int getID() const noexcept { return m_id; }

 protected:
  ThunderAutoModeStep();

  ThunderAutoModeStep(const ThunderAutoModeStep& other) = default;
  ThunderAutoModeStep& operator=(const ThunderAutoModeStep& other) noexcept = default;

 private:
  int m_id;
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

  bool editorDisplayTrueBranch = true;

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

  bool editorDisplayDefaultBranch = true;
  int editorDisplayCaseBranch = 0;

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

    explicit Node(DirectoryType dirType);

    bool operator==(const Node& other) const noexcept {
      if (directoryType != other.directoryType) {
        return false;
      }
      if (directoryType == DirectoryType::SWITCH_CASE && caseBranchValue != other.caseBranchValue) {
        return false;
      }
      return stepIndex == other.stepIndex;
    }

    /**
     * Returns true if this node is in the same directory as the other node (essentially the same as == but
     * ignores stepIndex).
     *
     * @param other The other node to compare.
     * @return True or false.
     */
    bool isSameDirectoryAs(const Node& other) const noexcept;

    /**
     * Returns a new node representing the previous step in the directory.
     *
     * @return The previous node.
     */
    Node prev() const;

    /**
     * Returns a new node representing the next step in the directory.
     *
     * @return The next node.
     */
    Node next() const;
  };

  std::vector<Node> path;

  bool operator==(const ThunderAutoModeStepPath& other) const noexcept = default;

  /**
   * Returns the parent path of this path.
   *
   * @return The parent path.
   */
  ThunderAutoModeStepPath parentPath() const;

  /**
   * Returns true if this path has the other path as its parent path.
   *
   * @param other The other path to check.
   * @return True or false.
   */
  bool hasParentPath(const ThunderAutoModeStepPath& other) const noexcept;

  /**
   * Returns true if this path is in the same directory as the other path.
   *
   * @param other The other path to check.
   * @return True or false.
   */
  bool isInSameDirectoryAs(const ThunderAutoModeStepPath& other) const noexcept;

  /**
   * Appends a node to the path and returns a new path.
   *
   * @param node The node to append.
   * @return The new path.
   */
  ThunderAutoModeStepPath operator/(Node node) const;

  /**
   * Appends a node to the path.
   *
   * @param node The node to append.
   * @return Reference to this path.
   */
  ThunderAutoModeStepPath& operator/=(Node node);

  /**
   * Returns a reference to the end node of the path.
   *
   * @return Reference to the end node.
   */
  Node& endNode() { return path.back(); }
  const Node& endNode() const { return path.back(); }
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
   * Gets a reference to the step at the given path. Throws an exception if the path is invalid.
   *
   * @param stepPath The path to the step.
   *
   * @return A reference to the step.
   */
  ThunderAutoModeStep& getStepAtPath(const ThunderAutoModeStepPath& stepPath);

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
