#pragma once

#include <ThunderLibCore/Auto/ThunderAutoTrajectorySkeleton.hpp>
#include <wpi/json.h>
#include <string>
#include <map>
#include <memory>
#include <list>
#include <utility>
#include <optional>

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

struct ThunderAutoModeStepTrajectoryBehavior {
  bool runsTrajectory = false;

  struct {
    // Whether a trajectory used in the step was not found in the available trajectories map.
    bool isTrajectoryMissing = false;

    // Whether the step contains a non-continuous sequence of trajectories (end+start poses don't match).
    bool containsNonContinuousSequence = false;

    explicit operator bool() const noexcept { return isTrajectoryMissing || containsNonContinuousSequence; }
  } errorInfo;

  /**
   * The start and end poses of the trajectory(ies) run by this step.
   *
   * std::nullopt may signify the following, ordered by priority:
   * 1. No trajectory is run by this step.
   * 2. This step as a problem (see errorInfo).
   * 3. The trajectory has multiple start/end poses (e.g., a branch step).
   */
  std::optional<frc::Pose2d> startPose, endPose;

  /**
   * For branch steps only: the indices of the first and last steps that run trajectories, or std::nullopt if
   * none do.
   */
  std::optional<std::pair<size_t, size_t>> trajectoryStepRange;
};

struct ThunderAutoModeStepTrajectoryBehaviorTreeNode {
  ThunderAutoModeStepTrajectoryBehavior behavior;
  std::vector<ThunderAutoModeStepTrajectoryBehaviorTreeNode> childrenVec;
  std::map<int, ThunderAutoModeStepTrajectoryBehaviorTreeNode> childrenMap;
};

/**
 * Represents a step in a ThunderAuto mode.
 */
struct ThunderAutoModeStep {
  virtual ~ThunderAutoModeStep() = default;
  virtual ThunderAutoModeStepType type() const noexcept = 0;

  virtual std::unique_ptr<ThunderAutoModeStep> clone() const = 0;

  int getID() const noexcept { return m_id; }

  virtual ThunderAutoModeStepTrajectoryBehavior getTrajectoryBehavior(
      std::optional<frc::Pose2d> previousStepEndPose,
      const std::map<std::string, ThunderAutoTrajectorySkeleton>& trajectories) const noexcept {
    return getTrajectoryBehaviorTree(previousStepEndPose, trajectories).behavior;
  }

  virtual ThunderAutoModeStepTrajectoryBehaviorTreeNode getTrajectoryBehaviorTree(
      std::optional<frc::Pose2d> previousStepEndPose,
      const std::map<std::string, ThunderAutoTrajectorySkeleton>& trajectories) const noexcept = 0;

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

  ThunderAutoModeStepTrajectoryBehaviorTreeNode getTrajectoryBehaviorTree(
      std::optional<frc::Pose2d> previousStepEndPose,
      const std::map<std::string, ThunderAutoTrajectorySkeleton>& trajectories) const noexcept override {
    ThunderAutoModeStepTrajectoryBehaviorTreeNode treeNode;
    treeNode.behavior.startPose = treeNode.behavior.endPose = previousStepEndPose;
    return treeNode;
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

  ThunderAutoModeStepTrajectoryBehavior getTrajectoryBehavior(
      std::optional<frc::Pose2d> previousStepEndPose,
      const std::map<std::string, ThunderAutoTrajectorySkeleton>& trajectories) const noexcept override;

  ThunderAutoModeStepTrajectoryBehaviorTreeNode getTrajectoryBehaviorTree(
      std::optional<frc::Pose2d> previousStepEndPose,
      const std::map<std::string, ThunderAutoTrajectorySkeleton>& trajectories) const noexcept override {
    ThunderAutoModeStepTrajectoryBehaviorTreeNode treeNode;
    treeNode.behavior = getTrajectoryBehavior(previousStepEndPose, trajectories);
    return treeNode;
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

  ThunderAutoModeStepTrajectoryBehaviorTreeNode getTrajectoryBehaviorTree(
      std::optional<frc::Pose2d> previousStepEndPose,
      const std::map<std::string, ThunderAutoTrajectorySkeleton>& trajectories) const noexcept override;
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

  ThunderAutoModeStepTrajectoryBehaviorTreeNode getTrajectoryBehaviorTree(
      std::optional<frc::Pose2d> previousStepEndPose,
      const std::map<std::string, ThunderAutoTrajectorySkeleton>& trajectories) const noexcept override;
};

class ThunderAutoModeStepPath;

class ThunderAutoModeStepDirectoryPath {
 public:
  struct Node {
    size_t stepIndex = 0;

    enum class Type {
      BOOL_TRUE,
      BOOL_ELSE,
      SWITCH_CASE,
      SWITCH_DEFAULT,
    } directoryType;

    int caseBranchValue = 0;

    bool operator==(const Node& other) const noexcept {
      if (stepIndex != other.stepIndex) {
        return false;
      }
      if (directoryType != other.directoryType) {
        return false;
      }
      if (directoryType == Type::SWITCH_CASE && caseBranchValue != other.caseBranchValue) {
        return false;
      }
      return true;
    }
  };

 private:
  std::vector<Node> m_path;

 public:
  ThunderAutoModeStepDirectoryPath() = default;
  explicit ThunderAutoModeStepDirectoryPath(std::vector<Node>&& path) : m_path(std::move(path)) {}

  bool operator==(const ThunderAutoModeStepDirectoryPath& other) const noexcept = default;

  size_t depth() const noexcept { return m_path.size(); }

  std::span<const Node> path() const noexcept { return m_path; }
  std::span<Node> path() noexcept { return m_path; }

  ThunderAutoModeStepDirectoryPath parentPath() const noexcept;

  bool hasParentPath(const ThunderAutoModeStepDirectoryPath& path) const noexcept;
  bool hasParentPath(const ThunderAutoModeStepPath& path) const noexcept;

  ThunderAutoModeStepPath step(int stepIndex) const noexcept;

  size_t getCommonPathDepth(const ThunderAutoModeStepDirectoryPath& otherPath) const noexcept;
  size_t getCommonPathDepth(const ThunderAutoModeStepPath& otherPath,
                            bool matchEndStepsWithDirNodes = true) const noexcept;
};

// Directory + End Node
class ThunderAutoModeStepPath {
  ThunderAutoModeStepDirectoryPath m_directory;
  size_t m_stepIndex = 0;

 public:
  explicit ThunderAutoModeStepPath(size_t stepIndex) : m_stepIndex(stepIndex) {}

  ThunderAutoModeStepPath(const ThunderAutoModeStepDirectoryPath& directory, size_t stepIndex)
      : m_directory(directory), m_stepIndex(stepIndex) {}

  bool operator==(const ThunderAutoModeStepPath& other) const noexcept = default;

  const ThunderAutoModeStepDirectoryPath& directoryPath() const noexcept { return m_directory; }
  ThunderAutoModeStepDirectoryPath& directoryPath() noexcept { return m_directory; }

  size_t stepIndex() const noexcept { return m_stepIndex; }
  void setStepIndex(size_t stepIndex) noexcept { m_stepIndex = stepIndex; }

  size_t depth() const noexcept { return m_directory.depth() + 1; }

  bool hasParentPath(const ThunderAutoModeStepDirectoryPath& path) const noexcept;
  bool hasParentPath(const ThunderAutoModeStepPath& path) const noexcept;

  void updateWithRemovalOfStep(const ThunderAutoModeStepPath& removedStepPath);

  ThunderAutoModeStepDirectoryPath boolBranch(bool b) const noexcept;
  ThunderAutoModeStepDirectoryPath switchBranchCase(int caseValue) const noexcept;
  ThunderAutoModeStepDirectoryPath switchBranchDefault() const noexcept;

  size_t getCommonPathDepth(const ThunderAutoModeStepDirectoryPath& otherPath,
                            bool matchEndStepsWithDirNodes = true) const noexcept;
  size_t getCommonPathDepth(const ThunderAutoModeStepPath& otherPath,
                            bool matchEndStepsWithDirNodes = true) const noexcept;
};

std::string ThunderAutoModeStepDirectoryPathToString(const ThunderAutoModeStepDirectoryPath& directory);
std::string ThunderAutoModeStepPathToString(const ThunderAutoModeStepPath& path);

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
  StepDirectory& findStepDirectoryAtPath(const ThunderAutoModeStepDirectoryPath& stepDirectoryPath);

  ThunderAutoModeStepTrajectoryBehavior getTrajectoryBehavior(
      const std::map<std::string, ThunderAutoTrajectorySkeleton>& trajectories) const noexcept;

  ThunderAutoModeStepTrajectoryBehaviorTreeNode getTrajectoryBehaviorTree(
      const std::map<std::string, ThunderAutoTrajectorySkeleton>& trajectories) const noexcept;
};

void to_json(wpi::json& json, const ThunderAutoMode& mode);
void from_json(const wpi::json& json, ThunderAutoMode& mode);

}  // namespace thunder::core
