#include <ThunderLibCore/Auto/ThunderAutoMode.hpp>
#include <ThunderLibCore/Error.hpp>
#include <ThunderLibCore/Math.hpp>
#include <fmt/format.h>
#include <random>
#include <queue>

namespace thunder::core {

const char* ThunderAutoModeStepTypeToString(ThunderAutoModeStepType type) noexcept {
  switch (type) {
    case ThunderAutoModeStepType::ACTION:
      return "Action";
    case ThunderAutoModeStepType::TRAJECTORY:
      return "Trajectory";
    case ThunderAutoModeStepType::BRANCH_BOOL:
      return "Boolean Condition";
    case ThunderAutoModeStepType::BRANCH_SWITCH:
      return "Switch Condition";
    default:
      return "Unknown";
  }
}

ThunderAutoModeStep::ThunderAutoModeStep() {
  static std::mt19937 generator{std::random_device{}()};
  static std::uniform_int_distribution<int> distribution{INT32_MIN, INT32_MAX};
  m_id = distribution(generator);
}

bool operator==(const ThunderAutoModeStep& lhs, const ThunderAutoModeStep& rhs) noexcept {
  if (lhs.type() != rhs.type()) {
    return false;
  }

  ThunderAutoModeStepType type = lhs.type();

  switch (type) {
    case ThunderAutoModeStepType::ACTION:
      return static_cast<const ThunderAutoModeActionStep&>(lhs) ==
             static_cast<const ThunderAutoModeActionStep&>(rhs);
    case ThunderAutoModeStepType::TRAJECTORY:
      return static_cast<const ThunderAutoModeTrajectoryStep&>(lhs) ==
             static_cast<const ThunderAutoModeTrajectoryStep&>(rhs);
    case ThunderAutoModeStepType::BRANCH_BOOL:
      return static_cast<const ThunderAutoModeBoolBranchStep&>(lhs) ==
             static_cast<const ThunderAutoModeBoolBranchStep&>(rhs);
    case ThunderAutoModeStepType::BRANCH_SWITCH:
      return static_cast<const ThunderAutoModeSwitchBranchStep&>(lhs) ==
             static_cast<const ThunderAutoModeSwitchBranchStep&>(rhs);
    default:
      ThunderLibCoreUnreachable("Unknown ThunderAutoModeStepType");
  }
}

bool operator==(const std::unique_ptr<ThunderAutoModeStep>& lhs,
                const std::unique_ptr<ThunderAutoModeStep>& rhs) noexcept {
  if (lhs.get() == rhs.get()) {
    return true;
  }
  if (!lhs || !rhs) {
    return false;
  }
  return (*lhs == *rhs);
}

ThunderAutoModeStepTrajectoryBehavior ThunderAutoModeTrajectoryStep::getTrajectoryBehavior(
    std::optional<frc::Pose2d> previousStepEndPose,
    const std::map<std::string, ThunderAutoTrajectorySkeleton>& trajectories) const noexcept {
  (void)previousStepEndPose;

  ThunderAutoModeStepTrajectoryBehavior behavior;
  auto it = trajectories.find(trajectoryName);
  if (it == trajectories.end()) {
    behavior.runsTrajectory = true;
    behavior.errorInfo.isTrajectoryMissing = true;
    return behavior;
  }

  ThunderAutoTrajectoryBehavior trajBehavior = it->second.getBehavior();
  behavior.runsTrajectory = true;
  behavior.startPose = trajBehavior.startPose;
  behavior.endPose = trajBehavior.endPose;

  return behavior;
}

static ThunderAutoModeStepTrajectoryBehaviorTreeNode GetStepSequenceTrajectoryBehaviorTree(
    std::optional<frc::Pose2d> previousStepEndPose,
    const std::list<std::unique_ptr<ThunderAutoModeStep>>& branch,
    const std::map<std::string, ThunderAutoTrajectorySkeleton>& trajectories) noexcept {
  ThunderAutoModeStepTrajectoryBehavior behavior;
  behavior.startPose = behavior.endPose = previousStepEndPose;
  std::vector<ThunderAutoModeStepTrajectoryBehaviorTreeNode> children;

  size_t firstTrajectoryStepIndex = 0, lastTrajectoryStepIndex = 0;

  int stepIndex = 0;
  for (auto it = branch.begin(); it != branch.end(); it++, stepIndex++) {
    const auto& step = *it;

    ThunderAutoModeStepTrajectoryBehaviorTreeNode stepBehaviorTreeNode =
        step->getTrajectoryBehaviorTree(behavior.endPose, trajectories);

    const ThunderAutoModeStepTrajectoryBehavior stepBehavior = stepBehaviorTreeNode.behavior;
    children.push_back(std::move(stepBehaviorTreeNode));

    // Ignore steps that don't run trajectories.
    if (!stepBehavior.runsTrajectory)
      continue;

    if (!behavior.runsTrajectory) {
      firstTrajectoryStepIndex = stepIndex;
    }
    lastTrajectoryStepIndex = stepIndex;

    if (behavior.errorInfo) {
      // Previous step had an error so skip processing further steps.
      // We never "break" in this loop because we still need to update lastTrajectoryStepIndex.
      continue;
    }

    bool previouslyRanTrajectory = behavior.runsTrajectory;
    behavior.runsTrajectory = true;

    if (stepBehavior.errorInfo) {
      behavior.errorInfo = stepBehavior.errorInfo;
      behavior.startPose = behavior.endPose = std::nullopt;
      continue;
    }

    if (previouslyRanTrajectory && !behavior.endPose.has_value()) {
      // Not possible to run another trajectory because the last step didn't end at a single pose.
      behavior.errorInfo.containsNonContinuousSequence = true;
      behavior.startPose = behavior.endPose = std::nullopt;
      continue;
    }

    if (stepBehavior.startPose.has_value()) {
      if (previouslyRanTrajectory || previousStepEndPose.has_value()) {
        // Check continuity between steps.
        if (!Pose2dEquals(behavior.endPose, stepBehavior.startPose.value())) {
          behavior.errorInfo.containsNonContinuousSequence = true;
          behavior.startPose = behavior.endPose = std::nullopt;
          continue;
        }

        behavior.endPose = stepBehavior.endPose;
      } else {
        behavior.startPose = stepBehavior.startPose;
        behavior.endPose = stepBehavior.endPose;
      }
    } else if (previouslyRanTrajectory || previousStepEndPose.has_value()) {
      // A trajectory was run in a previous step but this step's trajectory does not have a single start pose.
      behavior.errorInfo.containsNonContinuousSequence = true;
      behavior.startPose = behavior.endPose = std::nullopt;
      continue;
    } else {
      behavior.endPose = stepBehavior.endPose;
    }
  }

  if (behavior.runsTrajectory) {
    behavior.trajectoryStepRange = std::make_pair(firstTrajectoryStepIndex, lastTrajectoryStepIndex);
  }

  ThunderAutoModeStepTrajectoryBehaviorTreeNode root;
  root.behavior = behavior;
  root.childrenVec = std::move(children);
  return root;
}

ThunderAutoModeBoolBranchStep::ThunderAutoModeBoolBranchStep(const ThunderAutoModeBoolBranchStep& other)
    : ThunderAutoModeStep(other) {
  trueBranch.clear();
  for (const auto& step : other.trueBranch) {
    trueBranch.push_back(step->clone());
  }

  elseBranch.clear();
  for (const auto& step : other.elseBranch) {
    elseBranch.push_back(step->clone());
  }

  conditionName = other.conditionName;
  editorDisplayTrueBranch = other.editorDisplayTrueBranch;
}

ThunderAutoModeBoolBranchStep& ThunderAutoModeBoolBranchStep::operator=(
    const ThunderAutoModeBoolBranchStep& other) noexcept {
  ThunderAutoModeStep::operator=(other);
  if (this != &other) {
    trueBranch.clear();
    for (const auto& step : other.trueBranch) {
      trueBranch.push_back(step->clone());
    }

    elseBranch.clear();
    for (const auto& step : other.elseBranch) {
      elseBranch.push_back(step->clone());
    }

    conditionName = other.conditionName;
  }
  editorDisplayTrueBranch = other.editorDisplayTrueBranch;
  return *this;
}

ThunderAutoModeStepTrajectoryBehaviorTreeNode ThunderAutoModeBoolBranchStep::getTrajectoryBehaviorTree(
    std::optional<frc::Pose2d> previousStepEndPose,
    const std::map<std::string, ThunderAutoTrajectorySkeleton>& trajectories) const noexcept {
  ThunderAutoModeStepTrajectoryBehaviorTreeNode combinedBehaviorTree;
  combinedBehaviorTree.childrenMap[true] =
      GetStepSequenceTrajectoryBehaviorTree(previousStepEndPose, trueBranch, trajectories);

  combinedBehaviorTree.childrenMap[false] =
      GetStepSequenceTrajectoryBehaviorTree(previousStepEndPose, elseBranch, trajectories);

  ThunderAutoModeStepTrajectoryBehavior& trueBranchBehavior = combinedBehaviorTree.childrenMap[true].behavior;
  ThunderAutoModeStepTrajectoryBehavior& elseBranchBehavior =
      combinedBehaviorTree.childrenMap[false].behavior;
  ThunderAutoModeStepTrajectoryBehavior& combinedBehavior = combinedBehaviorTree.behavior;

  if (!trueBranchBehavior.runsTrajectory && !elseBranchBehavior.runsTrajectory) {
    combinedBehavior.runsTrajectory = false;
    combinedBehavior.startPose = combinedBehavior.endPose = previousStepEndPose;
    return combinedBehaviorTree;
  }
  combinedBehavior.runsTrajectory = true;

  auto trueErrorInfo = trueBranchBehavior.errorInfo;
  auto elseErrorInfo = elseBranchBehavior.errorInfo;

  if (trueErrorInfo || elseErrorInfo) {
    combinedBehavior.errorInfo.isTrajectoryMissing =
        trueErrorInfo.isTrajectoryMissing || elseErrorInfo.isTrajectoryMissing;
    combinedBehavior.errorInfo.containsNonContinuousSequence =
        trueErrorInfo.containsNonContinuousSequence || elseErrorInfo.containsNonContinuousSequence;
    return combinedBehaviorTree;
  }

  // If no trajectory gets run in either branch then the robot won't move from its last pose.
  if (!trueBranchBehavior.runsTrajectory) {
    /**
     * If this is the first step in the auto mode, then we assume that the branch which doesn't run a
     * trajectory will start wherever the other branch ends.
     */
    if (!previousStepEndPose.has_value()) {
      previousStepEndPose = elseBranchBehavior.endPose;
    }
    trueBranchBehavior.startPose = trueBranchBehavior.endPose = previousStepEndPose;
  }
  if (!elseBranchBehavior.runsTrajectory) {
    /**
     * If this is the first step in the auto mode, then we assume that the branch which doesn't run a
     * trajectory will start wherever the other branch ends.
     */
    if (!previousStepEndPose.has_value()) {
      previousStepEndPose = trueBranchBehavior.endPose;
    }
    elseBranchBehavior.startPose = elseBranchBehavior.endPose = previousStepEndPose;
  }

  if (trueBranchBehavior.startPose.has_value() && elseBranchBehavior.startPose.has_value() &&
      Pose2dEquals(trueBranchBehavior.startPose.value(), elseBranchBehavior.startPose.value())) {
    combinedBehavior.startPose = trueBranchBehavior.startPose;
  }

  if (trueBranchBehavior.endPose.has_value() && elseBranchBehavior.endPose.has_value() &&
      Pose2dEquals(trueBranchBehavior.endPose.value(), elseBranchBehavior.endPose.value())) {
    combinedBehavior.endPose = trueBranchBehavior.endPose;
  }

  return combinedBehaviorTree;
}

ThunderAutoModeSwitchBranchStep::ThunderAutoModeSwitchBranchStep(const ThunderAutoModeSwitchBranchStep& other)
    : ThunderAutoModeStep(other) {
  caseBranches.clear();
  for (const auto& [caseID, steps] : other.caseBranches) {
    std::list<std::unique_ptr<ThunderAutoModeStep>> clonedSteps;
    for (const auto& step : steps) {
      clonedSteps.push_back(step->clone());
    }
    caseBranches[caseID] = std::move(clonedSteps);
  }

  defaultBranch.clear();
  for (const auto& step : other.defaultBranch) {
    defaultBranch.push_back(step->clone());
  }

  conditionName = other.conditionName;

  editorDisplayDefaultBranch = other.editorDisplayDefaultBranch;
  editorDisplayCaseBranch = other.editorDisplayCaseBranch;
}

ThunderAutoModeSwitchBranchStep& ThunderAutoModeSwitchBranchStep::operator=(
    const ThunderAutoModeSwitchBranchStep& other) noexcept {
  ThunderAutoModeStep::operator=(other);
  if (this != &other) {
    caseBranches.clear();
    for (const auto& [caseID, steps] : other.caseBranches) {
      std::list<std::unique_ptr<ThunderAutoModeStep>> clonedSteps;
      for (const auto& step : steps) {
        clonedSteps.push_back(step->clone());
      }
      caseBranches[caseID] = std::move(clonedSteps);
    }

    defaultBranch.clear();
    for (const auto& step : other.defaultBranch) {
      defaultBranch.push_back(step->clone());
    }

    conditionName = other.conditionName;
  }
  editorDisplayDefaultBranch = other.editorDisplayDefaultBranch;
  editorDisplayCaseBranch = other.editorDisplayCaseBranch;
  return *this;
}

ThunderAutoModeStepTrajectoryBehaviorTreeNode ThunderAutoModeSwitchBranchStep::getTrajectoryBehaviorTree(
    std::optional<frc::Pose2d> previousStepEndPose,
    const std::map<std::string, ThunderAutoTrajectorySkeleton>& trajectories) const noexcept {
  ThunderAutoModeStepTrajectoryBehaviorTreeNode defaultBranchBehaviorTree =
      GetStepSequenceTrajectoryBehaviorTree(previousStepEndPose, defaultBranch, trajectories);

  ThunderAutoModeStepTrajectoryBehavior& defaultBranchBehavior = defaultBranchBehaviorTree.behavior;
  if (!defaultBranchBehavior.runsTrajectory) {
    defaultBranchBehavior.startPose = defaultBranchBehavior.endPose = previousStepEndPose;
  }

  ThunderAutoModeStepTrajectoryBehaviorTreeNode combinedBehaviorTree;
  combinedBehaviorTree.behavior = defaultBranchBehavior;
  combinedBehaviorTree.childrenVec.push_back(std::move(defaultBranchBehaviorTree));
  ThunderAutoModeStepTrajectoryBehavior& combinedBehavior = combinedBehaviorTree.behavior;

  std::optional<frc::Pose2d> endPose = defaultBranchBehavior.endPose;
  bool endPoseFound = endPose.has_value();

  for (const auto& [value, branch] : caseBranches) {
    combinedBehaviorTree.childrenMap[value] =
        GetStepSequenceTrajectoryBehaviorTree(previousStepEndPose, branch, trajectories);

    ThunderAutoModeStepTrajectoryBehavior& branchBehavior = combinedBehaviorTree.childrenMap[value].behavior;
    if (branchBehavior.runsTrajectory) {
      if (!endPoseFound) {
        endPose = branchBehavior.endPose;
        endPoseFound = true;
      } else if (!Pose2dEquals(endPose, branchBehavior.endPose)) {
        endPose = std::nullopt;
      }
    }
  }

  /**
   * If this is the first step in the auto mode, then we assume that branches which don't run trajectories
   * will start wherever the other branches end.
   */
  if (!previousStepEndPose.has_value()) {
    previousStepEndPose = endPose;
    combinedBehavior.endPose = endPose;
    if (!defaultBranchBehavior.runsTrajectory) {
      defaultBranchBehavior.endPose = endPose;
    }
  }

  for (auto& [value, branchBehaviorTree] : combinedBehaviorTree.childrenMap) {
    ThunderAutoModeStepTrajectoryBehavior& branchBehavior = branchBehaviorTree.behavior;

    combinedBehavior.runsTrajectory |= branchBehavior.runsTrajectory;

    if (branchBehavior.errorInfo) {
      combinedBehavior.errorInfo.isTrajectoryMissing |= branchBehavior.errorInfo.isTrajectoryMissing;
      combinedBehavior.errorInfo.containsNonContinuousSequence |=
          branchBehavior.errorInfo.containsNonContinuousSequence;

      combinedBehavior.startPose = combinedBehavior.endPose = std::nullopt;
      continue;
    }
    if (combinedBehavior.errorInfo) {
      continue;
    }

    // If no trajectory gets run in this branch, then the robot won't move from its last position.
    if (!branchBehavior.runsTrajectory) {
      branchBehavior.startPose = branchBehavior.endPose = previousStepEndPose;
    }

    if (!Pose2dEquals(combinedBehavior.startPose, branchBehavior.startPose)) {
      combinedBehavior.startPose = std::nullopt;
    }
    if (!Pose2dEquals(combinedBehavior.endPose, branchBehavior.endPose)) {
      combinedBehavior.endPose = std::nullopt;
    }
  }

  return combinedBehaviorTree;
}

ThunderAutoModeStepDirectoryPath ThunderAutoModeStepDirectoryPath::parentPath() const noexcept {
  if (depth() == 0) {
    return ThunderAutoModeStepDirectoryPath{};
  }

  std::vector<Node> path(m_path.begin(), std::prev(m_path.end()));
  return ThunderAutoModeStepDirectoryPath{std::move(path)};
}

bool ThunderAutoModeStepDirectoryPath::hasParentPath(
    const ThunderAutoModeStepDirectoryPath& path) const noexcept {
  const size_t pathDepth = path.depth();
  if (depth() <= pathDepth)
    return false;

  size_t commonPathDepth = getCommonPathDepth(path);
  return pathDepth == commonPathDepth;
}

bool ThunderAutoModeStepDirectoryPath::hasParentPath(const ThunderAutoModeStepPath& path) const noexcept {
  const size_t pathDepth = path.depth();
  if (depth() < pathDepth)  // Not <= because we consider steps to be parents of their branches,
                            // ex. /0/0 is parent to /0/0.true/
    return false;

  size_t commonPathDepth = getCommonPathDepth(path);
  return pathDepth == commonPathDepth;
}

ThunderAutoModeStepPath ThunderAutoModeStepDirectoryPath::step(int stepIndex) const noexcept {
  return ThunderAutoModeStepPath(*this, stepIndex);
}

size_t ThunderAutoModeStepDirectoryPath::getCommonPathDepth(
    const ThunderAutoModeStepDirectoryPath& otherPath) const noexcept {
  std::span<const Node> dir1 = m_path;
  std::span<const Node> dir2 = otherPath.path();

  const size_t maxDepth = std::min(dir1.size(), dir2.size());
  size_t commonPathDepth = 0;
  for (size_t i = 0; i < maxDepth; i++) {
    if (dir1[i] != dir2[i]) {
      return commonPathDepth;
    }
    commonPathDepth++;
  }
  return commonPathDepth;
}

size_t ThunderAutoModeStepDirectoryPath::getCommonPathDepth(const ThunderAutoModeStepPath& otherPath,
                                                            bool matchEndStepsWithDirNodes) const noexcept {
  std::span<const Node> dir1 = m_path;
  std::span<const Node> dir2 = otherPath.directoryPath().path();

  const size_t maxDepth = std::min(dir1.size(), dir2.size());
  size_t commonPathDepth = 0;
  for (size_t i = 0; i < maxDepth; i++) {
    if (dir1[i] != dir2[i]) {
      return commonPathDepth;
    }
    commonPathDepth++;
  }

  if (matchEndStepsWithDirNodes) {
    if ((otherPath.depth() == commonPathDepth + 1) && (commonPathDepth < dir1.size())) {
      if (otherPath.stepIndex() == dir1[commonPathDepth].stepIndex) {
        commonPathDepth++;
      }
    }
  }

  return commonPathDepth;
}

bool ThunderAutoModeStepPath::hasParentPath(const ThunderAutoModeStepDirectoryPath& path) const noexcept {
  const size_t pathDepth = path.depth();
  if (depth() <= pathDepth)
    return false;

  size_t commonPathDepth = getCommonPathDepth(path);
  return pathDepth == commonPathDepth;
}

bool ThunderAutoModeStepPath::hasParentPath(const ThunderAutoModeStepPath& path) const noexcept {
  const size_t pathDepth = path.depth();
  if (depth() <= pathDepth)
    return false;

  size_t commonPathDepth = getCommonPathDepth(path);
  return pathDepth == commonPathDepth;
}

void ThunderAutoModeStepPath::updateWithRemovalOfStep(const ThunderAutoModeStepPath& removedStepPath) {
  const size_t currentStepPathDepth = depth();
  const size_t removedStepPathDepth = removedStepPath.depth();
  ThunderLibCoreAssert(removedStepPathDepth > 0);

  const size_t commonPathDepth = getCommonPathDepth(removedStepPath, false);

  if (commonPathDepth == removedStepPathDepth) {
    throw LogicError::Construct("Cannot update step path because it is child to removed step path");
  }
  // Check if the common path is the parent directory of the removed step.
  else if (commonPathDepth == removedStepPathDepth - 1) {
    const size_t removedStepIndex = removedStepPath.stepIndex();

    ThunderLibCoreAssert(currentStepPathDepth >= commonPathDepth + 1);
    size_t& currentStepIndex = (currentStepPathDepth == commonPathDepth + 1)
                                   ? m_stepIndex
                                   : m_directory.path()[commonPathDepth].stepIndex;

    if (removedStepIndex < currentStepIndex) {
      currentStepIndex--;
    }
  }
}

ThunderAutoModeStepDirectoryPath ThunderAutoModeStepPath::boolBranch(bool b) const noexcept {
  using enum ThunderAutoModeStepDirectoryPath::Node::Type;

  ThunderAutoModeStepDirectoryPath::Node node{
      .stepIndex = m_stepIndex,
      .directoryType = b ? BOOL_TRUE : BOOL_ELSE,
  };

  std::span<const ThunderAutoModeStepDirectoryPath::Node> path = m_directory.path();
  std::vector<ThunderAutoModeStepDirectoryPath::Node> newPath(path.begin(), path.end());
  newPath.push_back(node);

  return ThunderAutoModeStepDirectoryPath(std::move(newPath));
}

ThunderAutoModeStepDirectoryPath ThunderAutoModeStepPath::switchBranchCase(int caseValue) const noexcept {
  ThunderAutoModeStepDirectoryPath::Node node{
      .stepIndex = m_stepIndex,
      .directoryType = ThunderAutoModeStepDirectoryPath::Node::Type::SWITCH_CASE,
      .caseBranchValue = caseValue,
  };

  std::span<const ThunderAutoModeStepDirectoryPath::Node> path = m_directory.path();
  std::vector<ThunderAutoModeStepDirectoryPath::Node> newPath(path.begin(), path.end());
  newPath.push_back(node);

  return ThunderAutoModeStepDirectoryPath(std::move(newPath));
}

ThunderAutoModeStepDirectoryPath ThunderAutoModeStepPath::switchBranchDefault() const noexcept {
  ThunderAutoModeStepDirectoryPath::Node node{
      .stepIndex = m_stepIndex,
      .directoryType = ThunderAutoModeStepDirectoryPath::Node::Type::SWITCH_DEFAULT,
  };

  std::span<const ThunderAutoModeStepDirectoryPath::Node> path = m_directory.path();
  std::vector<ThunderAutoModeStepDirectoryPath::Node> newPath(path.begin(), path.end());
  newPath.push_back(node);

  return ThunderAutoModeStepDirectoryPath(std::move(newPath));
}

size_t ThunderAutoModeStepPath::getCommonPathDepth(const ThunderAutoModeStepDirectoryPath& otherPath,
                                                   bool matchEndStepsWithDirNodes) const noexcept {
  return otherPath.getCommonPathDepth(*this, matchEndStepsWithDirNodes);
}

size_t ThunderAutoModeStepPath::getCommonPathDepth(const ThunderAutoModeStepPath& otherPath,
                                                   bool matchEndStepsWithDirNodes) const noexcept {
  std::span<const ThunderAutoModeStepDirectoryPath::Node> dir1 = m_directory.path();
  std::span<const ThunderAutoModeStepDirectoryPath::Node> dir2 = otherPath.directoryPath().path();

  const size_t maxDepth = std::min(dir1.size(), dir2.size());
  size_t commonPathDepth = 0;
  for (size_t i = 0; i < maxDepth; i++) {
    if (dir1[i] != dir2[i]) {
      return commonPathDepth;
    }
    commonPathDepth++;
  }

  if (dir1.size() == dir2.size()) {
    if (stepIndex() == otherPath.stepIndex()) {
      commonPathDepth++;
    }
  } else if (matchEndStepsWithDirNodes) {
    if (dir1.size() < dir2.size()) {
      if (stepIndex() == dir2[commonPathDepth].stepIndex) {
        commonPathDepth++;
      }
    } else if (dir1.size() > dir2.size()) {
      if (otherPath.stepIndex() == dir1[commonPathDepth].stepIndex) {
        commonPathDepth++;
      }
    }
  }

  return commonPathDepth;
}

std::string ThunderAutoModeStepDirectoryPathToString(const ThunderAutoModeStepDirectoryPath& directory) {
  std::string directoryStr;

  for (const ThunderAutoModeStepDirectoryPath::Node& node : directory.path()) {
    directoryStr += fmt::format("/{}", node.stepIndex);

    switch (node.directoryType) {
      using enum ThunderAutoModeStepDirectoryPath::Node::Type;
      case BOOL_TRUE:
        directoryStr += ".true";
        break;
      case BOOL_ELSE:
        directoryStr += ".false";
        break;
      case SWITCH_CASE:
        directoryStr += fmt::format(".case{}", node.caseBranchValue);
        break;
      case SWITCH_DEFAULT:
        directoryStr += ".default";
        break;
      default:
        ThunderLibCoreUnreachable("Invalid auto mode step directory path node directory type");
    }
  }

  return directoryStr;
}

std::string ThunderAutoModeStepPathToString(const ThunderAutoModeStepPath& path) {
  std::string pathStr = ThunderAutoModeStepDirectoryPathToString(path.directoryPath());
  pathStr += fmt::format("/{}", path.stepIndex());
  return pathStr;
}

ThunderAutoMode::ThunderAutoMode(const ThunderAutoMode& other) {
  steps.clear();
  for (const auto& step : other.steps) {
    steps.push_back(step->clone());
  }
}

ThunderAutoMode& ThunderAutoMode::operator=(const ThunderAutoMode& other) noexcept {
  if (this != &other) {
    steps.clear();
    for (const auto& step : other.steps) {
      steps.push_back(step->clone());
    }
  }
  return *this;
}

ThunderAutoMode::StepPosition ThunderAutoMode::findStepAtPath(const ThunderAutoModeStepPath& path) {
  StepDirectory& directory = findStepDirectoryAtPath(path.directoryPath());

  if (path.stepIndex() >= directory.size()) {
    throw InvalidArgumentError::Construct("Auto mode step path index {} is out of bounds (size {})",
                                          path.stepIndex(), directory.size());
  }

  StepDirectory::iterator stepIt = std::next(directory.begin(), path.stepIndex());

  return std::make_pair(&directory, stepIt);
}

ThunderAutoModeStep& ThunderAutoMode::getStepAtPath(const ThunderAutoModeStepPath& stepPath) {
  auto [stepsList, stepIt] = findStepAtPath(stepPath);
  return **stepIt;
}

ThunderAutoMode::StepDirectory& ThunderAutoMode::findStepDirectoryAtPath(
    const ThunderAutoModeStepDirectoryPath& path) {
  using enum ThunderAutoModeStepDirectoryPath::Node::Type;

  std::queue<ThunderAutoModeStepDirectoryPath::Node> nodesToVisit;
  for (const auto& node : path.path()) {
    nodesToVisit.push(node);
  }

  StepDirectory* currentDirectory = &steps;
  StepDirectory::iterator currentIt = currentDirectory->begin();

  while (!nodesToVisit.empty()) {
    ThunderAutoModeStepDirectoryPath::Node currentNode = nodesToVisit.front();
    nodesToVisit.pop();

    if (currentNode.stepIndex >= currentDirectory->size()) {
      throw InvalidArgumentError::Construct("Auto mode step path index {} is out of bounds (size {})",
                                            currentNode.stepIndex, currentDirectory->size());
    }

    currentIt = currentDirectory->begin();
    std::advance(currentIt, currentNode.stepIndex);

    ThunderAutoModeStepType stepType = (*currentIt)->type();
    switch (stepType) {
      using enum ThunderAutoModeStepType;
      case BRANCH_BOOL: {
        auto& branchBoolStep = static_cast<ThunderAutoModeBoolBranchStep&>(**currentIt);
        if (currentNode.directoryType == BOOL_TRUE) {
          currentDirectory = &branchBoolStep.trueBranch;
        } else if (currentNode.directoryType == BOOL_ELSE) {
          currentDirectory = &branchBoolStep.elseBranch;
        } else {
          throw InvalidArgumentError::Construct(
              "Auto mode step directory path node type {} is invalid for boolean branch",
              static_cast<int>(currentNode.directoryType));
        }
        break;
      }
      case BRANCH_SWITCH: {
        auto& branchSwitchStep = static_cast<ThunderAutoModeSwitchBranchStep&>(**currentIt);
        if (currentNode.directoryType == SWITCH_DEFAULT) {
          currentDirectory = &branchSwitchStep.defaultBranch;
        } else if (currentNode.directoryType == SWITCH_CASE) {
          auto caseIt = branchSwitchStep.caseBranches.find(currentNode.caseBranchValue);
          if (caseIt == branchSwitchStep.caseBranches.end()) {
            throw InvalidArgumentError::Construct(
                "Auto mode step directory path case branch value {} does not exist in switch branch",
                currentNode.caseBranchValue);
          }
          currentDirectory = &caseIt->second;
        } else {
          throw InvalidArgumentError::Construct(
              "Auto mode step directory path node type {} is invalid for switch branch",
              static_cast<int>(currentNode.directoryType));
        }
        break;
      }
      default:
        throw InvalidArgumentError::Construct(
            "Auto mode step directory path cannot descend into non-branch step of type {}",
            ThunderAutoModeStepTypeToString(stepType));
    }
  }

  return *currentDirectory;
}

ThunderAutoModeStepTrajectoryBehavior ThunderAutoMode::getTrajectoryBehavior(
    const std::map<std::string, ThunderAutoTrajectorySkeleton>& trajectories) const noexcept {
  ThunderAutoModeStepTrajectoryBehaviorTreeNode tree =
      GetStepSequenceTrajectoryBehaviorTree(std::nullopt, steps, trajectories);
  return tree.behavior;
}

ThunderAutoModeStepTrajectoryBehaviorTreeNode ThunderAutoMode::getTrajectoryBehaviorTree(
    const std::map<std::string, ThunderAutoTrajectorySkeleton>& trajectories) const noexcept {
  return GetStepSequenceTrajectoryBehaviorTree(std::nullopt, steps, trajectories);
}

static void to_json(wpi::json& json, const ThunderAutoModeStepType& stepType) {
  switch (stepType) {
    case ThunderAutoModeStepType::ACTION:
      json = "action";
      break;
    case ThunderAutoModeStepType::TRAJECTORY:
      json = "trajectory";
      break;
    case ThunderAutoModeStepType::BRANCH_BOOL:
      json = "branch_bool";
      break;
    case ThunderAutoModeStepType::BRANCH_SWITCH:
      json = "branch_switch";
      break;
    default:
      ThunderLibCoreUnreachable("Unknown ThunderAutoModeStepType");
  }
}

static void from_json(const wpi::json& json, ThunderAutoModeStepType& stepType) {
  std::string typeStr = json.get<std::string>();
  if (typeStr == "action") {
    stepType = ThunderAutoModeStepType::ACTION;
  } else if (typeStr == "trajectory") {
    stepType = ThunderAutoModeStepType::TRAJECTORY;
  } else if (typeStr == "branch_bool") {
    stepType = ThunderAutoModeStepType::BRANCH_BOOL;
  } else if (typeStr == "branch_switch") {
    stepType = ThunderAutoModeStepType::BRANCH_SWITCH;
  } else {
    throw RuntimeError::Construct("Unknown ThunderAutoModeStepType: {}", typeStr);
  }
}

static void to_json(wpi::json& json, const std::unique_ptr<ThunderAutoModeStep>& step);

static void to_json(wpi::json& json, const ThunderAutoModeStep& step) {
  json["type"] = step.type();

  switch (step.type()) {
    case ThunderAutoModeStepType::ACTION: {
      const auto& actionStep = static_cast<const ThunderAutoModeActionStep&>(step);
      json["action"] = actionStep.actionName;
      break;
    }
    case ThunderAutoModeStepType::TRAJECTORY: {
      const auto& trajectoryStep = static_cast<const ThunderAutoModeTrajectoryStep&>(step);
      json["trajectory"] = trajectoryStep.trajectoryName;
      break;
    }
    case ThunderAutoModeStepType::BRANCH_BOOL: {
      const auto& branchBoolStep = static_cast<const ThunderAutoModeBoolBranchStep&>(step);

      if (!branchBoolStep.trueBranch.empty()) {
        json["true_branch"] = branchBoolStep.trueBranch;
      }
      if (!branchBoolStep.elseBranch.empty()) {
        json["else_branch"] = branchBoolStep.elseBranch;
      }
      json["condition"] = branchBoolStep.conditionName;

      break;
    }
    case ThunderAutoModeStepType::BRANCH_SWITCH: {
      const auto& branchSwitchStep = static_cast<const ThunderAutoModeSwitchBranchStep&>(step);

      if (!branchSwitchStep.caseBranches.empty()) {
        wpi::json caseBranchesJson;
        for (const auto& [key, value] : branchSwitchStep.caseBranches) {
          caseBranchesJson[std::to_string(key)] = value;
        }
        json["case_branches"] = caseBranchesJson;
      }
      if (!branchSwitchStep.defaultBranch.empty()) {
        json["default_branch"] = branchSwitchStep.defaultBranch;
      }
      json["condition"] = branchSwitchStep.conditionName;

      break;
    }
    default:
      ThunderLibCoreUnreachable("Unknown ThunderAutoModeStepType");
  }
}

static void to_json(wpi::json& json, const std::unique_ptr<ThunderAutoModeStep>& step) {
  ThunderLibCoreAssert(step != nullptr, "Step is null?");
  json = *step;
}

static void from_json(const wpi::json& json, std::unique_ptr<ThunderAutoModeStep>& step) {
  ThunderAutoModeStepType stepType = json.at("type").get<ThunderAutoModeStepType>();
  switch (stepType) {
    case ThunderAutoModeStepType::ACTION: {
      auto actionStep = std::make_unique<ThunderAutoModeActionStep>();
      json.at("action").get_to(actionStep->actionName);
      step = std::move(actionStep);
      break;
    }
    case ThunderAutoModeStepType::TRAJECTORY: {
      auto trajectoryStep = std::make_unique<ThunderAutoModeTrajectoryStep>();
      json.at("trajectory").get_to(trajectoryStep->trajectoryName);
      step = std::move(trajectoryStep);
      break;
    }
    case ThunderAutoModeStepType::BRANCH_BOOL: {
      auto branchBoolStep = std::make_unique<ThunderAutoModeBoolBranchStep>();
      if (json.contains("true_branch")) {
        json.at("true_branch").get_to(branchBoolStep->trueBranch);
      }
      if (json.contains("else_branch")) {
        json.at("else_branch").get_to(branchBoolStep->elseBranch);
      }
      json.at("condition").get_to(branchBoolStep->conditionName);

      step = std::move(branchBoolStep);
      break;
    }
    case ThunderAutoModeStepType::BRANCH_SWITCH: {
      auto branchSwitchStep = std::make_unique<ThunderAutoModeSwitchBranchStep>();
      if (json.contains("case_branches")) {
        const wpi::json& nextStepsJson = json.at("case_branches");
        for (const auto& [key, value] : nextStepsJson.items()) {
          int caseValue = std::stoi(key);
          branchSwitchStep->caseBranches[caseValue] = value;
        }
      }
      if (json.contains("default_branch")) {
        json.at("default_branch").get_to(branchSwitchStep->defaultBranch);
      }
      json.at("condition").get_to(branchSwitchStep->conditionName);

      step = std::move(branchSwitchStep);
      break;
    }
    default:
      ThunderLibCoreUnreachable("Unknown ThunderAutoModeStepType");
  }
}

void to_json(wpi::json& json, const ThunderAutoMode& mode) {
  json["steps"] = mode.steps;
}

void from_json(const wpi::json& json, ThunderAutoMode& mode) {
  json.at("steps").get_to(mode.steps);
}

}  // namespace thunder::core
