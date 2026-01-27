#include <ThunderLib/Commands/ThunderAutoTrajectoryCommand.hpp>

namespace thunder {

ThunderAutoTrajectoryCommand::ThunderAutoTrajectoryCommand(const std::string& trajectoryName,
                                                           std::shared_ptr<ThunderAutoProject> project,
                                                           const TrajectoryRunnerProperties& properties)
    : m_project(project), m_runnerProperties(properties) {
  if (m_project) {
    m_trajectory = project->getTrajectory(trajectoryName);
  }

  if (!isValid())
    return;

  // Get start action command.
  {
    const std::string& startActionName = m_trajectory->getStartAction();
    if (!startActionName.empty()) {
      m_startActionCommand = m_project->getActionCommand(startActionName);
    }
  }

  // Get end action command.
  {
    const std::string& endActionName = m_trajectory->getEndAction();
    if (!endActionName.empty()) {
      m_endActionCommand = m_project->getActionCommand(endActionName);
    }
  }

  // Get stop action commands.
  {
    const std::map<units::time::second_t, std::string> stopActions = m_trajectory->getStopActions();

    for (const auto& [stopTime, actionName] : stopActions) {
      if (!actionName.empty()) {
        frc2::CommandPtr stopCommand = m_project->getActionCommand(actionName);
        m_stopActionCommands.push_back(StopActionCommand{stopTime, std::move(stopCommand)});
      }
    }
  }

  // Get positioned action commands.
  {
    const std::multimap<units::time::second_t, std::string>& actions = m_trajectory->getActions();

    for (const auto& [actionTime, actionName] : actions) {
      frc2::CommandPtr actionCommand = m_project->getActionCommand(actionName);
      m_positionedActionCommands.push_back(PositionedActionCommand{actionTime, std::move(actionCommand)});
    }
  }
}

bool ThunderAutoTrajectoryCommand::isValid() const {
  return m_project != nullptr && m_trajectory != nullptr && m_runnerProperties.isValid();
}

void ThunderAutoTrajectoryCommand::Initialize() {
  if (!isValid())
    return;

  m_alliance = frc::DriverStation::GetAlliance();

  if (m_runnerProperties.resetPose) {
    frc::Pose2d initialTrajectoryPose = m_trajectory->getInitialState().pose;

    frc::Pose2d initialPose = flipPoseForAlliance(
        initialTrajectoryPose, m_alliance, m_project->getFieldSymmetry(), m_project->getFieldDimensions());

    m_runnerProperties.resetPose(initialPose);
  }

  beginStartAction();
}

void ThunderAutoTrajectoryCommand::Execute() {
  if (!isValid())
    return;

  switch (m_executionState) {
    using enum ExecutionState;
    case START_ACTIONS:
      executeStartAction();
      break;
    case FOLLOW_TRAJECTORY:
      executeFollowTrajectory();
      break;
    case STOPPED:
      executeStopped();
      break;
    case END_ACTIONS:
      executeEndAction();
      break;
    case FINISHED:
      break;
  }
}

void ThunderAutoTrajectoryCommand::End(bool interrupted) {
  if (!isValid())
    return;

  switch (m_executionState) {
    using enum ExecutionState;
    case START_ACTIONS:
      endStartAction(interrupted);
      break;
    case FOLLOW_TRAJECTORY:
      endFollowTrajectory(interrupted);
      break;
    case STOPPED:
      endStopped(interrupted);
      break;
    case END_ACTIONS:
      endEndAction(interrupted);
      break;
    case FINISHED:
      break;
  }

  m_executionState = ExecutionState::FINISHED;
}

bool ThunderAutoTrajectoryCommand::IsFinished() {
  if (!isValid())
    return true;

  return m_executionState == ExecutionState::FINISHED;
}

void ThunderAutoTrajectoryCommand::beginStartAction() {
  m_executionState = ExecutionState::START_ACTIONS;

  m_startActionCommand.get()->Initialize();
}

void ThunderAutoTrajectoryCommand::executeStartAction() {
  if (m_startActionCommand.get()->IsFinished()) {
    m_startActionCommand.get()->End(false);
    beginFollowTrajectory();
    return;
  }

  m_startActionCommand.get()->Execute();
}

void ThunderAutoTrajectoryCommand::endStartAction(bool interrupted) {
  m_startActionCommand.get()->End(interrupted);
}

void ThunderAutoTrajectoryCommand::beginFollowTrajectory() {
  m_executionState = ExecutionState::FOLLOW_TRAJECTORY;

  m_timer.Reset();
  m_timer.Start();

  m_nextStop = m_stopActionCommands.begin();
  m_nextAction = m_positionedActionCommands.begin();
  m_runningActions.clear();
}

void ThunderAutoTrajectoryCommand::resumeFollowTrajectory() {
  m_executionState = ExecutionState::FOLLOW_TRAJECTORY;

  m_timer.Start();
}

void ThunderAutoTrajectoryCommand::executeFollowTrajectory() {
  units::second_t trajectoryTime = m_timer.Get();

  // Process stop points.

  if (m_nextStop != m_stopActionCommands.end()) {
    if (trajectoryTime >= m_nextStop->stopTime - 20_ms) {
      // Stop the robot.
      m_runnerProperties.setSpeeds(frc::ChassisSpeeds{});
      // Switch to the STOPPED state.
      beginStopped();
      return;
    }
  }

  // Process positioned actions.

  while ((m_nextAction != m_positionedActionCommands.end()) &&
         (trajectoryTime >= m_nextAction->actionTime - 20_ms)) {
    m_nextAction->command.get()->Initialize();
    m_runningActions.push_back(m_nextAction);
    m_nextAction++;
  }

  std::list<PositionedActionIterator> doneRunningActions;
  for (const PositionedActionIterator& actionIt : m_runningActions) {
    const auto& [_, actionCommand] = *actionIt;

    if (actionCommand.get()->IsFinished()) {
      actionCommand.get()->End(false);
      doneRunningActions.push_back(actionIt);
    } else {
      actionCommand.get()->Execute();
    }
  }

  std::for_each(doneRunningActions.begin(), doneRunningActions.end(),
                [&](const PositionedActionIterator& it) { m_runningActions.remove(it); });

  // Check if trajectory is complete.

  if (trajectoryTime >= m_trajectory->getDuration()) {  // TODO: Wait for it to reach final position?
    // Stop the robot.
    m_runnerProperties.setSpeeds(frc::ChassisSpeeds{});
    // Begin the end action.
    beginEndAction();
    return;
  }

  // Drive the robot.

  TrajectoryState state = m_trajectory->sample(trajectoryTime);

  const FieldSymmetry fieldSymmetry = m_project->getFieldSymmetry();
  const FieldDimensions fieldDimensions = m_project->getFieldDimensions();

  frc::Pose2d currentPose = m_runnerProperties.getPose();
  frc::Pose2d targetPose = flipPoseForAlliance(state.pose, m_alliance, fieldSymmetry, fieldDimensions);
  frc::Rotation2d heading = flipAngleForAlliance(state.heading, m_alliance, fieldSymmetry);

  frc::ChassisSpeeds velocities =
      m_runnerProperties.controller->Calculate(currentPose, frc::Pose2d(targetPose.Translation(), heading),
                                               state.linearVelocity, targetPose.Rotation());

  m_runnerProperties.setSpeeds(velocities);
}

void ThunderAutoTrajectoryCommand::endFollowTrajectory(bool interrupted) {
  // Stop the robot.

  m_runnerProperties.setSpeeds(frc::ChassisSpeeds{});

  // End any running actions

  for (const PositionedActionIterator& actionIt : m_runningActions) {
    const auto& [_, actionCommand] = *actionIt;
    actionCommand.get()->End(interrupted);
  }
}

void ThunderAutoTrajectoryCommand::beginStopped() {
  m_executionState = ExecutionState::STOPPED;

  m_timer.Stop();

  m_nextStop->command.get()->Initialize();
}

void ThunderAutoTrajectoryCommand::executeStopped() {
  const frc2::CommandPtr& stopActionCommand = m_nextStop->command;
  if (stopActionCommand.get()->IsFinished()) {
    stopActionCommand.get()->End(false);
    m_nextStop++;
    resumeFollowTrajectory();
    return;
  }

  stopActionCommand.get()->Execute();
}

void ThunderAutoTrajectoryCommand::endStopped(bool interrupted) {
  m_nextStop->command.get()->End(interrupted);
}

void ThunderAutoTrajectoryCommand::beginEndAction() {
  m_executionState = ExecutionState::END_ACTIONS;

  m_endActionCommand.get()->Initialize();
}

void ThunderAutoTrajectoryCommand::executeEndAction() {
  if (m_endActionCommand.get()->IsFinished()) {
    m_endActionCommand.get()->End(false);
    m_executionState = ExecutionState::FINISHED;
    return;
  }

  m_endActionCommand.get()->Execute();
}

void ThunderAutoTrajectoryCommand::endEndAction(bool interrupted) {
  m_endActionCommand.get()->End(interrupted);
}

CanonicalAngle ThunderAutoTrajectoryCommand::flipAngleForAlliance(
    CanonicalAngle originalAngle,
    std::optional<frc::DriverStation::Alliance> alliance,
    FieldSymmetry fieldSymmetry) {
  bool isRed = alliance.has_value() && (alliance.value() == frc::DriverStation::Alliance::kRed);
  if (!isRed) {
    return originalAngle;
  }

  switch (fieldSymmetry) {
    using enum FieldSymmetry;
    case ROTATIONAL:
      return originalAngle + CanonicalAngle(180_deg);
    case REFLECTIONAL:
      return CanonicalAngle(180_deg) - originalAngle;
    case NONE:
    default:
      return originalAngle;
  }
}

frc::Pose2d ThunderAutoTrajectoryCommand::flipPoseForAlliance(
    const frc::Pose2d& originalPose,
    std::optional<frc::DriverStation::Alliance> alliance,
    FieldSymmetry fieldSymmetry,
    FieldDimensions fieldDimensions) {
  bool isRed = alliance.has_value() && (alliance.value() == frc::DriverStation::Alliance::kRed);
  if (!isRed) {
    return originalPose;
  }

  units::meter_t x = originalPose.X();
  units::meter_t y = originalPose.Y();

  switch (fieldSymmetry) {
    using enum FieldSymmetry;
    case NONE:
      break;
    case ROTATIONAL:
      x = fieldDimensions.width - x;
      y = fieldDimensions.length - y;
      break;
    case REFLECTIONAL:
      x = fieldDimensions.width - x;
      break;
  }

  frc::Rotation2d rotation = flipAngleForAlliance(originalPose.Rotation(), alliance, fieldSymmetry);

  frc::Pose2d pose(x, y, rotation);
  return pose;
}

}  // namespace thunder
