#pragma once

#include <ThunderLib/Auto/ThunderAutoTrajectory.hpp>
#include <ThunderLib/Auto/ThunderAutoProject.hpp>
#include <ThunderLib/Trajectory/TrajectoryRunnerProperties.hpp>
#include <ThunderLib/Types/CanonicalAngle.hpp>
#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>
#include <frc2/command/CommandHelper.h>
#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include <memory>
#include <optional>
#include <list>

namespace thunder {

/**
 * A command that follows a ThunderAuto Trajectory using the specified TrajectoryRunnerProperties.
 */
class ThunderAutoTrajectoryCommand : public frc2::CommandHelper<frc2::Command, ThunderAutoTrajectoryCommand> {
 public:
  /**
   * Constructs a ThunderAutoTrajectoryCommand.
   *
   * @param trajectoryName The name of the trajectory to follow.
   * @param project The ThunderAutoProject that contains the trajectory.
   * @param properties The TrajectoryRunnerProperties to use for following the trajectory.
   */
  ThunderAutoTrajectoryCommand(const std::string& trajectoryName,
                               std::shared_ptr<ThunderAutoProject> project,
                               const TrajectoryRunnerProperties& properties);

  bool isValid() const;

  explicit operator bool() const { return isValid(); }

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

 private:
  std::unique_ptr<ThunderAutoTrajectory> m_trajectory;
  std::shared_ptr<ThunderAutoProject> m_project;
  TrajectoryRunnerProperties m_runnerProperties;

  frc::Timer m_timer;

  std::optional<frc::DriverStation::Alliance> m_alliance;  // Checked during initialization, not updated afterward

  enum class ExecutionState {
    START_ACTIONS,
    FOLLOW_TRAJECTORY,
    STOPPED,
    END_ACTIONS,
    FINISHED,
  };

  ExecutionState m_executionState = ExecutionState::START_ACTIONS;

  frc2::CommandPtr m_startActionCommand = frc2::cmd::None();
  frc2::CommandPtr m_endActionCommand = frc2::cmd::None();

  struct StopActionCommand {
    units::second_t stopTime;
    frc2::CommandPtr command;
  };

  std::list<StopActionCommand> m_stopActionCommands;
  using StopIterator = decltype(m_stopActionCommands)::const_iterator;
  StopIterator m_nextStop = m_stopActionCommands.end();

  struct PositionedActionCommand {
    units::second_t actionTime;
    frc2::CommandPtr command;
  };

  std::list<PositionedActionCommand> m_positionedActionCommands;
  using PositionedActionIterator = decltype(m_positionedActionCommands)::const_iterator;
  PositionedActionIterator m_nextAction = m_positionedActionCommands.end();

  std::list<PositionedActionIterator> m_runningActions;

 private:
  void beginStartActions();
  void executeStartActions();
  void endStartActions(bool interrupted);

  void beginFollowTrajectory();
  void resumeFollowTrajectory();
  void executeFollowTrajectory();
  void endFollowTrajectory(bool interrupted);

  void beginStopped();
  void executeStopped();
  void endStopped(bool interrupted);

  void beginEndActions();
  void executeEndActions();
  void endEndActions(bool interrupted);

  static CanonicalAngle flipAngleForAlliance(CanonicalAngle originalAngle,
                                             std::optional<frc::DriverStation::Alliance> alliance,
                                             FieldSymmetry fieldSymmetry);

  static frc::Pose2d flipPoseForAlliance(const frc::Pose2d& originalPose,
                                                std::optional<frc::DriverStation::Alliance> alliance,
                                                FieldSymmetry fieldSymmetry,
                                                FieldDimensions fieldDimensions);
};

}  // namespace thunder
