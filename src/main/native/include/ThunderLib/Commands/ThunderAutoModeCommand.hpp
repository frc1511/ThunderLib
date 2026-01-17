#pragma once

#include <ThunderLib/Auto/ThunderAutoMode.hpp>
#include <ThunderLib/Auto/ThunderAutoProject.hpp>
#include <ThunderLib/Trajectory/TrajectoryRunnerProperties.hpp>
#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>
#include <frc2/command/CommandHelper.h>
#include <memory>

namespace thunder {

/**
 * A command that executes a ThunderAuto Auto Mode.
 */
class ThunderAutoModeCommand : public frc2::CommandHelper<frc2::Command, ThunderAutoModeCommand> {
 public:
  /**
   * Constructs a ThunderAutoModeCommand.
   *
   * @param trajectoryName The name of the auto mode to follow.
   * @param project The ThunderAutoProject that contains the auto mode and all referenced trajectories,
   *                actions, and conditions.
   * @param properties The TrajectoryRunnerProperties to use for following all trajectories.
   */
  ThunderAutoModeCommand(const std::string& autoModeName,
                         std::shared_ptr<ThunderAutoProject> project,
                         const TrajectoryRunnerProperties& properties);

  bool isValid() const;

  explicit operator bool() const { return isValid(); }

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

 private:
  void nextStep();
  void setupCurrentStep();

 private:
  const std::string m_autoModeName;
  std::unique_ptr<ThunderAutoMode> m_autoMode;
  std::shared_ptr<ThunderAutoProject> m_project;
  TrajectoryRunnerProperties m_runnerProperties;
  TrajectoryRunnerProperties m_runnerPropertiesNoResetPose;

  bool m_isFinished = true;
  std::shared_ptr<ThunderAutoModeStep> m_currentStep;
  frc2::CommandPtr m_currentStepCommand = frc2::cmd::None();
  bool m_currentStepWasInitialized = false;
  bool m_firstTrajectoryWasSeen = false;
};

}  // namespace thunder
