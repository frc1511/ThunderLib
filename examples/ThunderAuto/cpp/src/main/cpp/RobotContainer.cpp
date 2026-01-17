// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include "commands/ExampleCommand.h"

#include <frc2/command/button/Trigger.h>
#include <frc2/command/Commands.h>
#include <frc/RobotState.h>

#include <cstdio>

RobotContainer::RobotContainer() {
  // Configure the button bindings
  ConfigureBindings();

  /**
   * Load the project saved as "ChargedUp.thunderauto" in the deploy directory.
   *
   * You can provide a path relative to the deploy directory, or an absolute path to a custom location.
   */
  autoProject = std::make_shared<thunder::ThunderAutoProject>("ChargedUp");

  /**
   * Register any action commands used by auto modes and trajectories in the project.
   *
   * If left unregistered, a None command will be used in its place.
   */
  autoProject->registerActionCommand("ScoreCone_High", frc2::cmd::Print("Scored Cone at High"));
  autoProject->registerActionCommand("ScoreCone_Middle", frc2::cmd::Print("Scored Cone at Middle"));
  autoProject->registerActionCommand(
      "ScoreCube_High",
      frc2::cmd::Print("Scoring Cube at High")
          .AndThen(frc2::cmd::Wait(0.5_s).RaceWith(frc2::cmd::Run([] { std::puts("Scoring..."); })))
          .AndThen(frc2::cmd::Print("Scored Cube at High")));

  autoProject->registerActionCommand("MoveArm_High", frc2::cmd::Print("Moving Arm to High")
                                                         .AndThen(frc2::cmd::Wait(0.5_s))
                                                         .AndThen(frc2::cmd::Print("Moved Arm to High")));

  autoProject->registerActionCommand("MoveArm_Middle", frc2::cmd::Print("Moving Arm to Middle")
                                                           .AndThen(frc2::cmd::Wait(0.5_s))
                                                           .AndThen(frc2::cmd::Print("Moved Arm to Middle")));

  autoProject->registerActionCommand("MoveArm_Low", frc2::cmd::Print("Moving Arm to Low")
                                                        .AndThen(frc2::cmd::Wait(0.5_s))
                                                        .AndThen(frc2::cmd::Print("Moved Arm to Low")));

  autoProject->registerActionCommand("ConeIntake_Start", frc2::cmd::Print("Cone Intake Started"));
  autoProject->registerActionCommand("ConeIntake_Stop", frc2::cmd::Print("Cone Intake Stopped"));

  autoProject->registerActionCommand("NotifyDone", frc2::cmd::Print("Notification Done"));

  /**
   * Register any conditions used by auto modes in the project.
   */
  autoProject->registerBooleanCondition("HasGamePiece", [] { return true; });

  /**
   * Create a sendable chooser at the SmartDashboard location "Auto_Mode".
   *
   * This chooser will automatically re-publish when changes are made.
   */
  autoChooser = std::make_shared<thunder::ThunderAutoSendableChooser>("Auto_Mode");

  /**
   * To add items from the project to the chooser, we must first include the project as a "source".
   */
  autoChooser->includeProjectSource(autoProject);

  /**
   * Now we can add trajectories and auto modes from the project to the chooser by referencing the project's
   * name.
   */
  autoChooser->addAutoModeFromProject(autoProject->getName(), "Barrier_3GP");
  autoChooser->addTrajectoryFromProject(autoProject->getName(), "BarrierStart_To_GP1");
  // You can also add non-ThunderAuto commands if desired.
  autoChooser->addCustomCommand("Example Command", std::make_shared<ExampleCommand>(&exampleSubsystem));

  /**
   * Finally, give the auto chooser the drivetrain's trajectory runner properties so that it can construct
   * trajectory following commands.
   */
  autoChooser->setTrajectoryRunnerProperties(driveSubsystem.GetTrajectoryRunnerProperties());

  ////////////////////////////////////////////////////////////////////////////////

  /**
   * Load a second project.
   */
  autoProject2 = std::make_shared<thunder::ThunderAutoProject>("RapidReact");

  autoProject2->registerActionCommand(
      "ShootAll",
      frc2::cmd::Print("Shoot started")
          .AndThen(frc2::cmd::Wait(1.0_s).RaceWith(frc2::cmd::Run([] { std::puts("Shooting..."); })))
          .AndThen(frc2::cmd::Print("Shoot all cargo")));
  autoProject2->registerActionCommand("StartIntake", frc2::cmd::Print("Intake Started"));
  autoProject2->registerActionCommand("StopIntake", frc2::cmd::Print("Intake Stopped"));

  autoChooser->includeProjectSource(autoProject2);
  autoChooser->addTrajectoryFromProject(autoProject2->getName(), "Full5Cargo");
}

void RobotContainer::ConfigureBindings() {
  // Swerve drive
  driveSubsystem.SetDefaultCommand(frc2::cmd::Run(
      [this] {
        if (!frc::RobotState::IsTeleop())
          return;

        const units::meters_per_second_t xSpeed =
            -frc::ApplyDeadband(driverController.GetLeftY(), OperatorConstants::kDeadband) *
            DriveConstants::kMaxVelocity;

        const units::meters_per_second_t ySpeed =
            -frc::ApplyDeadband(driverController.GetLeftX(), OperatorConstants::kDeadband) *
            DriveConstants::kMaxVelocity;

        const units::radians_per_second_t rot =
            -frc::ApplyDeadband(driverController.GetRightX(), OperatorConstants::kDeadband) *
            DriveConstants::kMaxAngularVelocity;

        driveSubsystem.Drive(xSpeed, ySpeed, rot, fieldRelative);
      },
      {&driveSubsystem}));

  // Triangle button to toggle field-relative driving
  driverController.Triangle().ToggleOnTrue(frc2::cmd::RunOnce([this] { fieldRelative = !fieldRelative; }));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return autoChooser->getSelectedCommand();
}
