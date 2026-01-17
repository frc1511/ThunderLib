// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandPS4Controller.h>

#include "Constants.h"
#include "subsystems/ExampleSubsystem.h"
#include "subsystems/DriveSubsystem.h"

#include <ThunderLib/Auto/ThunderAutoProject.hpp>
#include <ThunderLib/Auto/ThunderAutoSendableChooser.hpp>

class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

 private:
  frc2::CommandPS4Controller driverController{OperatorConstants::kDriverControllerPort};

  ExampleSubsystem exampleSubsystem;
  DriveSubsystem driveSubsystem;

  bool fieldRelative = true;

  std::shared_ptr<thunder::ThunderAutoProject> autoProject;
  std::shared_ptr<thunder::ThunderAutoProject> autoProject2;
  std::shared_ptr<thunder::ThunderAutoSendableChooser> autoChooser;

  void ConfigureBindings();
};
