// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ExampleCommand.h"
#include <cstdio>

ExampleCommand::ExampleCommand(ExampleSubsystem* subsystem) : m_subsystem{subsystem} {
  // Register that this command requires the subsystem.
  AddRequirements(m_subsystem);
}

// Called when the command is initially scheduled.
void ExampleCommand::Initialize() {
  std::puts("ExampleCommand initialized");
}

// Called every time the scheduler runs while the command is scheduled.
void ExampleCommand::Execute() {
  std::puts("ExampleCommand executing");
}

// Called once the command ends or is interrupted.
void ExampleCommand::End(bool interrupted) {
  std::printf("ExampleCommand ended %s\n", interrupted ? "due to interruption" : "normally");
}

// Returns true when the command should end.
bool ExampleCommand::IsFinished() {
  return false;
}
