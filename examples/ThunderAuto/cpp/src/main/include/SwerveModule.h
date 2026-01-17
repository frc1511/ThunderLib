#pragma once

// Modified version of SwerveModule class from WPILib SwerveBot example.

#include "Constants.h"

#include <frc/Encoder.h>
#include <frc/simulation/EncoderSim.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include <numbers>

class SwerveModule {
 public:
  SwerveModule(int driveMotorChannel,
               int turningMotorChannel,
               int driveEncoderChannelA,
               int driveEncoderChannelB,
               int turningEncoderChannelA,
               int turningEncoderChannelB);

  frc::SwerveModuleState GetState() const;
  frc::SwerveModulePosition GetPosition() const;
  void SetDesiredState(frc::SwerveModuleState& state);

 private:
  void SetDesiredStateReal(frc::SwerveModuleState& state);
  void SetDesiredStateSim(frc::SwerveModuleState& state);

 private:
  frc::PWMSparkMax driveMotor;
  frc::PWMSparkMax turningMotor;

  frc::Encoder driveEncoder;
  frc::Encoder turningEncoder;

  frc::sim::EncoderSim driveEncoderSim{driveEncoder};
  frc::sim::EncoderSim turningEncoderSim{turningEncoder};

  frc::PIDController drivePIDController{1.0, 0, 0};
  frc::ProfiledPIDController<units::radians> turningPIDController{
      1.0,
      0.0,
      0.0,
      {DriveConstants::kModuleMaxTurningVelocity, DriveConstants::kModuleMaxTurningAcceleration}};

  frc::SimpleMotorFeedforward<units::meters> driveFeedforward{1_V, 3_V / 1_mps};
  frc::SimpleMotorFeedforward<units::radians> turnFeedforward{1_V, 0.5_V / 1_rad_per_s};
};
