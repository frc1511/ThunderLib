#include "SwerveModule.h"

#include <numbers>

#include <frc/geometry/Rotation2d.h>
#include <frc/RobotBase.h>

SwerveModule::SwerveModule(const int driveMotorChannel,
                           const int turningMotorChannel,
                           const int driveEncoderChannelA,
                           const int driveEncoderChannelB,
                           const int turningEncoderChannelA,
                           const int turningEncoderChannelB)
    : driveMotor(driveMotorChannel),
      turningMotor(turningMotorChannel),
      driveEncoder(driveEncoderChannelA, driveEncoderChannelB),
      turningEncoder(turningEncoderChannelA, turningEncoderChannelB) {
  // Set the distance per pulse for the drive encoder. We can simply use the
  // distance traveled for one rotation of the wheel divided by the encoder
  // resolution.
  driveEncoder.SetDistancePerPulse(2 * std::numbers::pi * DriveConstants::kWheelRadius.value() /
                                   DriveConstants::kEncoderResolution);

  // Set the distance (in this case, angle) per pulse for the turning encoder.
  // This is the the angle through an entire rotation (2 * std::numbers::pi)
  // divided by the encoder resolution.
  turningEncoder.SetDistancePerPulse(2 * std::numbers::pi / DriveConstants::kEncoderResolution);

  // Limit the PID Controller's input range between -pi and pi and set the input
  // to be continuous.
  turningPIDController.EnableContinuousInput(-units::radian_t{std::numbers::pi},
                                             units::radian_t{std::numbers::pi});
}

frc::SwerveModuleState SwerveModule::GetState() const {
  return {units::meters_per_second_t{driveEncoder.GetRate()}, units::radian_t{turningEncoder.GetDistance()}};
}

frc::SwerveModulePosition SwerveModule::GetPosition() const {
  return {units::meter_t{driveEncoder.GetDistance()}, units::radian_t{turningEncoder.GetDistance()}};
}

void SwerveModule::SetDesiredState(frc::SwerveModuleState& referenceState) {
  if (frc::RobotBase::IsSimulation()) {
    SetDesiredStateSim(referenceState);
  } else {
    SetDesiredStateReal(referenceState);
  }
}

void SwerveModule::SetDesiredStateReal(frc::SwerveModuleState& referenceState) {
  frc::Rotation2d encoderRotation{units::radian_t{turningEncoder.GetDistance()}};

  // Optimize the reference state to avoid spinning further than 90 degrees
  referenceState.Optimize(encoderRotation);

  // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired direction of
  // travel that can occur when modules change directions. This results in smoother driving.
  referenceState.CosineScale(encoderRotation);

  // Calculate the drive output from the drive PID controller.
  const units::volt_t driveOutput{
      drivePIDController.Calculate(driveEncoder.GetRate(), referenceState.speed.value())};

  const units::volt_t driveFeedforwardOutput = driveFeedforward.Calculate(referenceState.speed);

  // Calculate the turning motor output from the turning PID controller.
  const units::volt_t turnOutput{turningPIDController.Calculate(units::radian_t{turningEncoder.GetDistance()},
                                                                referenceState.angle.Radians())};

  const units::volt_t turnFeedforwardOutput =
      turnFeedforward.Calculate(turningPIDController.GetSetpoint().velocity);

  // Set the motor outputs.
  driveMotor.SetVoltage(driveOutput + driveFeedforwardOutput);
  turningMotor.SetVoltage(turnOutput + turnFeedforwardOutput);
}

void SwerveModule::SetDesiredStateSim(frc::SwerveModuleState& referenceState) {
  frc::Rotation2d encoderRotation{units::radian_t{turningEncoder.GetDistance()}};

  // Optimize the reference state to avoid spinning further than 90 degrees
  referenceState.Optimize(encoderRotation);

  // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired direction of
  // travel that can occur when modules change directions. This results in smoother driving.
  referenceState.CosineScale(encoderRotation);

  // Simulate setting the speed and angle by directly setting the encoder values.
  driveEncoderSim.SetRate(referenceState.speed.value());
  driveEncoderSim.SetDistance(driveEncoder.GetDistance() +
                              referenceState.speed.value() * kPeriodicUpdateTime.value());
  turningEncoderSim.SetDistance(referenceState.angle.Radians().value());
}
