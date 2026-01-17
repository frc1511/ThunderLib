#include "subsystems/DriveSubsystem.h"

#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/RobotBase.h>

DriveSubsystem::DriveSubsystem() {
  gyro.Reset();
}

void DriveSubsystem::Periodic() {
  UpdateOdometry();
  SendFeedback();
}

void DriveSubsystem::SimulationPeriodic() {}

const thunder::TrajectoryRunnerProperties& DriveSubsystem::GetTrajectoryRunnerProperties() {
  return trajectoryRunnerProperties;
}

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot,
                           bool fieldRelative) {
  frc::ChassisSpeeds speeds(xSpeed, ySpeed, rot);

  if (fieldRelative) {
    // Invert speeds when controlling from the perspective of red driver station.
    // For testing, keep normal when no auto mode was run yet (pose was never set).
    if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed && wasPoseSet) {
      speeds.vx = -speeds.vx;
      speeds.vy = -speeds.vy;
    }

    speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(speeds, GetCurrentPose().Rotation());
  }

  frc::ChassisSpeeds discretizedSpeeds =
      frc::ChassisSpeeds::Discretize(speeds.vx, speeds.vy, speeds.omega, kPeriodicUpdateTime);

  SetChassisSpeeds(discretizedSpeeds);
}

frc::Pose2d DriveSubsystem::GetCurrentPose() const {
  return odometry.GetPose();
}

void DriveSubsystem::SetCurrentPose(const frc::Pose2d& pose) {
  odometry.ResetPosition(gyro.GetRotation2d(), GetModulePositions(), pose);

  // Important - reset controllers to prevent sudden jumps. ThunderLib does not do this for you.
  holonomicDriveController->GetXController().Reset();
  holonomicDriveController->GetYController().Reset();
  holonomicDriveController->GetThetaController().Reset(pose.Rotation().Radians());

  wasPoseSet = true;
}

void DriveSubsystem::SetChassisSpeeds(const frc::ChassisSpeeds& speeds) {
  // In simulation, update the gyro angle based on the chassis angular speed.
  if (frc::RobotBase::IsSimulation()) {
    units::degree_t newAngle = units::degree_t{gyro.GetAngle()} - speeds.omega * kPeriodicUpdateTime;
    gyroSim.SetAngle(newAngle.value());
  }

  wpi::array<frc::SwerveModuleState, 4> swerveModuleStates = kinematics.ToSwerveModuleStates(speeds);
  frc::SwerveDriveKinematics<4>::DesaturateWheelSpeeds(&swerveModuleStates, DriveConstants::kMaxVelocity);

  frontLeft.SetDesiredState(swerveModuleStates[0]);
  frontRight.SetDesiredState(swerveModuleStates[1]);
  backLeft.SetDesiredState(swerveModuleStates[2]);
  backRight.SetDesiredState(swerveModuleStates[3]);
}

wpi::array<frc::SwerveModulePosition, 4> DriveSubsystem::GetModulePositions() const {
  return {frontLeft.GetPosition(), frontRight.GetPosition(), backLeft.GetPosition(), backRight.GetPosition()};
}

void DriveSubsystem::UpdateOdometry() {
  odometry.Update(gyro.GetRotation2d(), GetModulePositions());
}

void DriveSubsystem::SendFeedback() {
  field.SetRobotPose(odometry.GetPose());
  frc::SmartDashboard::PutData("Field", &field);
}
