#pragma once

// Modified version of Drivetrain class from WPILib SwerveBot example.

#include "Constants.h"
#include "SwerveModule.h"

#include <ThunderLib/Trajectory/TrajectoryRunnerProperties.hpp>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/AnalogGyro.h>
#include <frc/simulation/AnalogGyroSim.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/HolonomicDriveController.h>

class DriveSubsystem : public frc2::SubsystemBase {
 public:
  DriveSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;

  /**
   * Gets the essential properties to run ThunderAuto trajectories on this
   * drivetrain.
   *
   * @return The ThunderTrajectoryRunnerProperties for this drivetrain.
   */
  const thunder::TrajectoryRunnerProperties& GetTrajectoryRunnerProperties();

  /**
   * Method to drive the robot using joystick info.
   *
   * When fieldRelative is true, xSpeed and ySpeed are relative to the field, from the perspective of the
   * current alliance station.
   *
   * @param xSpeed        Velocity along the x-axis. (Fwd is +)
   * @param ySpeed        Velocity along the y-axis. (Left is +)
   * @param rot           Angular velocity of the robot frame. (CCW is +)
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed,
             units::radians_per_second_t rot,
             bool fieldRelative);

  frc::Pose2d GetCurrentPose() const;

  void SetCurrentPose(const frc::Pose2d& pose);

 private:
  void SetChassisSpeeds(const frc::ChassisSpeeds& speeds);
  wpi::array<frc::SwerveModulePosition, 4> GetModulePositions() const;

  void UpdateOdometry();
  void SendFeedback();

 private:
  frc::Translation2d frontLeftLocation{+DriveConstants::kModuleSpacingX / 2.0,
                                       +DriveConstants::kModuleSpacingY / 2.0};
  frc::Translation2d frontRightLocation{+DriveConstants::kModuleSpacingX / 2.0,
                                        -DriveConstants::kModuleSpacingY / 2.0};
  frc::Translation2d backLeftLocation{-DriveConstants::kModuleSpacingX / 2.0,
                                      +DriveConstants::kModuleSpacingY / 2.0};
  frc::Translation2d backRightLocation{-DriveConstants::kModuleSpacingX / 2.0,
                                       -DriveConstants::kModuleSpacingY / 2.0};

  SwerveModule frontLeft{
      DriveConstants::kFLDriveMotorChannel,      DriveConstants::kFLTurningMotorChannel,
      DriveConstants::kFLDriveEncoderChannelA,   DriveConstants::kFLDriveEncoderChannelB,
      DriveConstants::kFLTurningEncoderChannelA, DriveConstants::kFLTurningEncoderChannelB};

  SwerveModule frontRight{
      DriveConstants::kFRDriveMotorChannel,      DriveConstants::kFRTurningMotorChannel,
      DriveConstants::kFRDriveEncoderChannelA,   DriveConstants::kFRDriveEncoderChannelB,
      DriveConstants::kFRTurningEncoderChannelA, DriveConstants::kFRTurningEncoderChannelB};

  SwerveModule backLeft{DriveConstants::kBLDriveMotorChannel,      DriveConstants::kBLTurningMotorChannel,
                        DriveConstants::kBLDriveEncoderChannelA,   DriveConstants::kBLDriveEncoderChannelB,
                        DriveConstants::kBLTurningEncoderChannelA, DriveConstants::kBLTurningEncoderChannelB};

  SwerveModule backRight{
      DriveConstants::kBRDriveMotorChannel,      DriveConstants::kBRTurningMotorChannel,
      DriveConstants::kBRDriveEncoderChannelA,   DriveConstants::kBRDriveEncoderChannelB,
      DriveConstants::kBRTurningEncoderChannelA, DriveConstants::kBRTurningEncoderChannelB};

  frc::AnalogGyro gyro{DriveConstants::kGyroChannel};
  frc::sim::AnalogGyroSim gyroSim{gyro};

  frc::SwerveDriveKinematics<4> kinematics{frontLeftLocation, frontRightLocation, backLeftLocation,
                                           backRightLocation};

  frc::SwerveDriveOdometry<4> odometry{kinematics, gyro.GetRotation2d(), GetModulePositions()};

  bool wasPoseSet = false;

  frc::Field2d field;

  std::shared_ptr<frc::HolonomicDriveController> holonomicDriveController =
      std::make_shared<frc::HolonomicDriveController>(
          // X controller
          frc::PIDController{3.25, 0.0, 0.0},
          // Y controller
          frc::PIDController{3.25, 0.0, 0.0},
          // Theta controller
          frc::ProfiledPIDController<units::radians>{
              8.0,
              0.0,
              0.0,
              // This profile is not important because ThunderAuto handles angular acceleration limits. You
              // should make sure these limits are >= those in ThunderAuto so they don't interfere.
              {DriveConstants::kMaxAngularVelocity, DriveConstants::kMaxAngularAcceleration}});

  const thunder::TrajectoryRunnerProperties trajectoryRunnerProperties{
      std::bind(&DriveSubsystem::GetCurrentPose, this),
      std::bind(&DriveSubsystem::SetCurrentPose, this, std::placeholders::_1),
      (thunder::TrajectoryRunnerProperties::SetSpeedsFunc)std::bind(&DriveSubsystem::SetChassisSpeeds,
                                                                    this,
                                                                    std::placeholders::_1),
      holonomicDriveController};
};
