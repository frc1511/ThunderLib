#pragma once

#include <units/length.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/time.h>
#include <numbers>

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace OperatorConstants {

inline constexpr int kDriverControllerPort = 0;

inline constexpr double kDeadband = 0.05;

}  // namespace OperatorConstants

namespace DriveConstants {

inline constexpr units::meters_per_second_t kMaxVelocity = 3.0_mps;
inline constexpr units::radians_per_second_t kMaxAngularVelocity = 2 * std::numbers::pi * 1_rad_per_s;
inline constexpr units::radians_per_second_squared_t kMaxAngularAcceleration = 2 * std::numbers::pi * 1_rad_per_s_sq;

inline constexpr units::meter_t kWheelRadius = 0.0508_m;
inline constexpr int kEncoderResolution = 4096;

inline constexpr units::radians_per_second_t kModuleMaxTurningVelocity = std::numbers::pi * 1_rad_per_s;
inline constexpr units::radians_per_second_squared_t kModuleMaxTurningAcceleration = 2 * std::numbers::pi * 1_rad_per_s_sq;
inline constexpr units::meter_t kModuleSpacingX = 0.762_m;
inline constexpr units::meter_t kModuleSpacingY = 0.762_m;

inline constexpr int kFLDriveMotorChannel = 1;
inline constexpr int kFLTurningMotorChannel = 2;
inline constexpr int kFRDriveMotorChannel = 3;
inline constexpr int kFRTurningMotorChannel = 4;
inline constexpr int kBLDriveMotorChannel = 5;
inline constexpr int kBLTurningMotorChannel = 6;
inline constexpr int kBRDriveMotorChannel = 7;
inline constexpr int kBRTurningMotorChannel = 8;

inline constexpr int kFLDriveEncoderChannelA = 1;
inline constexpr int kFLDriveEncoderChannelB = 2;
inline constexpr int kFLTurningEncoderChannelA = 3;
inline constexpr int kFLTurningEncoderChannelB = 4;

inline constexpr int kFRDriveEncoderChannelA = 5;
inline constexpr int kFRDriveEncoderChannelB = 6;
inline constexpr int kFRTurningEncoderChannelA = 7;
inline constexpr int kFRTurningEncoderChannelB = 8;

inline constexpr int kBLDriveEncoderChannelA = 9;
inline constexpr int kBLDriveEncoderChannelB = 10;
inline constexpr int kBLTurningEncoderChannelA = 11;
inline constexpr int kBLTurningEncoderChannelB = 12;

inline constexpr int kBRDriveEncoderChannelA = 13;
inline constexpr int kBRDriveEncoderChannelB = 14;
inline constexpr int kBRTurningEncoderChannelA = 15;
inline constexpr int kBRTurningEncoderChannelB = 16;

inline constexpr int kGyroChannel = 0;

}  // namespace DriveConstants

inline constexpr units::second_t kPeriodicUpdateTime = 0.02_s;
