#pragma once

#include <frc/controller/HolonomicDriveController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <functional>
#include <memory>

namespace thunder {

/**
 * A set of callback functions and properties required to run a trajectory.
 */
struct TrajectoryRunnerProperties {
  using GetPoseFunc = std::function<frc::Pose2d()>;
  using ResetPoseFunc = std::function<void(const frc::Pose2d&)>;
  using GetSpeedsFunc = std::function<frc::ChassisSpeeds()>;
  using SetSpeedsFunc = std::function<void(const frc::ChassisSpeeds&)>;

  /**
   * Gets the current pose of the robot (required).
   */
  GetPoseFunc getPose;

  /**
   * Resets the robot's pose to the specified pose (optional).
   *
   * If not provided (nullptr), the robot's pose will not be reset at the start of the trajectory.
   */
  ResetPoseFunc resetPose;

  /**
   * Control the robot's chassis speeds (required).
   *
   * Speeds are robot-relative, not field-relative.
   */
  SetSpeedsFunc setSpeeds;

  /**
   * Holonomic drive controller to use for following the trajectory (required).
   */
  std::shared_ptr<frc::HolonomicDriveController> controller;

  bool isValid() const { return (getPose != nullptr) && (setSpeeds != nullptr) && (controller != nullptr); }

  explicit operator bool() const { return isValid(); }
};

}  // namespace thunder
