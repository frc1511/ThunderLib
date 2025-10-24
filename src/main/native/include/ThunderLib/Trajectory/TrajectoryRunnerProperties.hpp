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

  GetPoseFunc getPose;
  ResetPoseFunc resetPose;
  GetSpeedsFunc getSpeeds;
  SetSpeedsFunc setSpeeds;

  std::shared_ptr<frc::HolonomicDriveController> controller;
};

}  // namespace thunder
