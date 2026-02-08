#pragma once

#include <ThunderLib/Types/CanonicalAngle.hpp>
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
  struct DriveControllerInputData {
    frc::Pose2d currentPose;
    frc::Pose2d targetPose;
    CanonicalAngle heading;
    units::meters_per_second_t linearVelocity;
  };

  using GetPoseFunc = std::function<frc::Pose2d()>;
  using ResetPoseFunc = std::function<void(const frc::Pose2d&)>;
  using GetSpeedsFunc = std::function<frc::ChassisSpeeds()>;
  using SetSpeedsFunc = std::function<void(const frc::ChassisSpeeds&)>;
  using SetSpeedsWithDiagnosticsFunc =
      std::function<void(const frc::ChassisSpeeds&, const DriveControllerInputData&)>;
  using ControlFunc = std::function<void(const DriveControllerInputData&)>;
  using StopFunc = std::function<void()>;

  /**
   * Gets the current pose of the robot.
   */
  const GetPoseFunc getPose = nullptr;

  /**
   * Resets the robot's pose to the specified pose (and PID controllers too!).
   *
   * If not provided (nullptr), the robot's pose will not be reset at the start of the trajectory.
   */
  const ResetPoseFunc resetPose = nullptr;

  /**
   * Control the robot's chassis speeds.
   *
   * Speeds are robot-relative, not field-relative.
   */
  const SetSpeedsFunc setSpeeds = nullptr;

  /**
   * Control the robot's chassis speeds with diagnostic data.
   *
   * Speeds are robot-relative, not field-relative.
   */
  const SetSpeedsWithDiagnosticsFunc setSpeedsWithDiagnostics = nullptr;

  /**
   * Holonomic drive controller to use for following the trajectory.
   *
   * @note ThunderLib will NOT reset the controller's x, y, or theta PID controllers; the user should do that
   * in the resetPose function if desired.
   */
  const std::shared_ptr<frc::HolonomicDriveController> controller = nullptr;

  /**
   * Control the robot.
   */
  const ControlFunc control = nullptr;

  /**
   * Stop the robot (at the end of the trajectory or on interruption).
   */
  const StopFunc stop = nullptr;

  /**
   * Constructor for TrajectoryRunnerProperties.
   *
   * @param getPose                  Function that returns the current robot pose.
   * @param resetPose                Function that resets the robot pose (and PID controllers too!)
   * @param setSpeeds                Function that controls the robot chassis speeds (robot-centric).
   * @param holonomicDriveController The holonomic drive controller for trajectory following. ThunderLib will
   *                                 NOT reset its x, y, or theta PID controllers; the user should do that in
   *                                 the resetPose function if desired.
   */
  TrajectoryRunnerProperties(GetPoseFunc getPoseFunc,
                             ResetPoseFunc resetPoseFunc,
                             SetSpeedsFunc setSpeedsFunc,
                             std::shared_ptr<frc::HolonomicDriveController> holonomicDriveController)
      : getPose(std::move(getPoseFunc)),
        resetPose(std::move(resetPoseFunc)),
        setSpeeds(std::move(setSpeedsFunc)),
        controller(std::move(holonomicDriveController)) {}

  /**
   * Constructor for TrajectoryRunnerProperties.
   *
   * @param getPose                  Function that returns the current robot pose.
   * @param resetPose                Function that resets the robot pose (and PID controllers too!)
   * @param setSpeedsWithDiagnostics Function that controls the robot chassis speeds (robot-centric) with
   *                                 diagnostic data.
   * @param holonomicDriveController The holonomic drive controller for trajectory following. ThunderLib will
   *                                 NOT reset its x, y, or theta PID controllers; the user should do that in
   *                                 the resetPose function if desired.
   */
  TrajectoryRunnerProperties(GetPoseFunc getPoseFunc,
                             ResetPoseFunc resetPoseFunc,
                             SetSpeedsWithDiagnosticsFunc setSpeedsWithDiagnosticsFunc,
                             std::shared_ptr<frc::HolonomicDriveController> holonomicDriveController)
      : getPose(std::move(getPoseFunc)),
        resetPose(std::move(resetPoseFunc)),
        setSpeedsWithDiagnostics(std::move(setSpeedsWithDiagnosticsFunc)),
        controller(std::move(holonomicDriveController)) {}

  /**
   * Constructor for TrajectoryRunnerProperties, with no drive controller.
   * Instead, a consumer of DriveControllerInputData must be provided instead of
   * the typical chassis speeds consumer, and the user is responsible for
   * calculating the appropriate speeds on their own.
   *
   * @param getPose   Function that returns the current robot pose.
   * @param resetPose Function that resets the robot pose (and PID controllers too!)
   * @param control   Function that controls the robot based on the provided DriveControllerInputData.
   * @param stop      Function that stops the robot.
   */
  TrajectoryRunnerProperties(GetPoseFunc getPose, ResetPoseFunc resetPose, ControlFunc control, StopFunc stop)
      : getPose(std::move(getPose)),
        resetPose(std::move(resetPose)),
        control(std::move(control)),
        stop(std::move(stop)) {}

  bool isValid() const {
    if (getPose == nullptr)
      return false;

    if (setSpeeds != nullptr || setSpeedsWithDiagnostics != nullptr) {
      if (controller == nullptr)
        return false;
    } else {
      return control != nullptr && stop != nullptr;
    }

    return true;
  }

  explicit operator bool() const { return isValid(); }
};

}  // namespace thunder
