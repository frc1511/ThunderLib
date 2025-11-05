#pragma once

#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/length.h>

namespace thunder {

/**
 * Field symmetry changes how trajectories are modified based on the selected alliance color.
 *
 * NONE: The trajectory remains the same regardless of alliance color.
 * ROTATIONAL: On red, both X and Y coordinates are inverted. Rotations are flipped by 180.
 * REFLECTIONAL: On red, only the X coordinate is inverted. Rotations are flipped by 180
 */
enum FieldSymmetry {
  NONE,
  ROTATIONAL,    // Rotated 180 degrees, like 2022 and 2025.
  REFLECTIONAL,  // Mirrored across the center line, like 2023 and 2024.
};

struct FieldDimensions {
  units::meter_t width = 0_m;   // X
  units::meter_t length = 0_m;  // Y
};

struct TrajectoryState {
  units::second_t time = 0_s;
  frc::Pose2d pose;
  frc::ChassisSpeeds chassisSpeeds; // Field-centric
  units::meters_per_second_t linearVelocity = 0.0_mps;
  frc::Rotation2d heading;
};

class Trajectory {
 public:
  virtual ~Trajectory() = default;

  virtual bool isValid() const noexcept = 0;

  operator bool() const noexcept { return isValid(); }

  /**
   * Samples the trajectory at a specified time.
   *
   * @param time The time at which to sample the trajectory.
   * @return The state of the robot at the specified time.
   */
  virtual TrajectoryState sample(units::second_t time) const noexcept = 0;

  /**
   * Returns the duration of the trajectory.
   *
   * @return The duration.
   */
  virtual units::second_t getDuration() const noexcept = 0;

  /**
   * Returns the initial state of the robot.
   *
   * @return The initial state.
   */
  virtual TrajectoryState getInitialState() const noexcept = 0;

  /**
   * Returns the final state of the robot.
   *
   * @return The final state.
   */
  virtual TrajectoryState getFinalState() const noexcept = 0;

 protected:
  Trajectory() = default;
};

}  // namespace thunder
