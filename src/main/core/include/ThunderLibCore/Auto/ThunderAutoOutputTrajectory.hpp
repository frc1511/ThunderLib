#pragma once

#include <ThunderLibCore/Auto/ThunderAutoTrajectorySkeleton.hpp>
#include <ThunderLibCore/Types.hpp>
#include <frc/kinematics/ChassisSpeeds.h>
#include <units/time.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>
#include <units/curvature.h>
#include <wpi/json.h>
#include <vector>
#include <memory>
#include <cstddef>
#include <string>
#include <cstring>

namespace thunder::core {

struct ThunderAutoOutputTrajectoryPoint {
  units::second_t time;
  Point2d position;
  units::meters_per_second_t linearVelocity;
  frc::ChassisSpeeds chassisSpeeds;
  CanonicalAngle heading;
  CanonicalAngle rotation;
  units::radians_per_second_t angularVelocity;

  std::vector<std::string> actions;

  units::meter_t distance;
  units::curvature_t curvature{0.0};
  units::meters_per_second_squared_t centripetalAcceleration;
};

struct ThunderAutoOutputTrajectorySegment {
  struct SampledPoint {
    units::meter_t distance;
    Point2d position;
    units::curvature_t curvature{0.0};
  };

  /**
   * The raw bezier curve points calculated for this segment, set and used internally at the start of the
   * trajectory build process, but may prove useful later. These points are not evenly spaced.
   */
  std::vector<SampledPoint> sampledPoints;

  units::meter_t length;

  /**
   * Indices in the output trajectory points vector defining the bounds of this segment (inclusive).
   */

  size_t startIndex;
  size_t endIndex;
};

struct ThunderAutoOutputTrajectory {
  // Timestamp when the trajectory was built.
  units::second_t buildTimestamp;

  units::meter_t totalDistance;
  units::second_t totalTime;

  std::vector<ThunderAutoOutputTrajectoryPoint> points;  // Ordered by time.
  std::vector<ThunderAutoOutputTrajectorySegment> segments;

  std::string startAction;
  std::string endAction;
  std::map<units::second_t, std::string> stopActions;
  std::multimap<units::second_t, std::string> actions;

  /**
   * Returns the index of an output trajectory point at the given trajectory position. This function completes
   * in constant O(1) time. This function requires that the points vector is not empty and that the segments
   * vector is correctly initialized.
   *
   * @param position A trajectory position.
   *
   * @return Index of the output trajectory point at the given trajectory position.
   */
  size_t trajectoryPositionToPointIndex(ThunderAutoTrajectoryPosition position) const;

  /**
   * Determines the trajectory position of an output trajectory point. This function completes in O(log(n))
   * time where n is the number of segments in the segments vector.
   *
   * @param pointIndex Index of a point in the points vector.
   *
   * @return The trajectory position of the given point.
   */
  ThunderAutoTrajectoryPosition pointIndexToTrajectoryPosition(size_t pointIndex) const;
};

struct ThunderAutoPartialOutputTrajectoryPoint {
  Point2d position;
  units::meter_t distance;
};

struct ThunderAutoPartialOutputTrajectory {
  units::second_t buildTimestamp;

  units::meter_t totalDistance;

  std::vector<ThunderAutoPartialOutputTrajectoryPoint> points;
  std::vector<ThunderAutoOutputTrajectorySegment> segments;

  /**
   * Returns the index of an output trajectory point at the given trajectory position. This function completes
   * in constant O(1) time. This function requires that the points vector is not empty and that the segments
   * vector is correctly initialized.
   *
   * @param position A trajectory position.
   *
   * @return Index of the output trajectory point at the given trajectory position.
   */
  size_t trajectoryPositionToPointIndex(ThunderAutoTrajectoryPosition position) const;

  /**
   * Determines the trajectory position of an output trajectory point. This function completes in O(log(n))
   * time where n is the number of segments in the segments vector.
   *
   * @param pointIndex Index of a point in the points vector.
   *
   * @return The trajectory position of the given point.
   */
  ThunderAutoTrajectoryPosition pointIndexToTrajectoryPosition(size_t pointIndex) const;
};

struct ThunderAutoOutputTrajectorySettings {
  /**
   * Number of samples per segment. This is used to initially calculate the
   * length of the trajectory.
   */
  size_t lengthSamples;

  /**
   * Number of samples per meter of the trajectory. This affects the resolution
   * of the output trajectory.
   */
  size_t samplesPerMeter;
};

static constexpr ThunderAutoOutputTrajectorySettings kPreviewOutputTrajectorySettings{
    .lengthSamples = 100,
    .samplesPerMeter = 25,
};

static constexpr ThunderAutoOutputTrajectorySettings kHighResOutputTrajectorySettings{
    .lengthSamples = 1000,
    .samplesPerMeter = 64,
};

/**
 * Builds an output trajectory from a trajectory skeleton. Calculates the trajectory points, linear and
 * angular velocities, times, etc. for each point in the trajectory. This function completes in O(n) time,
 * where n is the number of samples specified in settings.
 *
 * @param trajectorySkeleton The trajectory skeleton to build the output trajectory from.
 * @param settings The settings for building the output trajectory.
 *
 * @return A unique pointer to the built output trajectory.
 */
std::unique_ptr<ThunderAutoOutputTrajectory> BuildThunderAutoOutputTrajectory(
    const ThunderAutoTrajectorySkeleton& trajectorySkeleton,
    const ThunderAutoOutputTrajectorySettings& settings);

std::unique_ptr<ThunderAutoPartialOutputTrajectory> BuildThunderAutoPartialOutputTrajectory(
    const ThunderAutoTrajectorySkeleton& trajectorySkeleton,
    const ThunderAutoOutputTrajectorySettings& settings);

struct ThunderAutoCSVExportProperties {
  bool includeHeader = true;

  bool time = true;                      // time
  bool position = true;                  // x_pos, y_pos
  bool linearVelocity = true;            // velocity
  bool componentVelocities = false;      // vx, vy
  bool heading = false;                  // heading
  bool rotation = true;                  // rotation
  bool angularVelocity = true;           // angular_velocity
  bool actionsBitField = true;           // action
  bool distance = false;                 // distance
  bool curvature = false;                // curvature
  bool centripetalAcceleration = false;  // centripetal_accel

  static ThunderAutoCSVExportProperties Default() { return ThunderAutoCSVExportProperties{}; }

  // The default export properties pre-2026.
  static ThunderAutoCSVExportProperties LegacyDefault() {
    ThunderAutoCSVExportProperties properties{
        .includeHeader = true,
        .time = true,
        .position = true,
        .linearVelocity = true,
        .componentVelocities = false,
        .heading = false,
        .rotation = true,
        .angularVelocity = false,
        .actionsBitField = true,
        .distance = false,
        .curvature = false,
        .centripetalAcceleration = false,
    };
    return properties;
  }

  static ThunderAutoCSVExportProperties Full() {
    ThunderAutoCSVExportProperties properties{
        .includeHeader = true,
        .time = true,
        .position = true,
        .linearVelocity = true,
        .componentVelocities = true,
        .heading = true,
        .rotation = true,
        .angularVelocity = true,
        .actionsBitField = true,
        .distance = true,
        .curvature = true,
        .centripetalAcceleration = true,
    };
    return properties;
  }

  bool operator==(const ThunderAutoCSVExportProperties& other) const noexcept = default;
};

void from_json(const wpi::json& json, ThunderAutoCSVExportProperties& properties);
void to_json(wpi::json& json, const ThunderAutoCSVExportProperties& properties) noexcept;

/**
 * Exports the given ThunderAutoOutputTrajectory to a file at the specified path.
 * This function will overwrite any existing file at the path.
 * This function will throw an exception if a problem occurs while exporting the trajectory.
 *
 * @param output The ThunderAutoOutputTrajectory to export.
 * @param orderedActions A span of ordered action names (used to fill the actions bit field).
 * @param exportPath The path to the file where the trajectory will be exported.
 * @param properties The properties describing what to export in the CSV file.
 */
void CSVExportThunderAutoOutputTrajectory(const ThunderAutoOutputTrajectory& output,
                                          std::span<const std::string> orderedActions,
                                          const std::filesystem::path& exportPath,
                                          const ThunderAutoCSVExportProperties& properties);

/**
 * Builds an output trajectory from a trajectory skeleton and exports it to a CSV file.
 *
 * @param trajectorySkeleton The trajectory skeleton to build the output trajectory from.
 * @param buildSettings The settings for building the output trajectory.
 * @param orderedActions A span of ordered action names (used to fill the actions bit field).
 * @param exportPath The path to the file where the trajectory will be exported.
 * @param exportProperties The properties describing what to export in the CSV file.
 */
void BuildAndCSVExportThunderAutoOutputTrajectory(const ThunderAutoTrajectorySkeleton& trajectorySkeleton,
                                                  const ThunderAutoOutputTrajectorySettings& buildSettings,
                                                  std::span<const std::string> orderedActions,
                                                  const std::filesystem::path& exportPath,
                                                  const ThunderAutoCSVExportProperties& exportProperties);

}  // namespace thunder::core
