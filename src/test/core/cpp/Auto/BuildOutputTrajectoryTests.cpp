#include <ThunderLibCoreTests/ThunderLibCoreTests.hpp>

#include <ThunderLibCore/Auto/ThunderAutoProject.hpp>
#include <ThunderLibCore/Auto/ThunderAutoOutputTrajectory.hpp>

using namespace thunder::core;

TEST(BuildOutputTrajectoryTests, BuildTrajectory) {
  const std::filesystem::path kTestOutputPath =
      GetTestOutputPath() / "BuildOutputTrajectoryTests_BuildTrajectory";
  std::filesystem::create_directory(kTestOutputPath);

  std::vector<std::string> actions = {"action1", "action2", "action3", "action4"};

  // Make a trajectory skeleton (2025 field).
  // This trajectory has a stop point in the middle, a rotation waypoint 25% of the way through, and a few
  // actions.

  ThunderAutoTrajectorySkeleton trajectory;
  {
    trajectory.addStartAction("action1");
    trajectory.addStartAction("action2");
    trajectory.setStartRotation(180.0_deg);

    ThunderAutoTrajectorySkeletonWaypoint waypoint1(
        Point2d(2_m, 5_m), CanonicalAngle(0_deg),
        ThunderAutoTrajectorySkeletonWaypoint::HeadingWeights(1.0, 1.0));

    trajectory.appendPoint(waypoint1);

    trajectory.rotations().add(0.5, {.angle = 90_deg});

    ThunderAutoTrajectorySkeletonWaypoint waypoint2(
        Point2d(3_m, 1.5_m), 0_mps, ThunderAutoTrajectorySkeletonWaypoint::HeadingAngles(180_deg, 45_deg),
        ThunderAutoTrajectorySkeletonWaypoint::HeadingWeights(1.0, 1.0));
    waypoint2.setStopRotation(90.0_deg);
    waypoint2.addStopAction("action1");

    trajectory.appendPoint(waypoint2);
    trajectory.actions().add(1.25, {.action = "action3"});

    ThunderAutoTrajectorySkeletonWaypoint waypoint3(
        Point2d(6.0_m, 1.5_m), CanonicalAngle(-45_deg),
        ThunderAutoTrajectorySkeletonWaypoint::HeadingWeights(1.0, 0.5));

    trajectory.appendPoint(waypoint3);

    trajectory.addEndAction("action4");
    trajectory.setEndRotation(180.0_deg);
  }

  // Low-res settings for testing.
  ThunderAutoOutputTrajectorySettings settings = {
      .lengthSamples = 50,
      .samplesPerMeter = 25,
  };

  // Build the trajectory.

  std::unique_ptr<ThunderAutoOutputTrajectory> outputTrajectory;
  ASSERT_NO_THROW(outputTrajectory = BuildThunderAutoOutputTrajectory(trajectory, settings));
  ASSERT_NE(outputTrajectory, nullptr);

  // Export default properties to CSV.
  {
    const std::filesystem::path csvExportPath = kTestOutputPath / "simpleTrajectory_defaultProps.csv";
    ThunderAutoCSVExportProperties csvExportProps = ThunderAutoCSVExportProperties::Default();

    CSVExportThunderAutoOutputTrajectory(*outputTrajectory, actions, csvExportPath, csvExportProps);
  }

  // No header
  {
    const std::filesystem::path csvExportPath =
        kTestOutputPath / "simpleTrajectory_defaultProps_noHeader.csv";
    ThunderAutoCSVExportProperties csvExportProps = ThunderAutoCSVExportProperties::Default();
    csvExportProps.includeHeader = false;

    CSVExportThunderAutoOutputTrajectory(*outputTrajectory, actions, csvExportPath, csvExportProps);
  }

  // Export all properties to CSV.
  {
    const std::filesystem::path csvExportPath = kTestOutputPath / "simpleTrajectory_fullProps.csv";
    ThunderAutoCSVExportProperties csvExportProps = ThunderAutoCSVExportProperties::Full();

    CSVExportThunderAutoOutputTrajectory(*outputTrajectory, actions, csvExportPath, csvExportProps);
  }

  /**
   * I'm not adding checks to the output data for now... It's easy enough to verify manually in Excel whenever
   * build code gets changed.
   */
}

