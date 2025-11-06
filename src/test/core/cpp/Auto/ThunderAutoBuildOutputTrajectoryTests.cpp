#include <ThunderLibCoreTests/ThunderLibCoreTests.hpp>

#include <ThunderLibCore/Auto/ThunderAutoProject.hpp>
#include <ThunderLibCore/Auto/ThunderAutoOutputTrajectory.hpp>

using namespace thunder::core;

TEST(ThunderAutoBuildOutputTrajectoryTests, BuildTrajectory) {
  const std::filesystem::path kTestOutputPath =
      GetTestOutputPath() / "BuildOutputTrajectoryTests_BuildTrajectory";
  std::filesystem::create_directory(kTestOutputPath);

  std::vector<std::string> actions = {"action1", "action2", "action3", "action4"};

  // Make a trajectory skeleton (2025 field).
  // This trajectory has a stop point in the middle, a rotation waypoint 25% of the way through, and a few
  // actions.

  ThunderAutoTrajectorySkeleton trajectory;
  {
    trajectory.setStartAction("action1");
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
    waypoint2.setStopAction("action1");

    trajectory.appendPoint(waypoint2);
    trajectory.actions().add(1.25, {.action = "action3"});

    ThunderAutoTrajectorySkeletonWaypoint waypoint3(
        Point2d(6.0_m, 1.5_m), CanonicalAngle(-45_deg),
        ThunderAutoTrajectorySkeletonWaypoint::HeadingWeights(1.0, 0.5));

    trajectory.appendPoint(waypoint3);

    trajectory.setEndAction("action4");
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

#define DOUBLE_TOLERANCE 1e-2

TEST(ThunderAutoBuildOutputTrajectoryTests, BuildSimpleTrajectory) {
  std::filesystem::path projectPath = kTestResourcesPath / "ThunderAuto" / "SimpleTrajectory.thunderauto";

  std::unique_ptr<ThunderAutoProject> project;
  ASSERT_NO_THROW(project = LoadThunderAutoProject(projectPath));
  ASSERT_NE(project, nullptr);

  // Don't care about settings - that's tested elsewhere.

  // Get the trajectory skeleton.

  const std::string trajectoryName = "MySimpleTrajectory";

  const ThunderAutoProjectState& state = project->state();
  ASSERT_TRUE(state.trajectories.contains(trajectoryName));

  const ThunderAutoTrajectorySkeleton& skeleton = state.trajectories.at(trajectoryName);

  // Build the trajectory.

  std::unique_ptr<ThunderAutoOutputTrajectory> trajectory;
  ASSERT_NO_THROW(trajectory = BuildThunderAutoOutputTrajectory(skeleton, kHighResOutputTrajectorySettings));
  ASSERT_NE(trajectory, nullptr);

  EXPECT_NEAR(trajectory->totalTime.value(), 5.5, DOUBLE_TOLERANCE);

  // Start & end actions
  ASSERT_EQ(trajectory->startAction, "StartAction");
  ASSERT_EQ(trajectory->endAction, "EndAction");

  // Positioned action
  {
    ASSERT_EQ(trajectory->actions.size(), 1UL);
    const auto& [time, name] = *trajectory->actions.begin();
    EXPECT_NEAR(time.value(), 1.75, DOUBLE_TOLERANCE);
    EXPECT_EQ(name, "PositionedAction");
  }

  // Check states at various times.

  auto sample = [&trajectory](units::second_t time) {
    auto it = std::lower_bound(
        trajectory->points.begin(), trajectory->points.end(), time,
        [](const ThunderAutoOutputTrajectoryPoint& point, units::second_t t) { return point.time < t; });

    return it;
  };

  const frc::Pose2d expectedStartPose(7.8_m, 6.7_m, frc::Rotation2d(0_deg));

  // The start of the first segment.
  {
    auto it = sample(0.0_s);
    ASSERT_NE(it, trajectory->points.end());

    const ThunderAutoOutputTrajectoryPoint& point = *it;

    EXPECT_NEAR(point.time.value(), 0.0, DOUBLE_TOLERANCE);

    EXPECT_NEAR(point.distance.value(), 0.0, DOUBLE_TOLERANCE);
    EXPECT_NEAR(point.position.x.value(), expectedStartPose.X().value(), DOUBLE_TOLERANCE);
    EXPECT_NEAR(point.position.y.value(), expectedStartPose.Y().value(), DOUBLE_TOLERANCE);
    EXPECT_NEAR(std::abs(point.rotation.degrees().value()), expectedStartPose.Rotation().Degrees().value(),
                DOUBLE_TOLERANCE);

    EXPECT_NEAR(point.chassisSpeeds.vx.value(), 0.0, DOUBLE_TOLERANCE);
    EXPECT_NEAR(point.chassisSpeeds.vy.value(), 0.0, DOUBLE_TOLERANCE);

    EXPECT_NEAR(point.linearVelocity.value(), 0.0, DOUBLE_TOLERANCE);
    EXPECT_NEAR(point.angularVelocity.value(), 0.0, DOUBLE_TOLERANCE);

    EXPECT_NEAR(std::abs(point.heading.degrees().value()), 180,
                DOUBLE_TOLERANCE);  // -180 or +180, they're the same
  }

  // Acceleration
  {
    auto it = sample(0.5_s);
    ASSERT_NE(it, trajectory->points.end());

    const ThunderAutoOutputTrajectoryPoint& point = *it;

    EXPECT_NEAR(point.time.value(), 0.5, DOUBLE_TOLERANCE);

    frc::Pose2d expectedPose = expectedStartPose + frc::Transform2d(-0.25_m, 0_m, frc::Rotation2d(0.0_rad));
    EXPECT_NEAR(point.distance.value(), 0.25, DOUBLE_TOLERANCE);
    EXPECT_NEAR(point.position.x.value(), expectedPose.X().value(), DOUBLE_TOLERANCE);
    EXPECT_NEAR(point.position.y.value(), expectedPose.Y().value(), DOUBLE_TOLERANCE);
    EXPECT_GT(std::abs(point.rotation.degrees().value()), 0.0);  // Should be turning

    EXPECT_NEAR(point.chassisSpeeds.vx.value(), -1.0, DOUBLE_TOLERANCE);  // Halfway to peak velocity
    EXPECT_NEAR(point.chassisSpeeds.vy.value(), 0.0, DOUBLE_TOLERANCE);

    EXPECT_NEAR(point.linearVelocity.value(), 1.0, DOUBLE_TOLERANCE);  // Halfway to peak velocity
    EXPECT_GT(std::abs(point.angularVelocity.value()), 0.0);           // Should be turning

    EXPECT_NEAR(std::abs(point.heading.degrees().value()), 180,
                DOUBLE_TOLERANCE);  // -180 or +180, they're the same
  }

  // The center of the first segment.
  {
    auto it = sample(1.75_s);
    ASSERT_NE(it, trajectory->points.end());

    const ThunderAutoOutputTrajectoryPoint& point = *it;

    EXPECT_NEAR(point.time.value(), 1.75, DOUBLE_TOLERANCE);

    frc::Pose2d expectedPose = expectedStartPose + frc::Transform2d(-2.5_m, 0_m, frc::Rotation2d(90_deg));
    EXPECT_NEAR(point.distance.value(), 2.5, DOUBLE_TOLERANCE);
    EXPECT_NEAR(point.position.x.value(), expectedPose.X().value(), DOUBLE_TOLERANCE);
    EXPECT_NEAR(point.position.y.value(), expectedPose.Y().value(), DOUBLE_TOLERANCE);
    EXPECT_NEAR(std::abs(point.rotation.degrees().value()), expectedPose.Rotation().Degrees().value(),
                0.5);  // Turning one way or the other, don't care

    EXPECT_NEAR(point.chassisSpeeds.vx.value(), -2.0, DOUBLE_TOLERANCE);  // Peak (max configured) velocity
    EXPECT_NEAR(point.chassisSpeeds.vy.value(), 0.0, DOUBLE_TOLERANCE);

    EXPECT_NEAR(point.linearVelocity.value(), 2.0, DOUBLE_TOLERANCE);  // Peak (max configured) velocity
    EXPECT_GT(std::abs(point.angularVelocity.value()), 0.0);           // Should be turning

    EXPECT_NEAR(std::abs(point.heading.degrees().value()), 180,
                DOUBLE_TOLERANCE);  // -180 or +180, they're the same
  }
}
