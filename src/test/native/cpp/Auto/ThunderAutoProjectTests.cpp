#include <ThunderLibTests/ThunderLibTests.hpp>

#include <ThunderLib/Auto/ThunderAutoProject.hpp>

using namespace thunder;

#define DOUBLE_TOLERANCE 1e-2

TEST(ThunderAutoProjectTests, SimpleTrajectoryTest) {
  /**
   * Project with one trajectory, a simple L-shaped trajectory where the robot stops midway through, then
   * starts driving again. This trajectory has no constraints, so its straight lines should be ideal trapezoid
   * profiles that accenerate from 0 m/s to 2 m/s, cruise for a while, then decelerate back down to 0 m/s. The
   * robot will rotate 180 degrees during the first straightaway. There is a start action "StartAction", end
   * action "EndAction", stop action "StopAction", and positioned action "PositionedAction" at position 0.5.
   */
  std::filesystem::path projectPath = kTestResourcesPath / "ThunderAuto" / "SimpleTrajectory.thunderauto";

  auto project = std::make_shared<ThunderAutoProject>(projectPath);
  ASSERT_NE(project, nullptr);
  ASSERT_TRUE(project->isLoaded());
  ASSERT_EQ(project->getName(), "SimpleTrajectory");

  EXPECT_EQ(project->getFieldSymmetry(), FieldSymmetry::ROTATIONAL);

  const std::string trajectoryName = "MySimpleTrajectory";

  ASSERT_TRUE(project->hasTrajectory(trajectoryName));
  auto trajectory = std::unique_ptr<ThunderAutoTrajectory>(project->getTrajectory(trajectoryName));
  ASSERT_NE(trajectory, nullptr);
  ASSERT_TRUE(trajectory->isValid());

  EXPECT_NEAR(trajectory->getDuration().value(), 5.5, DOUBLE_TOLERANCE);

  {
    const TrajectoryState initialState = trajectory->getInitialState();

    EXPECT_NEAR(initialState.time.value(), 0.0, DOUBLE_TOLERANCE);
    EXPECT_NEAR(initialState.pose.X().value(), 7.8, DOUBLE_TOLERANCE);
    EXPECT_NEAR(initialState.pose.Y().value(), 6.7, DOUBLE_TOLERANCE);
    EXPECT_NEAR(initialState.pose.Rotation().Degrees().value(), 0.0, DOUBLE_TOLERANCE);
    EXPECT_NEAR(initialState.chassisSpeeds.vx.value(), 0.0, DOUBLE_TOLERANCE);
    EXPECT_NEAR(initialState.chassisSpeeds.vy.value(), 0.0, DOUBLE_TOLERANCE);
    EXPECT_NEAR(initialState.chassisSpeeds.omega.value(), 0.0, DOUBLE_TOLERANCE);
    EXPECT_NEAR(initialState.linearVelocity.value(), 0.0, DOUBLE_TOLERANCE);
    EXPECT_NEAR(initialState.heading.Degrees().value(), 180.0, DOUBLE_TOLERANCE);
  }

  {
    const TrajectoryState finalState = trajectory->getFinalState();

    EXPECT_NEAR(finalState.time.value(), 5.5, DOUBLE_TOLERANCE);
    EXPECT_NEAR(finalState.pose.X().value(), 2.8, DOUBLE_TOLERANCE);
    EXPECT_NEAR(finalState.pose.Y().value(), 4.7, DOUBLE_TOLERANCE);
    EXPECT_NEAR(finalState.pose.Rotation().Degrees().value(), 180.0, DOUBLE_TOLERANCE);
    EXPECT_NEAR(finalState.chassisSpeeds.vx.value(), 0.0, DOUBLE_TOLERANCE);
    EXPECT_NEAR(finalState.chassisSpeeds.vy.value(), 0.0, DOUBLE_TOLERANCE);
    EXPECT_NEAR(finalState.chassisSpeeds.omega.value(), 0.0, DOUBLE_TOLERANCE);
    EXPECT_NEAR(finalState.linearVelocity.value(), 0.0, DOUBLE_TOLERANCE);
    EXPECT_NEAR(finalState.heading.Degrees().value(), -90.0, DOUBLE_TOLERANCE);
  }

  // Stop point
  {
    const TrajectoryState stopState = trajectory->sample(3.5_s);

    EXPECT_NEAR(stopState.time.value(), 3.5, DOUBLE_TOLERANCE);
    EXPECT_NEAR(stopState.pose.X().value(), 2.8, DOUBLE_TOLERANCE);
    EXPECT_NEAR(stopState.pose.Y().value(), 6.7, DOUBLE_TOLERANCE);
    EXPECT_NEAR(stopState.pose.Rotation().Degrees().value(), 180.0, DOUBLE_TOLERANCE);
    EXPECT_NEAR(stopState.chassisSpeeds.vx.value(), 0.0, DOUBLE_TOLERANCE);
    EXPECT_NEAR(stopState.chassisSpeeds.vy.value(), 0.0, DOUBLE_TOLERANCE);
    EXPECT_NEAR(stopState.chassisSpeeds.omega.value(), 0.0, DOUBLE_TOLERANCE);
    EXPECT_NEAR(stopState.linearVelocity.value(), 0.0, DOUBLE_TOLERANCE);
    EXPECT_NEAR(stopState.heading.Degrees().value(), -90.0, DOUBLE_TOLERANCE);

    const std::map<units::second_t, std::string>& stopActions = trajectory->getStopActions();
    ASSERT_EQ(stopActions.size(), 1UL);
    const auto& [stopTime, stopActionName] = *stopActions.begin();
    EXPECT_NEAR(stopTime.value(), 3.5, DOUBLE_TOLERANCE);
    EXPECT_EQ(stopActionName, "StopAction");
  }

  // Start & end actions
  EXPECT_EQ(trajectory->getStartAction(), "StartAction");
  EXPECT_EQ(trajectory->getEndAction(), "EndAction");

  // Positioned action
  {
    const std::multimap<units::second_t, std::string>& actions = trajectory->getActions();
    ASSERT_EQ(actions.size(), 1UL);
    const auto& [time, name] = *actions.begin();
    EXPECT_NEAR(time.value(), 1.75, DOUBLE_TOLERANCE);
    EXPECT_EQ(name, "PositionedAction");
  }

  /**
   * Check state at one point. No need for more since the same trajectory build gets tested more thoroughly in
   * ThunderLibCore's ThunderAutoBuildOutputTrajectoryTests suite.
   */
  {
    const TrajectoryState state = trajectory->sample(1.75_s);

    EXPECT_NEAR(state.time.value(), 1.75, DOUBLE_TOLERANCE);
    EXPECT_NEAR(state.pose.X().value(), 7.8 - 2.5, DOUBLE_TOLERANCE);
    EXPECT_NEAR(state.pose.Y().value(), 6.7, DOUBLE_TOLERANCE);
    EXPECT_NEAR(std::abs(state.pose.Rotation().Degrees().value()), 90.0, 0.5);
    EXPECT_NEAR(state.chassisSpeeds.vx.value(), -2.0, DOUBLE_TOLERANCE);
    EXPECT_NEAR(state.chassisSpeeds.vy.value(), 0.0, DOUBLE_TOLERANCE);
    EXPECT_GT(std::abs(state.chassisSpeeds.omega.value()), 0.0);
    EXPECT_NEAR(state.linearVelocity.value(), 2.0, DOUBLE_TOLERANCE);
    EXPECT_NEAR(std::abs(state.heading.Degrees().value()), 180.0, DOUBLE_TOLERANCE);
  }
}
