#include <ThunderLibCoreTests/ThunderLibCoreTests.hpp>

#include <ThunderLibCore/Auto/ThunderAutoProject.hpp>

using namespace thunder::core;

using ::testing::Contains;

TEST(LoadProjectTests, SaveAndLoad) {
  const std::filesystem::path kTestOutputPath = GetTestOutputPath() / "LoadProjectTests_SaveAndLoad";
  std::filesystem::create_directory(kTestOutputPath);

  const std::filesystem::path projectSavePath = kTestOutputPath / "test.thunderauto";

  ThunderAutoProjectSettings settings;
  {
    settings.setProjectPath(projectSavePath);
    settings.fieldImage = ThunderAutoFieldImage(ThunderAutoBuiltinFieldImage::FIELD_2025);
    settings.driveController = DriveControllerType::HOLONOMIC;
    settings.robotSize = Measurement2d(0.8_m, 0.8_m);
  }

  ThunderAutoProjectState state;

  // Actions
  {
    ThunderAutoAction action1(ThunderAutoActionType::COMMAND);
    state.addAction("action1", action1);

    ThunderAutoAction action2(ThunderAutoActionType::CONCURRENT_ACTION_GROUP, {"action1"});
    state.addAction("action2", action2);

    ThunderAutoAction action3(ThunderAutoActionType::CONCURRENT_ACTION_GROUP, {"action1", "action2"});
    state.addAction("action3", action3);
  }

  // Waypoint Links
  {
    state.waypointLinks.insert("link1");
    state.waypointLinks.insert("link2");
  }

  // Add some trajectories
  {
    ThunderAutoTrajectorySkeleton trajectory;

    ThunderAutoTrajectorySkeletonWaypoint waypoint1(
        Point2d(7.57_m, 7.0_m), CanonicalAngle(180_deg),
        ThunderAutoTrajectorySkeletonWaypoint::HeadingWeights(1.0, 1.0));

    trajectory.appendPoint(waypoint1);

    ThunderAutoTrajectorySkeletonWaypoint waypoint2(
        Point2d(3.5_m, 6.5_m), CanonicalAngle(200_deg),
        ThunderAutoTrajectorySkeletonWaypoint::HeadingWeights(1.5, 1.0));

    trajectory.appendPoint(waypoint2);

    ThunderAutoTrajectorySkeletonWaypoint waypoint3(
        Point2d(2.0_m, 5.0_m), CanonicalAngle(240_deg),
        ThunderAutoTrajectorySkeletonWaypoint::HeadingWeights(1.0, 0.5));
    waypoint3.setLinkName("link1");

    trajectory.appendPoint(waypoint3);

    trajectory.setStartRotation(0.0_deg);
    trajectory.setEndRotation(90.0_deg);

    state.trajectories["Trajectory1"] = trajectory;
  }
  {
    ThunderAutoTrajectorySkeleton trajectory;

    ThunderAutoTrajectorySkeletonWaypoint waypoint1(
        Point2d(2.0_m, 5.0_m), CanonicalAngle(0_deg),
        ThunderAutoTrajectorySkeletonWaypoint::HeadingWeights(1.0, 1.0));
    waypoint1.setLinkName("link1");

    trajectory.appendPoint(waypoint1);

    ThunderAutoTrajectorySkeletonWaypoint waypoint2(
        Point2d(3.0_m, 1.5_m), CanonicalAngle(45_deg),
        ThunderAutoTrajectorySkeletonWaypoint::HeadingWeights(1.0, 1.0));
    waypoint2.setStopped(true);
    waypoint2.setStopRotation(90_deg);
    waypoint2.addStopAction("action1");

    trajectory.appendPoint(waypoint2);

    ThunderAutoTrajectorySkeletonWaypoint waypoint3(
        Point2d(6.0_m, 1.5_m), CanonicalAngle(315_deg),
        ThunderAutoTrajectorySkeletonWaypoint::HeadingWeights(1.0, 0.5));

    trajectory.appendPoint(waypoint3);

    state.trajectories["Trajectory2"] = trajectory;
  }

  // Add an auto mode
  {
    // Trajectory step
    auto step1 = std::make_shared<ThunderAutoModeTrajectoryStep>();
    step1->trajectoryName = "Trajectory1";

    // Action step
    auto step2 = std::make_shared<ThunderAutoModeActionStep>();
    step2->actionName = "action1";

    // Trajectory step
    auto step3 = std::make_shared<ThunderAutoModeTrajectoryStep>();
    step3->trajectoryName = "Trajectory2";

    auto step4True = std::make_shared<ThunderAutoModeActionStep>();
    step2->actionName = "action2";
    auto step4Else = std::make_shared<ThunderAutoModeActionStep>();
    step2->actionName = "action3";

    auto step4 = std::make_shared<ThunderAutoModeBoolBranchStep>();
    step4->trueBranch.push_back(step4True);
    step4->elseBranch.push_back(step4Else);

    auto step5 = std::make_shared<ThunderAutoModeSwitchBranchStep>();

    auto step5Case1 = std::make_shared<ThunderAutoModeActionStep>();
    step5Case1->actionName = "action1";
    auto step5Case2 = std::make_shared<ThunderAutoModeActionStep>();
    step5Case2->actionName = "action2";
    auto step5Default = std::make_shared<ThunderAutoModeActionStep>();
    step5Default->actionName = "action3";

    step5->caseBranches[1] = {step5Case1};
    step5->caseBranches[2] = {step5Case2};
    step5->defaultBranch = {step5Default};

    ThunderAutoMode autoMode;
    autoMode.steps = {step1, step2, step3, step4, step5};

    state.autoModes["AutoMode1"] = autoMode;
  }

  ASSERT_NO_THROW(SaveThunderAutoProject(settings, state));

  ThunderAutoProjectVersion loadedVersion;
  std::unique_ptr<ThunderAutoProject> loadedProject;
  ASSERT_NO_THROW(loadedProject = LoadThunderAutoProject(projectSavePath, &loadedVersion));
  ASSERT_NE(loadedProject, nullptr);

  EXPECT_EQ(loadedVersion.major, THUNDERAUTO_PROJECT_VERSION_MAJOR);
  EXPECT_EQ(loadedVersion.minor, THUNDERAUTO_PROJECT_VERSION_MINOR);

  EXPECT_EQ(loadedProject->settings(), settings);
  EXPECT_EQ(loadedProject->state(), state);
}

/**
 * Test loading a legacy ThunderAuto project file (2024+2025 format). Verify that ThunderLibCore reads and
 * converts all its data correctly.
 */
TEST(LoadProjectTests, LoadPre2026Project) {
  std::filesystem::path projectPath = kTestDataPath / "ThunderAuto" / "Pre2026.thunderauto";

  std::unique_ptr<ThunderAutoProject> project;
  ASSERT_NO_THROW(project = LoadThunderAutoProject(projectPath));
  ASSERT_NE(project, nullptr);

  //
  // Verify settings
  //

  ThunderAutoProjectSettings& settings = project->settings();
  EXPECT_EQ(settings.projectPath, projectPath);
  EXPECT_EQ(settings.directory, projectPath.parent_path());
  EXPECT_EQ(settings.name, "Pre2026");

  // Field Image
  {
    ThunderAutoFieldImage& fieldImage = settings.fieldImage;
    ASSERT_EQ(fieldImage.type(), ThunderAutoFieldImageType::BUILTIN);
    ASSERT_NO_THROW(fieldImage.builtinImage());
    EXPECT_EQ(fieldImage.builtinImage(), ThunderAutoBuiltinFieldImage::FIELD_2025);

    Measurement2d fieldSize = fieldImage.fieldSize();
    EXPECT_FLOAT_EQ(fieldSize.x(), 17.548249);
    EXPECT_FLOAT_EQ(fieldSize.y(), 8.077200);

    Rect fieldBounds = fieldImage.imageFieldBounds();
    EXPECT_FLOAT_EQ(fieldBounds.min.x, 0.0795935557);
    EXPECT_FLOAT_EQ(fieldBounds.min.y, 0.1139555361);
    EXPECT_FLOAT_EQ(fieldBounds.max.x, 1.0 - 0.0795935557);
    EXPECT_FLOAT_EQ(fieldBounds.max.y, 1.0 - 0.1139555361);
  }

  EXPECT_EQ(settings.driveController, DriveControllerType::HOLONOMIC);

  Measurement2d robotSize = settings.robotSize;
  EXPECT_FLOAT_EQ(robotSize.x(), 0.8);
  EXPECT_FLOAT_EQ(robotSize.y(), 0.8);

  EXPECT_EQ(settings.autoSave, true);
  EXPECT_EQ(settings.autoCSVExport, false);

  EXPECT_EQ(settings.csvExportProps.time, true);
  EXPECT_EQ(settings.csvExportProps.position, true);
  EXPECT_EQ(settings.csvExportProps.linearVelocity, true);
  EXPECT_EQ(settings.csvExportProps.componentVelocities, false);
  EXPECT_EQ(settings.csvExportProps.heading, false);
  EXPECT_EQ(settings.csvExportProps.rotation, true);
  EXPECT_EQ(settings.csvExportProps.angularVelocity, false);
  EXPECT_EQ(settings.csvExportProps.actionsBitField, true);
  EXPECT_EQ(settings.csvExportProps.distance, false);
  EXPECT_EQ(settings.csvExportProps.curvature, false);
  EXPECT_EQ(settings.csvExportProps.centripetalAcceleration, false);

  //
  // Verify state
  //

  ThunderAutoProjectState& state = project->state();

  // Trajectories
  {
    std::map<std::string, ThunderAutoTrajectorySkeleton>& trajectories = state.trajectories;
    EXPECT_EQ(trajectories.size(), 2U);

    // Trajectory1
    {
      ASSERT_TRUE(trajectories.contains("Trajectory1"));
      const ThunderAutoTrajectorySkeleton& trajectory = trajectories.at("Trajectory1");

      ASSERT_EQ(trajectory.numPoints(), 3U);

      ThunderAutoTrajectorySkeleton::const_iterator pointIt = trajectory.cbegin();
      // Point 1
      {
        Point2d position = pointIt->position();
        EXPECT_FLOAT_EQ(position.x(), 7.57);
        EXPECT_FLOAT_EQ(position.y(), 7.0);

        ThunderAutoTrajectorySkeletonWaypoint::HeadingAngles headings = pointIt->headings();
        EXPECT_EQ(headings.outgoingAngle(), CanonicalAngle(180_deg));

        ThunderAutoTrajectorySkeletonWaypoint::HeadingWeights headingWeights = pointIt->headingWeights();
        EXPECT_FLOAT_EQ(headingWeights.outgoingWeight(), 1.0);

        EXPECT_FALSE(pointIt->isStopped());

        EXPECT_FALSE(pointIt->isLinked());

        EXPECT_FALSE(pointIt->isEditorLocked());
      }

      ++pointIt;

      // Point 2
      {
        Point2d position = pointIt->position();
        EXPECT_FLOAT_EQ(position.x(), 3.5);
        EXPECT_FLOAT_EQ(position.y(), 6.5);

        ThunderAutoTrajectorySkeletonWaypoint::HeadingAngles headings = pointIt->headings();
        EXPECT_EQ(headings.incomingAngle(), CanonicalAngle(25_deg));
        EXPECT_EQ(headings.outgoingAngle(), CanonicalAngle(205_deg));

        ThunderAutoTrajectorySkeletonWaypoint::HeadingWeights headingWeights = pointIt->headingWeights();
        EXPECT_FLOAT_EQ(headingWeights.incomingWeight(), 1.5);
        EXPECT_FLOAT_EQ(headingWeights.outgoingWeight(), 1.0);

        EXPECT_FALSE(pointIt->isStopped());

        EXPECT_FALSE(pointIt->isLinked());

        EXPECT_TRUE(pointIt->isEditorLocked());
      }

      ++pointIt;

      // Point 3
      {
        Point2d position = pointIt->position();
        EXPECT_FLOAT_EQ(position.x(), 2.0);
        EXPECT_FLOAT_EQ(position.y(), 5.0);

        ThunderAutoTrajectorySkeletonWaypoint::HeadingAngles headings = pointIt->headings();
        EXPECT_EQ(headings.incomingAngle(), CanonicalAngle(60_deg));

        ThunderAutoTrajectorySkeletonWaypoint::HeadingWeights headingWeights = pointIt->headingWeights();
        EXPECT_FLOAT_EQ(headingWeights.incomingWeight(), 1.0);

        EXPECT_FALSE(pointIt->isStopped());

        EXPECT_TRUE(pointIt->isLinked());
        EXPECT_EQ(pointIt->linkName(), "MyLink1");

        EXPECT_FALSE(pointIt->isEditorLocked());
      }

      // Actions
      {
        const std::unordered_set<std::string>& startActions = trajectory.startActions();
        EXPECT_TRUE(startActions.empty());

        const ThunderAutoPositionedTrajectoryItemList<ThunderAutoTrajectoryAction>& actions =
            trajectory.actions();
        ASSERT_EQ(actions.size(), 1U);

        auto actionIt = actions.cbegin();
        {
          EXPECT_FLOAT_EQ(actionIt->first, 1.0);  // position

          ThunderAutoTrajectoryAction action = {.action = "Action1", .editorLocked = false};
          EXPECT_EQ(actionIt->second, action);  // action
        }

        const std::unordered_set<std::string>& endActions = trajectory.endActions();
        EXPECT_TRUE(endActions.empty());
      }

      // Rotations
      {
        EXPECT_EQ(trajectory.startRotation(), CanonicalAngle(180_deg));

        const ThunderAutoPositionedTrajectoryItemList<ThunderAutoTrajectoryRotation>& rotations =
            trajectory.rotations();
        ASSERT_EQ(rotations.size(), 1U);

        auto rotationIt = rotations.cbegin();
        {
          EXPECT_FLOAT_EQ(rotationIt->first, 0.65);  // position

          ThunderAutoTrajectoryRotation rotation = {.angle = CanonicalAngle(90_deg), .editorLocked = false};
          EXPECT_EQ(rotationIt->second, rotation);  // angle
        }

        EXPECT_EQ(trajectory.endRotation(), CanonicalAngle(0_deg));
      }
    }

    // Trajectory2
    {
      ASSERT_TRUE(trajectories.contains("Trajectory2"));
      const ThunderAutoTrajectorySkeleton& trajectory = trajectories.at("Trajectory2");

      ASSERT_EQ(trajectory.numPoints(), 3U);

      ThunderAutoTrajectorySkeleton::const_iterator pointIt = trajectory.cbegin();
      // Point 1
      {
        Point2d position = pointIt->position();
        EXPECT_FLOAT_EQ(position.x(), 2.0);
        EXPECT_FLOAT_EQ(position.y(), 5.0);

        ThunderAutoTrajectorySkeletonWaypoint::HeadingAngles headings = pointIt->headings();
        EXPECT_EQ(headings.outgoingAngle(), CanonicalAngle(0_deg));

        ThunderAutoTrajectorySkeletonWaypoint::HeadingWeights headingWeights = pointIt->headingWeights();
        EXPECT_FLOAT_EQ(headingWeights.outgoingWeight(), 1.0);

        EXPECT_FALSE(pointIt->isStopped());

        EXPECT_TRUE(pointIt->isLinked());
        EXPECT_EQ(pointIt->linkName(), "MyLink1");

        EXPECT_FALSE(pointIt->isEditorLocked());
      }

      ++pointIt;

      // Point 2
      {
        Point2d position = pointIt->position();
        EXPECT_FLOAT_EQ(position.x(), 3.0);
        EXPECT_FLOAT_EQ(position.y(), 1.5);

        ThunderAutoTrajectorySkeletonWaypoint::HeadingAngles headings = pointIt->headings();
        EXPECT_EQ(headings.incomingAngle(), CanonicalAngle(180_deg));
        EXPECT_EQ(headings.outgoingAngle(), CanonicalAngle(45_deg));

        ThunderAutoTrajectorySkeletonWaypoint::HeadingWeights headingWeights = pointIt->headingWeights();
        EXPECT_FLOAT_EQ(headingWeights.incomingWeight(), 1.0);
        EXPECT_FLOAT_EQ(headingWeights.outgoingWeight(), 1.0);

        EXPECT_TRUE(pointIt->isStopped());
        EXPECT_EQ(pointIt->stopRotation(), CanonicalAngle(90_deg));

        EXPECT_FALSE(pointIt->isLinked());

        EXPECT_FALSE(pointIt->isEditorLocked());
      }

      ++pointIt;

      // Point 3
      {
        Point2d position = pointIt->position();
        EXPECT_FLOAT_EQ(position.x(), 6.0);
        EXPECT_FLOAT_EQ(position.y(), 1.5);

        ThunderAutoTrajectorySkeletonWaypoint::HeadingAngles headings = pointIt->headings();
        EXPECT_EQ(headings.incomingAngle(), CanonicalAngle(135_deg));

        ThunderAutoTrajectorySkeletonWaypoint::HeadingWeights headingWeights = pointIt->headingWeights();
        EXPECT_FLOAT_EQ(headingWeights.incomingWeight(), 1.0);

        EXPECT_FALSE(pointIt->isStopped());

        EXPECT_FALSE(pointIt->isLinked());

        EXPECT_FALSE(pointIt->isEditorLocked());
      }

      // Actions
      {
        const std::unordered_set<std::string>& startActions = trajectory.startActions();
        EXPECT_TRUE(startActions.empty());

        const ThunderAutoPositionedTrajectoryItemList<ThunderAutoTrajectoryAction>& actions =
            trajectory.actions();
        ASSERT_EQ(actions.size(), 2U);

        auto actionIt = actions.cbegin();
        {
          EXPECT_FLOAT_EQ(actionIt->first, 1.0);  // position

          ThunderAutoTrajectoryAction action = {.action = "Action1", .editorLocked = false};
          EXPECT_EQ(actionIt->second, action);  // action
        }
        ++actionIt;
        {
          EXPECT_FLOAT_EQ(actionIt->first, 1.0);  // position

          ThunderAutoTrajectoryAction action = {.action = "Action2", .editorLocked = false};
          EXPECT_EQ(actionIt->second, action);  // action
        }

        const std::unordered_set<std::string>& endActions = trajectory.endActions();
        ASSERT_EQ(endActions.size(), 1U);
        EXPECT_THAT(endActions, Contains("Action2"));
      }

      // Rotations
      {
        EXPECT_EQ(trajectory.startRotation(), CanonicalAngle(0_deg));

        const ThunderAutoPositionedTrajectoryItemList<ThunderAutoTrajectoryRotation>& rotations =
            trajectory.rotations();
        /**
         * The waypoint rotation should not be converted to a rotation target because the waypoint is a "stop"
         * waypoint, so the new waypoint's stop rotation should be set instead.
         */
        ASSERT_TRUE(rotations.empty());

        EXPECT_EQ(trajectory.endRotation(), CanonicalAngle(0_deg));
      }
    }
  }

  EXPECT_TRUE(state.autoModes.empty());  // Auto Modes shouldn't exist. They were added in 2026

  EXPECT_EQ(state.actions.size(), 2U);
  EXPECT_TRUE(state.actions.contains("Action1"));
  EXPECT_TRUE(state.actions.contains("Action2"));

  EXPECT_EQ(state.waypointLinks.size(), 1U);
  EXPECT_TRUE(state.waypointLinks.contains("MyLink1"));

  //  ThunderAutoEditorState defaultEditorState;
  //  EXPECT_EQ(state.editorState, defaultEditorState);
}

