#include <ThunderLibCoreTests/ThunderLibCoreTests.hpp>

#include <ThunderLibCore/Auto/ThunderAutoMode.hpp>
#include <ThunderLibCore/Auto/ThunderAutoProject.hpp>

using namespace thunder::core;

using ::testing::Contains;

TEST(ThunderAutoModeTests, VerifyContinuityGoodAutoMode) {
  /*

  |- Action
  |- Boolean Branch
  |  |- True:
  |  |  |- (1,2,90_deg -> 1,3,135_deg) (Trajectory1)
  |  |  |- (1,3,135_deg -> 3,4,180_deg) (Trajectory2)
  |  |- False:
  |  |  |- (2,3,180_deg -> 3,4,180_deg) (Trajectory3)
  |- (3,4,180_deg -> 5,7,90_deg) (Trajectory4)
  |- Switch Branch
  |  |- Case 0:
  |  |  |- (5,7,90_deg -> 9,10,180_deg) (Trajectory5)
  |  |- Case 1:
  |  |  |- (5,7,90_deg -> 6,8,135_deg) (Trajectory6)
  |  |  |- (6,8,135_deg -> 9,10,180_deg) (Trajectory7)
  |  |- Default:
  |  |  |- (5,7,90_deg -> 9,10,180_deg) (Trajectory8)
  |- Boolean Branch
  |  |- True:
  |  |  |- (9,10,180_deg -> 9,11,180_deg) (Trajectory9)
  |  |  |- (9,11,180_deg -> 9,10,180_deg) (Trajectory10)
  |  |- False:

  */

  std::map<std::string, ThunderAutoTrajectorySkeleton> trajectories;
  trajectories["Trajectory1"] = ThunderAutoTrajectorySkeleton(
      {
          ThunderAutoTrajectorySkeletonWaypoint{Point2d(1_m, 2_m), 0_deg, {}},
          ThunderAutoTrajectorySkeletonWaypoint{Point2d(1_m, 3_m), 0_deg, {}},
      },
      90_deg, 135_deg);

  trajectories["Trajectory2"] = ThunderAutoTrajectorySkeleton(
      {
          ThunderAutoTrajectorySkeletonWaypoint{Point2d(1_m, 3_m), 0_deg, {}},
          ThunderAutoTrajectorySkeletonWaypoint{Point2d(3_m, 4_m), 0_deg, {}},
      },
      135_deg, 180_deg);

  trajectories["Trajectory3"] = ThunderAutoTrajectorySkeleton(
      {
          ThunderAutoTrajectorySkeletonWaypoint{Point2d(2_m, 3_m), 0_deg, {}},
          ThunderAutoTrajectorySkeletonWaypoint{Point2d(3_m, 4_m), 0_deg, {}},
      },
      180_deg, 180_deg);

  trajectories["Trajectory4"] = ThunderAutoTrajectorySkeleton(
      {
          ThunderAutoTrajectorySkeletonWaypoint{Point2d(3_m, 4_m), 0_deg, {}},
          ThunderAutoTrajectorySkeletonWaypoint{Point2d(5_m, 7_m), 0_deg, {}},
      },
      180_deg, 90_deg);

  trajectories["Trajectory5"] = ThunderAutoTrajectorySkeleton(
      {
          ThunderAutoTrajectorySkeletonWaypoint{Point2d(5_m, 7_m), 0_deg, {}},
          ThunderAutoTrajectorySkeletonWaypoint{Point2d(9_m, 10_m), 0_deg, {}},
      },
      90_deg, 180_deg);

  trajectories["Trajectory6"] = ThunderAutoTrajectorySkeleton(
      {
          ThunderAutoTrajectorySkeletonWaypoint{Point2d(5_m, 7_m), 0_deg, {}},
          ThunderAutoTrajectorySkeletonWaypoint{Point2d(6_m, 8_m), 0_deg, {}},
      },
      90_deg, 135_deg);

  trajectories["Trajectory7"] = ThunderAutoTrajectorySkeleton(
      {
          ThunderAutoTrajectorySkeletonWaypoint{Point2d(6_m, 8_m), 0_deg, {}},
          ThunderAutoTrajectorySkeletonWaypoint{Point2d(9_m, 10_m), 0_deg, {}},
      },
      135_deg, 180_deg);

  trajectories["Trajectory8"] = ThunderAutoTrajectorySkeleton(
      {
          ThunderAutoTrajectorySkeletonWaypoint{Point2d(5_m, 7_m), 0_deg, {}},
          ThunderAutoTrajectorySkeletonWaypoint{Point2d(9_m, 10_m), 0_deg, {}},
      },
      90_deg, 180_deg);

  trajectories["Trajectory9"] = ThunderAutoTrajectorySkeleton(
      {
          ThunderAutoTrajectorySkeletonWaypoint{Point2d(9_m, 10_m), 0_deg, {}},
          ThunderAutoTrajectorySkeletonWaypoint{Point2d(9_m, 11_m), 0_deg, {}},
      },
      180_deg, 180_deg);

  trajectories["Trajectory10"] = ThunderAutoTrajectorySkeleton(
      {
          ThunderAutoTrajectorySkeletonWaypoint{Point2d(9_m, 11_m), 0_deg, {}},
          ThunderAutoTrajectorySkeletonWaypoint{Point2d(9_m, 10_m), 0_deg, {}},
      },
      180_deg, 180_deg);

  ThunderAutoMode autoMode;
  {
    // Action step
    auto step1 = std::make_unique<ThunderAutoModeActionStep>();
    step1->actionName = "Action1";

    // Bool Branch step
    auto step2TrueStep1 = std::make_unique<ThunderAutoModeTrajectoryStep>();
    step2TrueStep1->trajectoryName = "Trajectory1";
    auto step2TrueStep2 = std::make_unique<ThunderAutoModeTrajectoryStep>();
    step2TrueStep2->trajectoryName = "Trajectory2";
    auto step2Else = std::make_unique<ThunderAutoModeTrajectoryStep>();
    step2Else->trajectoryName = "Trajectory3";

    auto step2 = std::make_unique<ThunderAutoModeBoolBranchStep>();
    step2->conditionName = "booleanCondition1";
    step2->trueBranch.push_back(std::move(step2TrueStep1));
    step2->trueBranch.push_back(std::move(step2TrueStep2));
    step2->elseBranch.push_back(std::move(step2Else));

    // Trajectory Step
    auto step3 = std::make_unique<ThunderAutoModeTrajectoryStep>();
    step3->trajectoryName = "Trajectory4";

    // Switch Branch step
    auto step4Case0 = std::make_unique<ThunderAutoModeTrajectoryStep>();
    step4Case0->trajectoryName = "Trajectory5";
    auto step4Case1Step1 = std::make_unique<ThunderAutoModeTrajectoryStep>();
    step4Case1Step1->trajectoryName = "Trajectory6";
    auto step4Case1Step2 = std::make_unique<ThunderAutoModeTrajectoryStep>();
    step4Case1Step2->trajectoryName = "Trajectory7";
    auto step4Default = std::make_unique<ThunderAutoModeTrajectoryStep>();
    step4Default->trajectoryName = "Trajectory8";

    auto step4 = std::make_unique<ThunderAutoModeSwitchBranchStep>();
    step4->conditionName = "switchCondition1";
    step4->caseBranches[0].push_back(std::move(step4Case0));
    step4->caseBranches[1].push_back(std::move(step4Case1Step1));
    step4->caseBranches[1].push_back(std::move(step4Case1Step2));
    step4->defaultBranch.push_back(std::move(step4Default));

    auto step5TrueStep1 = std::make_unique<ThunderAutoModeTrajectoryStep>();
    step5TrueStep1->trajectoryName = "Trajectory9";
    auto step5TrueStep2 = std::make_unique<ThunderAutoModeTrajectoryStep>();
    step5TrueStep2->trajectoryName = "Trajectory10";

    auto step5 = std::make_unique<ThunderAutoModeBoolBranchStep>();
    step5->conditionName = "booleanCondition2";
    step5->trueBranch.push_back(std::move(step5TrueStep1));
    step5->trueBranch.push_back(std::move(step5TrueStep2));

    autoMode.steps.push_back(std::move(step1));
    autoMode.steps.push_back(std::move(step2));
    autoMode.steps.push_back(std::move(step3));
    autoMode.steps.push_back(std::move(step4));
    autoMode.steps.push_back(std::move(step5));
  }

  ThunderAutoModeStepTrajectoryBehavior behavior = autoMode.getTrajectoryBehavior(trajectories);
  ASSERT_TRUE(behavior.runsTrajectory);
  ASSERT_FALSE(behavior.errorInfo);
  EXPECT_EQ(behavior.startPose, std::nullopt);
  EXPECT_EQ(behavior.endPose, frc::Pose2d(9_m, 10_m, 180_deg));
  ASSERT_TRUE(behavior.trajectoryStepRange.has_value());
  EXPECT_EQ(behavior.trajectoryStepRange->first, 1UL);
  EXPECT_EQ(behavior.trajectoryStepRange->second, 4UL);
}

TEST(ThunderAutoModeTests, VerifyContinuityBadAutoMode) {
  /*

  |- Switch Branch
  |  |- Case 0:
  |  |  |- (5,7,90_deg -> 9,10,180_deg) (Trajectory1)
  |  |- Case 1:
  |  |  |- (5,7,90_deg -> 6,8,135_deg) (Trajectory2)
  |  |  |- (7,8,135_deg -> 9,10,180_deg) (Trajectory3)
  |  |- Default:
  |  |  |- (5,7,90_deg -> 9,10,180_deg) (Trajectory4)

  */

  std::map<std::string, ThunderAutoTrajectorySkeleton> trajectories;
  trajectories["Trajectory1"] = ThunderAutoTrajectorySkeleton(
      {
          ThunderAutoTrajectorySkeletonWaypoint{Point2d(5_m, 7_m), 0_deg, {}},
          ThunderAutoTrajectorySkeletonWaypoint{Point2d(9_m, 10_m), 0_deg, {}},
      },
      90_deg, 180_deg);

  trajectories["Trajectory2"] = ThunderAutoTrajectorySkeleton(
      {
          ThunderAutoTrajectorySkeletonWaypoint{Point2d(5_m, 7_m), 0_deg, {}},
          ThunderAutoTrajectorySkeletonWaypoint{Point2d(6_m, 8_m), 0_deg, {}},
      },
      90_deg, 135_deg);

  trajectories["Trajectory3"] = ThunderAutoTrajectorySkeleton(
      {
          ThunderAutoTrajectorySkeletonWaypoint{Point2d(7_m, 8_m), 0_deg, {}},
          ThunderAutoTrajectorySkeletonWaypoint{Point2d(9_m, 10_m), 0_deg, {}},
      },
      135_deg, 180_deg);

  trajectories["Trajectory4"] = ThunderAutoTrajectorySkeleton(
      {
          ThunderAutoTrajectorySkeletonWaypoint{Point2d(5_m, 7_m), 0_deg, {}},
          ThunderAutoTrajectorySkeletonWaypoint{Point2d(9_m, 10_m), 0_deg, {}},
      },
      90_deg, 180_deg);

  ThunderAutoMode autoMode;
  {
    // Switch Branch step
    auto step1Case0 = std::make_unique<ThunderAutoModeTrajectoryStep>();
    step1Case0->trajectoryName = "Trajectory1";
    auto step1Case1Step1 = std::make_unique<ThunderAutoModeTrajectoryStep>();
    step1Case1Step1->trajectoryName = "Trajectory2";
    auto step1Case1Step2 = std::make_unique<ThunderAutoModeTrajectoryStep>();
    step1Case1Step2->trajectoryName = "Trajectory3";
    auto step1Default = std::make_unique<ThunderAutoModeTrajectoryStep>();
    step1Default->trajectoryName = "Trajectory4";

    auto step1 = std::make_unique<ThunderAutoModeSwitchBranchStep>();
    step1->conditionName = "switchCondition1";
    step1->caseBranches[0].push_back(std::move(step1Case0));
    step1->caseBranches[1].push_back(std::move(step1Case1Step1));
    step1->caseBranches[1].push_back(std::move(step1Case1Step2));
    step1->defaultBranch.push_back(std::move(step1Default));

    autoMode.steps.push_back(std::move(step1));
  }

  ThunderAutoModeStepTrajectoryBehavior behavior = autoMode.getTrajectoryBehavior(trajectories);
  ASSERT_TRUE(behavior.runsTrajectory);
  ASSERT_FALSE(behavior.errorInfo.isTrajectoryMissing);
  ASSERT_TRUE(behavior.errorInfo.containsNonContinuousSequence);
  EXPECT_EQ(behavior.startPose, std::nullopt);
  EXPECT_EQ(behavior.endPose, std::nullopt);
  ASSERT_TRUE(behavior.trajectoryStepRange.has_value());
  EXPECT_EQ(behavior.trajectoryStepRange->first, 0UL);
  EXPECT_EQ(behavior.trajectoryStepRange->second, 0UL);
}

static void VerifyThunderAutoModeStepPath(const ThunderAutoModeStepPath& actual,
                                          const ThunderAutoModeStepPath& expected) {
  std::span<const ThunderAutoModeStepDirectoryPath::Node> actualDirectoryPath = actual.directoryPath().path();
  std::span<const ThunderAutoModeStepDirectoryPath::Node> expectedDirectoryPath =
      expected.directoryPath().path();

  ASSERT_EQ(actualDirectoryPath.size(), expectedDirectoryPath.size());

  for (size_t i = 0; i < actualDirectoryPath.size(); i++) {
    EXPECT_EQ(actualDirectoryPath[i], expectedDirectoryPath[i]);
  }

  EXPECT_EQ(actual.stepIndex(), expected.stepIndex());
}

TEST(ThunderAutoModeTests, CommonPathDepth) {
  {
    const ThunderAutoModeStepPath path = ThunderAutoModeStepPath(2);

    EXPECT_EQ(path.getCommonPathDepth(ThunderAutoModeStepDirectoryPath()), 0UL);
    EXPECT_EQ(path.getCommonPathDepth(ThunderAutoModeStepPath(1)), 0UL);
    EXPECT_EQ(path.getCommonPathDepth(ThunderAutoModeStepPath(2)), 1UL);
    EXPECT_EQ(path.getCommonPathDepth(ThunderAutoModeStepPath(2).switchBranchCase(3)), 1UL);
    EXPECT_EQ(path.getCommonPathDepth(ThunderAutoModeStepPath(2).switchBranchCase(2).step(1)), 1UL);
  }

  {
    const ThunderAutoModeStepDirectoryPath path = ThunderAutoModeStepPath(2).boolBranch(true);

    EXPECT_EQ(path.getCommonPathDepth(ThunderAutoModeStepDirectoryPath()), 0UL);
    EXPECT_EQ(path.getCommonPathDepth(ThunderAutoModeStepPath(1)), 0UL);
    EXPECT_EQ(path.getCommonPathDepth(ThunderAutoModeStepPath(2)), 1UL);
    EXPECT_EQ(path.getCommonPathDepth(ThunderAutoModeStepPath(2).switchBranchCase(3)), 0UL);
    EXPECT_EQ(path.getCommonPathDepth(ThunderAutoModeStepPath(2).switchBranchCase(2).step(1)), 0UL);
    EXPECT_EQ(path.getCommonPathDepth(ThunderAutoModeStepPath(2).boolBranch(true)), 1UL);
    EXPECT_EQ(path.getCommonPathDepth(ThunderAutoModeStepPath(2).boolBranch(true).step(3)), 1UL);
  }

  {
    const ThunderAutoModeStepPath path = ThunderAutoModeStepPath(2).boolBranch(true).step(3);

    EXPECT_EQ(path.getCommonPathDepth(ThunderAutoModeStepDirectoryPath()), 0UL);
    EXPECT_EQ(path.getCommonPathDepth(ThunderAutoModeStepPath(1)), 0UL);
    EXPECT_EQ(path.getCommonPathDepth(ThunderAutoModeStepPath(2)), 1UL);
    EXPECT_EQ(path.getCommonPathDepth(ThunderAutoModeStepPath(2).switchBranchCase(3)), 0UL);
    EXPECT_EQ(path.getCommonPathDepth(ThunderAutoModeStepPath(2).switchBranchCase(2).step(1)), 0UL);
    EXPECT_EQ(path.getCommonPathDepth(ThunderAutoModeStepPath(2).boolBranch(true)), 1UL);
    EXPECT_EQ(path.getCommonPathDepth(ThunderAutoModeStepPath(2).boolBranch(true).step(3)), 2UL);
    EXPECT_EQ(path.getCommonPathDepth(ThunderAutoModeStepPath(2).boolBranch(true).step(3).boolBranch(false)),
              2UL);
  }

  // matchEndStepsWithDirNodes = false

  {
    const ThunderAutoModeStepPath path = ThunderAutoModeStepPath(2);

    EXPECT_EQ(path.getCommonPathDepth(ThunderAutoModeStepDirectoryPath(), false), 0UL);
    EXPECT_EQ(path.getCommonPathDepth(ThunderAutoModeStepPath(1), false), 0UL);
    EXPECT_EQ(path.getCommonPathDepth(ThunderAutoModeStepPath(2), false), 1UL);
    EXPECT_EQ(path.getCommonPathDepth(ThunderAutoModeStepPath(2).switchBranchCase(3), false), 0UL);
    EXPECT_EQ(path.getCommonPathDepth(ThunderAutoModeStepPath(2).switchBranchCase(2).step(1), false), 0UL);
  }

  {
    const ThunderAutoModeStepDirectoryPath path = ThunderAutoModeStepPath(2).boolBranch(true);

    EXPECT_EQ(path.getCommonPathDepth(ThunderAutoModeStepPath(1), false), 0UL);
    EXPECT_EQ(path.getCommonPathDepth(ThunderAutoModeStepPath(2), false), 0UL);
    EXPECT_EQ(path.getCommonPathDepth(ThunderAutoModeStepPath(2).switchBranchCase(2).step(1), false), 0UL);
    EXPECT_EQ(path.getCommonPathDepth(ThunderAutoModeStepPath(2).boolBranch(true).step(3), false), 1UL);
  }

  {
    const ThunderAutoModeStepPath path = ThunderAutoModeStepPath(2).boolBranch(true).step(3);

    EXPECT_EQ(path.getCommonPathDepth(ThunderAutoModeStepDirectoryPath(), false), 0UL);
    EXPECT_EQ(path.getCommonPathDepth(ThunderAutoModeStepPath(1), false), 0UL);
    EXPECT_EQ(path.getCommonPathDepth(ThunderAutoModeStepPath(2), false), 0UL);
    EXPECT_EQ(path.getCommonPathDepth(ThunderAutoModeStepPath(2).switchBranchCase(3), false), 0UL);
    EXPECT_EQ(path.getCommonPathDepth(ThunderAutoModeStepPath(2).switchBranchCase(2).step(1), false), 0UL);
    EXPECT_EQ(path.getCommonPathDepth(ThunderAutoModeStepPath(2).boolBranch(true), false), 1UL);
    EXPECT_EQ(path.getCommonPathDepth(ThunderAutoModeStepPath(2).boolBranch(true).step(3), false), 2UL);
    EXPECT_EQ(
        path.getCommonPathDepth(ThunderAutoModeStepPath(2).boolBranch(true).step(3).boolBranch(false), false),
        1UL);
  }
}

TEST(ThunderAutoModeTests, PathHasParentPath) {
  {
    ThunderAutoModeStepPath path = ThunderAutoModeStepPath(2).switchBranchCase(3).step(1);

    EXPECT_TRUE(path.hasParentPath(ThunderAutoModeStepDirectoryPath()));
    EXPECT_TRUE(path.hasParentPath(ThunderAutoModeStepPath(2)));
    EXPECT_FALSE(path.hasParentPath(ThunderAutoModeStepPath(1)));
    EXPECT_TRUE(path.hasParentPath(ThunderAutoModeStepPath(2).switchBranchCase(3)));
    EXPECT_FALSE(path.hasParentPath(ThunderAutoModeStepPath(2).switchBranchCase(2)));
    EXPECT_FALSE(path.hasParentPath(ThunderAutoModeStepPath(2).switchBranchCase(3).step(1)));
    EXPECT_FALSE(path.hasParentPath(ThunderAutoModeStepPath(2).switchBranchCase(3).step(1).boolBranch(true)));
  }

  {
    ThunderAutoModeStepPath path = ThunderAutoModeStepPath(2);

    EXPECT_TRUE(path.hasParentPath(ThunderAutoModeStepDirectoryPath()));
    EXPECT_FALSE(path.hasParentPath(ThunderAutoModeStepPath(2)));
    EXPECT_FALSE(path.hasParentPath(ThunderAutoModeStepPath(1)));
    EXPECT_FALSE(path.hasParentPath(ThunderAutoModeStepPath(2).switchBranchCase(3)));
    EXPECT_FALSE(path.hasParentPath(ThunderAutoModeStepPath(2).switchBranchCase(2)));
    EXPECT_FALSE(path.hasParentPath(ThunderAutoModeStepPath(2).switchBranchCase(3).step(1)));
    EXPECT_FALSE(path.hasParentPath(ThunderAutoModeStepPath(2).switchBranchCase(3).step(1).boolBranch(true)));
  }
}

TEST(ThunderAutoModeTests, DirectoryPathHasParentPath) {
  {
    ThunderAutoModeStepDirectoryPath path =
        ThunderAutoModeStepPath(2).switchBranchCase(3).step(1).boolBranch(false);

    EXPECT_TRUE(path.hasParentPath(ThunderAutoModeStepDirectoryPath()));
    EXPECT_TRUE(path.hasParentPath(ThunderAutoModeStepPath(2)));
    EXPECT_FALSE(path.hasParentPath(ThunderAutoModeStepPath(1)));
    EXPECT_TRUE(path.hasParentPath(ThunderAutoModeStepPath(2).switchBranchCase(3)));
    EXPECT_FALSE(path.hasParentPath(ThunderAutoModeStepPath(2).switchBranchCase(2)));
    EXPECT_TRUE(path.hasParentPath(ThunderAutoModeStepPath(2).switchBranchCase(3).step(1)));
    EXPECT_FALSE(
        path.hasParentPath(ThunderAutoModeStepPath(2).switchBranchCase(3).step(1).boolBranch(false)));
    EXPECT_FALSE(
        path.hasParentPath(ThunderAutoModeStepPath(2).switchBranchCase(3).step(1).boolBranch(false).step(2)));
  }

  {
    ThunderAutoModeStepDirectoryPath path = ThunderAutoModeStepDirectoryPath();

    EXPECT_FALSE(path.hasParentPath(ThunderAutoModeStepDirectoryPath()));
    EXPECT_FALSE(path.hasParentPath(ThunderAutoModeStepPath(2)));
    EXPECT_FALSE(path.hasParentPath(ThunderAutoModeStepPath(1)));
    EXPECT_FALSE(path.hasParentPath(ThunderAutoModeStepPath(2).switchBranchCase(3)));
    EXPECT_FALSE(path.hasParentPath(ThunderAutoModeStepPath(2).switchBranchCase(2)));
    EXPECT_FALSE(path.hasParentPath(ThunderAutoModeStepPath(2).switchBranchCase(3).step(1)));
  }
}

TEST(ThunderAutoModeTests, UpdateStepPathWithRemovalOfStep) {
  const ThunderAutoModeStepPath kStartPath = ThunderAutoModeStepPath(2).switchBranchCase(3).step(1);

  {
    ThunderAutoModeStepPath path = kStartPath;
    ThunderAutoModeStepPath pathToRemove = ThunderAutoModeStepPath(1);
    path.updateWithRemovalOfStep(pathToRemove);

    VerifyThunderAutoModeStepPath(path, ThunderAutoModeStepPath(1).switchBranchCase(3).step(1));
  }

  {
    ThunderAutoModeStepPath path = kStartPath;
    ThunderAutoModeStepPath pathToRemove = ThunderAutoModeStepPath(2).switchBranchCase(3).step(0);
    path.updateWithRemovalOfStep(pathToRemove);

    VerifyThunderAutoModeStepPath(path, ThunderAutoModeStepPath(2).switchBranchCase(3).step(0));
  }

  {
    ThunderAutoModeStepPath path = kStartPath;
    ThunderAutoModeStepPath pathToRemove = ThunderAutoModeStepPath(3);
    path.updateWithRemovalOfStep(pathToRemove);

    VerifyThunderAutoModeStepPath(path, kStartPath);
  }

  {
    ThunderAutoModeStepPath path = kStartPath;
    ThunderAutoModeStepPath pathToRemove = ThunderAutoModeStepPath(2).switchBranchCase(3).step(2);
    path.updateWithRemovalOfStep(pathToRemove);

    VerifyThunderAutoModeStepPath(path, kStartPath);
  }
}

static ThunderAutoMode CreateSimpleAutoMode() {
  /*

  |- (0) Trajectory1
  |- (1) Action1
  |- (2) Trajectory2
  |- (3) BooleanCondition1
  |      |- True:
  |      |  |- (0) Action2
  |      |- Else:
  |      |  |- (0) Action3
  |- (4) SwitchCondition1
  |      |- Case 1:
  |      |  |- (0) Action1
  |      |- Case 2:
  |      |  |- (0) Action2
  |      |- Default:
  |      |  |- (0) Action3

  */

  // Trajectory step
  auto step1 = std::make_unique<ThunderAutoModeTrajectoryStep>();
  step1->trajectoryName = "Trajectory1";

  // Action step
  auto step2 = std::make_unique<ThunderAutoModeActionStep>();
  step2->actionName = "Action1";

  // Trajectory step
  auto step3 = std::make_unique<ThunderAutoModeTrajectoryStep>();
  step3->trajectoryName = "Trajectory2";

  auto step4True = std::make_unique<ThunderAutoModeActionStep>();
  step4True->actionName = "Action2";
  auto step4Else = std::make_unique<ThunderAutoModeActionStep>();
  step4Else->actionName = "Action3";

  auto step4 = std::make_unique<ThunderAutoModeBoolBranchStep>();
  step4->conditionName = "BooleanCondition1";
  step4->trueBranch.push_back(std::move(step4True));
  step4->elseBranch.push_back(std::move(step4Else));

  auto step5 = std::make_unique<ThunderAutoModeSwitchBranchStep>();

  auto step5Case1 = std::make_unique<ThunderAutoModeActionStep>();
  step5Case1->actionName = "Action1";
  auto step5Case2 = std::make_unique<ThunderAutoModeActionStep>();
  step5Case2->actionName = "Action2";
  auto step5Default = std::make_unique<ThunderAutoModeActionStep>();
  step5Default->actionName = "Action3";

  step5->conditionName = "SwitchCondition1";
  step5->caseBranches[1].push_back(std::move(step5Case1));
  step5->caseBranches[2].push_back(std::move(step5Case2));
  step5->defaultBranch.push_back(std::move(step5Default));

  ThunderAutoMode autoMode;
  autoMode.steps.push_back(std::move(step1));
  autoMode.steps.push_back(std::move(step2));
  autoMode.steps.push_back(std::move(step3));
  autoMode.steps.push_back(std::move(step4));
  autoMode.steps.push_back(std::move(step5));

  return autoMode;
}

TEST(ThunderAutoModeTests, MoveSteps) {
  const ThunderAutoMode kStartAutoMode = CreateSimpleAutoMode();
  const std::string kAutoModeName = "AutoMode";

  ThunderAutoProjectState state;
  state.autoModes[kAutoModeName] = kStartAutoMode;

  state.editorState.view = ThunderAutoEditorState::View::AUTO_MODE;
  ThunderAutoModeEditorState& editorState = state.editorState.autoModeEditorState;
  editorState.currentAutoModeName = kAutoModeName;
  editorState.selectedStepPath = std::nullopt;

  bool res = false;
  ThunderAutoMode expectedAutoMode = kStartAutoMode;

  // Verify do nothing operations

  {
    // Move to same position (does nothing)
    ASSERT_NO_THROW(res = state.currentAutoModeMoveStepAfterOther(ThunderAutoModeStepPath(1),
                                                                  ThunderAutoModeStepPath(1)));
    EXPECT_FALSE(res);
    ASSERT_EQ(state.autoModes[kAutoModeName], kStartAutoMode);
  }

  {
    // Move to same position (does nothing)
    ASSERT_NO_THROW(res = state.currentAutoModeMoveStepAfterOther(ThunderAutoModeStepPath(1),
                                                                  ThunderAutoModeStepPath(0)));
    EXPECT_FALSE(res);
    ASSERT_EQ(state.autoModes[kAutoModeName], kStartAutoMode);
  }

  {
    // Move step under itself (not allowed)
    ASSERT_NO_THROW(res = state.currentAutoModeMoveStepAfterOther(
                        ThunderAutoModeStepPath(3), ThunderAutoModeStepPath(3).boolBranch(true).step(0)));
    EXPECT_FALSE(res);
    ASSERT_EQ(state.autoModes[kAutoModeName], kStartAutoMode);
  }

  {
    // Move to same position (does nothing)
    ASSERT_NO_THROW(res = state.currentAutoModeMoveStepBeforeOther(ThunderAutoModeStepPath(0),
                                                                   ThunderAutoModeStepPath(0)));
    EXPECT_FALSE(res);
    ASSERT_EQ(state.autoModes[kAutoModeName], kStartAutoMode);
  }

  {
    // Move to same position (does nothing)
    ASSERT_NO_THROW(res = state.currentAutoModeMoveStepBeforeOther(ThunderAutoModeStepPath(0),
                                                                   ThunderAutoModeStepPath(1)));
    EXPECT_FALSE(res);
    ASSERT_EQ(state.autoModes[kAutoModeName], kStartAutoMode);
  }

  {
    // Move step under itself (not allowed)
    ASSERT_NO_THROW(res = state.currentAutoModeMoveStepBeforeOther(
                        ThunderAutoModeStepPath(3), ThunderAutoModeStepPath(3).boolBranch(true).step(0)));
    EXPECT_FALSE(res);
    ASSERT_EQ(state.autoModes[kAutoModeName], kStartAutoMode);
  }

  {
    // Move step into current directory (does nothing)
    ASSERT_NO_THROW(res = state.currentAutoModeMoveStepIntoDirectory(ThunderAutoModeStepPath(0),
                                                                     ThunderAutoModeStepDirectoryPath()));
    EXPECT_FALSE(res);
    ASSERT_EQ(state.autoModes[kAutoModeName], kStartAutoMode);
  }

  {
    // Move step into current directory (does nothing)
    ASSERT_NO_THROW(
        res = state.currentAutoModeMoveStepIntoDirectory(ThunderAutoModeStepPath(3).boolBranch(true).step(0),
                                                         ThunderAutoModeStepPath(3).boolBranch(true)));
    EXPECT_FALSE(res);
    ASSERT_EQ(state.autoModes[kAutoModeName], kStartAutoMode);
  }

  {
    // Move step under itself (not allowed)
    ASSERT_NO_THROW(res = state.currentAutoModeMoveStepIntoDirectory(
                        ThunderAutoModeStepPath(3), ThunderAutoModeStepPath(3).boolBranch(true)));
    EXPECT_FALSE(res);
    ASSERT_EQ(state.autoModes[kAutoModeName], kStartAutoMode);
  }

  // Test simple move operations

  {
    ASSERT_NO_THROW(res = state.currentAutoModeMoveStepAfterOther(ThunderAutoModeStepPath(0),
                                                                  ThunderAutoModeStepPath(1)));
    ASSERT_TRUE(res);

    // The moved step gets selected.
    EXPECT_EQ(editorState.selectedStepPath, ThunderAutoModeStepPath(1));

    // Verify the auto mode.
    auto step0It = expectedAutoMode.steps.begin();
    auto step1It = std::next(step0It);
    std::swap(*step0It, *step1It);

    ASSERT_EQ(state.autoModes[kAutoModeName], expectedAutoMode);
  }

  {
    ASSERT_NO_THROW(res = state.currentAutoModeMoveStepBeforeOther(ThunderAutoModeStepPath(2),
                                                                   ThunderAutoModeStepPath(1)));
    ASSERT_TRUE(res);

    // The moved step gets selected.
    EXPECT_EQ(editorState.selectedStepPath, ThunderAutoModeStepPath(1));

    // Verify the auto mode.
    auto step1It = std::next(expectedAutoMode.steps.begin());
    auto step2It = std::next(step1It);
    std::swap(*step1It, *step2It);

    ASSERT_EQ(state.autoModes[kAutoModeName], expectedAutoMode);
  }

  {
    ASSERT_NO_THROW(res = state.currentAutoModeMoveStepBeforeOther(ThunderAutoModeStepPath(2),
                                                                   ThunderAutoModeStepPath(4)));
    ASSERT_TRUE(res);

    // The moved step gets selected.
    EXPECT_EQ(editorState.selectedStepPath, ThunderAutoModeStepPath(3));

    // Verify the auto mode.
    auto step2It = std::next(expectedAutoMode.steps.begin(), 2);
    auto step3It = std::next(step2It);
    std::swap(*step2It, *step3It);

    ASSERT_EQ(state.autoModes[kAutoModeName], expectedAutoMode);
  }

  {
    ASSERT_NO_THROW(
        res = state.currentAutoModeMoveStepIntoDirectory(ThunderAutoModeStepPath(2).boolBranch(true).step(0),
                                                         ThunderAutoModeStepPath(2).boolBranch(false)));
    ASSERT_TRUE(res);

    // The moved step gets selected.
    EXPECT_EQ(editorState.selectedStepPath, ThunderAutoModeStepPath(2).boolBranch(false).step(1));

    // Verify the auto mode.
    auto step2It = std::next(expectedAutoMode.steps.begin(), 2);
    ThunderAutoModeBoolBranchStep* step2 = static_cast<ThunderAutoModeBoolBranchStep*>(step2It->get());
    step2->elseBranch.push_back(std::move(step2->trueBranch.back()));
    step2->trueBranch.pop_back();

    ASSERT_EQ(state.autoModes[kAutoModeName], expectedAutoMode);
  }

  {
    // Move into empty directory
    ASSERT_NO_THROW(res = state.currentAutoModeMoveStepIntoDirectory(
                        ThunderAutoModeStepPath(0), ThunderAutoModeStepPath(2).boolBranch(true)));
    ASSERT_TRUE(res);

    // The moved step gets selected.
    EXPECT_EQ(editorState.selectedStepPath, ThunderAutoModeStepPath(1).boolBranch(true).step(0));

    // Verify the auto mode.
    auto step0It = expectedAutoMode.steps.begin();
    auto step2It = std::next(step0It, 2);
    ThunderAutoModeBoolBranchStep* step2 = static_cast<ThunderAutoModeBoolBranchStep*>(step2It->get());
    step2->trueBranch.push_back(std::move(*step0It));
    expectedAutoMode.steps.erase(step0It);

    ASSERT_EQ(state.autoModes[kAutoModeName], expectedAutoMode);
  }
}
