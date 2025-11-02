package com.thunder.lib.tests.auto;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Optional;

import com.thunder.lib.auto.ThunderAutoProject;
import com.thunder.lib.auto.ThunderAutoTrajectory;
import com.thunder.lib.tests.BaseTestSuite;
import com.thunder.lib.trajectory.ThunderTrajectoryState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ThunderAutoProjectTestSuite extends BaseTestSuite {
  public static final double TRAJ_DOUBLE_TOLERANCE = 1e-2;

  @Test
  void simpleTrajectoryTest() {
    String projectPath = TEST_RESOURCES_DIR + "/ThunderAuto/SimpleTrajectory.thunderauto";
    ThunderAutoProject project = new ThunderAutoProject(projectPath);
    project.disableRemoteUpdates();

    assertTrue(project.isLoaded());
    assertEquals("SimpleTrajectory", project.getName());

    String trajectoryName = "MySimpleTrajectory";
    assertTrue(project.hasTrajectory(trajectoryName));

    Optional<ThunderAutoTrajectory> trajectoryOptional = project.getTrajectory(trajectoryName);
    assertTrue(trajectoryOptional.isPresent());
    ThunderAutoTrajectory trajectory = trajectoryOptional.get();

    assertEquals(5.5, trajectory.getDurationSeconds(), TRAJ_DOUBLE_TOLERANCE);

    {
      ThunderTrajectoryState initialState = trajectory.getInitialState();

      assertEquals(0.0, initialState.getTimeSeconds(), TRAJ_DOUBLE_TOLERANCE);
      Pose2d pose = initialState.getPose();
      assertEquals(7.8, pose.getX(), TRAJ_DOUBLE_TOLERANCE);
      assertEquals(6.7, pose.getY(), TRAJ_DOUBLE_TOLERANCE);
      assertEquals(0.0, pose.getRotation().getDegrees(), TRAJ_DOUBLE_TOLERANCE);
      ChassisSpeeds chassisSpeeds = initialState.getChassisSpeeds();
      assertEquals(0.0, chassisSpeeds.vxMetersPerSecond, TRAJ_DOUBLE_TOLERANCE);
      assertEquals(0.0, chassisSpeeds.vyMetersPerSecond, TRAJ_DOUBLE_TOLERANCE);
      assertEquals(0.0, chassisSpeeds.omegaRadiansPerSecond, TRAJ_DOUBLE_TOLERANCE);
      assertEquals(0.0, initialState.getLinearVelocityMetersPerSecond(), TRAJ_DOUBLE_TOLERANCE);
      assertEquals(180.0, initialState.getHeading().getDegrees(), TRAJ_DOUBLE_TOLERANCE);
    }

    {
      ThunderTrajectoryState finalState = trajectory.getFinalState();

      assertEquals(5.5, finalState.getTimeSeconds(), TRAJ_DOUBLE_TOLERANCE);
      Pose2d pose = finalState.getPose();
      assertEquals(2.8, pose.getX(), TRAJ_DOUBLE_TOLERANCE);
      assertEquals(4.7, pose.getY(), TRAJ_DOUBLE_TOLERANCE);
      assertEquals(180.0, pose.getRotation().getDegrees(), TRAJ_DOUBLE_TOLERANCE);
      ChassisSpeeds chassisSpeeds = finalState.getChassisSpeeds();
      assertEquals(0.0, chassisSpeeds.vxMetersPerSecond, TRAJ_DOUBLE_TOLERANCE);
      assertEquals(0.0, chassisSpeeds.vyMetersPerSecond, TRAJ_DOUBLE_TOLERANCE);
      assertEquals(0.0, chassisSpeeds.omegaRadiansPerSecond, TRAJ_DOUBLE_TOLERANCE);
      assertEquals(0.0, finalState.getLinearVelocityMetersPerSecond(), TRAJ_DOUBLE_TOLERANCE);
      assertEquals(-90.0, finalState.getHeading().getDegrees(), TRAJ_DOUBLE_TOLERANCE);
    }

    // Stop point
    {
      ThunderTrajectoryState stopState = trajectory.sample(3.5);

      assertEquals(3.5, stopState.getTimeSeconds(), TRAJ_DOUBLE_TOLERANCE);
      Pose2d pose = stopState.getPose();
      assertEquals(2.8, pose.getX(), TRAJ_DOUBLE_TOLERANCE);
      assertEquals(6.7, pose.getY(), TRAJ_DOUBLE_TOLERANCE);
      assertEquals(180.0, pose.getRotation().getDegrees(), TRAJ_DOUBLE_TOLERANCE);
      ChassisSpeeds chassisSpeeds = stopState.getChassisSpeeds();
      assertEquals(0.0, chassisSpeeds.vxMetersPerSecond, TRAJ_DOUBLE_TOLERANCE);
      assertEquals(0.0, chassisSpeeds.vyMetersPerSecond, TRAJ_DOUBLE_TOLERANCE);
      assertEquals(0.0, chassisSpeeds.omegaRadiansPerSecond, TRAJ_DOUBLE_TOLERANCE);
      assertEquals(0.0, stopState.getLinearVelocityMetersPerSecond(), TRAJ_DOUBLE_TOLERANCE);
      assertEquals(-90.0, stopState.getHeading().getDegrees(), TRAJ_DOUBLE_TOLERANCE);

      ArrayList<Double> stopTimes = trajectory.getStopTimes();
      assertEquals(1, stopTimes.size());
      double stopTime = stopTimes.get(0);
      assertEquals(3.5, stopTime, TRAJ_DOUBLE_TOLERANCE);

      HashSet<String> stopActions = trajectory.getStopActions(stopTime);
      assertEquals(1, stopActions.size());
      String actionName = stopActions.iterator().next();
      assertEquals("StopAction", actionName);
    }

    // Start action
    {
      HashSet<String> startActions = trajectory.getStartActions();
      assertEquals(1, startActions.size());
      String actionName = startActions.iterator().next();
      assertEquals("StartAction", actionName);
    }

    // End action
    {
      HashSet<String> endActions = trajectory.getEndActions();
      assertEquals(1, endActions.size());
      String actionName = endActions.iterator().next();
      assertEquals("EndAction", actionName);
    }

    // Positioned action
    {
      ArrayList<Double> actionTimes = trajectory.getActionTimes();
      assertEquals(1, actionTimes.size());
      double actionTime = actionTimes.get(0);
      assertEquals(1.75, actionTime, TRAJ_DOUBLE_TOLERANCE);

      HashSet<String> actionsAtTime = trajectory.getActionsAtTime(actionTime);
      assertEquals(1, actionsAtTime.size());
      String actionName = actionsAtTime.iterator().next();
      assertEquals("PositionedAction", actionName);
    }

    {
      ThunderTrajectoryState state = trajectory.sample(1.75);

      assertEquals(1.75, state.getTimeSeconds(), TRAJ_DOUBLE_TOLERANCE);
      Pose2d pose = state.getPose();
      assertEquals(7.8 - 2.5, pose.getX(), TRAJ_DOUBLE_TOLERANCE);
      assertEquals(6.7, pose.getY(), TRAJ_DOUBLE_TOLERANCE);
      assertEquals(90.0, Math.abs(pose.getRotation().getDegrees()), 0.5);
      ChassisSpeeds chassisSpeeds = state.getChassisSpeeds();
      assertEquals(-2.0, chassisSpeeds.vxMetersPerSecond, TRAJ_DOUBLE_TOLERANCE);
      assertEquals(0.0, chassisSpeeds.vyMetersPerSecond, TRAJ_DOUBLE_TOLERANCE);
      assertTrue(Math.abs(chassisSpeeds.omegaRadiansPerSecond) > 0.0);
      assertEquals(2.0, state.getLinearVelocityMetersPerSecond(), TRAJ_DOUBLE_TOLERANCE);
      assertEquals(180.0, Math.abs(state.getHeading().getDegrees()), TRAJ_DOUBLE_TOLERANCE);
    }
  }
}
