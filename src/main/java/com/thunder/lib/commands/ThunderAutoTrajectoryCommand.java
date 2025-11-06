package com.thunder.lib.commands;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.Optional;
import java.util.Set;

import com.thunder.lib.auto.ThunderAutoProject;
import com.thunder.lib.auto.ThunderAutoTrajectory;
import com.thunder.lib.trajectory.FieldDimensions;
import com.thunder.lib.trajectory.FieldSymmetry;
import com.thunder.lib.trajectory.ThunderTrajectoryRunnerProperties;
import com.thunder.lib.trajectory.ThunderTrajectoryState;
import com.thunder.lib.types.CanonicalAngle;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ThunderAutoTrajectoryCommand extends Command {
  /**
   * Constructs a ThunderAutoTrajectoryCommand.
   *
   * @param trajectoryName The name of the trajectory to follow.
   * @param project        The ThunderAutoProject that contains the trajectory.
   * @param properties     The ThunderTrajectoryRunnerProperties to use for
   *                       following the trajectory.
   */
  public ThunderAutoTrajectoryCommand(
      String trajectoryName,
      ThunderAutoProject project,
      ThunderTrajectoryRunnerProperties properties) {

    m_project = project;
    m_runnerProperties = properties;

    Optional<ThunderAutoTrajectory> trajectoryOptional = project.getTrajectory(trajectoryName);
    if (!trajectoryOptional.isPresent()) {
      m_hasTrajectory = false;
      return;
    }

    m_trajectory = trajectoryOptional.get();
    m_hasTrajectory = true;

    // Get start action command.
    {
      Optional<String> startActionName = m_trajectory.getStartAction();
      if (startActionName.isPresent()) {
        m_startActionCommand = m_project.getActionCommand(startActionName.get());
      } else {
        m_startActionCommand = Commands.none();
      }
    }

    // Get end action command.
    {
      Optional<String> endActionName = m_trajectory.getEndAction();
      if (endActionName.isPresent()) {
        m_endActionCommand = m_project.getActionCommand(endActionName.get());
      } else {
        m_endActionCommand = Commands.none();
      }
    }

    // Get stop action commands.
    {
      ArrayList<Double> stopTimes = m_trajectory.getStopTimes();
      for (Double stopTime : stopTimes) {
        Optional<String> stopActionName = m_trajectory.getStopAction(stopTime);
        if (stopActionName.isPresent()) {
          Command stopActionCommand = m_project.getActionCommand(stopActionName.get());
          m_stopActionCommands.add(new StopActionCommand(stopTime, stopActionCommand));
        }
      }
    }

    // Get positioned action commands.
    {
      ArrayList<Double> actionTimes = m_trajectory.getActionTimes();
      for (Double actionTime : actionTimes) {
        Set<String> actionName = m_trajectory.getActionsAtTime(actionTime);
        for (String name : actionName) {
          Command actionCommand = m_project.getActionCommand(name);
          m_positionedActionCommands.add(new PositionedActionCommand(actionTime, actionCommand));
        }
      }
    }
  }

  /**
   * Checks if the command has a valid trajectory to follow.
   * 
   * @return True or false.
   */
  public boolean isValid() {
    return m_hasTrajectory;
  }

  @Override
  public void initialize() {
    if (!isValid())
      return;

    m_alliance = DriverStation.getAlliance();

    Pose2d initialTrajectoryPose = m_trajectory.getInitialState().getPose();
    Pose2d initialPose = flipPoseForAlliance(
        initialTrajectoryPose,
        m_alliance,
        m_project.getFieldSymmetry(),
        m_project.getFieldDimensions());

    m_runnerProperties.getResetPoseConsumer().accept(initialPose);

    beginStartAction();
  }

  @Override
  public void execute() {
    if (!isValid())
      return;

    switch (m_executionState) {
      case START_ACTION:
        executeStartAction();
        break;
      case FOLLOW_TRAJECTORY:
        executeFollowTrajectory();
        break;
      case STOPPED:
        executeStopped();
        break;
      case END_ACTION:
        executeEndAction();
        break;
      case FINISHED:
        break;
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (!isValid())
      return;

    switch (m_executionState) {
      case START_ACTION:
        endStartAction(interrupted);
        break;
      case FOLLOW_TRAJECTORY:
        endFollowTrajectory(interrupted);
        break;
      case STOPPED:
        endStopped(interrupted);
        break;
      case END_ACTION:
        endEndAction(interrupted);
        break;
      case FINISHED:
        break;
    }

    m_executionState = ExecutionState.FINISHED;
  }

  @Override
  public boolean isFinished() {
    if (!isValid())
      return true;

    return m_executionState == ExecutionState.FINISHED;
  }

  private boolean m_hasTrajectory;
  private ThunderAutoTrajectory m_trajectory;
  private ThunderAutoProject m_project;
  private ThunderTrajectoryRunnerProperties m_runnerProperties;

  private Timer m_timer = new Timer();
  private Optional<DriverStation.Alliance> m_alliance = Optional.empty();

  private enum ExecutionState {
    START_ACTION,
    FOLLOW_TRAJECTORY,
    STOPPED,
    END_ACTION,
    FINISHED,
  }

  private ExecutionState m_executionState = ExecutionState.FINISHED;

  private Command m_startActionCommand;
  private Command m_endActionCommand;

  private class StopActionCommand {
    public final double stopTime;
    public final Command command;

    public StopActionCommand(double stopTime, Command command) {
      this.stopTime = stopTime;
      this.command = command;
    }
  }

  private LinkedList<StopActionCommand> m_stopActionCommands = new LinkedList<>();
  private Iterator<StopActionCommand> m_nextStop;
  private Optional<StopActionCommand> m_currentStopAction = Optional.empty();

  private class PositionedActionCommand {
    public final double actionTime;
    public final Command command;

    public PositionedActionCommand(double actionTime, Command command) {
      this.actionTime = actionTime;
      this.command = command;
    }
  }

  private LinkedList<PositionedActionCommand> m_positionedActionCommands = new LinkedList<>();
  private Iterator<PositionedActionCommand> m_nextAction;
  private Optional<PositionedActionCommand> m_currentAction = Optional.empty();

  private HashSet<PositionedActionCommand> m_runningActions;

  private void beginStartAction() {
    m_executionState = ExecutionState.START_ACTION;

    m_startActionCommand.initialize();
  }

  private void executeStartAction() {
    if (m_startActionCommand.isFinished()) {
      m_startActionCommand.end(false);
      beginFollowTrajectory();
      return;
    }

    m_startActionCommand.execute();
  }

  private void endStartAction(boolean interrupted) {
    m_startActionCommand.end(interrupted);
  }

  private void beginFollowTrajectory() {
    m_executionState = ExecutionState.FOLLOW_TRAJECTORY;

    m_timer.reset();
    m_timer.start();

    m_nextStop = m_stopActionCommands.iterator();
    m_nextAction = m_positionedActionCommands.iterator();
    m_runningActions.clear();
  }

  private void resumeFollowTrajectory() {
    m_executionState = ExecutionState.FOLLOW_TRAJECTORY;

    m_timer.start();
  }

  private void executeFollowTrajectory() {
    double trajectoryTime = m_timer.get();

    // Process stop points.

    if (!m_currentStopAction.isPresent() && m_nextStop.hasNext()) {
      m_currentStopAction = Optional.of(m_nextStop.next());
    }
    if (m_currentStopAction.isPresent()) {
      if (trajectoryTime >= m_currentStopAction.get().stopTime - 0.02) {
        // Stop the robot
        m_runnerProperties.getSpeedsConsumer().accept(new ChassisSpeeds(0, 0, 0));
        // Switch to the STOPPED state.
        beginStopped();
        return;
      }
    }

    // Process positioned actions.

    while (m_currentAction.isPresent() || (!m_currentAction.isPresent() && m_nextAction.hasNext())) {
      if (!m_currentAction.isPresent()) {
        m_currentAction = Optional.of(m_nextAction.next());
      }

      PositionedActionCommand action = m_currentAction.get();
      if (trajectoryTime >= action.actionTime - 0.02) {
        action.command.initialize();
        m_runningActions.add(action);
        m_currentAction = Optional.empty();
      } else {
        break;
      }
    }

    HashSet<PositionedActionCommand> doneRunningActions = new HashSet<>();
    for (PositionedActionCommand action : m_runningActions) {
      if (action.command.isFinished()) {
        action.command.end(false);
        doneRunningActions.add(action);
      } else {
        action.command.execute();
      }
    }

    m_runningActions.removeAll(doneRunningActions);

    // Check if trajectory is complete.

    if (trajectoryTime >= m_trajectory.getDurationSeconds()) {
      // Stop the robot.
      m_runnerProperties.getSpeedsConsumer().accept(new ChassisSpeeds(0, 0, 0));
      // Begin the end action.
      beginEndAction();
      return;
    }

    // Drive the robot.

    ThunderTrajectoryState state = m_trajectory.sample(trajectoryTime);

    FieldSymmetry fieldSymmetry = m_project.getFieldSymmetry();
    FieldDimensions fieldDimensions = m_project.getFieldDimensions();

    Pose2d currentPose = m_runnerProperties.getPoseSupplier().get();
    Pose2d targetPose = flipPoseForAlliance(currentPose, m_alliance, fieldSymmetry, fieldDimensions);
    CanonicalAngle heading = flipAngleForAlliance(new CanonicalAngle(state.getHeading()), m_alliance, fieldSymmetry);

    ChassisSpeeds velocities = m_runnerProperties.getHolonomicDriveController().calculate(
        currentPose,
        new Pose2d(targetPose.getTranslation(), heading.toRotation2d()),
        state.getLinearVelocityMetersPerSecond(),
        targetPose.getRotation());

    m_runnerProperties.getSpeedsConsumer().accept(velocities);
  }

  private void endFollowTrajectory(boolean interrupted) {
    // Stop the robot.

    m_runnerProperties.getSpeedsConsumer().accept(new ChassisSpeeds(0, 0, 0));

    // End any running actions.

    for (PositionedActionCommand action : m_runningActions) {
      action.command.end(interrupted);
    }
  }

  private void beginStopped() {
    m_executionState = ExecutionState.STOPPED;

    m_timer.stop();

    m_currentStopAction.get().command.initialize();
  }

  private void executeStopped() {
    StopActionCommand action = m_currentStopAction.get();

    if (action.command.isFinished()) {
      action.command.end(false);
      m_currentStopAction = Optional.empty();
      resumeFollowTrajectory();
      return;
    }

    action.command.execute();
  }

  private void endStopped(boolean interrupted) {
    m_currentStopAction.get().command.end(interrupted);
  }

  private void beginEndAction() {
    m_executionState = ExecutionState.END_ACTION;

    m_endActionCommand.initialize();
  }

  private void executeEndAction() {
    if (m_endActionCommand.isFinished()) {
      m_endActionCommand.end(false);
      m_executionState = ExecutionState.FINISHED;
      return;
    }

    m_endActionCommand.execute();
  }

  private void endEndAction(boolean interrupted) {
    m_endActionCommand.end(interrupted);
  }

  private static CanonicalAngle flipAngleForAlliance(CanonicalAngle originalAngle,
      Optional<DriverStation.Alliance> alliance,
      FieldSymmetry fieldSymmetry) {
    boolean isRed = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    if (!isRed)
      return originalAngle;

    switch (fieldSymmetry) {
      case ROTATIONAL:
        return originalAngle.plus(CanonicalAngle.fromDegrees(180));
      case REFLECTIONAL:
        return CanonicalAngle.fromDegrees(180).minus(originalAngle);
      case NONE:
      default:
        return originalAngle;
    }
  }

  private static Pose2d flipPoseForAlliance(Pose2d originalPose,
      Optional<DriverStation.Alliance> alliance,
      FieldSymmetry fieldSymmetry,
      FieldDimensions fieldDimensions) {
    boolean isRed = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    if (!isRed)
      return originalPose;

    double x = originalPose.getX();
    double y = originalPose.getY();

    switch (fieldSymmetry) {
      case NONE:
        break;
      case ROTATIONAL:
        x = fieldDimensions.width - x;
        y = fieldDimensions.length - y;
        break;
      case REFLECTIONAL:
        x = fieldDimensions.width - x;
        break;
    }

    CanonicalAngle flippedAngle = flipAngleForAlliance(
        new CanonicalAngle(originalPose.getRotation()),
        alliance,
        fieldSymmetry);

    Pose2d pose = new Pose2d(x, y, flippedAngle.toRotation2d());
    return pose;
  }
}
