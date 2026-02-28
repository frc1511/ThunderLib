package com.thunder.lib.commands;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;

import com.thunder.lib.auto.ThunderAutoMode;
import com.thunder.lib.auto.ThunderAutoModeStep;
import com.thunder.lib.auto.ThunderAutoProject;
import com.thunder.lib.trajectory.ThunderTrajectoryRunnerProperties;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * A command that executes a ThunderAuto Auto Mode.
 */
public class ThunderAutoModeCommand extends Command {
  /**
   * Constructs a ThunderAutoModeCommand.
   *
   * @param autoModeName The name of the auto mode to follow.
   * @param project      The ThunderAutoProject that contains the auto mode and
   *                     all referenced trajectories, actions, and conditions.
   * @param properties   The ThunderTrajectoryRunnerProperties to use for
   *                     following trajectories.
   */
  public ThunderAutoModeCommand(
      String autoModeName,
      ThunderAutoProject project,
      ThunderTrajectoryRunnerProperties properties) {

    m_autoModeName = autoModeName;
    m_project = project;
    m_runnerProperties = properties;
  }

  /**
   * Checks if the command has a valid auto mode to follow.
   * 
   * @return True or false.
   */
  public boolean isValid() {
    return m_project.isLoaded() && m_autoMode.isPresent() && m_autoMode.get().isValid();
  }

  @Override
  public void initialize() {
    if (!m_project.isLoaded()) {
      System.err.println(
          "[ThunderLib] [ThunderAutoModeCommand] Cannot initialize command with unloaded ThunderAutoProject '"
              + m_project.getName() + "' for auto mode '" + m_autoModeName + "'");
      return;
    }

    m_autoMode = m_project.getAutoMode(m_autoModeName);
    if (m_autoMode.isEmpty() || !m_autoMode.get().isValid()) {
      System.err
          .println("[ThunderLib] [ThunderAutoModeCommand] Cannot initialize command for auto mode '" + m_autoModeName
              + "' that was not found in or could not be loaded from project '" + m_project.getName() + "'");
      return;
    }

    if (!m_autoMode.get().isRunnable(m_project)) {
      System.err.println("[ThunderLib] [ThunderAutoModeCommand] Auto mode '" + m_autoModeName + "' of project '"
          + m_project.getName() + "' is not runnable. "
          + "It may reference missing trajectories, or one or more sequences of non-continuous trajectory steps. "
          + "Open in ThunderAuto for more details.");
      return;
    }

    if (!isValid())
      return;

    m_isFinished = false;
    m_currentStep = Optional.empty();
    m_firstTrajectoryWasSeen = false;
    nextStep();
  }

  @Override
  public void execute() {
    if (!isValid())
      return;

    if (m_isFinished || m_currentStep.isEmpty())
      return;

    if (!CommandScheduler.getInstance().isScheduled(m_currentStepCommand)) {
      m_currentStepWasInitialized = false;
      nextStep();
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (!isValid())
      return;

    if (m_isFinished || m_currentStep.isEmpty())
      return;

    CommandScheduler.getInstance().cancel(m_currentStepCommand);
    m_isFinished = true;
  }

  @Override
  public boolean isFinished() {
    if (!isValid())
      return true;

    return m_isFinished;
  }

  private void nextStep() {
    if (!isValid())
      return;

    if (m_isFinished)
      return;

    if (m_currentStep.isPresent()) {
      m_currentStep = m_autoMode.get().getNextStep(m_currentStep.get());
    } else {
      m_currentStep = m_autoMode.get().getFirstStep();
    }

    if (m_currentStep.isEmpty()) {
      m_isFinished = true;
      return;
    }

    setupCurrentStep();

    if (!m_isFinished && !m_currentStepWasInitialized) {
      CommandScheduler.getInstance().schedule(m_currentStepCommand);
      m_currentStepWasInitialized = true; // Prevent double initialization during recursive calls.
    }
  }

  private void setupCurrentStep() {
    if (m_currentStep.isEmpty())
      return;

    ThunderAutoModeStep step = m_currentStep.get();
    switch (step.getType()) {
      case ACTION: {
        String actionName = step.getActionName();
        m_currentStepCommand = m_project.getActionCommand(actionName);
        break;
      }
      case TRAJECTORY: {
        String trajectoryName = step.getTrajectoryName();

        // Only reset the pose on the first trajectory step of the auto mode.
        boolean shouldResetPose = !m_firstTrajectoryWasSeen;
        m_firstTrajectoryWasSeen = true;

        m_currentStepCommand = new ThunderAutoTrajectoryCommand(trajectoryName, m_project, m_runnerProperties,
            shouldResetPose);
        break;
      }
      case BRANCH_BOOL: {
        String conditionName = step.getConditionName();
        Optional<BooleanSupplier> condition = m_project.getBooleanCondition(conditionName);
        if (condition.isEmpty()) {
          System.err.println("[ThunderLib] [ThunderAutoModeCommand] Boolean condition '" + conditionName
              + "' not registered to project '" + m_project.getName() + "' required by auto mode '" + m_autoModeName
              + "'. Stopping auto mode now");
          m_isFinished = true;
          break;
        }

        boolean conditionResult = condition.get().getAsBoolean();
        Optional<ThunderAutoModeStep> branchStep = m_autoMode.get().getFirstStepOfBranch(step, conditionResult);
        if (branchStep.isPresent()) {
          m_currentStep = branchStep;
          setupCurrentStep();
        } else {
          nextStep();
        }
        break;
      }
      case BRANCH_SWITCH: {
        String conditionName = step.getConditionName();
        Optional<IntSupplier> condition = m_project.getSwitchCondition(conditionName);
        if (condition.isEmpty()) {
          System.err.println("[ThunderLib] [ThunderAutoModeCommand] Switch condition '" + conditionName
              + "' not registered to project '" + m_project.getName() + "' required by auto mode '" + m_autoModeName
              + "'. Stopping auto mode now");
          m_isFinished = true;
          return;
        }

        int conditionResult = condition.get().getAsInt();
        Optional<ThunderAutoModeStep> branchStep = m_autoMode.get().getFirstStepOfBranch(step, conditionResult);
        if (branchStep.isPresent()) {
          m_currentStep = branchStep;
          setupCurrentStep();
        } else {
          nextStep();
        }
        break;
      }
      default:
        m_isFinished = true;
        break;
    }
  }

  private String m_autoModeName;
  private Optional<ThunderAutoMode> m_autoMode = Optional.empty();
  private ThunderAutoProject m_project;
  private ThunderTrajectoryRunnerProperties m_runnerProperties;

  private boolean m_isFinished = true;
  private Optional<ThunderAutoModeStep> m_currentStep;
  private Command m_currentStepCommand = Commands.none();
  private boolean m_currentStepWasInitialized = false;
  private boolean m_firstTrajectoryWasSeen = false;
}
