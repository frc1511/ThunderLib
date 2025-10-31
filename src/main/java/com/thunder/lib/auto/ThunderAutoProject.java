package com.thunder.lib.auto;

import java.lang.ref.Cleaner;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;

import com.thunder.lib.jni.ThunderLibJNI;
import com.thunder.lib.trajectory.FieldSymmetry;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Represents a ThunderAuto project.
 */
public class ThunderAutoProject {
  /**
   * Default constructor. Creates an empty ThunderAuto project that is not loaded.
   */
  public ThunderAutoProject() {
    m_handle = ThunderLibJNI.ThunderAutoProject_construct();
    m_cleaner.register(this, new ThunderAutoProjectCleanup(m_handle));
  }

  /**
   * Loads a ThunderAuto project from the specified path.
   *
   * @param projectPath The path to the ThunderAuto project. If an absolute path
   *                    is provided, it will be used as-is. If a relative path is
   *                    provided, it will be considered relative to the deploy
   *                    directory.
   */
  public ThunderAutoProject(String projectPath) {
    m_handle = ThunderLibJNI.ThunderAutoProject_constructWithPath(projectPath);
    m_cleaner.register(this, new ThunderAutoProjectCleanup(m_handle));
  }

  /**
   * Loads a ThunderAuto project from the specified path.
   *
   * @param projectPath The path to the ThunderAuto project. If an absolute path
   *                    is provided, it will be used as-is. If a relative path is
   *                    provided, it will be considered relative to the deploy
   *                    directory.
   *
   * @return True if the project was successfully loaded, false otherwise.
   */
  public boolean load(String projectPath) {
    return ThunderLibJNI.ThunderAutoProject_load(m_handle, projectPath);
  }

  /**
   * Scans the deploy directory for a ThunderAuto project and loads the first one
   * it finds.
   *
   * @return True if a project was found and loaded, false otherwise.
   */
  public boolean discoverAndLoadFromDeployDirectory() {
    return ThunderLibJNI.ThunderAutoProject_discoverAndLoadFromDeployDirectory(m_handle);
  }

  /**
   * Returns whether a project is successfully loaded.
   *
   * @return True if a project is loaded, false otherwise.
   */
  public boolean isLoaded() {
    return ThunderLibJNI.ThunderAutoProject_isLoaded(m_handle);
  }

  /**
   * Get the name of the loaded project.
   *
   * @return The name of the loaded project, or an empty string if no project is
   *         loaded.
   */
  public String getName() {
    return ThunderLibJNI.ThunderAutoProject_getName(m_handle);
  }

  /**
   * Register an action command that is used in a trajectory or auto mode. If an
   * action is already registered with the same name, it will be replaced. If an
   * action referenced during the execution of a trajectory or auto mode is not
   * found, nothing will be executed.
   *
   * @param actionName The name of the action.
   * @param command    The command to be executed when the action is called.
   */
  public void registerActionCommand(String actionName, Command command) {
    m_actionCommands.put(actionName, command);
  }

  /**
   * Check if an action command with the given name is registered.
   *
   * @param actionName The name of the action to check.
   *
   * @return True if the action command is registered, false otherwise.
   */
  public boolean isActionCommandRegistered(String actionName) {
    return m_actionCommands.containsKey(actionName);
  }

  /**
   * Check if an action with the given name exists in the project.
   * 
   * Note: This function has nothing to do with registered action commands, it is
   * simply checking the existence of an action in the project.
   *
   * @param actionName The name of the action to check.
   *
   * @return True if the action exists, false otherwise.
   */
  public boolean hasAction(String actionName) {
    return ThunderLibJNI.ThunderAutoProject_hasAction(m_handle, actionName);
  }

  /**
   * Get an action command by name.
   *
   * If the action name references an action group defined in the project, the
   * corresponding command group will be constructed and returned.
   *
   * If the action name references an action command that has been registered via
   * registerActionCommand(), the registered command will be returned.
   *
   * If the action name does not reference a valid action group in the project or
   * a registered command, a None command will be returned.
   *
   * @param actionName The name of the action to get.
   *
   * @return A CommandPtr representing the action command, or a none command if
   *         not found.
   */
  public Command getActionCommand(String actionName) {
    if (!hasAction(actionName)) {
      return Commands.none();
    }

    if (ThunderLibJNI.ThunderAutoProject_isActionCommand(m_handle, actionName)) {
      if (isActionCommandRegistered(actionName)) {
        return m_actionCommands.get(actionName);
      }
    } else if (ThunderLibJNI.ThunderAutoProject_isActionGroup(m_handle, actionName)) {
      ArrayList<String> groupActionNames = ThunderLibJNI.ThunderAutoProject_getActionGroup(m_handle, actionName);

      Command[] commands = new Command[groupActionNames.size()];
      for (int i = 0; i < groupActionNames.size(); i++) {
        commands[i] = getActionCommand(groupActionNames.get(i));
      }

      boolean isSequential = ThunderLibJNI.ThunderAutoProject_isSequentialActionGroup(m_handle, actionName);
      boolean isConcurrent = ThunderLibJNI.ThunderAutoProject_isConcurrentActionGroup(m_handle, actionName);
      if (isSequential) {
        return Commands.sequence(commands);
      } else if (isConcurrent) {
        return Commands.parallel(commands);
      }
    }

    return Commands.none();
  }

  /**
   * Register a boolean condition that is used during a branch step in an auto
   * mode to determine the next step to run. If a condition is already registered
   * with the same name, it will be replaced. If a condition that is referenced
   * during the execution of an auto mode is not found, the auto mode will stop
   * executing and an error will be logged.
   *
   * @param conditionName The name of the condition.
   * @param condition     The function that returns a boolean indicating the next
   *                      step to run.
   */
  public void registerBooleanCondition(String conditionName, BooleanSupplier condition) {
    m_booleanConditions.put(conditionName, condition);
  }

  /**
   * Check if a boolean condition with the given name is registered.
   * 
   * @param conditionName The name of the condition to check.
   * 
   * @return True if the boolean condition is registered, false otherwise.
   */
  public boolean isBooleanConditionRegistered(String conditionName) {
    return m_booleanConditions.containsKey(conditionName);
  }

  /**
   * Get a registered boolean condition by name.
   * 
   * @param conditionName The name of the condition to get.
   * 
   * @return The boolean condition function, or an empty optional if not found.
   */
  public Optional<BooleanSupplier> getBooleanCondition(String conditionName) {
    if (isBooleanConditionRegistered(conditionName)) {
      return Optional.of(m_booleanConditions.get(conditionName));
    } else {
      return Optional.empty();
    }
  }

  /**
   * Register a switch condition that is used during a branch step in an auto mode
   * to determine the next step to run. If a condition is already registered with
   * the same name, it will be replaced. If a condition that is referenced during
   * the execution of an auto mode is not found, the auto mode will stop executing
   * and an error will be logged.
   *
   * @param conditionName The name of the condition.
   * @param condition     The function that returns an integer indicating the next
   *                      step to run.
   */
  public void registerSwitchCondition(String conditionName, IntSupplier condition) {
    m_switchConditions.put(conditionName, condition);
  }

  /**
   * Check if a switch condition with the given name is registered.
   * 
   * @param conditionName The name of the condition to check.
   * 
   * @return True if the switch condition is registered, false otherwise.
   */
  public boolean isSwitchConditionRegistered(String conditionName) {
    return m_switchConditions.containsKey(conditionName);
  }

  /**
   * Get a registered switch condition by name.
   * 
   * @param conditionName The name of the condition to get.
   * 
   * @return The switch condition function, or an empty optional if not found.
   */
  public Optional<IntSupplier> getSwitchCondition(String conditionName) {
    if (isSwitchConditionRegistered(conditionName)) {
      return Optional.of(m_switchConditions.get(conditionName));
    } else {
      return Optional.empty();
    }
  }

  /**
   * Get a trajectory by name.
   *
   * @param trajectoryName The name of the trajectory to get.
   *
   * @return The trajectory if it exists, or an empty optional if it does not.
   */
  public Optional<ThunderAutoTrajectory> getTrajectory(String trajectoryName) {
    long trajectoryHandle = ThunderLibJNI.ThunderAutoProject_getTrajectory(m_handle, trajectoryName);
    if (trajectoryHandle == 0) {
      return Optional.empty();
    } else {
      return Optional.of(new ThunderAutoTrajectory(trajectoryHandle));
    }
  }

  /**
   * Check if a trajectory with the given name exists.
   *
   * @param trajectoryName The name of the trajectory to check.
   *
   * @return True if the trajectory exists, false otherwise.
   */
  public boolean hasTrajectory(String trajectoryName) {
    return ThunderLibJNI.ThunderAutoProject_hasTrajectory(m_handle, trajectoryName);
  }

  /**
   * Get the names of all trajectories in the project.
   *
   * @return A set of trajectory names.
   */
  public HashSet<String> getTrajectoryNames() {
    return ThunderLibJNI.ThunderAutoProject_getTrajectoryNames(m_handle);
  }

  /**
   * Get an autonomous mode by name.
   *
   * @param autoModeName The name of the autonomous mode to get.
   *
   * @return A ThunderAutoMode if it exists, or an empty optional if it does not.
   */
  public Optional<ThunderAutoMode> getAutoMode(String autoModeName) {
    long autoModeHandle = ThunderLibJNI.ThunderAutoProject_getAutoMode(m_handle, autoModeName);
    if (autoModeHandle == 0) {
      return Optional.empty();
    } else {
      return Optional.of(new ThunderAutoMode(autoModeHandle, this));
    }
  }

  /**
   * Check if an autonomous mode with the given name exists.
   *
   * @param autoModeName The name of the autonomous mode to check.
   *
   * @return True if the autonomous mode exists, false otherwise.
   */
  public boolean hasAutoMode(String autoModeName) {
    return ThunderLibJNI.ThunderAutoProject_hasAutoMode(m_handle, autoModeName);
  }

  /**
   * Get the names of all autonomous modes in the project.
   *
   * @return A set of autonomous mode names.
   */
  public HashSet<String> getAutoModeNames() {
    return ThunderLibJNI.ThunderAutoProject_getAutoModeNames(m_handle);
  }

  /**
   * Get the symmetry of the configured field for the loaded project.
   * 
   * @return The field symmetry.
   */
  public FieldSymmetry getFieldSymmetry() {
    int symmetryInt = ThunderLibJNI.ThunderAutoProject_getFieldSymmetry(m_handle);
    return FieldSymmetry.values()[symmetryInt];
  }

  /**
   * Get the size of the configured field for the loaded project.
   * 
   * @return The field size.
   */
  public Pair<Double, Double> getFieldDimensions() {
    return ThunderLibJNI.ThunderAutoProject_getFieldDimensions(m_handle);
  }

  /**
   * Set whether remote project updates from ThunderAuto are enabled (enabled by
   * default).
   * 
   * Remote updates will only occur only when the robot is in Disabled mode and
   * FMS is not connected.
   *
   * @param enabled True to enable remote updates, false to disable them.
   */
  public void setRemoteUpdatesEnabled(boolean enabled) {
    ThunderLibJNI.ThunderAutoProject_setRemoteUpdatesEnabled(m_handle, enabled);
  }

  /**
   * Enable remote project updates from ThunderAuto (enabled by default).
   */
  public void enableRemoteUpdates() {
    ThunderLibJNI.ThunderAutoProject_enableRemoteUpdates(m_handle);
  }

  /**
   * Disable remote project updates from ThunderAuto.
   */
  public void disableRemoteUpdates() {
    ThunderLibJNI.ThunderAutoProject_disableRemoteUpdates(m_handle);
  }

  /**
   * Check if remote project updates from ThunderAuto are enabled (enabled by
   * default).
   *
   * @return True if remote updates are enabled, false otherwise.
   */
  public boolean areRemoteUpdatesEnabled() {
    return ThunderLibJNI.ThunderAutoProject_areRemoteUpdatesEnabled(m_handle);
  }

  /**
   * Register a callback function to be called when the project is updated
   * remotely from ThunderAuto.
   *
   * @param callback The callback runnable to register.
   *
   * @return A subscriber ID that can be used to unregister the callback, or 0 on
   *         failure.
   */
  public long registerRemoteUpdateSubscriber(Runnable callback) {
    return ThunderLibJNI.ThunderAutoProject_registerRemoteUpdateSubscriber(m_handle, callback);
  }

  /**
   * Unregister a previously registered remote update subscriber.
   *
   * @param id The subscriber ID returned by registerRemoteUpdateSubscriber().
   *
   * @return True if the subscriber was successfully unregistered, false
   *         otherwise.
   */
  public boolean unregisterRemoteUpdateSubscriber(long id) {
    return ThunderLibJNI.ThunderAutoProject_unregisterRemoteUpdateSubscriber(m_handle, id);
  }

  /**
   * Get the native handle for this ThunderAutoProject.
   * 
   * @return The native handle.
   */
  long getHandle() {
    return m_handle;
  }

  private long m_handle = 0;

  HashMap<String, Command> m_actionCommands = new HashMap<>();
  HashMap<String, BooleanSupplier> m_booleanConditions = new HashMap<>();
  HashMap<String, IntSupplier> m_switchConditions = new HashMap<>();

  private final Cleaner m_cleaner = Cleaner.create();

  private static class ThunderAutoProjectCleanup implements Runnable {
    private final long m_handle;

    ThunderAutoProjectCleanup(long handle) {
      m_handle = handle;
    }

    @Override
    public void run() {
      ThunderLibJNI.ThunderAutoProject_delete(m_handle);
    }
  }
}
