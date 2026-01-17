package com.thunder.lib.auto;

import java.lang.ref.Cleaner;

import com.thunder.lib.jni.ThunderLibJNI;

/**
 * Represents a ThunderAuto autonomous mode.
 */
public class ThunderAutoModeStep {
  ThunderAutoModeStep(long handle) {
    m_handle = handle;
    m_cleaner.register(this, new ThunderAutoModeStepCleanup(m_handle));
  }

  /**
   * The type of the step.
   */
  public enum Type {
    /**
     * Invalid type.
     */
    UNKNOWN,
    /**
     * Execute an action.
     */
    ACTION,
    /**
     * Follow a trajectory.
     */
    TRAJECTORY,
    /**
     * Execute a branch based on a boolean condition.
     */
    BRANCH_BOOL,
    /**
     * Execute a branch based on a switch condition.
     */
    BRANCH_SWITCH,
  }

  /**
   * Checks if the auto mode step is valid.
   * 
   * @return True or false.
   */
  public boolean isValid() {
    return m_handle != 0 && getType() != Type.UNKNOWN;
  }

  /**
   * Returns the type of the auto mode step.
   * 
   * @return Auto mode step type.
   */
  public Type getType() {
    return ThunderLibJNI.ThunderAutoModeStep_getType(m_handle);
  }

  /**
   * Returns the name of the action to run at this ACTION step.
   * 
   * @return Action name
   */
  public String getActionName() {
    return ThunderLibJNI.ThunderAutoModeStep_getItemName(m_handle);
  }

  /**
   * Returns the name of the trajectory to run at this TRAJECTORY step.
   * 
   * @return Trajectory name
   */
  public String getTrajectoryName() {
    return ThunderLibJNI.ThunderAutoModeStep_getItemName(m_handle);
  }

  /**
   * Returns the name of the condition to run at this BRANCH_BOOL/BRANCH_SWITCH
   * step.
   * 
   * @return Condition name
   */
  public String getConditionName() {
    return ThunderLibJNI.ThunderAutoModeStep_getItemName(m_handle);
  }

  /**
   * Get the native handle for this ThunderAutoModeStep.
   * 
   * @return The native handle.
   */
  long getHandle() {
    return m_handle;
  }

  private long m_handle;

  private final Cleaner m_cleaner = Cleaner.create();

  private static class ThunderAutoModeStepCleanup implements Runnable {
    private final long m_handle;

    ThunderAutoModeStepCleanup(long handle) {
      m_handle = handle;
    }

    @Override
    public void run() {
      ThunderLibJNI.ThunderAutoModeStep_delete(m_handle);
    }
  }
}
