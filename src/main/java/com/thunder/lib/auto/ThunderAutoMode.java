package com.thunder.lib.auto;

import java.lang.ref.Cleaner;
import java.util.Optional;

import com.thunder.lib.jni.ThunderLibJNI;

/**
 * Represents a ThunderAuto autonomous mode.
 */
public class ThunderAutoMode {
  ThunderAutoMode(long handle) {
    m_handle = handle;
    m_cleaner.register(this, new ThunderAutoModeCleanup(m_handle));
  }

  /**
   * Checks if the auto mode is valid.
   * 
   * @return True or false.
   */
  public boolean isValid() {
    return m_handle != 0;
  }

  /**
   * Returns the first step of the auto mode.
   * 
   * @return The first step of the auto mode, or an empty optional if none exists.
   */
  public Optional<ThunderAutoModeStep> getFirstStep() {
    long stepHandle = ThunderLibJNI.ThunderAutoMode_getFirstStep(m_handle);
    if (stepHandle == 0) {
      return Optional.empty();
    }
    return Optional.of(new ThunderAutoModeStep(stepHandle));
  }

  /**
   * Gets the next step in the auto mode.
   * 
   * @param previousStep The step before the one to get.
   * @return The next step of the auto mode, or an empty optional if previousStep
   *         was the final step.
   */
  public Optional<ThunderAutoModeStep> getNextStep(ThunderAutoModeStep previousStep) {
    long stepHandle = ThunderLibJNI.ThunderAutoMode_getNextStep(m_handle, previousStep.getHandle());
    if (stepHandle == 0) {
      return Optional.empty();
    }
    return Optional.of(new ThunderAutoModeStep(stepHandle));
  }

  /**
   * From the given boolean condition step, gets the first step of either the true
   * or false branch depending on booleanCondition.
   * 
   * @param branchStep       A boolean condition step.
   * @param booleanCondition Which branch to get.
   * @return The first step of the branch, or an empty optional if the branch is
   *         empty or there was a problem.
   */
  public Optional<ThunderAutoModeStep> getFirstStepOfBranch(ThunderAutoModeStep branchStep, boolean booleanCondition) {
    long stepHandle = ThunderLibJNI.ThunderAutoMode_getFirstStepOfBoolBranch(m_handle, branchStep.getHandle(),
        booleanCondition);
    if (stepHandle == 0) {
      return Optional.empty();
    }
    return Optional.of(new ThunderAutoModeStep(stepHandle));
  }

  /**
   * From the given switch condition step, gets the first step of a case branch or
   * the default branch depending on switchCondition.
   * 
   * @param branchStep      A switch condition step.
   * @param switchCondition Which branch to get.
   * @return The first step of the branch, or an empty optional if the branch is
   *         empty or there was a problem.
   */
  public Optional<ThunderAutoModeStep> getFirstStepOfBranch(ThunderAutoModeStep branchStep, int switchCondition) {
    long stepHandle = ThunderLibJNI.ThunderAutoMode_getFirstStepOfSwitchBranch(m_handle, branchStep.getHandle(),
        switchCondition);
    if (stepHandle == 0) {
      return Optional.empty();
    }
    return Optional.of(new ThunderAutoModeStep(stepHandle));
  }

  /**
   * Checks if the auto mode is runnable (i.e., all referenced trajectories exist
   * and are continuous).
   * 
   * @param project The ThunderAutoProject containing all trajectories referenced
   *                by this auto mode.
   * 
   * @return True or false.
   */
  public boolean isRunnable(ThunderAutoProject project) {
    return ThunderLibJNI.ThunderAutoMode_isRunnable(m_handle, project.getHandle());
  }

  /**
   * Get the native handle for this ThunderAutoMode.
   * 
   * @return The native handle.
   */
  long getHandle() {
    return m_handle;
  }

  private long m_handle;

  private final Cleaner m_cleaner = Cleaner.create();

  private static class ThunderAutoModeCleanup implements Runnable {
    private final long m_handle;

    ThunderAutoModeCleanup(long handle) {
      m_handle = handle;
    }

    @Override
    public void run() {
      ThunderLibJNI.ThunderAutoMode_delete(m_handle);
    }
  }
}
