package com.thunder.lib.auto;

import java.lang.ref.Cleaner;
import java.util.ArrayList;
import java.util.HashSet;

import com.thunder.lib.jni.ThunderLibJNI;
import com.thunder.lib.trajectory.ThunderTrajectory;
import com.thunder.lib.trajectory.ThunderTrajectoryState;

/**
 * Represents a ThunderAuto trajectory.
 */
public class ThunderAutoTrajectory implements ThunderTrajectory {
  ThunderAutoTrajectory(long handle) {
    m_handle = handle;
    m_cleaner.register(this, new ThunderAutoTrajectoryCleanup(m_handle));
  }

  @Override
  public boolean isValid() {
    return m_handle != 0;
  }

  @Override
  public ThunderTrajectoryState sample(double timeSeconds) {
    return ThunderLibJNI.ThunderAutoTrajectory_sample(m_handle, timeSeconds);
  }

  @Override
  public double getDurationSeconds() {
    return ThunderLibJNI.ThunderAutoTrajectory_getDurationSeconds(m_handle);
  }

  @Override
  public ThunderTrajectoryState getInitialState() {
    return ThunderLibJNI.ThunderAutoTrajectory_getInitialState(m_handle);
  }

  @Override
  public ThunderTrajectoryState getFinalState() {
    return ThunderLibJNI.ThunderAutoTrajectory_getFinalState(m_handle);
  }

  /**
   * Get actions to perform before starting the trajectory.
   * 
   * @return Set of action names
   */
  public HashSet<String> getStartActions() {
    return ThunderLibJNI.ThunderAutoTrajectory_getStartActions(m_handle);
  }

  /**
   * Get actions to perform after completing the trajectory.
   * 
   * @return Set of action names
   */
  public HashSet<String> getEndActions() {
    return ThunderLibJNI.ThunderAutoTrajectory_getEndActions(m_handle);
  }

  /**
   * Get times at which the robot comes to a complete stop during the trajectory.
   * 
   * @return List of stop times in seconds
   */
  public ArrayList<Double> getStopTimes() {
    return ThunderLibJNI.ThunderAutoTrajectory_getStopTimes(m_handle);
  }

  /**
   * Get actions to perform when the robot is stopped at the specified time before
   * it resumes driving.
   * 
   * If the robot is not stopped at the specified time, an empty set is returned.
   * 
   * @param stopTimeSeconds Stop time in seconds
   * 
   * @return Set of action names
   */
  public HashSet<String> getStopActions(double stopTimeSeconds) {
    return ThunderLibJNI.ThunderAutoTrajectory_getStopActions(m_handle, stopTimeSeconds);
  }

  /**
   * Get times at which actions are to be performed during the trajectory.
   * 
   * @return List of action times in seconds
   */
  public ArrayList<Double> getActionTimes() {
    return ThunderLibJNI.ThunderAutoTrajectory_getActionTimes(m_handle);
  }

  /**
   * Get actions to perform at the specified time during the trajectory.
   * 
   * This function simply returns the actions scheduled to start at the specified
   * time, which is usually retrieved from getActionTimes(). Actions may continue
   * running beyond the specified time, but this function will not return those.
   * 
   * @param timeSeconds Time in seconds
   * 
   * @return Set of action names
   */
  public HashSet<String> getActionsAtTime(double timeSeconds) {
    return ThunderLibJNI.ThunderAutoTrajectory_getActionsAtTime(m_handle, timeSeconds);
  }

  private long m_handle;

  private final Cleaner m_cleaner = Cleaner.create();

  private static class ThunderAutoTrajectoryCleanup implements Runnable {
    private final long m_handle;

    ThunderAutoTrajectoryCleanup(long handle) {
      m_handle = handle;
    }

    @Override
    public void run() {
      ThunderLibJNI.ThunderAutoTrajectory_delete(m_handle);
    }
  }
}
