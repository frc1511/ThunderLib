package com.thunder.lib.trajectory;

/**
 * Interface representing a trajectory for a robot to follow.
 */
public interface ThunderTrajectory {
  /**
   * Checks if the trajectory is valid.
   * 
   * @return True or false.
   */
  public boolean isValid();

  /**
   * Samples the trajectory at a specified time.
   *
   * @param timeSeconds The time at which to sample the trajectory.
   * 
   * @return The state of the robot at the specified time.
   */
  public ThunderTrajectoryState sample(double timeSeconds);

  /**
   * Returns the duration of the trajectory.
   *
   * @return The duration.
   */
  public double getDurationSeconds();

  /**
   * Returns the initial state of the robot.
   *
   * @return The initial state.
   */
  public ThunderTrajectoryState getInitialState();

  /**
   * Returns the final state of the robot.
   *
   * @return The final state.
   */
  public ThunderTrajectoryState getFinalState();
}
