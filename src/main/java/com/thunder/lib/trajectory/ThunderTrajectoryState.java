package com.thunder.lib.trajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Class representing a state in a trajectory for a robot.
 */
public class ThunderTrajectoryState {
  private double m_timeSeconds = 0.0;
  private Pose2d m_pose = new Pose2d();
  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds();
  private double m_linearVelocityMetersPerSecond = 0.0;
  private Rotation2d m_heading = new Rotation2d();

  /**
   * Default constructor for ThunderTrajectoryState.
   */
  public ThunderTrajectoryState() {
  }

  /**
   * Parameterized constructor for ThunderTrajectoryState.
   * 
   * @param timeSeconds                   Time in seconds
   * @param pose                          Robot pose
   * @param chassisSpeeds                 Robot chassis speeds
   * @param linearVelocityMetersPerSecond Robot linear velocity in m/s
   * @param heading                       Robot heading
   */
  public ThunderTrajectoryState(
      double timeSeconds,
      Pose2d pose,
      ChassisSpeeds chassisSpeeds,
      double linearVelocityMetersPerSecond,
      Rotation2d heading) {
    m_timeSeconds = timeSeconds;
    m_pose = pose;
    m_chassisSpeeds = chassisSpeeds;
    m_linearVelocityMetersPerSecond = linearVelocityMetersPerSecond;
    m_heading = heading;
  }

  /**
   * Get the time in seconds of this state in the trajectory.
   * 
   * @return Time in seconds
   */
  public double getTimeSeconds() {
    return m_timeSeconds;
  }

  /**
   * Set the time in seconds of this state in the trajectory.
   * 
   * @param timeSeconds Time in seconds
   */
  public void setTimeSeconds(double timeSeconds) {
    m_timeSeconds = timeSeconds;
  }

  /**
   * Get the robot pose at this state in the trajectory.
   * 
   * @return Robot pose
   */
  public Pose2d getPose() {
    return m_pose;
  }

  /**
   * Set the robot pose at this state in the trajectory.
   * 
   * @param pose Robot pose
   */
  public void setPose(Pose2d pose) {
    m_pose = pose;
  }

  /**
   * Get the robot chassis speeds at this state in the trajectory.
   * 
   * @return Robot chassis speeds
   */
  public ChassisSpeeds getChassisSpeeds() {
    return m_chassisSpeeds;
  }

  /**
   * Set the robot chassis speeds at this state in the trajectory.
   * 
   * @param chassisSpeeds Robot chassis speeds
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds;
  }

  /**
   * Get the robot linear velocity in meters per second at this state in the trajectory.
   * 
   * @return Robot linear velocity in m/s
   */
  public double getLinearVelocityMetersPerSecond() {
    return m_linearVelocityMetersPerSecond;
  }

  /**
   * Set the robot linear velocity in meters per second at this state in the trajectory.
   * 
   * @param linearVelocityMetersPerSecond Robot linear velocity in m/s
   */
  public void setLinearVelocityMetersPerSecond(double linearVelocityMetersPerSecond) {
    m_linearVelocityMetersPerSecond = linearVelocityMetersPerSecond;
  }

  /**
   * Get the robot heading at this state in the trajectory.
   * 
   * @return Robot heading
   */
  public Rotation2d getHeading() {
    return m_heading;
  }

  /**
   * Set the robot heading at this state in the trajectory.
   * 
   * @param heading Robot heading
   */
  public void setHeading(Rotation2d heading) {
    m_heading = heading;
  }
}
