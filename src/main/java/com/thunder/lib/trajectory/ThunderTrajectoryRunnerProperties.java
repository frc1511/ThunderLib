package com.thunder.lib.trajectory;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * A set of callback functions and properties required to run a trajectory.
 */
public class ThunderTrajectoryRunnerProperties {
  private Supplier<Pose2d> m_getPose;
  private Consumer<Pose2d> m_resetPose;
  private Consumer<ChassisSpeeds> m_setSpeeds;
  private HolonomicDriveController m_holonomicDriveController;

  /**
   * Constructor for ThunderTrajectoryRunnerProperties.
   * 
   * @param getPose                  Function that returns the current robot pose.
   * @param resetPose                Function that resets the robot pose.
   * @param setSpeeds                Function that controls the robot chassis
   *                                 speeds (robot-centric).
   * @param holonomicDriveController The holonomic drive controller for trajectory
   *                                 following.
   */
  public ThunderTrajectoryRunnerProperties(
      Supplier<Pose2d> getPose,
      Consumer<Pose2d> resetPose,
      Consumer<ChassisSpeeds> setSpeeds,
      HolonomicDriveController holonomicDriveController) {
    m_getPose = getPose;
    m_resetPose = resetPose;
    m_setSpeeds = setSpeeds;
    m_holonomicDriveController = holonomicDriveController;
  }

  /**
   * Gets the pose supplier function.
   * 
   * @return The pose supplier.
   */
  public Supplier<Pose2d> getPoseSupplier() {
    return m_getPose;
  }

  /**
   * Gets the reset pose consumer function.
   * 
   * @return The reset pose consumer.
   */
  public Consumer<Pose2d> getResetPoseConsumer() {
    return m_resetPose;
  }

  /**
   * Gets the chassis speeds consumer function.
   * 
   * @return The chassis speeds consumer.
   */
  public Consumer<ChassisSpeeds> getSpeedsConsumer() {
    return m_setSpeeds;
  }

  /**
   * Gets the holonomic drive controller.
   * 
   * @return The holonomic drive controller.
   */
  public HolonomicDriveController getHolonomicDriveController() {
    return m_holonomicDriveController;
  }
}
