package com.thunder.lib.trajectory;

import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.thunder.lib.types.CanonicalAngle;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * A set of callback functions and properties required to run a trajectory.
 */
public class ThunderTrajectoryRunnerProperties {
  private Supplier<Pose2d> m_getPose;
  private Consumer<Pose2d> m_resetPose;
  private Optional<Consumer<ChassisSpeeds>> m_setSpeeds = Optional.empty();
  private Optional<BiConsumer<ChassisSpeeds, DriveControllerInputData>> m_setSpeedsWithDiagnostics = Optional.empty();
  private Optional<HolonomicDriveController> m_holonomicDriveController = Optional.empty();

  private Optional<Consumer<DriveControllerInputData>> m_control = Optional.empty();
  private Optional<Runnable> m_stop = Optional.empty();

  /**
   * Class containing
   */
  public static class DriveControllerInputData {
    /**
     * The current robot pose.
     */
    public final Pose2d currentPose;

    /**
     * The (alliance-adjusted) target pose.
     */
    public final Pose2d targetPose;

    /**
     * The (alliance-adjusted) target heading.
     */
    public final CanonicalAngle heading;

    /**
     * The target linear velocity in meters per second.
     */
    public final double linearVelocityMetersPerSecond;

    /**
     * Constructor for DiagnosticSpeedData.
     * 
     * @param currentPose                   The current robot pose.
     * @param targetPose                    The (alliance-adjusted) target pose.
     * @param heading                       The (alliance-adjusted) target heading.
     * @param linearVelocityMetersPerSecond The target linear velocity in meters per
     *                                      second.
     */
    public DriveControllerInputData(Pose2d currentPose,
        Pose2d targetPose,
        CanonicalAngle heading,
        double linearVelocityMetersPerSecond) {
      this.currentPose = currentPose;
      this.targetPose = targetPose;
      this.heading = heading;
      this.linearVelocityMetersPerSecond = linearVelocityMetersPerSecond;
    }

    /**
     * Default constructor for DriveControllerInputData.
     */
    public DriveControllerInputData() {
      this.currentPose = new Pose2d();
      this.targetPose = new Pose2d();
      this.heading = new CanonicalAngle();
      this.linearVelocityMetersPerSecond = 0;
    }
  }

  /**
   * Constructor for ThunderTrajectoryRunnerProperties.
   * 
   * @param getPose                  Function that returns the current robot pose.
   * @param resetPose                Function that resets the robot pose (and PID
   *                                 controllers too!)
   * @param setSpeeds                Function that controls the robot chassis
   *                                 speeds (robot-centric).
   * @param holonomicDriveController The holonomic drive controller for trajectory
   *                                 following. ThunderLib will NOT reset its x,
   *                                 y, or theta PID controllers; the user should
   *                                 do that in the resetPose function if desired.
   */
  public ThunderTrajectoryRunnerProperties(
      Supplier<Pose2d> getPose,
      Consumer<Pose2d> resetPose,
      Consumer<ChassisSpeeds> setSpeeds,
      HolonomicDriveController holonomicDriveController) {
    m_getPose = getPose;
    m_resetPose = resetPose;
    m_setSpeeds = Optional.of(setSpeeds);
    m_holonomicDriveController = Optional.of(holonomicDriveController);
  }

  /**
   * Constructor for ThunderTrajectoryRunnerProperties.
   * 
   * @param getPose                  Function that returns the current robot pose.
   * @param resetPose                Function that resets the robot pose (and PID
   *                                 controllers too!)
   * @param setSpeedsWithDiagnostics Function that controls the robot chassis
   *                                 speeds (robot-centric) with diagnostic data.
   * @param holonomicDriveController The holonomic drive controller for trajectory
   *                                 following. ThunderLib will NOT reset its x,
   *                                 y, or theta PID controllers; the user should
   *                                 do that in the resetPose function if desired.
   */
  public ThunderTrajectoryRunnerProperties(
      Supplier<Pose2d> getPose,
      Consumer<Pose2d> resetPose,
      BiConsumer<ChassisSpeeds, DriveControllerInputData> setSpeedsWithDiagnostics,
      HolonomicDriveController holonomicDriveController) {
    m_getPose = getPose;
    m_resetPose = resetPose;
    m_setSpeedsWithDiagnostics = Optional.of(setSpeedsWithDiagnostics);
    m_holonomicDriveController = Optional.of(holonomicDriveController);
  }

  /**
   * Constructor for ThunderTrajectoryRunnerProperties, with no drive controller.
   * Instead, a consumer of DriveControllerInputData must be provided instead of
   * the typical chassis speeds consumer, and the user is responsible for
   * calculating the appropriate speeds on their own.
   * 
   * @param getPose   Function that returns the current robot pose.
   * @param resetPose Function that resets the robot pose (and PID controllers
   *                  too!)
   * @param control   Function that controls the robot based on the provided
   *                  DriveControllerInputData.
   * @param stop      Function that stops the robot.
   */
  public ThunderTrajectoryRunnerProperties(
      Supplier<Pose2d> getPose,
      Consumer<Pose2d> resetPose,
      Consumer<DriveControllerInputData> control,
      Runnable stop) {
    m_getPose = getPose;
    m_resetPose = resetPose;
    m_control = Optional.of(control);
    m_stop = Optional.of(stop);
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
  public Optional<Consumer<ChassisSpeeds>> getSpeedsConsumer() {
    return m_setSpeeds;
  }

  /**
   * Gets the chassis speeds consumer function with diagnostics.
   * 
   * @return The chassis speeds consumer with diagnostics.
   */
  public Optional<BiConsumer<ChassisSpeeds, DriveControllerInputData>> getSpeedsWithDiagnosticsConsumer() {
    return m_setSpeedsWithDiagnostics;
  }

  /**
   * Gets the holonomic drive controller.
   * 
   * @return The holonomic drive controller.
   */
  public Optional<HolonomicDriveController> getHolonomicDriveController() {
    return m_holonomicDriveController;
  }

  /**
   * Gets the control consumer function with diagnostics.
   * 
   * @return The control consumer with diagnostics.
   */
  public Optional<Consumer<DriveControllerInputData>> getControlConsumer() {
    return m_control;
  }

  /**
   * Gets the stop runnable.
   * 
   * @return The stop runnable.
   */
  public Optional<Runnable> getStopRunnable() {
    return m_stop;
  }
}