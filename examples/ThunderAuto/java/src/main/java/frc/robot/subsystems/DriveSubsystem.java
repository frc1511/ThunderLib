package frc.robot.subsystems;

// Modified version of Drivetrain class from WPILib SwerveBot example.

import com.thunder.lib.trajectory.ThunderTrajectoryRunnerProperties;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModule;

import static frc.robot.Constants.*;
import static frc.robot.Constants.DriveConstants.*;

public class DriveSubsystem extends SubsystemBase {
  private final Translation2d frontLeftLocation = new Translation2d(kModuleSpacingX / 2, kModuleSpacingY / 2);
  private final Translation2d frontRightLocation = new Translation2d(kModuleSpacingX / 2, -kModuleSpacingY / 2);
  private final Translation2d backLeftLocation = new Translation2d(-kModuleSpacingX / 2, kModuleSpacingY / 2);
  private final Translation2d backRightLocation = new Translation2d(-kModuleSpacingX / 2, -kModuleSpacingY / 2);

  private final SwerveModule frontLeft = new SwerveModule(kFLDriveMotorChannel, kFLTurningMotorChannel,
      kFLDriveEncoderChannelA, kFLDriveEncoderChannelB, kFLTurningEncoderChannelA, kFLTurningEncoderChannelB);
  private final SwerveModule frontRight = new SwerveModule(kFRDriveMotorChannel, kFRTurningMotorChannel,
      kFRDriveEncoderChannelA, kFRDriveEncoderChannelB, kFRTurningEncoderChannelA, kFRTurningEncoderChannelB);
  private final SwerveModule backLeft = new SwerveModule(kBLDriveMotorChannel, kBLTurningMotorChannel,
      kBLDriveEncoderChannelA, kBLDriveEncoderChannelB, kBLTurningEncoderChannelA, kBLTurningEncoderChannelB);
  private final SwerveModule backRight = new SwerveModule(kBRDriveMotorChannel, kBRTurningMotorChannel,
      kBRDriveEncoderChannelA, kBRDriveEncoderChannelB, kBRTurningEncoderChannelA, kBRTurningEncoderChannelB);

  private final AnalogGyro gyro = new AnalogGyro(kGyroChannel);
  private final AnalogGyroSim gyroSim = new AnalogGyroSim(gyro);

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

  private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(
      kinematics,
      gyro.getRotation2d(),
      getModulePositions());

  private boolean wasPoseSet = false;

  private Field2d field = new Field2d();

  private PIDController xController = new PIDController(3.25, 0.0, 0.0);
  private PIDController yController = new PIDController(3.25, 0.0, 0.0);
  private ProfiledPIDController thetaController = new ProfiledPIDController(8.0, 0.0, 0.0,
      // This profile is not important because ThunderAuto handles angular
      // acceleration limits. You should make sure these limits are >= those in
      // ThunderAuto so they don't interfere.
      new TrapezoidProfile.Constraints(
          kMaxAngularVelocity, kMaxAngularAcceleration));

  private HolonomicDriveController holonomicDriveController = new HolonomicDriveController(
      xController, yController, thetaController);

  private ThunderTrajectoryRunnerProperties trajectoryRunnerProperties = new ThunderTrajectoryRunnerProperties(
      this::getCurrentPose,
      this::setCurrentPose,
      this::setChassisSpeeds,
      holonomicDriveController);

  public DriveSubsystem() {
    gyro.reset();
  }

  /**
   * Gets the essential properties to run ThunderAuto trajectories on this
   * drivetrain.
   * 
   * @return The ThunderTrajectoryRunnerProperties for this drivetrain.
   */
  public ThunderTrajectoryRunnerProperties getTrajectoryRunnerProperties() {
    return trajectoryRunnerProperties;
  }

  /**
   * Method to drive the robot using joystick info.
   * 
   * When fieldRelative is true, xSpeed and ySpeed are relative to the field, from
   * the perspective of the current alliance station.
   *
   * @param xSpeed        Velocity along the x-axis. (Fwd is +)
   * @param ySpeed        Velocity along the y-axis. (Left is +)
   * @param rot           Angular velocity of the robot frame. (CCW is +)
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, rot);

    if (fieldRelative) {
      // Invert speeds when controlling from the perspective of red driver station.
      // For testing, keep normal when no auto mode was run yet (pose was never set).
      if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red && wasPoseSet) {
        speeds.vxMetersPerSecond = -speeds.vxMetersPerSecond;
        speeds.vyMetersPerSecond = -speeds.vyMetersPerSecond;
      }

      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getCurrentPose().getRotation());
    }

    ChassisSpeeds discretizedSpeeds = ChassisSpeeds.discretize(speeds, kPeriodSeconds);
    setChassisSpeeds(discretizedSpeeds);
  }

  private void setChassisSpeeds(ChassisSpeeds speeds) {
    // In simulation, update the gyro angle based on the chassis angular speed.
    if (RobotBase.isSimulation()) {
      double newAngle = gyro.getAngle() - Math.toDegrees(speeds.omegaRadiansPerSecond) * kPeriodSeconds;
      gyroSim.setAngle(newAngle);
    }

    SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxVelocity);

    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);
  }

  private SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()
    };
  }

  /** Updates the field relative position of the robot. */
  private void updateOdometry() {
    odometry.update(gyro.getRotation2d(), getModulePositions());
  }

  public Pose2d getCurrentPose() {
    return odometry.getPoseMeters();
  }

  public void setCurrentPose(Pose2d pose) {
    odometry.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);

    // Important - reset controllers to prevent sudden jumps. ThunderLib does not do
    // this for you.
    xController.reset();
    yController.reset();
    thetaController.reset(pose.getRotation().getRadians());

    wasPoseSet = true;
  }

  public void sendFeedback() {
    field.setRobotPose(odometry.getPoseMeters());
    SmartDashboard.putData("Field", field);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    updateOdometry();
    sendFeedback();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
