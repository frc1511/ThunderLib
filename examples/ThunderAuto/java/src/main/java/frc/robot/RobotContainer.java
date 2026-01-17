// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.thunder.lib.auto.ThunderAutoProject;
import com.thunder.lib.auto.ThunderAutoSendableChooser;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

public class RobotContainer {
  private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();

  private final CommandPS4Controller driverController = new CommandPS4Controller(
      OperatorConstants.kDriverControllerPort);

  private boolean fieldRelative = true;

  private ThunderAutoProject thunderAutoProject;
  private ThunderAutoProject thunderAutoProject2;
  private ThunderAutoSendableChooser autoChooser;

  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    /**
     * Load the project saved as "ChargedUp.thunderauto" in the deploy
     * directory.
     *
     * You can provide a path relative to the deploy directory, or an absolute path
     * to a custom location.
     */
    thunderAutoProject = new ThunderAutoProject("ChargedUp");

    /**
     * Register any action commands used by auto modes and trajectories in the
     * project.
     * 
     * If left unregistered, a None command will be used in its place.
     */
    thunderAutoProject.registerActionCommand("ScoreCone_High", Commands.print("Scored Cone at High"));
    thunderAutoProject.registerActionCommand("ScoreCone_Middle", Commands.print("Scored Cone at Middle"));
    thunderAutoProject.registerActionCommand("ScoreCube_High",
        Commands.print("Scoring Cube at High")
            .andThen(Commands.waitSeconds(0.5).raceWith(Commands.run(() -> System.out.println("Scoring..."))))
            .andThen(Commands.print("Scored Cube at High")));

    thunderAutoProject.registerActionCommand("MoveArm_High",
        Commands.print("Moving Arm to High")
            .andThen(Commands.waitSeconds(0.5))
            .andThen(Commands.print("Moved Arm to High")));
    thunderAutoProject.registerActionCommand("MoveArm_Middle",
        Commands.print("Moving Arm to Middle")
            .andThen(Commands.waitSeconds(0.5))
            .andThen(Commands.print("Moved Arm to Middle")));
    thunderAutoProject.registerActionCommand("MoveArm_Low",
        Commands.print("Moving Arm to Low")
            .andThen(Commands.waitSeconds(0.5))
            .andThen(Commands.print("Moved Arm to Low")));

    thunderAutoProject.registerActionCommand("ConeIntake_Start", Commands.print("Cone Intake Started"));
    thunderAutoProject.registerActionCommand("ConeIntake_Stop", Commands.print("Cone Intake Stopped"));

    thunderAutoProject.registerActionCommand("NotifyDone", Commands.print("Notification Done"));

    /**
     * Register any conditions used by auto modes in the project.
     */
    thunderAutoProject.registerBooleanCondition("HasGamePiece", () -> true);

    /**
     * Create a sendable chooser at the SmartDashboard location "Auto_Mode".
     *
     * This chooser will automatically re-publish when changes are made.
     */
    autoChooser = new ThunderAutoSendableChooser("Auto_Mode");

    /**
     * To add items from the project to the chooser, we must first include the
     * project as a "source".
     */
    autoChooser.includeProjectSource(thunderAutoProject);

    /**
     * Now we can add trajectories and auto modes from the project to the chooser by
     * referencing the project's name.
     */
    autoChooser.addAutoModeFromProject(thunderAutoProject.getName(), "Barrier_3GP");
    autoChooser.addTrajectoryFromProject(thunderAutoProject.getName(), "BarrierStart_To_GP1");
    // You can also add non-ThunderAuto commands if desired.
    autoChooser.addCustomCommand("Example Command", new ExampleCommand(exampleSubsystem));

    /**
     * Finally, give the auto chooser the drivetrain's trajectory runner properties
     * so that it can construct trajectory following commands.
     */
    autoChooser.setTrajectoryRunnerProperties(driveSubsystem.getTrajectoryRunnerProperties());

    ////////////////////////////////////////////////////////////////////////////////

    /**
     * Load a second project.
     */
    thunderAutoProject2 = new ThunderAutoProject("RapidReact");

    thunderAutoProject2.registerActionCommand("ShootAll",
        Commands.print("Shoot started")
            .andThen(Commands.waitSeconds(1.0).raceWith(Commands.run(() -> System.out.println("Shooting..."))))
            .andThen(Commands.print("Shot all cargo")));
    thunderAutoProject2.registerActionCommand("StartIntake", Commands.print("Intake Started"));
    thunderAutoProject2.registerActionCommand("StopIntake", Commands.print("Intake Stopped"));

    autoChooser.includeProjectSource(thunderAutoProject2);

    autoChooser.addTrajectoryFromProject(thunderAutoProject2.getName(), "Full5Cargo");
  }

  private void configureBindings() {
    // Swerve drive
    driveSubsystem.setDefaultCommand(
        Commands.run(
            () -> {
              if (!RobotState.isTeleop())
                return;

              final double x = -MathUtil.applyDeadband(driverController.getLeftY(), OperatorConstants.kDeadband);
              final double y = -MathUtil.applyDeadband(driverController.getLeftX(), OperatorConstants.kDeadband);
              final double rot = -MathUtil.applyDeadband(driverController.getRightX(), OperatorConstants.kDeadband);

              final double xSpeed = x * DriveConstants.kMaxVelocity;
              final double ySpeed = y * DriveConstants.kMaxVelocity;
              final double rotSpeed = rot * DriveConstants.kMaxAngularVelocity;

              driveSubsystem.drive(xSpeed, ySpeed, rotSpeed, fieldRelative);
            }, driveSubsystem));

    // Triangle button to toggle field-relative driving
    driverController.triangle().toggleOnTrue(Commands.runOnce(() -> fieldRelative = !fieldRelative));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelectedCommand();
  }
}
