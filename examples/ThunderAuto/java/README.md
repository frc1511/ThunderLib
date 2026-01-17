# ThunderLib Java Example - ThunderAuto

This robot project demonstrates how to load a ThunderAuto project, use the ThunderAutoSendableChooser, and run trajectories and auto modes.

This project is based off the basic WPILib Java Command Robot template, as well as the SwerveBot example.

The ThunderAuto project file used in this example, see `src/main/deploy/ChargedUp.thunderauto`, is based on the 2023 FRC game field. The project includes several trajectories, and an auto mode that runs them in sequence. The auto mode demonstrates the use of branches, changing behavior if the robot fails to pick up a game piece. Several actions are also run during trajectories and at different points in the auto mode.

## Simulation

This project was built to be functional in simulation. If you are unfamiliar with robot simulation, check out [WPILib Introduction to Robot Simulation](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/introduction.html).

To start the simulation, open the command palette in VS Code (Ctrl+Shift+P) and select `WPILib: Simulate Robot Code`. Open the `Sim GUI`, and you should see the simulation window appear. Select an auto mode to run from the `Auto_Mode` chooser, then switch to Autonomous mode and watch the robot drive around the field. See the terminal in VS Code for printed messages from the robot code.

See the following video of the simulation in action:
![ThunderAuto Simulation](media/simulation.mp4)
