// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import java.io.File;

public class RobotContainer {

  // Subsystems
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
  public final VisionSubsystem visionSubsystem = new VisionSubsystem(drivebase);

  // Controllers
  private final CommandXboxController driverController = new CommandXboxController(0);
  // private final CommandXboxController auxiliaryController = new CommandXboxController(1);

  public RobotContainer() {
    // NamedCommands.registerCommand("ExampleCommand", new ExampleCommand(subsystem));

    configureBindings();

    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverController.getRightX() * 0.95);

    // Command driveFieldOrientedAnglularVelocityAprilTagAlignment = drivebase.driveCommand(
    //     () -> MathUtil.applyDeadband(driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
    //     () -> MathUtil.applyDeadband(driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
    //     () -> driverController.getRightX() * 0.95,
    //     () -> driverController.start().getAsBoolean(),
    //     visionSubsystem.returnCamera()
    // ); 

    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
  }

  private void configureBindings() {
    driverController.back().onTrue(Commands.runOnce(drivebase::zeroGyro));

    // Schedule `exampleMethodCommand` when the Driver Controller's B button is pressed, cancelling on release.
    driverController.b().whileTrue(exampleSubsystem.exampleMethodCommand());

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(exampleSubsystem::exampleCondition)
      .onTrue(new ExampleCommand(exampleSubsystem));
  }

  public Command getAutonomousCommand() {
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
