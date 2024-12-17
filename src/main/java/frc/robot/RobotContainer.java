// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.auton.ExampleAuton;
import frc.robot.commands.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.SwerveSubsystem;

import java.io.File;

public class RobotContainer
{

  // Controller(s)
  final CommandXboxController driverController = new CommandXboxController(0);

  // Subsystems
  public final static SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the rotational velocity 
  // buttons are quick rotation positions to different ways to face
  // WARNING: default buttons are on the same buttons as the ones defined in configureBindings
  AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                 () -> -MathUtil.applyDeadband(driverController.getLeftY(),
                                                                                               OperatorConstants.LEFT_Y_DEADBAND),
                                                                 () -> -MathUtil.applyDeadband(driverController.getLeftX(),
                                                                                               OperatorConstants.LEFT_X_DEADBAND),
                                                                 () -> -MathUtil.applyDeadband(driverController.getRightX(),
                                                                                               OperatorConstants.RIGHT_X_DEADBAND),
                                                                 driverController.getHID()::getYButtonPressed,
                                                                 driverController.getHID()::getAButtonPressed,
                                                                 driverController.getHID()::getXButtonPressed,
                                                                 driverController.getHID()::getBButtonPressed);

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot
  Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
      () -> MathUtil.applyDeadband(driverController.getLeftY() * 1, OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(driverController.getLeftX() * 1, OperatorConstants.LEFT_X_DEADBAND),
      () -> driverController.getRightX() * 1);

  public RobotContainer()
  {
    configureBindings();
  }

  private void configureBindings()
  {
    driverController.back().onTrue(Commands.runOnce(drivebase::zeroGyro));
    driverController.start().whileTrue(drivebase.centerModulesCommand());
    // driverController.povUp().whileTrue(drivebase.aimAtSpeaker(2));

    drivebase.setDefaultCommand(closedAbsoluteDriveAdv);
  }

  public Command getAutonomousCommand()
  {
    return new ExampleAuton();
  }

  public void setDriveMode()
  {
    configureBindings();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
