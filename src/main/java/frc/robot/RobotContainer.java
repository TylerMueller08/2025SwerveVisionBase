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
import frc.robot.commands.drivebase.FieldCentricDrive;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

import java.io.File;

public class RobotContainer
{

  // Controller(s)
  final CommandXboxController driverController = new CommandXboxController(0);

  // Subsystems
  public final static SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  /**
   * Swerve Drive Command with full field-centric mode and heading correction.
   */
  FieldCentricDrive fieldCentricDrive = new FieldCentricDrive(drivebase,
                                                                 () -> -MathUtil.applyDeadband(driverController.getLeftY(),
                                                                                               OperatorConstants.LEFT_Y_DEADBAND),
                                                                 () -> -MathUtil.applyDeadband(driverController.getLeftX(),
                                                                                               OperatorConstants.DEADBAND),
                                                                 () -> -MathUtil.applyDeadband(driverController.getRightX(),
                                                                                               OperatorConstants.RIGHT_X_DEADBAND),
                                                                 driverController.getHID()::getYButtonPressed,
                                                                 driverController.getHID()::getAButtonPressed,
                                                                 driverController.getHID()::getXButtonPressed,
                                                                 driverController.getHID()::getBButtonPressed);

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverController.getLeftY() * 1,
                                                                () -> driverController.getLeftX() * 1)
                                                              .withControllerRotationAxis(driverController::getRightX)
                                                              .deadband(OperatorConstants.DEADBAND)
                                                              .scaleTranslation(0.8)
                                                              .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverController::getRightX,
                                                                                             driverController::getRightY)
                                                                                            .headingWhile(true);

  Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

  public RobotContainer()
  {
    configureBindings();
  }

  private void configureBindings()
  {
    // (Condition) ? Return-On-True : Return-On-False
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    driverController.back().onTrue(Commands.runOnce(drivebase::zeroGyro));
    driverController.start().whileTrue(drivebase.centerModulesCommand());
    // driverController.povUp().whileTrue(drivebase.aimAtSpeaker(2));
    // driverController.povDown().whileTrue(drivebase.centerModulesCommand());
  }

  public Command getAutonomousCommand()
  {
    return new ExampleAuton();
    // return drivebase.getAutonomousCommand("New Auto");
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
