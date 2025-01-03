// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double FIELD_LENGTH = 16.54; // Meters
  public static final double ROBOT_MASS = (60) * 0.453592; // 60 lbs
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; // Seconds, 20ms + 110ms spark max velocity lag
  public static final double MAX_SPEED = Units.feetToMeters(14.5); // Maximum speed of the robot in meters per second, used to limit acceleration

  public static final class ModuleConfiguration {
    public static final double MOI = 6.883;
    public static final double WHEEL_COF = 1.19;
    public static final double WHEEL_RADIUS = Units.inchesToMeters(2);
    public static final double TRACK_WIDTH = 0.530225;
  }

  public static final class RobotConfiguration {
    public static final double MAX_DRIVE_VELOCITY = 3.0;
    public static final double MAX_DRIVE_ACCELERATION = 4.0;
    public static final double MAX_ANGULAR_VELOCITY = Units.degreesToRadians(540);
    public static final double MAX_ANGULAR_ACCELERATION = Units.degreesToRadians(720);
  }

  public static final class AutonConstants {
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
  }

  public static final class DrivebaseConstants {
    // Hold time on motor brakes when disabled, in seconds.
    public static final double WHEEL_LOCK_TIME = 10;
  }

  public static class OperatorConstants {
    // Joystick Deadband.
    public static final double DEADBAND         = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }
}
