// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // Robot tick rate in seconds
  public static final double ROBOT_LOOP_PERIOD = 1.0 / 60.0;

  // Controller deadband
  public static final double CONTROLLER_DEADBAND = 0.02;

  // Drive specs
  public static final double WHEELBASE = 1.0;
  public static final double TRACK_WIDTH = 1.0;
  public static final double DRIVE_WHEEL_DIAMETER_METERS = 0.1524;
  public static final double DRIVE_GEAR_RATIO = 1.0;
  
  // Controller ports
  public static final int PRIMARY_CONTROLLER_PORT = 0;

  // Drive motor ports
  public static final int FRONT_LEFT_MOTOR_PORT = 2;
  public static final int FRONT_RIGHT_MOTOR_PORT = 3;
  public static final int REAR_LEFT_MOTOR_PORT = 4;
  public static final int REAR_RIGHT_MOTOR_PORT = 5;

  // Accessories
  public static final int BLINKIN_LED_CONTROLLER_PORT = 0;

  // Lift motor ports
  public static final int LIFT_LEFT_MOTOR_PORT = 6;
  public static final int LIFT_RIGHT_MOTOR_PORT = 6;
  public static final int LIFT_TILT_MOTOR_PORT = 6;
  
  // Lift motor limits
  public static final double LIFT_LOWER_POSITION_LIMIT = 0.0;
  public static final double LIFT_UPPER_POSITION_LIMIT = 1.0;
  public static final double LIFT_WINCH_OUT_LIMIT = 1.0;
  public static final double LIFT_WINCH_IN_LIMIT = 0.0;
}
