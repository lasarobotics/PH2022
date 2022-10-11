// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

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
  public static final double CONTROLLER_DEADBAND = 0.15;

  // Spline interpolator
  private static final SplineInterpolator SPLINE_INTERPOLATOR = new SplineInterpolator();

  // Motor RPMs, encoder values, and gear ratios
  public static final int NEO_MAX_RPM = 5880;
  public static final int REV_NEO_ENCODER_TICKS_PER_ROTATION = 42;

  // Drive specs
  public static final double TRACK_WIDTH = 1.0;
  public static final double WHEELBASE = 1.0;
  public static final double DRIVE_WHEEL_DIAMETER_METERS = 0.1524;
  public static final double DRIVE_GEAR_RATIO = 4608.0 / 450.0;
  public static final double DRIVE_TICKS_PER_METER = (REV_NEO_ENCODER_TICKS_PER_ROTATION * DRIVE_GEAR_RATIO) * (1 / (DRIVE_WHEEL_DIAMETER_METERS * Math.PI)); // 898.285
  public static final double DRIVE_METERS_PER_TICK = 1 / DRIVE_TICKS_PER_METER; // 2.041e-5
  public static final double DRIVE_METERS_PER_ROTATION = DRIVE_METERS_PER_TICK * REV_NEO_ENCODER_TICKS_PER_ROTATION; // 0.046755
  public static final double DRIVETRAIN_EFFICIENCY = 0.85;
  public static final double DRIVE_MAX_LINEAR_SPEED = (NEO_MAX_RPM / 60) * DRIVE_METERS_PER_ROTATION * DRIVETRAIN_EFFICIENCY; // 3.895 m/s

  // Drive PID values
  public static final double DRIVE_kP = 0.015;
  public static final double DRIVE_kD = 0.0;
  public static final double DRIVE_TURN_SCALAR = 45.0;
  public static final double DRIVE_LOOKAHEAD = 16;

  private static final double DRIVE_THROTTLE_INPUT_CURVE_X[] = { 0.0, 0.5,   1.0 };
  private static final double DRIVE_THROTTLE_INPUT_CURVE_Y[] = { 0.0, 1.948, 3.896 };
  private static final double DRIVE_TRACTION_CONTROL_CURVE_X[] = { 0.0, 1.948, 3.896 };
  private static final double DRIVE_TRACTION_CONTROL_CURVE_Y[] = { 0.0, 0.5,   1.0 };
  private static final double DRIVE_TURN_INPUT_CURVE_X[] = { 0.0, 0.100, 0.200, 0.300, 0.400, 0.500, 0.600, 0.700, 0.800, 0.900, 1.0 };
  private static final double DRIVE_TURN_INPUT_CURVE_Y[] = { 0.0, 0.008, 0.032, 0.072, 0.128, 0.200, 0.288, 0.392, 0.512, 0.768, 1.0 };

  public static final PolynomialSplineFunction DRIVE_THROTTLE_INPUT_CURVE = SPLINE_INTERPOLATOR.interpolate(DRIVE_THROTTLE_INPUT_CURVE_X, DRIVE_THROTTLE_INPUT_CURVE_Y);
  public static final PolynomialSplineFunction DRIVE_TRACTION_CONTROL_CURVE = SPLINE_INTERPOLATOR.interpolate(DRIVE_TRACTION_CONTROL_CURVE_X, DRIVE_TRACTION_CONTROL_CURVE_Y);
  public static final PolynomialSplineFunction DRIVE_TURN_INPUT_CURVE = SPLINE_INTERPOLATOR.interpolate(DRIVE_TURN_INPUT_CURVE_X, DRIVE_TURN_INPUT_CURVE_Y);
  
  // Controller ports
  public static final int PRIMARY_CONTROLLER_PORT = 0;

  // Drive motor ports
  public static final int FRONT_LEFT_MOTOR_PORT = 11;
  public static final int FRONT_RIGHT_MOTOR_PORT = 3;
  public static final int REAR_LEFT_MOTOR_PORT = 5;
  public static final int REAR_RIGHT_MOTOR_PORT = 9;

  // Accessories
  public static final int BLINKIN_LED_CONTROLLER_PORT = 0;
}
