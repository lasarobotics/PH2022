// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.SparkMax;
import frc.robot.utils.TractionControlController;
import frc.robot.utils.TurnPIDController;

public class DriveSubsystem extends SubsystemBase implements AutoCloseable {
  public static class Hardware {
    private boolean isHardwareReal;
    private SparkMax lFrontMotor, rFrontMotor;
    private SparkMax lRearMotor, rRearMotor;
    private AHRS navx;

    public Hardware(boolean isHardwareReal,
                    SparkMax lFrontMotor, 
                    SparkMax rFrontMotor, 
                    SparkMax lRearMotor,
                    SparkMax rRearMotor,
                    AHRS navx) {
      this.isHardwareReal = isHardwareReal;
      this.lFrontMotor = lFrontMotor;
      this.rFrontMotor = rFrontMotor;
      this.lRearMotor = lRearMotor;
      this.rRearMotor = rRearMotor;
      this.navx = navx;
    }
  }

  private SparkMax m_lFrontMotor, m_rFrontMotor;
  private SparkMax m_lRearMotor, m_rRearMotor;
  private AHRS m_navx;

  private TurnPIDController m_turnPIDController;
  private TractionControlController m_tractionControlController;

  private MecanumDrive m_drivetrain;
  private MecanumDriveKinematics m_kinematics;
  private MecanumDriveOdometry m_odometry;

  private final double TOLERANCE = 0.125;
  private final double MAX_VOLTAGE = 12.0;

  /**
   * Create an instance of DriveSubsystem
   * <p>
   * NOTE: ONLY ONE INSTANCE SHOULD EXIST AT ANY TIME!
   * <p>
   * @param drivetrainHardware Hardware devices required by drivetrain
   * @param gearRatio Gear ratio between motor and drive wheels
   * @param wheelDiameter Wheel diameter in meters
   * @param trackWidth Track width of robot in meters
   * @param wheelbase Wheelbase of robot in meters
   * @param kP Proportional gain
   * @param kD Derivative gain
   * @param turnScalar Scalar for turn input (degrees)
   * @param deadband Deadband for controller input
   * @param lookAhead Turn PID lookahead, in number of loops
  * @param tractionControlCurve Spline function characterising traction of the robot
   * @param throttleInputCurve Spline function characterising throttle input
   * @param turnInputCurve Spline function characterising turn input
   */
  public DriveSubsystem(Hardware drivetrainHardware, double gearRatio, double wheelDiameter, double trackWidth, double wheelbase,
                        double kP, double kD, double turnScalar, double deadband, double lookAhead, 
                        PolynomialSplineFunction tractionControlCurve, PolynomialSplineFunction throttleInputCurve, PolynomialSplineFunction turnInputCurve) {
    m_turnPIDController = new TurnPIDController(kP, kD, turnScalar, deadband, lookAhead, turnInputCurve);
    m_tractionControlController = new TractionControlController(deadband, tractionControlCurve.getKnots()[tractionControlCurve.getN()], tractionControlCurve, throttleInputCurve);

    this.m_lFrontMotor = drivetrainHardware.lFrontMotor;
    this.m_rFrontMotor = drivetrainHardware.rFrontMotor;
    this.m_lRearMotor = drivetrainHardware.lRearMotor;
    this.m_rRearMotor = drivetrainHardware.rRearMotor;
    this.m_navx = drivetrainHardware.navx;

    this.m_drivetrain = new MecanumDrive(m_lFrontMotor, m_lRearMotor, m_rFrontMotor, m_rRearMotor);

    // Only do this stuff if hardware is real
    if (drivetrainHardware.isHardwareReal) {
      // Set position and velocity conversion factor
      double conversionFactor = (wheelDiameter * Math.PI) / gearRatio;
      m_lFrontMotor.getEncoder().setPositionConversionFactor(conversionFactor);
      m_rFrontMotor.getEncoder().setPositionConversionFactor(conversionFactor);
      m_lRearMotor.getEncoder().setPositionConversionFactor(conversionFactor);
      m_rRearMotor.getEncoder().setPositionConversionFactor(conversionFactor);
      m_lFrontMotor.getEncoder().setVelocityConversionFactor(conversionFactor);
      m_rFrontMotor.getEncoder().setVelocityConversionFactor(conversionFactor);
      m_lRearMotor.getEncoder().setVelocityConversionFactor(conversionFactor);
      m_rRearMotor.getEncoder().setVelocityConversionFactor(conversionFactor);
    }

    // Invert right side
    m_rFrontMotor.setInverted(true);
    m_rRearMotor.setInverted(true);

    // Enable voltage compensation
    m_lFrontMotor.enableVoltageCompensation(MAX_VOLTAGE);
    m_rFrontMotor.enableVoltageCompensation(MAX_VOLTAGE);
    m_lRearMotor.enableVoltageCompensation(MAX_VOLTAGE);
    m_rRearMotor.enableVoltageCompensation(MAX_VOLTAGE);

    // Set all motors to brake mode
    m_lFrontMotor.setIdleMode(IdleMode.kBrake);
    m_rFrontMotor.setIdleMode(IdleMode.kBrake);
    m_lRearMotor.setIdleMode(IdleMode.kBrake);
    m_rRearMotor.setIdleMode(IdleMode.kBrake);

    // Set deadband
    m_drivetrain.setDeadband(deadband);

    m_kinematics = new MecanumDriveKinematics(new Translation2d(+wheelbase / 2.0, +trackWidth / 2.0), 
                                              new Translation2d(+wheelbase / 2.0, -trackWidth / 2.0),
                                              new Translation2d(-wheelbase / 2.0, +trackWidth / 2.0),
                                              new Translation2d(-wheelbase / 2.0, -trackWidth / 2.0));

    // Calibrate NAVX and initialize PID setpoint
    m_navx.calibrate();
    resetAngle();
    m_turnPIDController.setSetpoint(0.0);
    m_turnPIDController.setTolerance(TOLERANCE);

    m_odometry = new MecanumDriveOdometry(m_kinematics, new Rotation2d());
  }

  /**
   * Initialize hardware devices for drive subsystem
   * @param isHardwareReal True if hardware objects are real, false if unit testing
   * @return hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware(boolean isHardwareReal) {
    Hardware drivetrainHardware = new Hardware(isHardwareReal,
                                               new SparkMax(Constants.FRONT_LEFT_MOTOR_PORT, MotorType.kBrushless),
                                               new SparkMax(Constants.FRONT_RIGHT_MOTOR_PORT, MotorType.kBrushless),
                                               new SparkMax(Constants.REAR_LEFT_MOTOR_PORT, MotorType.kBrushless),
                                               new SparkMax(Constants.REAR_RIGHT_MOTOR_PORT, MotorType.kBrushless),
                                               new AHRS(SPI.Port.kMXP));
    return drivetrainHardware;
  }

  /**
   * Reset NAVX angle
   */
  private void resetAngle() {
    m_navx.reset();
  }

  private void resetEncoders() {
    m_lFrontMotor.getEncoder().setPosition(0.0);
    m_rFrontMotor.getEncoder().setPosition(0.0);
    m_lRearMotor.getEncoder().setPosition(0.0);
    m_rRearMotor.getEncoder().setPosition(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Call this repeatedly to drive during teleop
   * @param ySpeed desired speed in Y direction [-1.0, +1.0]
   * @param xSpeed desired speed in X direction [-1.0, +1.0]
   * @param zRotation desired rotation in Z axis [-1.0, +1.0]
   */
  public void teleop(double ySpeed, double xSpeed, double zRotation) {
    m_drivetrain.driveCartesian(ySpeed, xSpeed, zRotation);
  }

  /**
   * Call this repeatedly to drive using PID during teleoperation
   * @param speedRequest Desired speed [-1.0, +1.0]
   * @param turnRequest Turn input [-1.0, +1.0]
   */
  public void teleopPID(double yRequest, double zRequest) {
    // Calculate next speed output
    double yOutput = m_tractionControlController.calculate(getInertialVelocity(), yRequest);

    // Calculate next PID turn output
    double zOutput = m_turnPIDController.calculate(getAngle(), getTurnRate(), zRequest);

    // Run motors with appropriate values
    m_drivetrain.driveCartesian(yOutput, 0.0, zOutput);
  }

  /**
   * Get orientation of robot
   * @return angle in degrees
   */
  public double getAngle() {
    return m_navx.getAngle();
  }

  /**
   * Get orientation of robot
   * @return angle as Rotation2d object
   */
  public Rotation2d getRotation2d() {
    return m_navx.getRotation2d();
  }

  /**
   * Returns the turn rate of the robot.
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_navx.getRate();
  }

  /**
   * Returns inertial velocity of the robot.
   * @return Velocity of the robot as measured by the NAVX
   */
  public double getInertialVelocity() {
    return m_navx.getVelocityY();
  }

  /**
   * Obtain current linear wheel speeds of the robot
   * @return Current wheel speeds
   */
  public MecanumDriveWheelSpeeds getWheelSpeeds() {
    MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds(m_lFrontMotor.getEncoder().getVelocity(),
                                                                      m_rFrontMotor.getEncoder().getVelocity(),
                                                                      m_lRearMotor.getEncoder().getVelocity(),
                                                                      m_rRearMotor.getEncoder().getVelocity());
    return wheelSpeeds;
  }

  /**
   * Reset drive subsystem odometry
   * @param pose pose to reset to
   */
  public void resetOdometry(Pose2d pose) {
    resetAngle();
    resetEncoders();
    m_odometry.resetPosition(pose, getRotation2d());
  }

  /**
   * Update robot odometry
   * <p>
   * Repeatedly call this method to keep track of robot's position
   */
  public void updateOdometry() {
    m_odometry.update(getRotation2d(), getWheelSpeeds());
  }

  /**
   * Obtain currently estimated pose of the robot
   * <p>
   * This method is periodically called by the Ramsete command to obtain the latest pose
   * @return latest robot pose
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Get kinematics of drivetrain
   * @return Object with robot kinematics
   */
  public MecanumDriveKinematics getKinematics() {
    return m_kinematics;
  }

  @Override
  public void close() {
    m_lFrontMotor.close();
    m_rFrontMotor.close();
    m_lRearMotor.close();
    m_rRearMotor.close();
  }
}
