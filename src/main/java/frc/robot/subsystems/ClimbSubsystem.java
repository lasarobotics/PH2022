// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.SparkMax;
import com.revrobotics.SparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase implements AutoCloseable {
  public static class Hardware {
    private SparkMax lLiftMotor, rLiftMotor, tiltMotor;


    public Hardware(
                  SparkMax lLiftMotor,
                  SparkMax rLiftMotor,
                  SparkMax tiltMotor) {
        this.lLiftMotor = lLiftMotor;
        this.rLiftMotor = rLiftMotor;
        this.tiltMotor = tiltMotor;
    }
  }


  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem(Hardware ClimbSubsystem, double deadband) {
    this.m_lLiftMotor = ClimbSubsystem.lLiftMotor; 
    this.m_rLiftMotor = ClimbSubsystem.rLiftMotor; 
    this.m_tiltMotor = ClimbSubsystem.liftMotor;
    this.m_lLiftMotor.follow(this.m_rLiftMotor, true);
  }

  public static Hardware initializeHardware() {
    Hardware climbHardware = new Hardware(
      new SparkMax(Constants.LIFT_RIGHT_MOTOR_PORT, MotorType.kBrushless),
      new SparkMax(Constants.LIFT_LEFT_MOTOR_PORT, MotorType.kBrushless),
      new SparkMax(Constants.LIFT_TILT_MOTOR_PORT, MotorType.kBrushless),
    );
    return climbHardware;
  }

  public void init() {
    climbSetPosition(lLiftMotor, 0);
    climbSetPosition(rLiftMotor, 0);
    climbSetPosition(tiltMotor, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void moveClimbUp() {
    climbSetPosition(this.m_rLiftMotor, Constants.LIFT_UPPER_POSITION_LIMIT);
  }

  public void moveClimbDown() {
    climbSetPosition(this.m_rLiftMotor, Constants.LIFT_LOWER_POSITION_LIMIT);
  }

  public void stopClimb() {
    m_rLiftMotor.stopMotor();
    m_lLiftMotor.stopMotor();
    m_rLiftMotor.overrideSoftLimitsEnable(true);
    m_lLiftMotor.overrideSoftLimitsEnable(true);
  }


  public void winchIn() {
    climbSetPosition(this.tiltMotor, Constants.LIFT_WINCH_IN_LIMIT);
  }

  public void winchOut() {
    climbSetPosition(this.tiltMotor, Constants.LIFT_WINCH_OUT_LIMIT);
  }

  public void winchStop() {
    m_tiltMotor.stopMotor();
    m_tiltMotor.overrideSoftLimitsEnable(true);
  }

  public void winchSetPosition(double pos) {
    pos = MathUtil.clamp(pos, Constants.LIFT_WINCH_IN_LIMIT, Constants.LIFT_WINCH_OUT_LIMIT);
    m_tiltMotor.set(pos, SparkMax.kPosition);
  }

  private void climbSetPosition(SparkMax motor, double position){
    motor.set(position, SparkMax.kPosition);
  }

  @Override
  public void close() {
  }
}
