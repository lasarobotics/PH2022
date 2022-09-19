// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase implements AutoCloseable {
  public static class Hardware {
    private CANSparkMax lFrontMotor, rFrontMotor;
    private CANSparkMax lRearMotor, rRearMotor;

    public Hardware(CANSparkMax lFrontMotor, 
                    CANSparkMax rFrontMotor, 
                    CANSparkMax lRearMotor,
                    CANSparkMax rRearMotor) {
      this.lFrontMotor = lFrontMotor;
      this.rFrontMotor = rFrontMotor;
      this.lRearMotor = lRearMotor;
      this.rRearMotor = rRearMotor;
    }
  }

  private CANSparkMax m_lFrontMotor, m_rFrontMotor;
  private CANSparkMax m_lRearMotor, m_rRearMotor;

  private MecanumDrive m_drivetrain;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(Hardware drivetrainHardware, double deadband) {
    this.m_lFrontMotor = drivetrainHardware.lFrontMotor;
    this.m_rFrontMotor = drivetrainHardware.rFrontMotor;
    this.m_lRearMotor = drivetrainHardware.lRearMotor;
    this.m_rRearMotor = drivetrainHardware.rRearMotor;

    this.m_drivetrain = new MecanumDrive(m_lFrontMotor, m_lRearMotor, m_rFrontMotor, m_rRearMotor);

    m_drivetrain.setDeadband(deadband);
  }

  public static Hardware initializeHardware() {
    Hardware drivetrainHardware = new Hardware(new CANSparkMax(Constants.FRONT_LEFT_MOTOR_PORT, MotorType.kBrushless),
                                               new CANSparkMax(Constants.FRONT_RIGHT_MOTOR_PORT, MotorType.kBrushless),
                                               new CANSparkMax(Constants.REAR_LEFT_MOTOR_PORT, MotorType.kBrushless),
                                               new CANSparkMax(Constants.REAR_RIGHT_MOTOR_PORT, MotorType.kBrushless));
    return drivetrainHardware;
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

  @Override
  public void close() {
    m_lFrontMotor.close();
    m_rFrontMotor.close();
    m_lRearMotor.close();
    m_rRearMotor.close();
  }
}
