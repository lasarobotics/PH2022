// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;
import org.mockito.AdditionalMatchers;

import frc.robot.Constants;
import frc.robot.utils.SparkMax;

@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
public class DriveSubsystemTest {

  private final double DELTA = 1e-3;
  private DriveSubsystem m_driveSubsystem;
  private DriveSubsystem.Hardware m_drivetrainHardware;

  private SparkMax m_lFrontMotor;
  private SparkMax m_rFrontMotor;
  private SparkMax m_lRearMotor;
  private SparkMax m_rRearMotor;

  @BeforeEach
  public void setup() {
    // Create mock hardware devices
    m_lFrontMotor = mock(SparkMax.class);
    m_rFrontMotor = mock(SparkMax.class);
    m_lRearMotor = mock(SparkMax.class);
    m_rRearMotor = mock(SparkMax.class);

    // Create Hardware object using mock objects
    m_drivetrainHardware = new DriveSubsystem.Hardware(false, m_lFrontMotor, m_rFrontMotor, m_lRearMotor, m_rRearMotor);

    // Create DriveSubsystem object
    m_driveSubsystem = new DriveSubsystem(m_drivetrainHardware, 
                                          Constants.CONTROLLER_DEADBAND);
  }

  @AfterEach
  public void close() {
    m_driveSubsystem.close();
    m_driveSubsystem = null;
  }

  @Test
  @Order(1)
  @DisplayName("Test if robot can move forward")
  public void forward() {
    // Try to drive forward
    m_driveSubsystem.teleop(+1.0, 0.0, 0.0);

    // Verify that left and right motors are being driven with expected values
    verify(m_lFrontMotor, times(1)).set(AdditionalMatchers.eq(+1.0, DELTA));
    verify(m_rFrontMotor, times(1)).set(AdditionalMatchers.eq(+1.0, DELTA));
    verify(m_lRearMotor, times(1)).set(AdditionalMatchers.eq(+1.0, DELTA));
    verify(m_rRearMotor, times(1)).set(AdditionalMatchers.eq(+1.0, DELTA));
  }

  @Test
  @Order(2)
  @DisplayName("Test if robot can move forward")
  public void reverse() {
    // Try to drive in reverse
    m_driveSubsystem.teleop(-1.0, 0.0, 0.0);

    // Verify that left and right motors are being driven with expected values
    verify(m_lFrontMotor, times(1)).set(AdditionalMatchers.eq(-1.0, DELTA));
    verify(m_rFrontMotor, times(1)).set(AdditionalMatchers.eq(-1.0, DELTA));
    verify(m_lRearMotor, times(1)).set(AdditionalMatchers.eq(-1.0, DELTA));
    verify(m_rRearMotor, times(1)).set(AdditionalMatchers.eq(-1.0, DELTA));
  }

  @Test
  @Order(3)
  @DisplayName("Test if robot can stop")
  public void stop() {
    // Try to stop
    m_driveSubsystem.teleop(0.0, 0.0, 0.0);

    // Verify that left and right motors are being driven with expected values
    verify(m_lFrontMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA));
    verify(m_rFrontMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA));
    verify(m_lRearMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA));
    verify(m_rRearMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA));
  }

  @Test
  @Order(4)
  @DisplayName("Test if robot can turn left")
  public void turnLeft() {
    // Try to turn left
    m_driveSubsystem.teleop(0.0, 0.0, -1.0);

    // Verify that left and right motors are being driven with expected values
    verify(m_lFrontMotor, times(1)).set(AdditionalMatchers.lt(0.0));
    verify(m_rFrontMotor, times(1)).set(AdditionalMatchers.gt(0.0));
    verify(m_lRearMotor, times(1)).set(AdditionalMatchers.lt(0.0));
    verify(m_rRearMotor, times(1)).set(AdditionalMatchers.gt(0.0));
  }

  @Test
  @Order(5)
  @DisplayName("Test if robot can turn right")
  public void turnRight() {
    // Try to turn right
    m_driveSubsystem.teleop(0.0, 0.0, +1.0);

    // Verify that left and right motors are being driven with expected values
    verify(m_lFrontMotor, times(1)).set(AdditionalMatchers.gt(0.0));
    verify(m_rFrontMotor, times(1)).set(AdditionalMatchers.lt(0.0));
    verify(m_lRearMotor, times(1)).set(AdditionalMatchers.gt(0.0));
    verify(m_rRearMotor, times(1)).set(AdditionalMatchers.lt(0.0));
  }

  @Test
  @Order(6)
  @DisplayName("Test if robot can strafe left")
  public void strafeLeft() {
    // Try to strafe left
    m_driveSubsystem.teleop(0.0, -1.0, 0.0);

    // Verify that left and right motors are being driven with expected values
    verify(m_lFrontMotor, times(1)).set(AdditionalMatchers.lt(0.0));
    verify(m_rFrontMotor, times(1)).set(AdditionalMatchers.gt(0.0));
    verify(m_lRearMotor, times(1)).set(AdditionalMatchers.gt(0.0));
    verify(m_rRearMotor, times(1)).set(AdditionalMatchers.lt(0.0));
  }

  @Test
  @Order(7)
  @DisplayName("Test if robot can strafe right")
  public void strafeRight() {
    // Try to strafe right
    m_driveSubsystem.teleop(0.0, +1.0, 0.0);

    // Verify that left and right motors are being driven with expected values
    verify(m_lFrontMotor, times(1)).set(AdditionalMatchers.gt(0.0));
    verify(m_rFrontMotor, times(1)).set(AdditionalMatchers.lt(0.0));
    verify(m_lRearMotor, times(1)).set(AdditionalMatchers.lt(0.0));
    verify(m_rRearMotor, times(1)).set(AdditionalMatchers.gt(0.0));
  }
}
