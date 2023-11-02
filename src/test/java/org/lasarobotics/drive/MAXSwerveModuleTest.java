// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;
import org.lasarobotics.drive.MAXSwerveModule.GearRatio;
import org.lasarobotics.drive.MAXSwerveModule.ModuleLocation;
import org.lasarobotics.hardware.SparkMax;
import org.lasarobotics.hardware.SparkMax.SparkMaxInputs;
import org.lasarobotics.utils.GlobalConstants;
import org.mockito.AdditionalMatchers;
import org.mockito.ArgumentMatchers;

import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
public class MAXSwerveModuleTest {
  private final double DELTA = 1e-5;
  private final Rotation2d ROTATION_PI = Rotation2d.fromRadians(Math.PI);

  private final GearRatio GEAR_RATIO = MAXSwerveModule.GearRatio.High;
  private final double SLIP_RATIO = 0.08;
  private final double WHEELBASE = 0.6;
  private final double TRACK_WIDTH = 0.6;

  private SparkMax m_lFrontDriveMotor, m_lFrontRotateMotor;
  private SparkMax m_rFrontDriveMotor, m_rFrontRotateMotor;
  private SparkMax m_lRearDriveMotor, m_lRearRotateMotor;
  private SparkMax m_rRearDriveMotor, m_rRearRotateMotor;

  private MAXSwerveModule m_lFrontModule;
  private MAXSwerveModule m_rFrontModule;
  private MAXSwerveModule m_lRearModule;
  private MAXSwerveModule m_rRearModule;


  @BeforeEach
  public void setup() {
    // Create mock hardware devices
    m_lFrontDriveMotor = mock(SparkMax.class);
    m_lFrontRotateMotor = mock(SparkMax.class);
    m_rFrontDriveMotor = mock(SparkMax.class);
    m_rFrontRotateMotor = mock(SparkMax.class);
    m_lRearDriveMotor = mock(SparkMax.class);
    m_lRearRotateMotor = mock(SparkMax.class);
    m_rRearDriveMotor = mock(SparkMax.class);
    m_rRearRotateMotor = mock(SparkMax.class);

    // Create hardware objects using mock devices
    m_lFrontModule = new MAXSwerveModule(
      new MAXSwerveModule.Hardware(m_lFrontDriveMotor, m_lFrontRotateMotor),
      MAXSwerveModule.ModuleLocation.LeftFront,
      GEAR_RATIO,
      SLIP_RATIO,
      WHEELBASE,
      TRACK_WIDTH
    );
    m_rFrontModule = new MAXSwerveModule(
      new MAXSwerveModule.Hardware(m_rFrontDriveMotor, m_rFrontRotateMotor),
      MAXSwerveModule.ModuleLocation.RightFront,
      GEAR_RATIO,
      SLIP_RATIO,
      WHEELBASE,
      TRACK_WIDTH
    );
    m_lRearModule = new MAXSwerveModule(
     new MAXSwerveModule.Hardware(m_lRearDriveMotor, m_lRearRotateMotor),
      MAXSwerveModule.ModuleLocation.LeftRear,
      GEAR_RATIO,
      SLIP_RATIO,
      WHEELBASE,
      TRACK_WIDTH
    );
    m_rRearModule = new MAXSwerveModule(
      new MAXSwerveModule.Hardware(m_rRearDriveMotor, m_rRearRotateMotor),
      MAXSwerveModule.ModuleLocation.RightRear,
      GEAR_RATIO,
      SLIP_RATIO,
      WHEELBASE,
      TRACK_WIDTH
    );

    // Disable traction control for unit tests
    m_lFrontModule.disableTractionControl();
    m_rFrontModule.disableTractionControl();
    m_lRearModule.disableTractionControl();
    m_rRearModule.disableTractionControl();
  }

  @Test
  @Order(1)
  @DisplayName("Test if module location is set correctly")
  public void moduleLocation() {
    // Check if all module locations are set
    assertEquals(new Translation2d(+WHEELBASE / 2, +TRACK_WIDTH / 2), m_lFrontModule.getModuleCoordinate());
    assertEquals(new Translation2d(+WHEELBASE / 2, -TRACK_WIDTH / 2), m_rFrontModule.getModuleCoordinate());
    assertEquals(new Translation2d(-WHEELBASE / 2, +TRACK_WIDTH / 2), m_lRearModule.getModuleCoordinate());
    assertEquals(new Translation2d(-WHEELBASE / 2, -TRACK_WIDTH / 2), m_rRearModule.getModuleCoordinate());
  }

  @Test
  @Order(2)
  @DisplayName("Test if module can set state")
  public void set() {
    // Hardcode sensor values
    SparkMaxInputs sparkMaxInputs = new SparkMaxInputs();
    when(m_lFrontRotateMotor.getInputs()).thenReturn(sparkMaxInputs);
    when(m_rFrontRotateMotor.getInputs()).thenReturn(sparkMaxInputs);
    when(m_lRearRotateMotor.getInputs()).thenReturn(sparkMaxInputs);
    when(m_rRearRotateMotor.getInputs()).thenReturn(sparkMaxInputs);

    // Try to set module state
    SwerveModuleState state = new SwerveModuleState(+2.0, Rotation2d.fromRadians(+Math.PI));
    m_lFrontModule.set(state);
    m_rFrontModule.set(state);
    m_lRearModule.set(state);
    m_rRearModule.set(state);

    // Verify that motors are being driven with expected values
    verify(m_lFrontDriveMotor, times(1)).set(AdditionalMatchers.eq(+2.0, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 2, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rFrontDriveMotor, times(1)).set(AdditionalMatchers.eq(-2.0, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_lRearDriveMotor, times(1)).set(AdditionalMatchers.eq(+2.0, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lRearRotateMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rRearDriveMotor, times(1)).set(AdditionalMatchers.eq(-2.0, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rRearRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 2, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
  }

  @Test
  @Order(3)
  @DisplayName("Test if module will auto-lock")
  public void autoLock() {
    // Hardcode sensor values
    SparkMaxInputs sparkMaxInputs = new SparkMaxInputs();
    when(m_lFrontRotateMotor.getInputs()).thenReturn(sparkMaxInputs);
    when(m_rFrontRotateMotor.getInputs()).thenReturn(sparkMaxInputs);
    when(m_lRearRotateMotor.getInputs()).thenReturn(sparkMaxInputs);
    when(m_rRearRotateMotor.getInputs()).thenReturn(sparkMaxInputs);

    // Try to set module state
    SwerveModuleState state = new SwerveModuleState(0.0, Rotation2d.fromRadians(+Math.PI));
    m_lFrontModule.set(state);
    m_rFrontModule.set(state);
    m_lRearModule.set(state);
    m_rRearModule.set(state);

    // Verify that motors are being driven with expected values
    verify(m_lFrontDriveMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rFrontDriveMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_lRearDriveMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lRearRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rRearDriveMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rRearRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
  }

  @Test
  @Order(4)
  @DisplayName("Test if module works in simulation")
  public void simulation() {
    // Hardcode sensor values
    SparkMaxInputs defaultInputs = new SparkMaxInputs();
    SparkMaxInputs lFrontRotateMotorInputs = new SparkMaxInputs();
    lFrontRotateMotorInputs.absoluteEncoderPosition = ModuleLocation.LeftFront.offset;
    when(m_lFrontDriveMotor.getInputs()).thenReturn(defaultInputs);
    when(m_lFrontRotateMotor.getInputs()).thenReturn(lFrontRotateMotorInputs);

    // Set state
    SwerveModuleState desiredState = new SwerveModuleState(2.0, Rotation2d.fromRadians(+Math.PI));
    m_lFrontModule.set(desiredState);

    // Run in simulation
    m_lFrontModule.simulationPeriodic();

    // Verify module reports expected position
    assertEquals(new SwerveModulePosition(-desiredState.speedMetersPerSecond * GlobalConstants.ROBOT_LOOP_PERIOD, desiredState.angle.minus(ROTATION_PI)), m_lFrontModule.getPosition());
  }
}