// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
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
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;
import org.lasarobotics.hardware.revrobotics.SparkInputsAutoLogged;
import org.lasarobotics.utils.GlobalConstants;
import org.mockito.AdditionalMatchers;
import org.mockito.ArgumentMatchers;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Mass;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.Timer;

@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
public class MAXSwerveModuleTest {
  private final double DELTA = 1e-3;
  private final Rotation2d ROTATION_PI = Rotation2d.fromRadians(Math.PI);

  private final GearRatio GEAR_RATIO = MAXSwerveModule.GearRatio.L3;
  private final DriveWheel DRIVE_WHEEL = new DriveWheel(Units.Inches.of(3.0), Units.Value.of(1.0), Units.Value.of(0.8));
  private final Measure<Distance> WHEELBASE = Units.Meters.of(0.6);
  private final Measure<Distance> TRACK_WIDTH = Units.Meters.of(0.6);
  private final Measure<Mass> MASS = Units.Pounds.of(110.0);
  private final Measure<Time> AUTO_LOCK_TIME = Units.Seconds.of(3.0);
  private final Measure<Current> DRIVE_CURRENT_LIMIT = Units.Amps.of(50.0);
  private final Measure<Dimensionless> SLIP_RATIO = Units.Percent.of(8.0);
  private final Spark.ID LEFT_FRONT_DRIVE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/LeftFront/Drive", 2);
  private final Spark.ID LEFT_FRONT_ROTATE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/LeftFront/Rotate", 3);
  private final Spark.ID RIGHT_FRONT_DRIVE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/RightFront/Drive", 4);
  private final Spark.ID RIGHT_FRONT_ROTATE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/RightFront/Rotate", 5);
  private final Spark.ID LEFT_REAR_DRIVE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/LeftRear/Drive", 6);
  private final Spark.ID LEFT_REAR_ROTATE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/LeftRear/Rotate", 7);
  private final Spark.ID RIGHT_REAR_DRIVE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/RightRear/Drive", 8);
  private final Spark.ID RIGHT_REAR_ROTATE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/RightRear/Rotate", 9);

  private final Measure<Velocity<Distance>> NEO_MAX_LINEAR_SPEED = Units.MetersPerSecond.of(4.327);
  private final Measure<Velocity<Distance>> VORTEX_MAX_LINEAR_SPEED = Units.MetersPerSecond.of(5.172);

  private Spark m_lFrontDriveMotor, m_lFrontRotateMotor;
  private Spark m_rFrontDriveMotor, m_rFrontRotateMotor;
  private Spark m_lRearDriveMotor, m_lRearRotateMotor;
  private Spark m_rRearDriveMotor, m_rRearRotateMotor;

  private MAXSwerveModule m_lFrontModule;
  private MAXSwerveModule m_rFrontModule;
  private MAXSwerveModule m_lRearModule;
  private MAXSwerveModule m_rRearModule;


  @BeforeEach
  public void setup() {
    // Create mock hardware devices
    m_lFrontDriveMotor = mock(Spark.class);
    m_lFrontRotateMotor = mock(Spark.class);
    m_rFrontDriveMotor = mock(Spark.class);
    m_rFrontRotateMotor = mock(Spark.class);
    m_lRearDriveMotor = mock(Spark.class);
    m_lRearRotateMotor = mock(Spark.class);
    m_rRearDriveMotor = mock(Spark.class);
    m_rRearRotateMotor = mock(Spark.class);

    // Hardcode motor IDs
    when(m_lFrontDriveMotor.getID()).thenReturn(LEFT_FRONT_DRIVE_MOTOR_ID);
    when(m_lFrontRotateMotor.getID()).thenReturn(LEFT_FRONT_ROTATE_MOTOR_ID);
    when(m_rFrontDriveMotor.getID()).thenReturn(RIGHT_FRONT_DRIVE_MOTOR_ID);
    when(m_rFrontRotateMotor.getID()).thenReturn(RIGHT_FRONT_ROTATE_MOTOR_ID);
    when(m_lRearDriveMotor.getID()).thenReturn(LEFT_REAR_DRIVE_MOTOR_ID);
    when(m_lRearRotateMotor.getID()).thenReturn(LEFT_REAR_ROTATE_MOTOR_ID);
    when(m_rRearDriveMotor.getID()).thenReturn(RIGHT_REAR_DRIVE_MOTOR_ID);
    when(m_rRearRotateMotor.getID()).thenReturn(RIGHT_REAR_ROTATE_MOTOR_ID);

    // Hardcode drive motor kind
    when(m_lFrontDriveMotor.getKind()).thenReturn(MotorKind.NEO);
    when(m_rFrontDriveMotor.getKind()).thenReturn(MotorKind.NEO);
    when(m_lRearDriveMotor.getKind()).thenReturn(MotorKind.NEO_VORTEX);
    when(m_rRearDriveMotor.getKind()).thenReturn(MotorKind.NEO_VORTEX);

    // Hardcode rotate motor kind
    when(m_lFrontRotateMotor.getKind()).thenReturn(MotorKind.NEO_550);
    when(m_rFrontRotateMotor.getKind()).thenReturn(MotorKind.NEO_550);
    when(m_lRearRotateMotor.getKind()).thenReturn(MotorKind.NEO_550);
    when(m_rRearRotateMotor.getKind()).thenReturn(MotorKind.NEO_550);

    // Hardcode sample ID
    Spark.ID id = new Spark.ID("moduleName", 0);
    when(m_lFrontDriveMotor.getID()).thenReturn(id);
    when(m_rFrontDriveMotor.getID()).thenReturn(id);
    when(m_lRearDriveMotor.getID()).thenReturn(id);
    when(m_rRearDriveMotor.getID()).thenReturn(id);

    // Create hardware objects using mock devices
    m_lFrontModule = new MAXSwerveModule(
      new MAXSwerveModule.Hardware(m_lFrontDriveMotor, m_lFrontRotateMotor),
      ModuleLocation.LeftFront,
      GEAR_RATIO,
      DRIVE_WHEEL,
      SLIP_RATIO,
      MASS,
      WHEELBASE,
      TRACK_WIDTH,
      AUTO_LOCK_TIME,
      DRIVE_CURRENT_LIMIT
    );
    m_rFrontModule = new MAXSwerveModule(
      new MAXSwerveModule.Hardware(m_rFrontDriveMotor, m_rFrontRotateMotor),
      ModuleLocation.RightFront,
      GEAR_RATIO,
      DRIVE_WHEEL,
      SLIP_RATIO,
      MASS,
      WHEELBASE,
      TRACK_WIDTH,
      AUTO_LOCK_TIME,
      DRIVE_CURRENT_LIMIT
    );
    m_lRearModule = new MAXSwerveModule(
     new MAXSwerveModule.Hardware(m_lRearDriveMotor, m_lRearRotateMotor),
      ModuleLocation.LeftRear,
      GEAR_RATIO,
      DRIVE_WHEEL,
      SLIP_RATIO,
      MASS,
      WHEELBASE,
      TRACK_WIDTH,
      AUTO_LOCK_TIME,
      DRIVE_CURRENT_LIMIT
    );
    m_rRearModule = new MAXSwerveModule(
      new MAXSwerveModule.Hardware(m_rRearDriveMotor, m_rRearRotateMotor),
      ModuleLocation.RightRear,
      GEAR_RATIO,
      DRIVE_WHEEL,
      SLIP_RATIO,
      MASS,
      WHEELBASE,
      TRACK_WIDTH,
      AUTO_LOCK_TIME,
      DRIVE_CURRENT_LIMIT
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
    assertEquals(new Translation2d(WHEELBASE.divide(+2), TRACK_WIDTH.divide(+2)), m_lFrontModule.getModuleCoordinate());
    assertEquals(new Translation2d(WHEELBASE.divide(+2), TRACK_WIDTH.divide(-2)), m_rFrontModule.getModuleCoordinate());
    assertEquals(new Translation2d(WHEELBASE.divide(-2), TRACK_WIDTH.divide(+2)), m_lRearModule.getModuleCoordinate());
    assertEquals(new Translation2d(WHEELBASE.divide(-2), TRACK_WIDTH.divide(-2)), m_rRearModule.getModuleCoordinate());
  }

  @Test
  @Order(2)
  @DisplayName("Test if module can set state")
  public void set() {
    // Hardcode sensor values
    SparkInputsAutoLogged lFrontSparkInputs = new SparkInputsAutoLogged();
    SparkInputsAutoLogged rFrontSparkInputs = new SparkInputsAutoLogged();
    SparkInputsAutoLogged lRearSparkInputs = new SparkInputsAutoLogged();
    SparkInputsAutoLogged rRearSparkInputs = new SparkInputsAutoLogged();

    lFrontSparkInputs.absoluteEncoderPosition = GlobalConstants.ROTATION_PI.minus(ModuleLocation.LeftFront.offset).getRadians();
    when(m_lFrontRotateMotor.getInputs()).thenReturn(lFrontSparkInputs);
    rFrontSparkInputs.absoluteEncoderPosition = GlobalConstants.ROTATION_PI.minus(ModuleLocation.RightFront.offset).getRadians();
    when(m_rFrontRotateMotor.getInputs()).thenReturn(rFrontSparkInputs);
    lRearSparkInputs.absoluteEncoderPosition = GlobalConstants.ROTATION_PI.minus(ModuleLocation.LeftRear.offset).getRadians();
    when(m_lRearRotateMotor.getInputs()).thenReturn(lRearSparkInputs);
    rRearSparkInputs.absoluteEncoderPosition = GlobalConstants.ROTATION_PI.minus(ModuleLocation.RightRear.offset).getRadians();
    when(m_rRearRotateMotor.getInputs()).thenReturn(rRearSparkInputs);

    // Try to set module state
    SwerveModuleState state = new SwerveModuleState(+2.0, Rotation2d.fromRadians(+Math.PI));
    m_lFrontModule.set(state);
    m_rFrontModule.set(state);
    m_lRearModule.set(state);
    m_rRearModule.set(state);

    // Update modules
    m_lFrontModule.periodic();
    m_rFrontModule.periodic();
    m_lRearModule.periodic();
    m_rRearModule.periodic();

    // Verify that motors are being driven with expected values
    verify(m_lFrontDriveMotor, times(1)).set(AdditionalMatchers.eq(-2.0, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(-Math.PI / 2, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rFrontDriveMotor, times(1)).set(AdditionalMatchers.eq(+2.0, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
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
    SparkInputsAutoLogged sparkInputs = new SparkInputsAutoLogged();
    when(m_lFrontRotateMotor.getInputs()).thenReturn(sparkInputs);
    when(m_rFrontRotateMotor.getInputs()).thenReturn(sparkInputs);
    when(m_lRearRotateMotor.getInputs()).thenReturn(sparkInputs);
    when(m_rRearRotateMotor.getInputs()).thenReturn(sparkInputs);

    // Advance sim time
    Timer.delay(AUTO_LOCK_TIME.in(Units.Seconds));

    // Try to set module state
    SwerveModuleState state = new SwerveModuleState(0.0, Rotation2d.fromRadians(+Math.PI));
    m_lFrontModule.set(state);
    m_rFrontModule.set(state);
    m_lRearModule.set(state);
    m_rRearModule.set(state);

    // Update modules
    m_lFrontModule.periodic();
    m_rFrontModule.periodic();
    m_lRearModule.periodic();
    m_rRearModule.periodic();

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
    SparkInputsAutoLogged defaultInputs = new SparkInputsAutoLogged();
    SparkInputsAutoLogged lFrontRotateMotorInputs = new SparkInputsAutoLogged();
    lFrontRotateMotorInputs.absoluteEncoderPosition = ModuleLocation.LeftFront.offset.getRadians();
    when(m_lFrontDriveMotor.getInputs()).thenReturn(defaultInputs);
    when(m_lFrontRotateMotor.getInputs()).thenReturn(lFrontRotateMotorInputs);
    when(m_rFrontDriveMotor.getInputs()).thenReturn(defaultInputs);
    when(m_rFrontRotateMotor.getInputs()).thenReturn(defaultInputs);
    when(m_lRearDriveMotor.getInputs()).thenReturn(defaultInputs);
    when(m_lRearRotateMotor.getInputs()).thenReturn(defaultInputs);
    when(m_rRearDriveMotor.getInputs()).thenReturn(defaultInputs);
    when(m_rRearRotateMotor.getInputs()).thenReturn(defaultInputs);

    // Set state
    SwerveModuleState desiredState = new SwerveModuleState(2.0, Rotation2d.fromRadians(+Math.PI));
    m_lFrontModule.set(desiredState);
    m_lFrontModule.periodic();

    // Verify module reports expected position
    assertEquals(new SwerveModulePosition(-desiredState.speedMetersPerSecond * MAXSwerveModule.DEFAULT_PERIOD.in(Units.Seconds), desiredState.angle.minus(ROTATION_PI)), m_lFrontModule.getPosition());
  }

  @Test
  @Order(5)
  @DisplayName("Test if module calculates correct maximum linear speed")
  public void maxLinearSpeed() {
    SparkInputsAutoLogged sparkInputs = new SparkInputsAutoLogged();
    when(m_lFrontRotateMotor.getInputs()).thenReturn(sparkInputs);
    when(m_rFrontRotateMotor.getInputs()).thenReturn(sparkInputs);
    when(m_lRearRotateMotor.getInputs()).thenReturn(sparkInputs);
    when(m_rRearRotateMotor.getInputs()).thenReturn(sparkInputs);

    assertTrue(m_lFrontModule.getMaxLinearSpeed().isNear(NEO_MAX_LINEAR_SPEED, DELTA));
    assertTrue(m_rFrontModule.getMaxLinearSpeed().isNear(NEO_MAX_LINEAR_SPEED, DELTA));
    assertTrue(m_lRearModule.getMaxLinearSpeed().isNear(VORTEX_MAX_LINEAR_SPEED, DELTA));
    assertTrue(m_rRearModule.getMaxLinearSpeed().isNear(VORTEX_MAX_LINEAR_SPEED, DELTA));
  }
}
