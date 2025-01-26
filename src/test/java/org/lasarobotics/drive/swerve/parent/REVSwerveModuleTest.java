// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.drive.swerve.parent;

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
import org.lasarobotics.drive.swerve.DriveWheel;
import org.lasarobotics.drive.swerve.SwerveModule;
import org.lasarobotics.drive.swerve.child.MAXSwerveModule;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;
import org.lasarobotics.hardware.revrobotics.SparkInputsAutoLogged;
import org.lasarobotics.utils.FFConstants;
import org.lasarobotics.utils.GlobalConstants;
import org.lasarobotics.utils.PIDConstants;
import org.mockito.AdditionalMatchers;
import org.mockito.ArgumentMatchers;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;

@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
public class REVSwerveModuleTest {
  private final double DELTA = 1e-3;

  private final MAXSwerveModule.GearRatio GEAR_RATIO = MAXSwerveModule.GearRatio.L3;
  private final DriveWheel DRIVE_WHEEL = DriveWheel.create(Units.Inches.of(3.0), Units.Value.of(1.0), Units.Value.of(0.8));
  private final PIDConstants DRIVE_PID = PIDConstants.of(0.3, 0.0, 0.001, 0.0, 0.0);
  private final FFConstants DRIVE_FF = FFConstants.of(0.2, 0.0, 0.0, 0.5);
  private final PIDConstants ROTATE_PID = PIDConstants.of(2.0, 0.0, 0.1, 0.0, 0.0);
  private final FFConstants ROTATE_FF = FFConstants.of(0.2, 0.0, 0.0, 0.01);
  private final Distance WHEELBASE = Units.Meters.of(0.6);
  private final Distance TRACK_WIDTH = Units.Meters.of(0.6);
  private final Mass MASS = Units.Pounds.of(110.0);
  private final Time AUTO_LOCK_TIME = Units.Seconds.of(3.0);
  private final Current DRIVE_CURRENT_LIMIT = Units.Amps.of(50.0);
  private final Dimensionless SLIP_RATIO = Units.Percent.of(8.0);
  private final Spark.ID LEFT_FRONT_DRIVE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/LeftFront/Drive", 2);
  private final Spark.ID LEFT_FRONT_ROTATE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/LeftFront/Rotate", 3);
  private final Spark.ID RIGHT_FRONT_DRIVE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/RightFront/Drive", 4);
  private final Spark.ID RIGHT_FRONT_ROTATE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/RightFront/Rotate", 5);
  private final Spark.ID LEFT_REAR_DRIVE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/LeftRear/Drive", 6);
  private final Spark.ID LEFT_REAR_ROTATE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/LeftRear/Rotate", 7);
  private final Spark.ID RIGHT_REAR_DRIVE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/RightRear/Drive", 8);
  private final Spark.ID RIGHT_REAR_ROTATE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/RightRear/Rotate", 9);

  private final LinearVelocity NEO_MAX_LINEAR_SPEED = Units.MetersPerSecond.of(4.327);
  private final LinearVelocity VORTEX_MAX_LINEAR_SPEED = Units.MetersPerSecond.of(5.172);

  private Spark m_lFrontDriveMotor, m_lFrontRotateMotor;
  private Spark m_rFrontDriveMotor, m_rFrontRotateMotor;
  private Spark m_lRearDriveMotor, m_lRearRotateMotor;
  private Spark m_rRearDriveMotor, m_rRearRotateMotor;

  private REVSwerveModule m_lFrontModule;
  private REVSwerveModule m_rFrontModule;
  private REVSwerveModule m_lRearModule;
  private REVSwerveModule m_rRearModule;


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
    m_lFrontModule = MAXSwerveModule.create(
      new REVSwerveModule.Hardware(m_lFrontDriveMotor, m_lFrontRotateMotor),
      SwerveModule.Location.LeftFront,
      GEAR_RATIO,
      DRIVE_WHEEL,
      DRIVE_PID,
      FFConstants.of(DRIVE_FF.kS, DRIVE_FF.kG, 12.0 / NEO_MAX_LINEAR_SPEED.in(Units.MetersPerSecond), DRIVE_FF.kA),
      ROTATE_PID,
      ROTATE_FF,
      SLIP_RATIO,
      MASS,
      WHEELBASE,
      TRACK_WIDTH,
      AUTO_LOCK_TIME,
      DRIVE_CURRENT_LIMIT
    );
    m_rFrontModule = MAXSwerveModule.create(
      new REVSwerveModule.Hardware(m_rFrontDriveMotor, m_rFrontRotateMotor),
      SwerveModule.Location.RightFront,
      GEAR_RATIO,
      DRIVE_WHEEL,
      DRIVE_PID,
      FFConstants.of(DRIVE_FF.kS, DRIVE_FF.kG, 12.0 / NEO_MAX_LINEAR_SPEED.in(Units.MetersPerSecond), DRIVE_FF.kA),
      ROTATE_PID,
      ROTATE_FF,
      SLIP_RATIO,
      MASS,
      WHEELBASE,
      TRACK_WIDTH,
      AUTO_LOCK_TIME,
      DRIVE_CURRENT_LIMIT
    );
    m_lRearModule = MAXSwerveModule.create(
     new REVSwerveModule.Hardware(m_lRearDriveMotor, m_lRearRotateMotor),
      SwerveModule.Location.LeftRear,
      GEAR_RATIO,
      DRIVE_WHEEL,
      DRIVE_PID,
      FFConstants.of(DRIVE_FF.kS, DRIVE_FF.kG, 12.0 / VORTEX_MAX_LINEAR_SPEED.in(Units.MetersPerSecond), DRIVE_FF.kA),
      ROTATE_PID,
      ROTATE_FF,
      SLIP_RATIO,
      MASS,
      WHEELBASE,
      TRACK_WIDTH,
      AUTO_LOCK_TIME,
      DRIVE_CURRENT_LIMIT
    );
    m_rRearModule = MAXSwerveModule.create(
      new REVSwerveModule.Hardware(m_rRearDriveMotor, m_rRearRotateMotor),
      SwerveModule.Location.RightRear,
      GEAR_RATIO,
      DRIVE_WHEEL,
      DRIVE_PID,
      FFConstants.of(DRIVE_FF.kS, DRIVE_FF.kG, 12.0 / VORTEX_MAX_LINEAR_SPEED.in(Units.MetersPerSecond), DRIVE_FF.kA),
      ROTATE_PID,
      ROTATE_FF,
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
    assertEquals(new Translation2d(WHEELBASE.div(+2), TRACK_WIDTH.div(+2)), m_lFrontModule.getModuleCoordinate());
    assertEquals(new Translation2d(WHEELBASE.div(+2), TRACK_WIDTH.div(-2)), m_rFrontModule.getModuleCoordinate());
    assertEquals(new Translation2d(WHEELBASE.div(-2), TRACK_WIDTH.div(+2)), m_lRearModule.getModuleCoordinate());
    assertEquals(new Translation2d(WHEELBASE.div(-2), TRACK_WIDTH.div(-2)), m_rRearModule.getModuleCoordinate());
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

    lFrontSparkInputs.absoluteEncoderPosition = Units.Radians.of(Math.PI).minus(MAXSwerveModule.ZERO_OFFSET.get(SwerveModule.Location.LeftFront)).in(Units.Radians);
    when(m_lFrontRotateMotor.getInputs()).thenReturn(lFrontSparkInputs);
    rFrontSparkInputs.absoluteEncoderPosition = Units.Radians.of(Math.PI).minus(MAXSwerveModule.ZERO_OFFSET.get(SwerveModule.Location.RightFront)).in(Units.Radians);
    when(m_rFrontRotateMotor.getInputs()).thenReturn(rFrontSparkInputs);
    lRearSparkInputs.absoluteEncoderPosition = Units.Radians.of(Math.PI).minus(MAXSwerveModule.ZERO_OFFSET.get(SwerveModule.Location.LeftRear)).in(Units.Radians);
    when(m_lRearRotateMotor.getInputs()).thenReturn(lRearSparkInputs);
    rRearSparkInputs.absoluteEncoderPosition = Units.Radians.of(Math.PI).minus(MAXSwerveModule.ZERO_OFFSET.get(SwerveModule.Location.RightRear)).in(Units.Radians);
    when(m_rRearRotateMotor.getInputs()).thenReturn(rRearSparkInputs);

    // Try to set module state
    SwerveModuleState state = new SwerveModuleState(+2.0, Rotation2d.fromRadians(+Math.PI));
    m_lFrontModule.set(state);
    m_rFrontModule.set(state);
    m_lRearModule.set(state);
    m_rRearModule.set(state);

    // Verify that motors are being driven with expected values
    verify(m_lFrontDriveMotor, times(1)).set(AdditionalMatchers.eq(-2.0, DELTA), ArgumentMatchers.eq(ControlType.kVelocity), ArgumentMatchers.anyDouble(), ArgumentMatchers.eq(ArbFFUnits.kVoltage));
    verify(m_lFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(-Math.PI / 2, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rFrontDriveMotor, times(1)).set(AdditionalMatchers.eq(+2.0, DELTA), ArgumentMatchers.eq(ControlType.kVelocity), ArgumentMatchers.anyDouble(), ArgumentMatchers.eq(ArbFFUnits.kVoltage));
    verify(m_rFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_lRearDriveMotor, times(1)).set(AdditionalMatchers.eq(+2.0, DELTA), ArgumentMatchers.eq(ControlType.kVelocity), ArgumentMatchers.anyDouble(), ArgumentMatchers.eq(ArbFFUnits.kVoltage));
    verify(m_lRearRotateMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rRearDriveMotor, times(1)).set(AdditionalMatchers.eq(-2.0, DELTA), ArgumentMatchers.eq(ControlType.kVelocity), ArgumentMatchers.anyDouble(), ArgumentMatchers.eq(ArbFFUnits.kVoltage));
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

    // Verify that motors are being driven with expected values
    verify(m_lFrontDriveMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kVelocity), ArgumentMatchers.anyDouble(), ArgumentMatchers.eq(ArbFFUnits.kVoltage));
    verify(m_lFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rFrontDriveMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kVelocity), ArgumentMatchers.anyDouble(), ArgumentMatchers.eq(ArbFFUnits.kVoltage));
    verify(m_rFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_lRearDriveMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kVelocity), ArgumentMatchers.anyDouble(), ArgumentMatchers.eq(ArbFFUnits.kVoltage));
    verify(m_lRearRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rRearDriveMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kVelocity), ArgumentMatchers.anyDouble(), ArgumentMatchers.eq(ArbFFUnits.kVoltage));
    verify(m_rRearRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
  }

  @Test
  @Order(4)
  @DisplayName("Test if module works in simulation")
  public void simulation() {
    // Hardcode sensor values
    SparkInputsAutoLogged defaultInputs = new SparkInputsAutoLogged();
    SparkInputsAutoLogged lFrontRotateMotorInputs = new SparkInputsAutoLogged();
    lFrontRotateMotorInputs.absoluteEncoderPosition = 0.0;
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
    m_lFrontModule.updateSimPosition();
    m_lFrontModule.getPosition();

    // Verify module reports expected position
    assertEquals(
      new SwerveModulePosition(
        desiredState.speedMetersPerSecond * GlobalConstants.ROBOT_LOOP_HZ.asPeriod().in(Units.Seconds),
        desiredState.angle.minus(
          Rotation2d.fromRadians(MAXSwerveModule.ZERO_OFFSET.get(SwerveModule.Location.LeftFront).in(Units.Radians)).unaryMinus()
      )),
      m_lFrontModule.getPosition()
    );
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

    assertTrue(m_lFrontModule.getMaxLinearVelocity().isNear(NEO_MAX_LINEAR_SPEED, DELTA));
    assertTrue(m_rFrontModule.getMaxLinearVelocity().isNear(NEO_MAX_LINEAR_SPEED, DELTA));
    assertTrue(m_lRearModule.getMaxLinearVelocity().isNear(VORTEX_MAX_LINEAR_SPEED, DELTA));
    assertTrue(m_rRearModule.getMaxLinearVelocity().isNear(VORTEX_MAX_LINEAR_SPEED, DELTA));
  }

  @Test
  @Order(6)
  @DisplayName("Test if module limits speed")
  public void tractionControl() {
    // Hardcode sensor values
    SparkInputsAutoLogged sparkInputs = new SparkInputsAutoLogged();
    when(m_lFrontRotateMotor.getInputs()).thenReturn(sparkInputs);
    sparkInputs.encoderVelocity = 3.0;
    when(m_lFrontDriveMotor.getInputs()).thenReturn(sparkInputs);

    // Hardcode other inputs
    var moduleState = new SwerveModuleState(NEO_MAX_LINEAR_SPEED, Rotation2d.fromDegrees(0.0));
    var realSpeeds = new ChassisSpeeds(Units.MetersPerSecond.of(-0.1), Units.MetersPerSecond.of(-3.0), Units.DegreesPerSecond.zero());

    // Attempt to drive module
    m_lFrontModule.set(moduleState, realSpeeds);

    // Verify that motors are being driven with expected values
    verify(m_lFrontDriveMotor).set(AdditionalMatchers.leq(NEO_MAX_LINEAR_SPEED.in(Units.MetersPerSecond)), ArgumentMatchers.eq(ControlType.kVelocity), ArgumentMatchers.anyDouble(), ArgumentMatchers.eq(ArbFFUnits.kVoltage));
    verify(m_lFrontRotateMotor).set(AdditionalMatchers.eq(-Math.PI / 2, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
  }
}
