// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.drive.swerves.revrobotics;

import java.time.Duration;
import java.time.Instant;

import org.lasarobotics.drive.TractionControlController;
import org.lasarobotics.drive.swerves.ModuleLocation;
import org.lasarobotics.hardware.ctre.CANCoder;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;
import org.lasarobotics.hardware.revrobotics.SparkPIDConfig;
import org.lasarobotics.utils.GlobalConstants;
import org.lasarobotics.utils.PIDConstants;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;

/** SDSMK4 module */
public class SDSMK4SwerveModule implements AutoCloseable {
  /**
   * SDSMK4 module hardware
   */
  public static class Hardware {
    private Spark driveMotor;
    private Spark rotateMotor;
    private CANCoder absoluteEncoder;

    public Hardware(Spark driveMotor, Spark rotateMotor, CANCoder absoluteEncoder) {
      this.driveMotor = driveMotor;
      this.rotateMotor = rotateMotor;
      this.absoluteEncoder = absoluteEncoder;
    }
  }

  public static class ModuleConfig {
    private GearRatio gearRatio;
    private boolean inverted;


  }

  /**
   * SDSMK4Swerve gear ratio
   */
  public enum GearRatio {
    /** 8.14:1 */
    L1(8.14),
    /** 6.75:1 */
    L2(6.75),
    /** 6.12:1 */
    L3(6.12),
    /** 5.14:1 */
    L4(5.14);

    public final double value;
    private GearRatio(double value) {
      this.value = value;
    }
  }

  private final double EPSILON = 5e-3;
  private final int DRIVE_MOTOR_CURRENT_LIMIT;
  private final int ROTATE_MOTOR_CURRENT_LIMIT = 30;
  private final double MK4_ROTATE_RATIO = 12.8;
  private final double MK4I_ROTATE_RATIO = 150.0 / 7.0;
  private final Rotation2d LOCK_POSITION = Rotation2d.fromRadians(Math.PI / 4);

  private static final double DRIVE_WHEEL_DIAMETER_METERS = Units.Inches.of(4).in(Units.Meters); // 4" wheels
  private static final double DRIVETRAIN_EFFICIENCY = 0.90;
  private static final double MAX_AUTO_LOCK_TIME = 10.0;
  private final double DRIVE_TICKS_PER_METER;
  private final double DRIVE_METERS_PER_TICK;
  private final double DRIVE_METERS_PER_ROTATION;
  private final double DRIVE_MAX_LINEAR_SPEED;

  // Swerve velocity PID settings
  private static final double DRIVE_VELOCITY_kP = 0.04;
  private static final double DRIVE_VELOCITY_TOLERANCE = 0.01;
  private static final boolean DRIVE_VELOCITY_SENSOR_PHASE = false;
  private static final boolean DRIVE_INVERT_MOTOR = false;

  // Swerve rotate PID settings
  private static final PIDConstants DRIVE_ROTATE_PID = new PIDConstants(0.5, 0.0, 0.0, 0.0);
  private static final double DRIVE_ROTATE_TOLERANCE = 0.02;
  private static final double DRIVE_ROTATE_LOWER_LIMIT = 0.0;
  private static final double DRIVE_ROTATE_UPPER_LIMIT = 0.0;
  private static final boolean DRIVE_ROTATE_SOFT_LIMITS = false;
  private static final boolean DRIVE_ROTATE_SENSOR_PHASE = false;
  private static final boolean DRIVE_ROTATE_INVERT_MOTOR = false;

  private Spark m_driveMotor;
  private Spark m_rotateMotor;
  private CANCoder m_absoluteEncoder;
  private Translation2d m_moduleCoordinate;
  private ModuleLocation m_location;
  private Rotation2d m_previousRotatePosition;

  private GearRatio m_driveGearRatio;
  private double m_driveConversionFactor;
  private double m_rotateConversionFactor;
  private double m_simDrivePosition;
  private double m_simRotatePosition;
  private double m_radius;
  private double m_autoLockTime;
  private boolean m_autoLock;

  private TractionControlController m_tractionControlController;
  private Instant m_autoLockTimer;
  private int m_moduleSynchronizationCounter = 0;

  /**
   * Create an instance of a SDSM4SwerveModule
   * @param swerveHardware Hardware devices required by swerve module
   * @param location Location of module
   * @param driveGearRatio Gear ratio for driving wheel
   * @param wheelbase Robot wheelbase
   * @param trackWidth Robot track width
   * @param autoLockTime Time before rotating module to locked position [0.0, 10.0]
   * @param maxSlippingTime Maximum time that wheel is allowed to slip
   * @param driveMotorCurrentLimit Desired current limit for the drive motor
   * @param inverted True if motors are mounted upside down, MK4i
   * @param slipRatio Desired slip ratio [+0.01, +0.40]
   */
  public SDSMK4SwerveModule(Hardware swerveHardware, ModuleLocation location, GearRatio driveGearRatio,
                            Measure<Distance> wheelbase, Measure<Distance> trackWidth, Measure<Time> autoLockTime,
                            Measure<Time> maxSlippingTime, Measure<Current> driveMotorCurrentLimit, boolean inverted, double slipRatio) {
    int encoderTicksPerRotation = swerveHardware.driveMotor.getKind().equals(MotorKind.NEO)
      ? GlobalConstants.NEO_ENCODER_TICKS_PER_ROTATION
      : GlobalConstants.VORTEX_ENCODER_TICKS_PER_ROTATION;
    DRIVE_TICKS_PER_METER = (encoderTicksPerRotation * driveGearRatio.value) * (1 / (DRIVE_WHEEL_DIAMETER_METERS * Math.PI));
    DRIVE_METERS_PER_TICK = 1 / DRIVE_TICKS_PER_METER;
    DRIVE_METERS_PER_ROTATION = DRIVE_METERS_PER_TICK * encoderTicksPerRotation;
    DRIVE_MAX_LINEAR_SPEED = (GlobalConstants.NEO_MAX_RPM / 60) * DRIVE_METERS_PER_ROTATION * DRIVETRAIN_EFFICIENCY;
    DRIVE_MOTOR_CURRENT_LIMIT = (int)driveMotorCurrentLimit.in(Units.Amps);

    this.m_driveMotor = swerveHardware.driveMotor;
    this.m_rotateMotor = swerveHardware.rotateMotor;
    this.m_absoluteEncoder = swerveHardware.absoluteEncoder;
    this.m_location = location;
    this.m_driveGearRatio = driveGearRatio;
    this.m_autoLock = true;
    this.m_simDrivePosition = 0.0;
    this.m_simRotatePosition = 0.0;
    this.m_autoLockTime = MathUtil.clamp(autoLockTime.in(Units.Milliseconds), 0.0, MAX_AUTO_LOCK_TIME * 1000);
    this.m_previousRotatePosition = LOCK_POSITION;
    this.m_tractionControlController =  new TractionControlController(Units.MetersPerSecond.of(DRIVE_MAX_LINEAR_SPEED), maxSlippingTime, slipRatio);
    this.m_autoLockTimer = Instant.now();

    // Set drive encoder conversion factor
    m_driveConversionFactor = DRIVE_WHEEL_DIAMETER_METERS * Math.PI / m_driveGearRatio.value;
    m_driveMotor.setPositionConversionFactor(Spark.FeedbackSensor.NEO_ENCODER, m_driveConversionFactor);
    m_driveMotor.setVelocityConversionFactor(Spark.FeedbackSensor.NEO_ENCODER, m_driveConversionFactor / 60);

    // Set rotate encoder conversion factor
    m_rotateConversionFactor = (2 * Math.PI) / (inverted ? MK4I_ROTATE_RATIO : MK4_ROTATE_RATIO);
    m_rotateMotor.setPositionConversionFactor(Spark.FeedbackSensor.NEO_ENCODER, m_rotateConversionFactor);
    m_rotateMotor.setVelocityConversionFactor(Spark.FeedbackSensor.NEO_ENCODER, m_rotateConversionFactor / 60);

    // Enable PID wrapping
    m_rotateMotor.enablePIDWrapping(-Math.PI, +Math.PI);

    // Create PID configs
    SparkPIDConfig driveMotorConfig = new SparkPIDConfig(
      new PIDConstants(
        DRIVE_VELOCITY_kP,
        0.0,
        0.0,
        1 / ((m_driveMotor.getKind().getMaxRPM() / 60) * m_driveConversionFactor)
      ),
      DRIVE_VELOCITY_SENSOR_PHASE,
      DRIVE_INVERT_MOTOR,
      DRIVE_VELOCITY_TOLERANCE
    );
    SparkPIDConfig rotateMotorConfig = new SparkPIDConfig(
      DRIVE_ROTATE_PID,
      DRIVE_ROTATE_SENSOR_PHASE,
      DRIVE_ROTATE_INVERT_MOTOR,
      DRIVE_ROTATE_TOLERANCE,
      DRIVE_ROTATE_LOWER_LIMIT,
      DRIVE_ROTATE_UPPER_LIMIT,
      DRIVE_ROTATE_SOFT_LIMITS
    );

    // Initialize PID
    m_driveMotor.initializeSparkPID(driveMotorConfig, Spark.FeedbackSensor.NEO_ENCODER);
    m_rotateMotor.initializeSparkPID(rotateMotorConfig, Spark.FeedbackSensor.NEO_ENCODER);

    // Set drive motor to coast
    m_driveMotor.setIdleMode(IdleMode.kCoast);

    // Set rotate motor to brake
    m_rotateMotor.setIdleMode(IdleMode.kBrake);

    // Set current limits
    m_driveMotor.setSmartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);
    m_rotateMotor.setSmartCurrentLimit(ROTATE_MOTOR_CURRENT_LIMIT);

    // Reset encoder
    resetDriveEncoder();


    m_absoluteEncoder.periodic();
    m_rotateMotor.resetEncoder(m_absoluteEncoder.getInputs().absolutePosition);

    // Calculate module coordinate
    switch (location) {
      case LeftFront:
        m_moduleCoordinate = new Translation2d(+wheelbase.in(Units.Meters) / 2, +trackWidth.in(Units.Meters) / 2);
        break;
      case RightFront:
        m_moduleCoordinate = new Translation2d(+wheelbase.in(Units.Meters) / 2, -trackWidth.in(Units.Meters) / 2);
        break;
      case LeftRear:
        m_moduleCoordinate = new Translation2d(-wheelbase.in(Units.Meters) / 2, +trackWidth.in(Units.Meters) / 2);
        break;
      case RightRear:
        m_moduleCoordinate = new Translation2d(-wheelbase.in(Units.Meters) / 2, -trackWidth.in(Units.Meters) / 2);
        break;
      default:
        m_moduleCoordinate = new Translation2d();
        break;
    }

    // Get distance from center of robot
    m_radius = m_moduleCoordinate.getNorm();

    // Make sure settings are burned to flash
    m_driveMotor.burnFlash();
    m_rotateMotor.burnFlash();
  }

  /**
   * Initialize hardware devices for SDSMK4 Swerve module
   * @param driveMotorID Drive motor ID
   * @param rotateMotorID Rotate motor ID
   * @param driveMotorKind Kind of drive motor
   * @return Hardware object containing all necessary objects for a MAXSwerve module
   * @throws IllegalArgumentException If specified drive motor is not supported
   */
  public static Hardware initializeHardware(Spark.ID driveMotorID, Spark.ID rotateMotorID, MotorKind driveMotorKind, MotorKind rotateMotorKind, CANCoder.ID absoluteEncoderID) {
    if (driveMotorKind != MotorKind.NEO && driveMotorKind != MotorKind.NEO_VORTEX)
      throw new IllegalArgumentException("Drive motor MUST be a NEO or a NEO Vortex!");
    if (rotateMotorKind != MotorKind.NEO && rotateMotorKind != MotorKind.NEO_VORTEX)
      throw new IllegalArgumentException("Rotate motor MUST be a NEO or a NEO Vortex!");

    Hardware swerveModuleHardware = new Hardware(
      new Spark(driveMotorID, driveMotorKind),
      new Spark(rotateMotorID, rotateMotorKind),
      new CANCoder(absoluteEncoderID)
    );

    return swerveModuleHardware;
  }

  /**
   * Get real speed of module
   * @param inertialVelocity Inertial velocity of robot (m/s)
   * @param rotateRate Rotate rate of robot (degrees/s)
   * @return Speed of module (m/s)
   */
  private Measure<Velocity<Distance>> calculateRealSpeed(double inertialVelocity, double rotateRate) {
    return Units.MetersPerSecond.of(inertialVelocity + Math.toRadians(rotateRate) * m_radius);
  }

  /**
   * Call this method periodically
   */
  public void periodic() {
    m_driveMotor.periodic();
    m_rotateMotor.periodic();
    m_absoluteEncoder.periodic();

    if (Math.abs(getRotationVelocity().magnitude()) <= EPSILON && ++m_moduleSynchronizationCounter > 5) {
      m_rotateMotor.resetEncoder(m_absoluteEncoder.getInputs().absolutePosition);
      m_moduleSynchronizationCounter = 0;
    }
  }


  /**
   * Call this method periodically during simulation
   */
  public void simulationPeriodic() {
    m_driveMotor.getInputs().encoderPosition = m_simDrivePosition;
    m_rotateMotor.getInputs().absoluteEncoderPosition = m_simRotatePosition;
  }

  /**
   * Set swerve module direction and speed
   * @param state Desired swerve module state
   */
  public void set(SwerveModuleState state) {
    // Auto lock modules if auto lock enabled, speed not requested, and time has elapsed
    if (m_autoLock && state.speedMetersPerSecond < EPSILON) {
      state.speedMetersPerSecond = 0.0;
      // Time's up, lock now...
      if (Duration.between(m_autoLockTimer, Instant.now()).toMillis() > m_autoLockTime)
        state.angle = LOCK_POSITION.minus(m_location.offset);
      // Waiting to lock...
      else state.angle = m_previousRotatePosition.minus(m_location.offset);
    } else {
      // Not locking this loop, restart timer...
      m_autoLockTimer = Instant.now();
    }

    // Apply chassis angular offset to the requested state.
    SwerveModuleState desiredState = new SwerveModuleState(
      state.speedMetersPerSecond,
      state.angle.plus(m_location.offset)
    );

    // Optimize swerve module rotation state
    // Rotate motor returns an angle in radians
    desiredState = SwerveModuleState.optimize(desiredState, Rotation2d.fromRadians(m_rotateMotor.getInputs().encoderPosition));

    // Set rotate motor position
    m_rotateMotor.set(desiredState.angle.getRadians(), ControlType.kPosition);

    // Set drive motor speed
    m_driveMotor.set(desiredState.speedMetersPerSecond, ControlType.kVelocity);

    // Save drive and rotate position for simulation purposes only
    m_simDrivePosition += desiredState.speedMetersPerSecond * GlobalConstants.ROBOT_LOOP_PERIOD;
    m_simRotatePosition = desiredState.angle.getRadians();

    // Save rotate position
    m_previousRotatePosition = desiredState.angle;
  }

  /**
   * Set swerve module direction and speed, automatically applying traction control
   * @param state Desired swerve module state
   * @param inertialVelocity Current inertial velocity
   * @param rotateRate Current rotate rate
   */
  public void set(SwerveModuleState state, Measure<Velocity<Distance>> inertialVelocity, Measure<Velocity<Angle>> rotateRate) {
    // Apply traction control
    state.speedMetersPerSecond = m_tractionControlController.calculate(
      Units.MetersPerSecond.of(state.speedMetersPerSecond),
      calculateRealSpeed(inertialVelocity.in(Units.MetersPerSecond), rotateRate.in(Units.DegreesPerSecond)),
      getDriveVelocity()
    ).in(Units.MetersPerSecond);

    // Set swerve module state
    set(state);
  }

  /**
   * Set swerve module direction and speed
   * @param states Array of states for all swerve modules
   */
  public void set(SwerveModuleState[] states) {
    set(states[m_location.index]);
  }

  /**
   * Set swerve module direction and speed, automatically applying traction control
   * @param states Array of states for all swerve modules
   * @param inertialVelocity Current inertial velocity
   * @param rotateRate Current turn rate
   */
  public void set(SwerveModuleState[] states, Measure<Velocity<Distance>> inertialVelocity, Measure<Velocity<Angle>> rotateRate) {
    set(states[m_location.index], inertialVelocity, rotateRate);
  }

  /**
   * Get velocity of drive wheel
   * @return velocity of drive wheel in m/s
   */
  public Measure<Velocity<Distance>> getDriveVelocity() {
    return Units.MetersPerSecond.of(m_driveMotor.getInputs().encoderVelocity);
  }

  /**
   * Get velocity of rotation motor
   * @return velocity of rotation motor in rad/s
   */
  public Measure<Velocity<Angle>> getRotationVelocity() {
    return Units.RadiansPerSecond.of(m_rotateMotor.getInputs().encoderVelocity);
  }

  /**
   * Get current module state
   * @return Current module state
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
      getDriveVelocity(),
      Rotation2d.fromRadians(m_rotateMotor.getInputs().encoderPosition).minus(m_location.offset)
    );
  }

  /**
   * Get module position
   * @return Current module position
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      m_driveMotor.getInputs().encoderPosition,
      Rotation2d.fromRadians(m_rotateMotor.getInputs().encoderPosition).minus(m_location.offset)
    );
  }

  /**
   * Reset drive motor encoder
   */
  public void resetDriveEncoder() {
    m_driveMotor.resetEncoder();
    m_simDrivePosition = 0.0;
  }

  /**
   * Lock swerve module
   */
  public void lock() {
    set(new SwerveModuleState(0.0, LOCK_POSITION.minus(m_location.offset)));
  }

  /**
   * Reset swerve module to 0 degrees
   */
  public void reset() {
    set(new SwerveModuleState(0.0, m_location.offset));
  }

  /**
   * Toggle traction control
   */
  public void toggleTractionControl() {
    m_tractionControlController.toggleTractionControl();
    if (m_tractionControlController.isEnabled()) m_driveMotor.setIdleMode(IdleMode.kCoast);
    else m_driveMotor.setIdleMode(IdleMode.kBrake);
  }

  /**
   * Enable traction control
   */
  public void enableTractionControl() {
    m_tractionControlController.enableTractionControl();
    m_driveMotor.setIdleMode(IdleMode.kCoast);
  }

  /**
   * Disable traction control
   */
  public void disableTractionControl() {
    m_tractionControlController.disableTractionControl();
    m_driveMotor.setIdleMode(IdleMode.kBrake);
  }

  /**
   * Get whether traction control is enabled
   * @return True if enabled
   */
  public boolean isTractionControlEnabled() {
    return m_tractionControlController.isEnabled();
  }

  /**
   * Get maximum drive speed of module
   * @return Max linear speed (m/s)
   */
  public Measure<Velocity<Distance>> getMaxLinearSpeed() {
    return Units.MetersPerSecond.of(DRIVE_MAX_LINEAR_SPEED);
  }

  /**
   * Get coordinate of module relative to the center of the robot
   * @return X, Y coordinate in meters
   */
  public Translation2d getModuleCoordinate() {
    return m_moduleCoordinate;
  }

  /**
   * Stop swerve module
   */
  public void stop() {
    m_rotateMotor.stopMotor();
    m_driveMotor.stopMotor();
  }

  @Override
  public void close() {
    m_driveMotor.close();
    m_rotateMotor.close();
  }
}
