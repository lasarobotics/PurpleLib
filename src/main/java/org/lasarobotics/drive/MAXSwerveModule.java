// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.drive;

import org.lasarobotics.hardware.SparkMax;
import org.lasarobotics.utils.GlobalConstants;
import org.lasarobotics.utils.PIDConstants;
import org.lasarobotics.utils.SparkPIDConfig;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;

/**
 * REV MAXSwerve module
 */
public class MAXSwerveModule implements AutoCloseable {
  public static class Hardware {
    private SparkMax driveMotor;
    private SparkMax rotateMotor;

    public Hardware(SparkMax driveMotor, SparkMax rotateMotor) {
      this.driveMotor = driveMotor;
      this.rotateMotor = rotateMotor;
    }
  }

  public enum ModuleLocation {
    LeftFront(0, -Math.PI / 2),
    RightFront(1, +0.0),
    LeftRear(2, +Math.PI),
    RightRear(3, +Math.PI / 2);

    public final int index;
    public final double offset;
    private ModuleLocation(int index, double offset) {
      this.index = index;
      this.offset = offset;
    }
  }

  public enum GearRatio {
    Low(5.50),
    Med(5.08),
    High(4.71);

    public final double value;
    private GearRatio(double value) {
      this.value = value;
    }
  }

  private final double LOCK_POSITION = Math.PI / 4;
  private final double EPSILON = 2e-3;
  private final int DRIVE_MOTOR_CURRENT_LIMIT = 50;
  private final int ROTATE_MOTOR_CURRENT_LIMIT = 20;

  private static final double DRIVE_WHEEL_DIAMETER_METERS = 0.0762; // 3" wheels
  private static final double DRIVETRAIN_EFFICIENCY = 0.90;
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
  private static final PIDConstants DRIVE_ROTATE_PID = new PIDConstants(1.0, 0.0, 0.0, 0.0);
  private static final double DRIVE_ROTATE_TOLERANCE = 0.01;
  private static final double DRIVE_ROTATE_LOWER_LIMIT = 0.0;
  private static final double DRIVE_ROTATE_UPPER_LIMIT = 0.0;
  private static final boolean DRIVE_ROTATE_SOFT_LIMITS = false;
  private static final boolean DRIVE_ROTATE_SENSOR_PHASE = true;
  private static final boolean DRIVE_ROTATE_INVERT_MOTOR = false;

  private SparkMax m_driveMotor;
  private SparkMax m_rotateMotor;
  private Translation2d m_moduleCoordinate;
  private ModuleLocation m_location;

  private GearRatio m_driveGearRatio;
  private double m_driveConversionFactor;
  private double m_rotateConversionFactor;
  private double m_simDrivePosition;
  private double m_simRotatePosition;
  private double m_radius;
  private boolean m_autoLock;

  private TractionControlController m_tractionControlController;

  /**
   * Create an instance of a MAXSwerveModule
   * @param swerveHardware Hardware devices required by swerve module
   * @param location Location of module
   * @param driveGearRatio Gear ratio for driving wheel
   * @param slipRatio Desired slip ratio
   * @param wheelbase Robot wheelbase in meters
   * @param trackWidth Robot track width in meters
   */
  public MAXSwerveModule(Hardware swerveHardware, ModuleLocation location, GearRatio driveGearRatio,
                         double slipRatio, double wheelbase, double trackWidth) {
    DRIVE_TICKS_PER_METER = (GlobalConstants.NEO_ENCODER_TICKS_PER_ROTATION * driveGearRatio.value) * (1 / (DRIVE_WHEEL_DIAMETER_METERS * Math.PI));
    DRIVE_METERS_PER_TICK = 1 / DRIVE_TICKS_PER_METER;
    DRIVE_METERS_PER_ROTATION = DRIVE_METERS_PER_TICK * GlobalConstants.NEO_ENCODER_TICKS_PER_ROTATION;
    DRIVE_MAX_LINEAR_SPEED = (GlobalConstants.NEO_MAX_RPM / 60) * DRIVE_METERS_PER_ROTATION * DRIVETRAIN_EFFICIENCY;

    this.m_driveMotor = swerveHardware.driveMotor;
    this.m_rotateMotor = swerveHardware.rotateMotor;
    this.m_location = location;
    this.m_driveGearRatio = driveGearRatio;
    this.m_autoLock = true;
    this.m_simDrivePosition = 0.0;
    this.m_simRotatePosition = 0.0;
    this.m_tractionControlController =  new TractionControlController(slipRatio, DRIVE_MAX_LINEAR_SPEED);

    // Set drive motor to coast
    m_driveMotor.setIdleMode(IdleMode.kCoast);

    // Set rotate motor to brake
    m_rotateMotor.setIdleMode(IdleMode.kBrake);

    // Set current limits
    m_driveMotor.setSmartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);
    m_rotateMotor.setSmartCurrentLimit(ROTATE_MOTOR_CURRENT_LIMIT);

    // Reset encoder
    resetDriveEncoder();

    // Set drive encoder conversion factor
    m_driveConversionFactor = DRIVE_WHEEL_DIAMETER_METERS * Math.PI / m_driveGearRatio.value;
    m_driveMotor.setPositionConversionFactor(SparkMax.FeedbackSensor.NEO_ENCODER, m_driveConversionFactor);
    m_driveMotor.setVelocityConversionFactor(SparkMax.FeedbackSensor.NEO_ENCODER, m_driveConversionFactor / 60);

    // Set rotate encoder conversion factor
    m_rotateConversionFactor = 2 * Math.PI;
    m_rotateMotor.setPositionConversionFactor(SparkMax.FeedbackSensor.THROUGH_BORE_ENCODER, m_rotateConversionFactor);
    m_rotateMotor.setVelocityConversionFactor(SparkMax.FeedbackSensor.THROUGH_BORE_ENCODER, m_rotateConversionFactor / 60);

    // Create PID configs
    SparkPIDConfig driveMotorConfig = new SparkPIDConfig(
      new PIDConstants(DRIVE_VELOCITY_kP, 0.0, 0.0, 1 / ((GlobalConstants.NEO_MAX_RPM / 60) * m_driveConversionFactor)),
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
    m_driveMotor.initializeSparkPID(driveMotorConfig, SparkMax.FeedbackSensor.NEO_ENCODER);
    m_rotateMotor.initializeSparkPID(rotateMotorConfig, SparkMax.FeedbackSensor.THROUGH_BORE_ENCODER);

    // Enable PID wrapping
    m_rotateMotor.enablePIDWrapping(0.0, 2 * Math.PI);

    // Add motors to REVPhysicsSim
    m_driveMotor.addToSimulation(DCMotor.getNEO(1));
    m_rotateMotor.addToSimulation(DCMotor.getNeo550(1));

    // Calculate module coordinate
    switch (location) {
      case LeftFront:
        m_moduleCoordinate = new Translation2d(+wheelbase / 2, +trackWidth / 2);
        break;
      case RightFront:
        m_moduleCoordinate = new Translation2d(+wheelbase / 2, -trackWidth / 2);
        break;
      case LeftRear:
        m_moduleCoordinate = new Translation2d(-wheelbase / 2, +trackWidth / 2);
        break;
      case RightRear:
        m_moduleCoordinate = new Translation2d(-wheelbase / 2, -trackWidth / 2);
        break;
      default:
        m_moduleCoordinate = new Translation2d();
        break;
    }

    // Get distance from center of robot
    m_radius = m_moduleCoordinate.getNorm();
  }

  /**
   * Initialize hardware devices for MAXSwerve module
   * @param driveMotorID Drive motor ID
   * @param rotateMotorID Rotate motor ID
   * @return Hardware object containing all necessary objects for a MAXSwerve module
   */
  public static Hardware initializeHardware(SparkMax.ID driveMotorID, SparkMax.ID rotateMotorID) {
    Hardware swerveModuleHardware = new Hardware(
      new SparkMax(driveMotorID, MotorType.kBrushless),
      new SparkMax(rotateMotorID, MotorType.kBrushless)
    );

    return swerveModuleHardware;
  }

  /**
   * Get real speed of module
   * @param inertialVelocity Inertial velocity of robot (m/s)
   * @param rotateRate Rotate rate of robot (degrees/s)
   * @return Speed of module (m/s)
   */
  private double calculateRealSpeed(double inertialVelocity, double rotateRate) {
    return inertialVelocity + Math.toRadians(rotateRate) * m_radius;
  }

  /**
   * Call this method periodically
   */
  public void periodic() {
    m_driveMotor.periodic();
    m_rotateMotor.periodic();
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
    // Auto lock modules if enabled and speed not requested
    if (m_autoLock && state.speedMetersPerSecond < EPSILON) {
      state.speedMetersPerSecond = 0.0;
      state.angle = Rotation2d.fromRadians(LOCK_POSITION - m_location.offset);
    }

    // Apply chassis angular offset to the requested state.
    SwerveModuleState desiredState = new SwerveModuleState(
      state.speedMetersPerSecond,
      state.angle.plus(Rotation2d.fromRadians(m_location.offset))
    );

    // Optimize swerve module rotation state
    // REV encoder returns an angle in radians
    desiredState = SwerveModuleState.optimize(desiredState, Rotation2d.fromRadians(m_rotateMotor.getInputs().absoluteEncoderPosition));

    // Set rotate motor position
    m_rotateMotor.set(desiredState.angle.getRadians(), ControlType.kPosition);

    // Set drive motor speed
    m_driveMotor.set(desiredState.speedMetersPerSecond, ControlType.kVelocity);

    // Save drive and rotate position for simulation purposes only
    m_simDrivePosition += desiredState.speedMetersPerSecond * GlobalConstants.ROBOT_LOOP_PERIOD;
    m_simRotatePosition = desiredState.angle.getRadians();
  }

  /**
   * Set swerve module direction and speed, automatically applying traction control
   * @param state Desired swerve module state
   * @param inertialVelocity Current inertial velocity (m/s)
   * @param rotateRate Current rotate rate (degrees/s)
   */
  public void set(SwerveModuleState state, double inertialVelocity, double rotateRate) {
    // Apply traction control
    state.speedMetersPerSecond = m_tractionControlController.calculate(
      state.speedMetersPerSecond,
      calculateRealSpeed(inertialVelocity, rotateRate),
      getDriveVelocity()
    );

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
   * @param inertialVelocity Current inertial velocity (m/s)
   * @param rotateRate Current turn rate (degrees/s)
   */
  public void set(SwerveModuleState[] states, double inertialVelocity, double rotateRate) {
    set(states[m_location.index], inertialVelocity, rotateRate);
  }

  /**
   * Get velocity of drive wheel
   * @return velocity of drive wheel in m/s
   */
  public double getDriveVelocity() {
    return m_driveMotor.getInputs().encoderVelocity;
  }

  /**
   * Get current module state
   * @return Current module state
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
      getDriveVelocity(),
      Rotation2d.fromRadians(m_rotateMotor.getInputs().absoluteEncoderPosition - m_location.offset)
    );
  }

  /**
   * Get module position
   * @return Current module position
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      m_driveMotor.getInputs().encoderPosition,
      Rotation2d.fromRadians(m_rotateMotor.getInputs().absoluteEncoderPosition - m_location.offset)
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
    set(new SwerveModuleState(0.0, Rotation2d.fromRadians(LOCK_POSITION - m_location.offset)));
  }

  /**
   * Reset swerve module to 0 degrees
   */
  public void reset() {
    set(new SwerveModuleState(0.0, Rotation2d.fromRadians(m_location.offset)));
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
   * Get coordinate of module relative to the center of the robot
   * @return X, Y coordinate in meters
   */
  public Translation2d getModuleCoordinate() {
    return m_moduleCoordinate;
  }

  /**
   * Get drive gear ratio
   * @return Gear ratio for driving wheel
   */
  public GearRatio getDriveGearRatio() {
    return m_driveGearRatio;
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
