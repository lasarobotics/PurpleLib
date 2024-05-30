// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.drive;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.time.Duration;
import java.time.Instant;

import org.lasarobotics.hardware.PurpleManager;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;
import org.lasarobotics.hardware.revrobotics.SparkPIDConfig;
import org.lasarobotics.utils.GlobalConstants;
import org.lasarobotics.utils.PIDConstants;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

/** REV MAXSwerve module */
public class MAXSwerveModule implements AutoCloseable {
  /**
   * MAXSwerve module hardware
   */
  public static class Hardware {
    private Spark driveMotor;
    private Spark rotateMotor;

    public Hardware(Spark driveMotor, Spark rotateMotor) {
      this.driveMotor = driveMotor;
      this.rotateMotor = rotateMotor;
    }
  }

  /** Module location */
  public enum ModuleLocation {
    LeftFront(0, Rotation2d.fromRadians(-Math.PI / 2)),
    RightFront(1, Rotation2d.fromRadians(+0.0)),
    LeftRear(2, Rotation2d.fromRadians(+Math.PI)),
    RightRear(3, Rotation2d.fromRadians(+Math.PI / 2));

    /** Module index */
    public final int index;
    /** Module orientation chassis offset */
    public final Rotation2d offset;
    private ModuleLocation(int index, Rotation2d offset) {
      this.index = index;
      this.offset = offset;
    }
  }

  /**
   * MAXSwerve gear ratio
   */
  public enum GearRatio {
    /** 5.50:1 */
    L1(5.50),
    /** 5.08:1 */
    L2(5.08),
    /** 4.71:1 */
    L3(4.71),
    /** 4.50:1 */
    L4(4.50),
    /** 4.29:1 */
    L5(4.29),
    /** 4.00:1 */
    L6(4.00),
    /** 3.75:1 */
    L7(3.75),
    /** 3.56:1 */
    L8(3.56);

    public final double value;
    private GearRatio(double value) {
      this.value = value;
    }
  }

  private final double EPSILON = 5e-3;
  private final Measure<Current> DRIVE_MOTOR_CURRENT_LIMIT;
  private final Measure<Current> ROTATE_MOTOR_CURRENT_LIMIT = Units.Amps.of(18.0);
  private final Rotation2d LOCK_POSITION = Rotation2d.fromRadians(Math.PI / 4);
  private static final Measure<Time> DEFAULT_PERIOD = Units.Milliseconds.of(5.0);

  private static final String IS_SLIPPING_LOG_ENTRY = "/IsSlipping";
  private static final String ODOMETER_LOG_ENTRY = "/Odometer";
  private static final double DRIVE_WHEEL_DIAMETER_METERS = 0.0762; // 3" wheels
  private static final double DRIVETRAIN_EFFICIENCY = 0.90;
  private static final double MAX_AUTO_LOCK_TIME = 10.0;
  private final double DRIVE_TICKS_PER_METER;
  private final double DRIVE_METERS_PER_TICK;
  private final double DRIVE_METERS_PER_ROTATION;
  private final double DRIVE_MAX_LINEAR_SPEED;

  // Swerve velocity PID settings
  private static final double DRIVE_VELOCITY_kP = 0.18;
  private static final double DRIVE_VELOCITY_kD = 0.001;
  private static final double DRIVE_VELOCITY_TOLERANCE = 0.01;
  private static final boolean DRIVE_VELOCITY_SENSOR_PHASE = false;
  private static final boolean DRIVE_INVERT_MOTOR = false;

  // Swerve rotate PID settings
  private static final PIDConstants DRIVE_ROTATE_PID = new PIDConstants(2.1, 0.0, 0.2, 0.0, 0.0);
  private static final double DRIVE_ROTATE_TOLERANCE = 0.01;
  private static final double DRIVE_ROTATE_LOWER_LIMIT = 0.0;
  private static final double DRIVE_ROTATE_UPPER_LIMIT = 0.0;
  private static final boolean DRIVE_ROTATE_SOFT_LIMITS = false;
  private static final boolean DRIVE_ROTATE_SENSOR_PHASE = true;
  private static final boolean DRIVE_ROTATE_INVERT_MOTOR = false;

  private Spark m_driveMotor;
  private Spark m_rotateMotor;
  private Translation2d m_moduleCoordinate;
  private ModuleLocation m_location;
  private Rotation2d m_previousRotatePosition;

  private volatile double m_simDrivePosition;
  private volatile SwerveModuleState m_desiredState;

  private GearRatio m_driveGearRatio;
  private double m_driveConversionFactor;
  private double m_rotateConversionFactor;
  private double m_radius;
  private double m_autoLockTime;
  private boolean m_autoLock;
  private double m_runningOdometer;
  private String m_odometerOutputPath;

  private TractionControlController m_tractionControlController;
  private Instant m_autoLockTimer;

  /**
   * Create an instance of a MAXSwerveModule
   * @param swerveHardware Hardware devices required by swerve module
   * @param location Location of module
   * @param driveGearRatio Gear ratio for driving wheel
   * @param wheelbase Robot wheelbase
   * @param trackWidth Robot track width
   * @param autoLockTime Time before rotating module to locked position [0.0, 10.0]
   * @param maxSlippingTime Maximum time that wheel is allowed to slip
   * @param driveMotorCurrentLimit Desired current limit for the drive motor
   * @param slipRatio Desired slip ratio [+0.01, +0.40]
   */
  public MAXSwerveModule(Hardware swerveHardware, ModuleLocation location, GearRatio driveGearRatio,
                         Measure<Distance> wheelbase, Measure<Distance> trackWidth, Measure<Time> autoLockTime,
                         Measure<Time> maxSlippingTime, Measure<Current> driveMotorCurrentLimit, double slipRatio) {
    int encoderTicksPerRotation = swerveHardware.driveMotor.getKind().equals(MotorKind.NEO)
      ? GlobalConstants.NEO_ENCODER_TICKS_PER_ROTATION
      : GlobalConstants.VORTEX_ENCODER_TICKS_PER_ROTATION;
    DRIVE_MOTOR_CURRENT_LIMIT = driveMotorCurrentLimit;
    DRIVE_TICKS_PER_METER =
      (encoderTicksPerRotation * driveGearRatio.value)
      * (1 / (DRIVE_WHEEL_DIAMETER_METERS * Math.PI));
    DRIVE_METERS_PER_TICK = 1 / DRIVE_TICKS_PER_METER;
    DRIVE_METERS_PER_ROTATION = DRIVE_METERS_PER_TICK * encoderTicksPerRotation;
    DRIVE_MAX_LINEAR_SPEED = (swerveHardware.driveMotor.getKind().getMaxRPM() / 60) * DRIVE_METERS_PER_ROTATION * DRIVETRAIN_EFFICIENCY;

    this.m_driveMotor = swerveHardware.driveMotor;
    this.m_rotateMotor = swerveHardware.rotateMotor;
    this.m_location = location;
    this.m_driveGearRatio = driveGearRatio;
    this.m_autoLock = true;
    this.m_simDrivePosition = 0.0;
    this.m_desiredState = new SwerveModuleState(Units.MetersPerSecond.of(0.0), LOCK_POSITION);
    this.m_autoLockTime = MathUtil.clamp(autoLockTime.in(Units.Milliseconds), 0.0, MAX_AUTO_LOCK_TIME * 1000);
    this.m_previousRotatePosition = LOCK_POSITION;
    this.m_tractionControlController =  new TractionControlController(Units.MetersPerSecond.of(DRIVE_MAX_LINEAR_SPEED), maxSlippingTime, slipRatio);
    this.m_autoLockTimer = Instant.now();
    this.m_runningOdometer = 0.0;

    // Set drive encoder conversion factor
    m_driveConversionFactor = DRIVE_WHEEL_DIAMETER_METERS * Math.PI / m_driveGearRatio.value;
    m_driveMotor.setPositionConversionFactor(Spark.FeedbackSensor.NEO_ENCODER, m_driveConversionFactor);
    m_driveMotor.setVelocityConversionFactor(Spark.FeedbackSensor.NEO_ENCODER, m_driveConversionFactor / 60);

    // Set rotate encoder conversion factor
    m_rotateConversionFactor = 2 * Math.PI;
    m_rotateMotor.setPositionConversionFactor(Spark.FeedbackSensor.THROUGH_BORE_ENCODER, m_rotateConversionFactor);
    m_rotateMotor.setVelocityConversionFactor(Spark.FeedbackSensor.THROUGH_BORE_ENCODER, m_rotateConversionFactor / 60);

    // Enable PID wrapping
    m_rotateMotor.enablePIDWrapping(0.0, m_rotateConversionFactor);

    // Create PID configs
    SparkPIDConfig driveMotorConfig = new SparkPIDConfig(
      new PIDConstants(
        DRIVE_VELOCITY_kP,
        0.0,
        DRIVE_VELOCITY_kD,
        1 / ((m_driveMotor.getKind().getMaxRPM() / 60) * m_driveConversionFactor),
        0.0
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
    m_rotateMotor.initializeSparkPID(rotateMotorConfig, Spark.FeedbackSensor.THROUGH_BORE_ENCODER);

    // Set drive motor to coast
    m_driveMotor.setIdleMode(IdleMode.kCoast);

    // Set rotate motor to brake
    m_rotateMotor.setIdleMode(IdleMode.kBrake);

    // Set current limits
    m_driveMotor.setSmartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);
    m_rotateMotor.setSmartCurrentLimit(ROTATE_MOTOR_CURRENT_LIMIT);

    // Set status frame rates
    m_driveMotor.setPeriodicFrameRate(PeriodicFrame.kStatus1, DEFAULT_PERIOD);
    m_driveMotor.setPeriodicFrameRate(PeriodicFrame.kStatus2, DEFAULT_PERIOD);
    m_rotateMotor.setPeriodicFrameRate(PeriodicFrame.kStatus5, DEFAULT_PERIOD);
    m_rotateMotor.setPeriodicFrameRate(PeriodicFrame.kStatus6, DEFAULT_PERIOD);

    // Reset encoder
    resetDriveEncoder();

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

    // Add callbacks to PurpleManager
    PurpleManager.addCallback(() -> periodic());

    // Setup disabled triggers
    RobotModeTriggers.disabled().onTrue(Commands.runOnce(() -> disabledInit()).ignoringDisable(true));
    RobotModeTriggers.disabled().onFalse(Commands.runOnce(() -> disabledExit()).ignoringDisable(true));

    // Get distance from center of robot
    m_radius = m_moduleCoordinate.getNorm();

    // Make sure settings are burned to flash
    m_driveMotor.burnFlash();
    m_rotateMotor.burnFlash();

    // Read odometer file if exists
    m_odometerOutputPath = (m_driveMotor.getID().name + "-odometer.txt").replace('/', '-');
    File file = new File(m_odometerOutputPath);
    Measure<Distance> previousDistanceTraveled = Units.Meters.of(0.0);
    if (file.exists()) {
      try {
        previousDistanceTraveled =
          Units.Meters.of(Double.parseDouble(
            new String(Files.readAllBytes(Paths.get(m_odometerOutputPath)), StandardCharsets.UTF_8)
          ));
      } catch (IOException e) {
        e.printStackTrace();
      }
    }
    m_runningOdometer += previousDistanceTraveled.in(Units.Meters);
  }

  /**
   * Initialize hardware devices for MAXSwerve module
   * @param driveMotorID Drive motor ID
   * @param rotateMotorID Rotate motor ID
   * @param driveMotorKind Kind of drive motor
   * @return Hardware object containing all necessary objects for a MAXSwerve module
   * @throws IllegalArgumentException If specified drive motor is not supported
   */
  public static Hardware initializeHardware(Spark.ID driveMotorID, Spark.ID rotateMotorID, MotorKind driveMotorKind) {
    if (driveMotorKind != MotorKind.NEO && driveMotorKind != MotorKind.NEO_VORTEX)
      throw new IllegalArgumentException("Drive motor MUST be a NEO or a NEO Vortex!");
    //var period = RobotBase.isReal() ? DEFAULT_PERIOD : Units.Seconds.of(GlobalConstants.ROBOT_LOOP_PERIOD);
    Hardware swerveModuleHardware = new Hardware(
      new Spark(driveMotorID, driveMotorKind, DEFAULT_PERIOD),
      new Spark(rotateMotorID, MotorKind.NEO_550, DEFAULT_PERIOD)
    );

    return swerveModuleHardware;
  }

  /**
   * Get desired swerve module state
   * @param requestedState Requested state
   * @return Actual desired state for module
   */
  private SwerveModuleState getDesiredState(SwerveModuleState requestedState) {
    // Apply chassis angular offset to the requested state.
    var desiredState = new SwerveModuleState(
      requestedState.speedMetersPerSecond,
      requestedState.angle.plus(m_location.offset)
    );

    // Get current module angle
    var currentAngle = Rotation2d.fromRadians(m_rotateMotor.getInputs().absoluteEncoderPosition);

    // Optimize swerve module rotation state
    // REV encoder returns an angle in radians
    desiredState = SwerveModuleState.optimize(desiredState, currentAngle);

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother driving.
    desiredState.speedMetersPerSecond *= desiredState.angle.minus(currentAngle).getCos();

    // Return corrected desired swerve module state
    return desiredState;
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
  private void periodic() {
    Logger.recordOutput(m_driveMotor.getID().name + IS_SLIPPING_LOG_ENTRY, isSlipping());
    Logger.recordOutput(m_driveMotor.getID().name + ODOMETER_LOG_ENTRY, m_runningOdometer);
  }

  /**
   * Call method during initialization of disabled mode to set drive motor to brake mode and log odometry
   */
  private void disabledInit() {
    m_driveMotor.setIdleMode(IdleMode.kBrake);

    try {
      FileWriter fileWriter = new FileWriter(m_odometerOutputPath);
      fileWriter.write(String.valueOf(m_runningOdometer));
      fileWriter.close();
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  /**
   * Call method when exiting disabled mode to set drive motor to coast mode
   */
  private void disabledExit() {
    m_driveMotor.setIdleMode(IdleMode.kCoast);
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

    // Get desired state
    m_desiredState = getDesiredState(state);

    // Set rotate motor position
    m_rotateMotor.set(m_desiredState.angle.getRadians(), ControlType.kPosition);

    // Set drive motor speed
    m_driveMotor.set(m_desiredState.speedMetersPerSecond, ControlType.kVelocity);

    // Save rotate position
    m_previousRotatePosition = m_desiredState.angle;

    // Increment odometer
    m_runningOdometer += Math.abs(m_desiredState.speedMetersPerSecond) * GlobalConstants.ROBOT_LOOP_PERIOD;
  }

  /**
   * Set swerve module direction and speed, automatically applying traction control
   * @param state Desired swerve module state
   * @param inertialVelocity Current inertial velocity
   * @param rotateRate Desired robot rotate rate
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
   * Get current module state
   * @return Current module state
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
      getDriveVelocity(),
      Rotation2d.fromRadians(m_rotateMotor.getInputs().absoluteEncoderPosition).minus(m_location.offset)
    );
  }

  /**
   * Get module position
   * <p>
   * Must be called periodically in simulation to keep position updated.
   * Usually this is automatically done by the pose estimator service.
   * @return Current module position
   */
  public SwerveModulePosition getPosition() {
    if (RobotBase.isReal()) {
      return new SwerveModulePosition(
        m_driveMotor.getInputs().encoderPosition,
        Rotation2d.fromRadians(m_rotateMotor.getInputs().absoluteEncoderPosition).minus(m_location.offset)
      );
    }

    m_simDrivePosition += m_desiredState.speedMetersPerSecond * DEFAULT_PERIOD.in(Units.Seconds);
    synchronized (m_driveMotor.getInputs()) {
      m_driveMotor.getInputs().encoderPosition = m_simDrivePosition;
      synchronized (m_rotateMotor.getInputs()) {
        m_rotateMotor.getInputs().absoluteEncoderPosition = m_desiredState.angle.getRadians();
        return new SwerveModulePosition(
          m_driveMotor.getInputs().encoderPosition,
          Rotation2d.fromRadians(m_rotateMotor.getInputs().absoluteEncoderPosition).minus(m_location.offset)
        );
      }
    }
  }

  /**
   * Get if drive wheel is slipping
   * @return True if wheel is slipping excessively
   */
  public boolean isSlipping() {
    return m_tractionControlController.isSlipping();
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
   * @return Max linear speed
   */
  public Measure<Velocity<Distance>> getMaxLinearSpeed() {
    return Units.MetersPerSecond.of(DRIVE_MAX_LINEAR_SPEED);
  }

  /**
   * Get location of module on robot chassis
   * @return Location of module
   */
  public ModuleLocation getModuleLocation() {
    return m_location;
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
   * Reset the total distance the swerve module has traveled
   */
  public void clearRunningOdometer() {
    m_runningOdometer = 0.0;
    try {
      FileWriter fileWriter = new FileWriter(m_odometerOutputPath);
      fileWriter.write(String.valueOf(m_runningOdometer));
      fileWriter.close();
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  /**
   * Get the total distance the swerve module has traveled
   * @return Odometer value for the swerve module
   */
  public Measure<Distance> getRunningOdometer() {
    return Units.Meters.of(m_runningOdometer);
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
