// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.drive.swerve.parent;

import java.time.Duration;
import java.time.Instant;

import org.lasarobotics.drive.TractionControlController;
import org.lasarobotics.drive.swerve.DriveWheel;
import org.lasarobotics.drive.swerve.SwerveModule;
import org.lasarobotics.drive.swerve.SwerveModuleSim;
import org.lasarobotics.hardware.PurpleManager;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;
import org.lasarobotics.utils.FFConstants;
import org.lasarobotics.utils.GlobalConstants;
import org.lasarobotics.utils.PIDConstants;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

/** REV MAXSwerve module */
public class REVSwerveModule extends SwerveModule implements Sendable {
  /**
   * REV swerve module hardware
   */
  public static class Hardware {
    public final Spark driveMotor;
    public final Spark rotateMotor;

    public Hardware(Spark driveMotor, Spark rotateMotor) {
      this.driveMotor = driveMotor;
      this.rotateMotor = rotateMotor;
    }
  }

  private static final Frequency UPDATE_RATE = Units.Hertz.of(100.0);
  public static final Time DEFAULT_SIGNAL_PERIOD = UPDATE_RATE.asPeriod();
  public static final Time SLOW_SIGNAL_PERIOD = Units.Seconds.one();
  public static final double DRIVETRAIN_EFFICIENCY = 0.90;

  private final double EPSILON = 5e-3;
  private final Current DRIVE_MOTOR_CURRENT_LIMIT;
  private final Current ROTATE_MOTOR_CURRENT_LIMIT = Units.Amps.of(20.0);

  private static final String ROTATE_ERROR_LOG_ENTRY = "/RotateError";
  private static final String MAX_LINEAR_VELOCITY_LOG_ENTRY = "/MaxLinearVelocity";
  private static final double MAX_AUTO_LOCK_TIME = 10.0;
  private final double DRIVE_TICKS_PER_METER;
  private final double DRIVE_METERS_PER_TICK;
  private final double DRIVE_METERS_PER_ROTATION;
  private final double DRIVE_MAX_LINEAR_SPEED;

  private Spark m_driveMotor;
  private Spark m_rotateMotor;
  private SwerveModuleSim m_moduleSim;
  private SimpleMotorFeedforward m_driveFF;
  private SparkBaseConfig m_driveMotorConfig;
  private SparkBaseConfig m_rotateMotorConfig;
  private Rotation2d m_zeroOffset;

  private SwerveModule.Location m_location;
  private Rotation2d m_previousRotatePosition;

  private volatile MutDistance m_simDrivePosition;
  private volatile SwerveModulePosition m_simModulePosition;
  private volatile SwerveModuleState m_desiredState;

  private double m_driveConversionFactor;
  private double m_rotateConversionFactor;
  private double m_autoLockTime;

  private Instant m_autoLockTimer;

  /**
   * Create an instance of a REV swerve module
   * @param swerveHardware Hardware devices required by swerve module
   * @param location Module location
   * @param motorOrientation Motor mount orientation
   * @param encoderOrientation Encoder orientation
   * @param gearRatio Module gear ratio
   * @param driveWheel Wheel installed in swerve module
   * @param zeroOffset Chassis zero offset due to module calibration position
   * @param drivePID Drive motor PID gains
   * @param driveFF Drive motor feed forward gains
   * @param rotatePID Rotate motor PID gains
   * @param rotateFF Rotate motor feed forward gains
   * @param slipRatio Desired slip ratio [1%, 40%]
   * @param mass Robot mass
   * @param wheelbase Robot wheelbase
   * @param trackWidth Robot track width
   * @param autoLockTime Time before automatically rotating module to locked position (10 seconds max)
   * @param driveCurrentLimit Desired stator current limit for the drive motor
   */
  public REVSwerveModule(Hardware swerveHardware,
                         SwerveModule.Location location,
                         SwerveModule.MountOrientation motorOrientation,
                         SwerveModule.MountOrientation encoderOrientation,
                         SwerveModule.GearRatio gearRatio,
                         DriveWheel driveWheel, Angle zeroOffset,
                         PIDConstants drivePID, FFConstants driveFF,
                         PIDConstants rotatePID, FFConstants rotateFF,
                         Dimensionless slipRatio, Mass mass,
                         Distance wheelbase, Distance trackWidth,
                         Time autoLockTime, Current driveCurrentLimit) {
    super(location, gearRatio, driveWheel, zeroOffset, wheelbase, trackWidth, swerveHardware.driveMotor.getID().name);
    int encoderTicksPerRotation = swerveHardware.driveMotor.getKind().equals(MotorKind.NEO_VORTEX)
      ? GlobalConstants.VORTEX_ENCODER_TICKS_PER_ROTATION
      : GlobalConstants.NEO_ENCODER_TICKS_PER_ROTATION;
    DRIVE_MOTOR_CURRENT_LIMIT = driveCurrentLimit;
    DRIVE_TICKS_PER_METER =
      (encoderTicksPerRotation * gearRatio.getDriveRatio())
      * (1 / (driveWheel.diameter.in(Units.Meters) * Math.PI));
    DRIVE_METERS_PER_TICK = 1 / DRIVE_TICKS_PER_METER;
    DRIVE_METERS_PER_ROTATION = DRIVE_METERS_PER_TICK * encoderTicksPerRotation;
    DRIVE_MAX_LINEAR_SPEED = (swerveHardware.driveMotor.getKind().getMaxRPM() / 60) * DRIVE_METERS_PER_ROTATION * DRIVETRAIN_EFFICIENCY;

    // Set traction control controller
    super.setTractionControlController(new TractionControlController(driveWheel, slipRatio, mass, Units.MetersPerSecond.of(DRIVE_MAX_LINEAR_SPEED)));

    this.m_driveMotor = swerveHardware.driveMotor;
    this.m_rotateMotor = swerveHardware.rotateMotor;
    this.m_moduleSim = new SwerveModuleSim(
      m_driveMotor.getKind().motor,
      driveFF.withKA((driveFF.kA <= 0.0) ? SwerveModule.MIN_SIM_kA : driveFF.kA),
      m_rotateMotor.getKind().motor, rotateFF.withKA((rotateFF.kA <= 0.0) ? SwerveModule.MIN_SIM_kA : rotateFF.kA)
    );
    this.m_driveFF = new SimpleMotorFeedforward(driveFF.kS, driveFF.kV, driveFF.kA);
    this.m_location = location;
    this.m_zeroOffset = Rotation2d.fromRadians(zeroOffset.in(Units.Radians));
    this.m_simDrivePosition = Units.Meters.zero().mutableCopy();
    this.m_simModulePosition = new SwerveModulePosition();
    this.m_desiredState = new SwerveModuleState(Units.MetersPerSecond.of(0.0), m_zeroOffset.plus(m_location.getLockPosition()));
    this.m_autoLockTime = MathUtil.clamp(autoLockTime.in(Units.Milliseconds), 0.0, MAX_AUTO_LOCK_TIME * 1000);
    this.m_previousRotatePosition = m_zeroOffset.plus(m_location.getLockPosition());
    this.m_autoLockTimer = Instant.now();

    Logger.recordOutput(m_driveMotor.getID().name + MAX_LINEAR_VELOCITY_LOG_ENTRY, DRIVE_MAX_LINEAR_SPEED);

    m_driveMotorConfig = (m_driveMotor.getKind().equals(MotorKind.NEO_VORTEX)) ? new SparkFlexConfig() : new SparkMaxConfig();
    m_rotateMotorConfig = (m_rotateMotor.getKind().equals(MotorKind.NEO_VORTEX)) ? new SparkFlexConfig() : new SparkMaxConfig();

    // Set drive encoder config
    m_driveConversionFactor = driveWheel.diameter.in(Units.Meters) * Math.PI / super.getGearRatio().getDriveRatio();
    m_driveMotorConfig.encoder.positionConversionFactor(m_driveConversionFactor);
    m_driveMotorConfig.encoder.velocityConversionFactor(m_driveConversionFactor / 60);

    // Set rotate encoder config
    m_rotateConversionFactor = 2 * Math.PI;
    m_rotateMotorConfig.absoluteEncoder.positionConversionFactor(m_rotateConversionFactor);
    m_rotateMotorConfig.absoluteEncoder.velocityConversionFactor(m_rotateConversionFactor / 60);
    m_rotateMotorConfig.absoluteEncoder.inverted(!encoderOrientation.equals(motorOrientation));

    // Invert drive motor if necessary
    m_driveMotorConfig.inverted(motorOrientation.equals(MountOrientation.INVERTED));

    // Set sensor to use for closed loop control
    m_driveMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    m_rotateMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    // Set gains for drive PID
    m_driveMotorConfig.closedLoop.pidf(
      drivePID.kP,
      drivePID.kI,
      drivePID.kD,
      drivePID.kF
    );

    // Set gains for rotate PID and enable wrapping
    m_rotateMotorConfig.closedLoop.pid(rotatePID.kP, rotatePID.kI, rotatePID.kD);
    m_rotateMotorConfig.closedLoop.positionWrappingEnabled(true);
    m_rotateMotorConfig.closedLoop.positionWrappingInputRange(0.0, m_rotateConversionFactor);

    // Set drive motor to coast
    m_driveMotorConfig.idleMode(IdleMode.kCoast);

    // Set rotate motor to brake
    m_rotateMotorConfig.idleMode(IdleMode.kBrake);

    // Set current limits
    m_driveMotorConfig.smartCurrentLimit((int)DRIVE_MOTOR_CURRENT_LIMIT.in(Units.Amps));
    m_rotateMotorConfig.smartCurrentLimit((int)ROTATE_MOTOR_CURRENT_LIMIT.in(Units.Amps));

    // Set status frame rates
    m_driveMotorConfig.signals.primaryEncoderPositionPeriodMs((int)DEFAULT_SIGNAL_PERIOD.in(Units.Milliseconds));
    m_driveMotorConfig.signals.primaryEncoderVelocityPeriodMs((int)DEFAULT_SIGNAL_PERIOD.in(Units.Milliseconds));
    m_driveMotorConfig.signals.absoluteEncoderPositionPeriodMs((int)SLOW_SIGNAL_PERIOD.in(Units.Milliseconds));
    m_driveMotorConfig.signals.absoluteEncoderVelocityPeriodMs((int)SLOW_SIGNAL_PERIOD.in(Units.Milliseconds));
    m_driveMotorConfig.signals.analogPositionPeriodMs((int)SLOW_SIGNAL_PERIOD.in(Units.Milliseconds));
    m_driveMotorConfig.signals.analogVelocityPeriodMs((int)SLOW_SIGNAL_PERIOD.in(Units.Milliseconds));
    m_driveMotorConfig.signals.limitsPeriodMs((int)SLOW_SIGNAL_PERIOD.in(Units.Milliseconds));
    m_rotateMotorConfig.signals.primaryEncoderPositionPeriodMs((int)SLOW_SIGNAL_PERIOD.in(Units.Milliseconds));
    m_rotateMotorConfig.signals.primaryEncoderVelocityPeriodMs((int)SLOW_SIGNAL_PERIOD.in(Units.Milliseconds));
    m_rotateMotorConfig.signals.absoluteEncoderPositionPeriodMs((int)DEFAULT_SIGNAL_PERIOD.in(Units.Milliseconds));
    m_rotateMotorConfig.signals.absoluteEncoderVelocityPeriodMs((int)DEFAULT_SIGNAL_PERIOD.in(Units.Milliseconds));
    m_rotateMotorConfig.signals.analogPositionPeriodMs((int)SLOW_SIGNAL_PERIOD.in(Units.Milliseconds));
    m_rotateMotorConfig.signals.analogVelocityPeriodMs((int)SLOW_SIGNAL_PERIOD.in(Units.Milliseconds));
    m_rotateMotorConfig.signals.limitsPeriodMs((int)SLOW_SIGNAL_PERIOD.in(Units.Milliseconds));

    // Reset encoder
    resetDriveEncoder();

    // Configure motors with desired config
    m_driveMotor.configure(m_driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rotateMotor.configure(m_rotateMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Add callbacks to PurpleManager
    PurpleManager.addCallback(() -> periodic());
    PurpleManager.addCallbackSim(() -> simulationPeriodic());
  }

  /**
   * Initialize hardware devices for MAXSwerve module
   * @param driveMotorID Drive motor ID
   * @param rotateMotorID Rotate motor ID
   * @param driveMotorKind Kind of drive motor
   * @return Hardware object containing all necessary objects for a REV swerve module
   * @throws IllegalArgumentException If specified drive motor is not supported
   */
  public static Hardware initializeHardware(Spark.ID driveMotorID,
                                            Spark.ID rotateMotorID,
                                            MotorKind driveMotorKind,
                                            MotorKind rotateMotorKind) {
    if (driveMotorKind != MotorKind.NEO && driveMotorKind != MotorKind.NEO_VORTEX)
      throw new IllegalArgumentException("Drive motor MUST be a NEO or a NEO Vortex!");
    if (rotateMotorKind != MotorKind.NEO && rotateMotorKind != MotorKind.NEO_VORTEX && rotateMotorKind != MotorKind.NEO_550)
      throw new IllegalArgumentException("Rotate motor MUST be a NEO 550, NEO, or NEO Vortex!");
    var frequency = RobotBase.isReal() ? DEFAULT_SIGNAL_PERIOD.asFrequency() : GlobalConstants.ROBOT_LOOP_HZ;
    Hardware swerveModuleHardware = new Hardware(
      new Spark(driveMotorID, driveMotorKind, frequency),
      new Spark(rotateMotorID, rotateMotorKind, frequency)
    );

    return swerveModuleHardware;
  }

  /**
   * Update position in simulation
   */
  void updateSimPosition() {
    m_simDrivePosition.mut_plus(m_desiredState.speedMetersPerSecond * GlobalConstants.ROBOT_LOOP_HZ.asPeriod().in(Units.Seconds), Units.Meters);
    synchronized (m_driveMotor.getInputs()) {
      m_driveMotor.getInputs().encoderPosition = m_simDrivePosition.in(Units.Meters);
      m_driveMotor.getInputs().encoderVelocity = m_desiredState.speedMetersPerSecond;
      synchronized (m_rotateMotor.getInputs()) {
        m_rotateMotor.getInputs().absoluteEncoderPosition = m_desiredState.angle.getRadians();
        m_simModulePosition = new SwerveModulePosition(m_simDrivePosition, m_desiredState.angle.minus(m_zeroOffset));
      }
    }
  }

  /**
   * Call this method periodically
   */
  private void periodic() {
    super.logOutputs();
    Logger.recordOutput(m_rotateMotor.getID().name + ROTATE_ERROR_LOG_ENTRY, m_desiredState.angle.minus(Rotation2d.fromRadians(m_rotateMotor.getInputs().absoluteEncoderPosition)));
  }

 /**
   * Call this method periodically in simulation
   */
  private void simulationPeriodic() {
    var vbus = Units.Volts.of(RobotController.getBatteryVoltage());
    m_driveMotor.getSim().enable();
    m_rotateMotor.getSim().enable();

    m_driveMotor.getSim().iterate(m_moduleSim.getDriveMotorVelocity().in(Units.RPM), vbus.in(Units.Volts), GlobalConstants.ROBOT_LOOP_HZ.asPeriod().in(Units.Seconds));
    m_rotateMotor.getSim().iterate(m_moduleSim.getRotateMotorVelocity().in(Units.RPM), vbus.in(Units.Volts), GlobalConstants.ROBOT_LOOP_HZ.asPeriod().in(Units.Seconds));

    m_moduleSim.update(
      vbus.times(m_driveMotor.getSim().getAppliedOutput()),
      vbus.times(m_rotateMotor.getSim().getAppliedOutput())
    );

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_moduleSim.getTotalCurrentDraw().in(Units.Amps)));

    m_driveMotor.getSim().setMotorCurrent(m_moduleSim.getDriveMotorCurrentDraw().in(Units.Amps));
    m_rotateMotor.getSim().setMotorCurrent(m_moduleSim.getRotateMotorCurrentDraw().in(Units.Amps));

    updateSimPosition();
    Logger.recordOutput(m_driveMotor.getID().name + "/DriveSimPosition", m_simModulePosition);
  }

  /**
   * Allow for adding swerve module as a sendable object for dashboard interactivity
   * @param builder
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSafeState(this::lock);
    builder.setActuator(true);
    // Control drive velocity
    builder.addDoubleProperty(
      "Velocity",
      () -> m_desiredState.speedMetersPerSecond,
      (value) -> set(new SwerveModuleState(Units.MetersPerSecond.of(value), m_desiredState.angle))
    );
    // Control rotation
    builder.addDoubleProperty(
      "Orientation",
      () -> m_desiredState.angle.getRadians(),
      (value) -> set(new SwerveModuleState(m_desiredState.speedMetersPerSecond, Rotation2d.fromRadians(value)))
    );
    // Configure drive kP
    builder.addDoubleProperty(
      "Drive kP",
      () -> m_driveMotor.getConfigAccessor().closedLoop.getP(),
      (value) -> {
        m_driveMotorConfig.closedLoop.p(value);
        m_driveMotor.configure(m_driveMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      }
    );
    // Configure drive kI
    builder.addDoubleProperty(
      "Drive kI",
      () -> m_driveMotor.getConfigAccessor().closedLoop.getI(),
      (value) -> {
        m_driveMotorConfig.closedLoop.i(value);
        m_driveMotor.configure(m_driveMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      }
    );
    // Configure drive kD
    builder.addDoubleProperty(
      "Drive kD",
      () -> m_driveMotor.getConfigAccessor().closedLoop.getD(),
      (value) -> {
        m_driveMotorConfig.closedLoop.d(value);
        m_driveMotor.configure(m_driveMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      }
    );
    // Configure drive kS
    builder.addDoubleProperty(
      "Drive kS",
      () -> m_driveFF.getKs(),
      (value) -> {
        m_driveFF = new SimpleMotorFeedforward(value, m_driveFF.getKv(), m_driveFF.getKa());
      }
    );
    // Configure drive kV
    builder.addDoubleProperty(
      "Drive kV",
      () -> m_driveFF.getKv(),
      (value) -> {
        m_driveFF = new SimpleMotorFeedforward(m_driveFF.getKs(), value, m_driveFF.getKa());
      }
    );
    // Configure drive kA
    builder.addDoubleProperty(
      "Drive kA",
      () -> m_driveFF.getKa(),
      (value) -> {
        m_driveFF = new SimpleMotorFeedforward(m_driveFF.getKs(), m_driveFF.getKv(), value);
      }
    );
    // Configure rotate kP
    builder.addDoubleProperty(
      "Rotate kP",
      () -> m_rotateMotor.getConfigAccessor().closedLoop.getP(),
      (value) -> {
        m_rotateMotorConfig.closedLoop.p(value);
        m_rotateMotor.configure(m_rotateMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      }
    );
    // Configure rotate kI
    builder.addDoubleProperty(
      "Rotate kI",
      () -> m_rotateMotor.getConfigAccessor().closedLoop.getI(),
      (value) -> {
        m_rotateMotorConfig.closedLoop.i(value);
        m_rotateMotor.configure(m_rotateMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      }
    );
    // Configure rotate kD
    builder.addDoubleProperty(
      "Rotate kD",
      () -> m_rotateMotor.getConfigAccessor().closedLoop.getD(),
      (value) -> {
        m_rotateMotorConfig.closedLoop.d(value);
        m_rotateMotor.configure(m_rotateMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      }
    );
  }

  @Override
  public Frequency getUpdateRate() {
    return UPDATE_RATE;
  }

  @Override
  public void enableBrakeMode(boolean enable) {
    m_driveMotor.setIdleMode((enable) ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setDriveSysID(Voltage volts) {
    m_rotateMotor.set(m_zeroOffset.getRadians(), ControlType.kPosition);
    m_driveMotor.set(volts.in(Units.Volts), ControlType.kVoltage);
  }

  @Override
  public void setRotateSysID(Voltage volts) {
    m_rotateMotor.set(volts.in(Units.Volts), ControlType.kVoltage);
  }

  @Override
  public void set(SwerveModuleState state) {
    // Auto lock modules if auto lock enabled, speed not requested, and time has elapsed
    if (super.isAutoLockEnabled() && state.speedMetersPerSecond < EPSILON) {
      state.speedMetersPerSecond = 0.0;
      // Time's up, lock now...
      if (Duration.between(m_autoLockTimer, Instant.now()).toMillis() > m_autoLockTime)
        state.angle = m_location.getLockPosition();
      // Waiting to lock...
      else state.angle = m_previousRotatePosition.minus(m_zeroOffset);
    } else {
      // Not locking this loop, restart timer...
      m_autoLockTimer = Instant.now();
    }

    // Save previous state
    var oldState = m_desiredState;

    // Get desired state
    m_desiredState = getDesiredState(state, Rotation2d.fromRadians(m_rotateMotor.getInputs().absoluteEncoderPosition));

    // Set rotate motor position
    m_rotateMotor.set(m_desiredState.angle.getRadians(), ControlType.kPosition);

    // Calculate drive FF
    var driveFF = m_driveFF.calculateWithVelocities(
      oldState.speedMetersPerSecond,
      m_desiredState.speedMetersPerSecond
    );
    driveFF *= REAL_ROBOT;

    // Set drive motor speed
    m_driveMotor.set(
      m_desiredState.speedMetersPerSecond, ControlType.kVelocity,
      driveFF, ArbFFUnits.kVoltage
    );

    // Save rotate position
    m_previousRotatePosition = m_desiredState.angle;

    // Increment odometer
    super.incrementOdometer(Math.abs(m_desiredState.speedMetersPerSecond) * GlobalConstants.ROBOT_LOOP_HZ.asPeriod().in(Units.Seconds));
  }

  @Override
  public LinearVelocity getDriveVelocity() {
    return Units.MetersPerSecond.of(m_driveMotor.getInputs().encoderVelocity);
  }

  @Override
  public SwerveModuleState getState() {
    return new SwerveModuleState(
      getDriveVelocity(),
      Rotation2d.fromRadians(m_rotateMotor.getInputs().absoluteEncoderPosition).minus(m_zeroOffset)
    );
  }

  @Override
  public SwerveModulePosition getPosition() {
    if (RobotBase.isSimulation()) return m_simModulePosition;
    return new SwerveModulePosition(
      m_driveMotor.getInputs().encoderPosition,
      Rotation2d.fromRadians(m_rotateMotor.getInputs().absoluteEncoderPosition).minus(m_zeroOffset)
    );
  }

  @Override
  public void resetDriveEncoder() {
    m_driveMotor.resetEncoder();
    m_simDrivePosition = Units.Meters.zero().mutableCopy();
    m_simModulePosition = new SwerveModulePosition(m_simDrivePosition, m_previousRotatePosition);
  }

  @Override
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