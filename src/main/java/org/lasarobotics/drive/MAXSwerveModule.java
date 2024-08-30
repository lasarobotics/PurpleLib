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
import org.lasarobotics.hardware.revrobotics.Spark.FeedbackSensor;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;
import org.lasarobotics.hardware.revrobotics.SparkPIDConfig;
import org.lasarobotics.hardware.revrobotics.SparkSim;
import org.lasarobotics.utils.FFConstants;
import org.lasarobotics.utils.GlobalConstants;
import org.lasarobotics.utils.PIDConstants;
import org.lasarobotics.utils.SimDynamics;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

/** REV MAXSwerve module */
public class MAXSwerveModule implements Sendable, AutoCloseable {
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

  public static final Measure<Time> DEFAULT_PERIOD = Units.Milliseconds.of(10.0);

  private final double EPSILON = 5e-3;
  private final double DRIVE_FF_SCALAR = 1.5;
  private final Measure<Current> DRIVE_MOTOR_CURRENT_LIMIT;
  private final Measure<Current> ROTATE_MOTOR_CURRENT_LIMIT = Units.Amps.of(18.0);
  private final Rotation2d LOCK_POSITION = Rotation2d.fromRadians(Math.PI / 4);

  private static final String IS_SLIPPING_LOG_ENTRY = "/IsSlipping";
  private static final String ODOMETER_LOG_ENTRY = "/Odometer";
  private static final double DRIVETRAIN_EFFICIENCY = 0.90;
  private static final double MAX_AUTO_LOCK_TIME = 10.0;
  private final double DRIVE_TICKS_PER_METER;
  private final double DRIVE_METERS_PER_TICK;
  private final double DRIVE_METERS_PER_ROTATION;
  private final double DRIVE_MAX_LINEAR_SPEED;
  private final int COSINE_CORRECTION;

  // Swerve velocity PID settings
  private static final double DRIVE_VELOCITY_kP = 0.18;
  private static final double DRIVE_VELOCITY_kD = 0.001;
  private static final double DRIVE_VELOCITY_kS = 0.2;
  private static final double DRIVE_VELOCITY_kA = 0.5;
  private static final double DRIVE_VELOCITY_TOLERANCE = 0.01;
  private static final boolean DRIVE_VELOCITY_SENSOR_PHASE = false;
  private static final boolean DRIVE_INVERT_MOTOR = false;

  // Swerve rotate PID settings
  private static final PIDConstants DRIVE_ROTATE_PID = new PIDConstants(2.1, 0.0, 0.2, 0.0, 0.0);
  private static final double DRIVE_ROTATE_kS = 0.2;
  private static final double DRIVE_ROTATE_kA = 0.01;
  private static final double DRIVE_ROTATE_TOLERANCE = 0.01;
  private static final double DRIVE_ROTATE_LOWER_LIMIT = 0.0;
  private static final double DRIVE_ROTATE_UPPER_LIMIT = 0.0;
  private static final boolean DRIVE_ROTATE_SOFT_LIMITS = false;
  private static final boolean DRIVE_ROTATE_SENSOR_PHASE = true;
  private static final boolean DRIVE_ROTATE_INVERT_MOTOR = false;
  private static final double DRIVE_ROTATE_GEAR_RATIO = 9424.0 / 203.0;

  private Spark m_driveMotor;
  private Spark m_rotateMotor;
  private SparkSim m_driveMotorSim;
  private SparkSim m_rotateMotorSim;
  private SwerveModuleSim m_moduleSim;
  private SparkPIDConfig m_driveMotorConfig;
  private SparkPIDConfig m_rotateMotorConfig;
  private Translation2d m_moduleCoordinate;
  private ModuleLocation m_location;
  private Rotation2d m_previousRotatePosition;

  private volatile double m_simDrivePosition;
  private volatile SwerveModuleState m_desiredState;

  private GearRatio m_driveGearRatio;
  private double m_driveConversionFactor;
  private double m_rotateConversionFactor;
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
   * @param driveWheel Wheel installed in swerve module
   * @param slipRatio Desired slip ratio [1%, 40%]
   * @param mass Robot mass
   * @param wheelbase Robot wheelbase
   * @param trackWidth Robot track width
   * @param autoLockTime Time before automatically rotating module to locked position (10 seconds max)
   * @param driveMotorCurrentLimit Desired current limit for the drive motor
   */
  public MAXSwerveModule(Hardware swerveHardware, ModuleLocation location, GearRatio driveGearRatio, DriveWheel driveWheel,
                         Measure<Dimensionless> slipRatio, Measure<Mass> mass,
                         Measure<Distance> wheelbase, Measure<Distance> trackWidth,
                         Measure<Time> autoLockTime, Measure<Current> driveMotorCurrentLimit) {
    int encoderTicksPerRotation = swerveHardware.driveMotor.getKind().equals(MotorKind.NEO_VORTEX)
      ? GlobalConstants.VORTEX_ENCODER_TICKS_PER_ROTATION
      : GlobalConstants.NEO_ENCODER_TICKS_PER_ROTATION;
    DRIVE_MOTOR_CURRENT_LIMIT = driveMotorCurrentLimit;
    DRIVE_TICKS_PER_METER =
      (encoderTicksPerRotation * driveGearRatio.value)
      * (1 / (driveWheel.diameter.in(Units.Meters) * Math.PI));
    DRIVE_METERS_PER_TICK = 1 / DRIVE_TICKS_PER_METER;
    DRIVE_METERS_PER_ROTATION = DRIVE_METERS_PER_TICK * encoderTicksPerRotation;
    DRIVE_MAX_LINEAR_SPEED = (swerveHardware.driveMotor.getKind().getMaxRPM() / 60) * DRIVE_METERS_PER_ROTATION * DRIVETRAIN_EFFICIENCY;
    COSINE_CORRECTION = RobotBase.isReal() ? 1 : 0;

    this.m_driveMotor = swerveHardware.driveMotor;
    this.m_rotateMotor = swerveHardware.rotateMotor;
    this.m_location = location;
    this.m_driveGearRatio = driveGearRatio;
    this.m_autoLock = true;
    this.m_simDrivePosition = 0.0;
    this.m_desiredState = new SwerveModuleState(Units.MetersPerSecond.of(0.0), LOCK_POSITION);
    this.m_autoLockTime = MathUtil.clamp(autoLockTime.in(Units.Milliseconds), 0.0, MAX_AUTO_LOCK_TIME * 1000);
    this.m_previousRotatePosition = LOCK_POSITION;
    this.m_tractionControlController =  new TractionControlController(driveWheel, slipRatio, mass, Units.MetersPerSecond.of(DRIVE_MAX_LINEAR_SPEED));
    this.m_autoLockTimer = Instant.now();
    this.m_runningOdometer = 0.0;

    // Set drive encoder conversion factor
    m_driveConversionFactor = driveWheel.diameter.in(Units.Meters) * Math.PI / m_driveGearRatio.value;
    m_driveMotor.setPositionConversionFactor(Spark.FeedbackSensor.NEO_ENCODER, m_driveConversionFactor);
    m_driveMotor.setVelocityConversionFactor(Spark.FeedbackSensor.NEO_ENCODER, m_driveConversionFactor / 60);

    // Set rotate encoder conversion factor
    m_rotateConversionFactor = 2 * Math.PI;
    m_rotateMotor.setPositionConversionFactor(Spark.FeedbackSensor.THROUGH_BORE_ENCODER, m_rotateConversionFactor);
    m_rotateMotor.setVelocityConversionFactor(Spark.FeedbackSensor.THROUGH_BORE_ENCODER, m_rotateConversionFactor / 60);

    // Enable PID wrapping
    m_rotateMotor.enablePIDWrapping(0.0, m_rotateConversionFactor);

    // Create PID configs
    m_driveMotorConfig = new SparkPIDConfig(
      new PIDConstants(
        DRIVE_VELOCITY_kP,
        0.0,
        DRIVE_VELOCITY_kD,
        (1 / ((m_driveMotor.getKind().getMaxRPM() / 60) * m_driveConversionFactor)) * DRIVE_FF_SCALAR,
        0.0
      ),
      DRIVE_VELOCITY_SENSOR_PHASE,
      DRIVE_INVERT_MOTOR,
      DRIVE_VELOCITY_TOLERANCE
    );
    m_rotateMotorConfig = new SparkPIDConfig(
      DRIVE_ROTATE_PID,
      DRIVE_ROTATE_SENSOR_PHASE,
      DRIVE_ROTATE_INVERT_MOTOR,
      DRIVE_ROTATE_TOLERANCE,
      DRIVE_ROTATE_LOWER_LIMIT,
      DRIVE_ROTATE_UPPER_LIMIT,
      DRIVE_ROTATE_SOFT_LIMITS
    );

    // Initialize PID
    m_driveMotor.initializeSparkPID(m_driveMotorConfig, Spark.FeedbackSensor.NEO_ENCODER);
    m_rotateMotor.initializeSparkPID(m_rotateMotorConfig, Spark.FeedbackSensor.THROUGH_BORE_ENCODER);

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
    PurpleManager.addCallbackSim(() -> simulationPeriodic());

    // Setup disabled triggers
    RobotModeTriggers.disabled().onTrue(Commands.runOnce(() -> disabledInit()).ignoringDisable(true));
    RobotModeTriggers.disabled().onFalse(Commands.runOnce(() -> disabledExit()).ignoringDisable(true));

    // Make sure settings are burned to flash
    m_driveMotor.burnFlash();
    m_rotateMotor.burnFlash();

    // Setup sim
    double rotate_kV = 1 / ((m_rotateMotor.getKind().getMaxRPM() / 60) * (m_rotateConversionFactor / DRIVE_ROTATE_GEAR_RATIO)) * 10;
    m_moduleSim = new SwerveModuleSim(
      m_driveMotor.getKind().motor.withReduction(m_driveGearRatio.value),
      new FFConstants(DRIVE_VELOCITY_kS, 0.0, m_driveMotorConfig.getF() * 10, DRIVE_VELOCITY_kA),
      m_rotateMotor.getKind().motor.withReduction(DRIVE_ROTATE_GEAR_RATIO),
      new FFConstants(DRIVE_ROTATE_kS, 0.0, rotate_kV * 500, DRIVE_ROTATE_kA * 500)
    );
    m_driveMotorSim = new SparkSim(m_driveMotor, SimDynamics.fromSim(m_moduleSim.getDriveSim()));
    m_rotateMotorSim = new SparkSim(m_rotateMotor, SimDynamics.fromSim(m_moduleSim.getRotateSim()));

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
    var period = RobotBase.isReal() ? DEFAULT_PERIOD : Units.Seconds.of(GlobalConstants.ROBOT_LOOP_PERIOD);
    Hardware swerveModuleHardware = new Hardware(
      new Spark(driveMotorID, driveMotorKind, period),
      new Spark(rotateMotorID, MotorKind.NEO_550, period)
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
    desiredState.speedMetersPerSecond *= Math.pow(desiredState.angle.minus(currentAngle).getCos(), COSINE_CORRECTION);

    // Return corrected desired swerve module state
    return desiredState;
  }

  /**
   * Call this method periodically
   */
  private void periodic() {
    Logger.recordOutput(m_driveMotor.getID().name + IS_SLIPPING_LOG_ENTRY, isSlipping());
    Logger.recordOutput(m_driveMotor.getID().name + ODOMETER_LOG_ENTRY, m_runningOdometer);
  }

 /**
   * Call this method periodically in simulation
   */
  private void simulationPeriodic() {
    var vbus = Units.Volts.of(RobotController.getBatteryVoltage());
    m_driveMotorSim.enable();
    m_rotateMotorSim.enable();

    m_driveMotorSim.update(Units.Value.of(m_moduleSim.getDriveMotorVelocity().in(Units.RPM)), vbus);
    m_rotateMotorSim.update(Units.Value.of(m_moduleSim.getRotateMotorVelocity().in(Units.RPM)), vbus);

    m_moduleSim.update(
      m_driveMotorSim.getAppliedOutput() * vbus.in(Units.Volts),
      m_rotateMotorSim.getAppliedOutput() * vbus.in(Units.Volts)
    );

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_moduleSim.getTotalCurrentDraw().in(Units.Amps)));

    m_driveMotorSim.setMotorCurrent(m_moduleSim.getDriveMotorCurrentDraw());
    m_rotateMotorSim.setMotorCurrent(m_moduleSim.getRotateMotorCurrentDraw());
  }

  /**
   * Allow for adding MAXSwerveModule as a sendable object for dashboard interactivity
   * @param builder
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSafeState(this::lock);
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
      () -> m_driveMotorConfig.getP(),
      (value) -> {
        m_driveMotorConfig.setP(value);
        m_driveMotor.initializeSparkPID(m_driveMotorConfig, FeedbackSensor.NEO_ENCODER);
      }
    );
    // Configure drive kI
    builder.addDoubleProperty(
      "Drive kI",
      () -> m_driveMotorConfig.getI(),
      (value) -> {
        m_driveMotorConfig.setI(value);
        m_driveMotor.initializeSparkPID(m_driveMotorConfig, FeedbackSensor.NEO_ENCODER);
      }
    );
    // Configure drive kD
    builder.addDoubleProperty(
      "Drive kD",
      () -> m_driveMotorConfig.getD(),
      (value) -> {
        m_driveMotorConfig.setD(value);
        m_driveMotor.initializeSparkPID(m_driveMotorConfig, FeedbackSensor.NEO_ENCODER);
      }
    );
    // Configure drive kF
    builder.addDoubleProperty(
      "Drive kF",
      () -> m_driveMotorConfig.getF(),
      (value) -> {
        m_driveMotorConfig.setF(value);
        m_driveMotor.initializeSparkPID(m_driveMotorConfig, FeedbackSensor.NEO_ENCODER);
      }
    );
    // Configure rotate kP
    builder.addDoubleProperty(
      "Rotate kP",
      () -> m_rotateMotorConfig.getP(),
      (value) -> {
        m_rotateMotorConfig.setP(value);
        m_rotateMotor.initializeSparkPID(m_rotateMotorConfig, FeedbackSensor.THROUGH_BORE_ENCODER);
      }
    );
    // Configure rotate kI
    builder.addDoubleProperty(
      "Rotate kI",
      () -> m_rotateMotorConfig.getI(),
      (value) -> {
        m_rotateMotorConfig.setI(value);
        m_rotateMotor.initializeSparkPID(m_rotateMotorConfig, FeedbackSensor.THROUGH_BORE_ENCODER);
      }
    );
    // Configure rotate kD
    builder.addDoubleProperty(
      "Rotate kD",
      () -> m_rotateMotorConfig.getD(),
      (value) -> {
        m_rotateMotorConfig.setD(value);
        m_rotateMotor.initializeSparkPID(m_rotateMotorConfig, FeedbackSensor.THROUGH_BORE_ENCODER);
      }
    );
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
   * @param state Desired swerve module state representing desired speed
   * @param realSpeeds Real speeds of robot from IMU
   */
  public void set(SwerveModuleState state, ChassisSpeeds realSpeeds) {
    var inertialVelocity = Units.MetersPerSecond.of(AdvancedSwerveKinematics.getRealModuleSpeed(m_moduleCoordinate, realSpeeds).speedMetersPerSecond);
    // Apply traction control
    state.speedMetersPerSecond = m_tractionControlController.calculate(
      Units.MetersPerSecond.of(state.speedMetersPerSecond),
      inertialVelocity,
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
   * @param states Array of states for all swerve modules representing desired speed
   * @param realSpeeds Real speeds of robot from IMU
   */
  public void set(SwerveModuleState[] states, ChassisSpeeds realSpeeds) {
    set(states[m_location.index], realSpeeds);
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
   * Enables auto locking. If enabled, the wheels will rotate to a state that prevents translation when not driving.
   */
  public void enableAutoLock() {
    m_autoLock = true;
  }

  /**
   * Disables auto locking. If disabled, the wheels will not automatically rotate when not driving.
   */
  public void disableAutoLock() {
    m_autoLock = false;
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