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
import org.lasarobotics.hardware.ctre.CANcoder;
import org.lasarobotics.hardware.ctre.TalonFX;
import org.lasarobotics.utils.FFConstants;
import org.lasarobotics.utils.GlobalConstants;
import org.lasarobotics.utils.PIDConstants;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
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
public class CTRESwerveModule extends SwerveModule implements Sendable {
  /**
   * CTRE swerve module hardware
   */
  public static class Hardware {
    public final TalonFX driveMotor;
    public final TalonFX rotateMotor;
    public final CANcoder canCoder;

    public Hardware(TalonFX driveMotor, TalonFX rotateMotor, CANcoder canCoder) {
      this.driveMotor = driveMotor;
      this.rotateMotor = rotateMotor;
      this.canCoder = canCoder;
    }
  }

  private static final Frequency UPDATE_RATE = Units.Hertz.of(200.0);
  public static final Frequency DEFAULT_SIGNAL_HZ = Units.Hertz.of(200.0);
  public static final Frequency ALTERNATE_SIGNAL_PERIOD = Units.Hertz.of(0.1);
  public static final double DRIVETRAIN_EFFICIENCY = 0.90;

  private final double EPSILON = 5e-3;
  private final Current ROTATE_MOTOR_CURRENT_LIMIT = Units.Amps.of(20.0);
  private final Current DRIVE_MOTOR_PEAK_SUPPLY_CURRENT_LIMIT = Units.Amps.of(90.0);
  private final Current DRIVE_MOTOR_SUSTAINED_SUPPLY_CURRENT_LIMIT = Units.Amps.of(40.0);
  private final Time DRIVE_MOTOR_PEAK_CURRENT_TIME = Units.Seconds.one();

  private static final String ROTATE_ERROR_LOG_ENTRY = "/RotateError";
  private static final String MAX_LINEAR_VELOCITY_LOG_ENTRY = "/MaxLinearVelocity";
  private static final double MAX_AUTO_LOCK_TIME = 10.0;
  private final double DRIVE_TICKS_PER_METER;
  private final double DRIVE_METERS_PER_TICK;
  private final double DRIVE_METERS_PER_ROTATION;
  private final double DRIVE_MAX_LINEAR_SPEED;

  private TalonFX m_driveMotor;
  private TalonFX m_rotateMotor;
  private CANcoder m_canCoder;
  private SwerveModuleSim m_moduleSim;
  private TalonFXConfiguration m_driveMotorConfig;
  private TalonFXConfiguration m_rotateMotorConfig;
  private CANcoderConfiguration m_canCoderConfig;
  private Rotation2d m_zeroOffset;

  private SwerveModule.Location m_location;
  private Rotation2d m_previousRotatePosition;

  private volatile MutDistance m_simDrivePosition;
  private volatile SwerveModulePosition m_simModulePosition;
  private volatile SwerveModuleState m_desiredState;

  private SwerveModule.GearRatio m_gearRatio;
  private double m_autoLockTime;
  private final boolean m_isPhoenixPro;

  private Instant m_autoLockTimer;

  private PositionVoltage m_rotatePositionSetter;
  private VelocityVoltage m_driveVelocitySetter;
  private VoltageOut m_driveVoltageSetter;
  private VoltageOut m_rotateVoltageSetter;

  /**
   * Create an instance of a CTRE swerve module
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
  public CTRESwerveModule(Hardware swerveHardware,
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
    this.m_isPhoenixPro = swerveHardware.driveMotor.isProLicensed();
    double maxRPM = (m_isPhoenixPro)
      ? Units.RadiansPerSecond.of(DCMotor.getKrakenX60Foc(1).freeSpeedRadPerSec).in(Units.RPM)
      : Units.RadiansPerSecond.of(DCMotor.getKrakenX60(1).freeSpeedRadPerSec).in(Units.RPM);
    int encoderTicksPerRotation = GlobalConstants.KRAKEN_X60_ENCODER_TICKS_PER_ROTATION;
    DRIVE_TICKS_PER_METER =
      (encoderTicksPerRotation * gearRatio.getDriveRatio())
      * (1 / (driveWheel.diameter.in(Units.Meters) * Math.PI));
    DRIVE_METERS_PER_TICK = 1 / DRIVE_TICKS_PER_METER;
    DRIVE_METERS_PER_ROTATION = DRIVE_METERS_PER_TICK * encoderTicksPerRotation;
    DRIVE_MAX_LINEAR_SPEED = (maxRPM / 60) * DRIVE_METERS_PER_ROTATION * DRIVETRAIN_EFFICIENCY;

    // Set traction control controller
    super.setTractionControlController(new TractionControlController(driveWheel, slipRatio, mass, Units.MetersPerSecond.of(DRIVE_MAX_LINEAR_SPEED)));

    this.m_driveMotor = swerveHardware.driveMotor;
    this.m_rotateMotor = swerveHardware.rotateMotor;
    this.m_canCoder = swerveHardware.canCoder;
    this.m_moduleSim = new SwerveModuleSim(
      m_isPhoenixPro ? DCMotor.getKrakenX60Foc(1) : DCMotor.getKrakenX60(1),
      driveFF.withKA((driveFF.kA <= 0.0) ? SwerveModule.MIN_SIM_kA : driveFF.kA),
      m_isPhoenixPro ? DCMotor.getKrakenX60Foc(1) : DCMotor.getKrakenX60(1),
      rotateFF.withKA((rotateFF.kA <= 0.0) ? SwerveModule.MIN_SIM_kA : rotateFF.kA)
    );
    this.m_location = location;
    this.m_gearRatio = gearRatio;
    this.m_zeroOffset = Rotation2d.fromRadians(zeroOffset.in(Units.Radians));
    this.m_simDrivePosition = Units.Meters.zero().mutableCopy();
    this.m_simModulePosition = new SwerveModulePosition();
    this.m_desiredState = new SwerveModuleState(Units.MetersPerSecond.of(0.0), m_zeroOffset.plus(m_location.getLockPosition()));
    this.m_autoLockTime = MathUtil.clamp(autoLockTime.in(Units.Milliseconds), 0.0, MAX_AUTO_LOCK_TIME * 1000);
    this.m_previousRotatePosition = m_zeroOffset.plus(m_location.getLockPosition());
    this.m_autoLockTimer = Instant.now();
    this.m_rotatePositionSetter = new PositionVoltage(Units.Radians.zero());
    this.m_driveVelocitySetter = new VelocityVoltage(Units.RotationsPerSecond.zero());
    this.m_rotateVoltageSetter = new VoltageOut(Units.Volts.zero());
    this.m_driveVoltageSetter = new VoltageOut(Units.Volts.zero());

    Logger.recordOutput(m_driveMotor.getID().name + MAX_LINEAR_VELOCITY_LOG_ENTRY, DRIVE_MAX_LINEAR_SPEED);

    m_driveMotorConfig = new TalonFXConfiguration();
    m_rotateMotorConfig = new TalonFXConfiguration();
    m_canCoderConfig = new CANcoderConfiguration();

    // Set rotate encoder config
    m_canCoderConfig.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Units.Rotations.of(1));
    m_canCoderConfig.MagnetSensor.withSensorDirection(!encoderOrientation.equals(motorOrientation) ? SensorDirectionValue.Clockwise_Positive
                                                                        : SensorDirectionValue.CounterClockwise_Positive);
    m_canCoder.applyConfigs(m_canCoderConfig);


    // Set sensor to use for closed loop control
    m_driveMotorConfig.Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);
    m_rotateMotorConfig.Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                                .withRotorToSensorRatio(m_gearRatio.getRotateRatio())
                                .withFeedbackRemoteSensorID(m_canCoder.getID().deviceID);

    // Invert drive motor if necessary
    m_driveMotorConfig.MotorOutput.Inverted = (motorOrientation.equals(MountOrientation.INVERTED))
                                              ? InvertedValue.Clockwise_Positive
                                              : InvertedValue.CounterClockwise_Positive;

    // Set gains for drive PID
    m_driveMotorConfig.Slot0
      .withKP(drivePID.kP)
      .withKI(drivePID.kI)
      .withKD(drivePID.kD)
      .withKS(driveFF.kS)
      .withKV(driveFF.kV)
      .withKA(driveFF.kA);

    // Set gains for rotate PID and enable wrapping
    m_rotateMotorConfig.Slot0
      .withKP(rotatePID.kP)
      .withKI(rotatePID.kI)
      .withKD(rotatePID.kD);
    m_rotateMotorConfig.ClosedLoopGeneral.ContinuousWrap = true;

    // Set drive motor to coast
    m_driveMotorConfig.MotorOutput.withNeutralMode(NeutralModeValue.Coast);

    // Set rotate motor to brake
    m_rotateMotorConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

    // Set current limits
    m_driveMotorConfig.CurrentLimits.withSupplyCurrentLimitEnable(true);
    m_driveMotorConfig.CurrentLimits.withSupplyCurrentLimit(DRIVE_MOTOR_PEAK_SUPPLY_CURRENT_LIMIT);
    m_driveMotorConfig.CurrentLimits.withSupplyCurrentLowerLimit(DRIVE_MOTOR_SUSTAINED_SUPPLY_CURRENT_LIMIT);
    m_driveMotorConfig.CurrentLimits.withSupplyCurrentLowerTime(DRIVE_MOTOR_PEAK_CURRENT_TIME);
    m_driveMotorConfig.CurrentLimits.withStatorCurrentLimit(driveCurrentLimit);
    m_rotateMotorConfig.CurrentLimits.withStatorCurrentLimit(ROTATE_MOTOR_CURRENT_LIMIT);

    // Reset encoder
    resetDriveEncoder();

    // Configure motors with desired config
    m_driveMotor.applyConfigs(m_driveMotorConfig);
    m_rotateMotor.applyConfigs(m_rotateMotorConfig);

    // Add callbacks to PurpleManager
    PurpleManager.addCallback(() -> periodic());
    PurpleManager.addCallbackSim(() -> simulationPeriodic());
  }

  /**
   * Initialize hardware devices for MAXSwerve module
   * @param driveMotorID Drive motor ID
   * @param rotateMotorID Rotate motor ID
   * @param canCoderID CANCoder ID
   * @return Hardware object containing all necessary objects for a MAXSwerve module
   */
  public static Hardware initializeHardware(TalonFX.ID driveMotorID,
                                            TalonFX.ID rotateMotorID,
                                            CANcoder.ID canCoderID) {
    var updateRate = RobotBase.isReal() ? DEFAULT_SIGNAL_HZ : GlobalConstants.ROBOT_LOOP_FREQUENCY;
    Hardware swerveModuleHardware = new Hardware(
      new TalonFX(driveMotorID, updateRate),
      new TalonFX(rotateMotorID, updateRate),
      new CANcoder(canCoderID, updateRate)
    );

    return swerveModuleHardware;
  }

  /**
   * Update position in simulation
   */
  void updateSimPosition() {
    m_simDrivePosition.mut_plus(m_desiredState.speedMetersPerSecond * GlobalConstants.ROBOT_LOOP_FREQUENCY.asPeriod().in(Units.Seconds), Units.Meters);
    synchronized (m_driveMotor.getInputs()) {
      m_driveMotor.getInputs().selectedSensorPosition.mut_replace(distanceToAngle(m_simDrivePosition));
      m_driveMotor.getInputs().selectedSensorVelocity.mut_replace(linearToAngularVelocity(Units.MetersPerSecond.of(m_desiredState.speedMetersPerSecond)));
      synchronized (m_rotateMotor.getInputs()) {
        m_rotateMotor.getInputs().selectedSensorPosition.mut_replace(m_desiredState.angle.getMeasure());
        m_canCoder.getInputs().absolutePosition.mut_replace(m_desiredState.angle.getMeasure());
        m_simModulePosition = new SwerveModulePosition(m_simDrivePosition, m_desiredState.angle.minus(m_zeroOffset));
      }
    }
  }

  /**
   * Convert anglular position to linear distance
   * @param angle Angular position
   * @return Distance
   */
  private Distance angleToDistance(Angle angle) {
    // Apply gear ratio to input rotations
    var gearedRadians = angle.in(Units.Radians) / m_gearRatio.getDriveRatio();
    // Then multiply the wheel radius by radians of rotation to get distance
    return getDriveWheel().radius.times(gearedRadians);
  }

  /**
   * Convert linear distance to angular position
   * @param distance Distsance
   * @return Angular position
   */
  private Angle distanceToAngle(Distance distance) {
    // Divide the distance by the wheel radius to get radians
    var wheelRadians = distance.in(Units.Meters) / getDriveWheel().radius.in(Units.Meters);
    // Then multiply by gear ratio to get rotor rotations
    return Units.Radians.of(wheelRadians * m_gearRatio.getDriveRatio());
  }

  /**
   * Convert angular velocity to linear velocity
   * @param angularVelocity
   * @return Linear velocity
   */
  private LinearVelocity angularToLinearVelocity(AngularVelocity angularVelocity) {
    // Apply gear ratio to input rotations
    double gearedRotations = angularVelocity.in(Units.RadiansPerSecond) / m_gearRatio.getDriveRatio();
    // Then multiply the wheel radius by radians of rotation to get distance
    return getDriveWheel().radius.per(Units.Seconds).times(gearedRotations);
  }

  /**
   * Convert linear velocity to angular velocity
   * @param velocity Linear velocity
   * @return Angular velocity
   */
  private AngularVelocity linearToAngularVelocity(LinearVelocity velocity) {
    // Divide the distance by the wheel radius to get radians
    var wheelRadians = velocity.in(Units.MetersPerSecond) / getDriveWheel().radius.in(Units.Meters);
    // Then multiply by gear ratio to get rotor rotations
    return Units.RadiansPerSecond.of(wheelRadians * m_gearRatio.getDriveRatio());
  }

  /**
   * Call this method periodically
   */
  private void periodic() {
    super.logOutputs();
    Logger.recordOutput(m_rotateMotor.getID().name + ROTATE_ERROR_LOG_ENTRY, m_desiredState.angle.minus(Rotation2d.fromRadians(m_rotateMotor.getInputs().selectedSensorPosition.in(Units.Radians))));
  }

 /**
   * Call this method periodically in simulation
   */
  private void simulationPeriodic() {
    var vbus = Units.Volts.of(RobotController.getBatteryVoltage());

    m_driveMotor.getSimState().setSupplyVoltage(vbus);
    m_rotateMotor.getSimState().setSupplyVoltage(vbus);
    m_canCoder.getSimState().setSupplyVoltage(vbus);

    m_driveMotor.getSimState().setRotorVelocity(m_moduleSim.getDriveMotorVelocity());
    m_rotateMotor.getSimState().setRotorVelocity(m_moduleSim.getRotateMotorVelocity());

    m_moduleSim.update(
      m_driveMotor.getSimState().getMotorVoltageMeasure(),
      m_rotateMotor.getSimState().getMotorVoltageMeasure()
    );

    RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage(m_moduleSim.getTotalCurrentDraw().in(Units.Amps)));

    updateSimPosition();
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
      () -> m_driveMotorConfig.Slot0.kP,
      (value) -> {
        m_driveMotorConfig.Slot0.withKP(value);
        m_driveMotor.applyConfigs(m_driveMotorConfig);
      }
    );
    // Configure drive kI
    builder.addDoubleProperty(
      "Drive kI",
      () -> m_driveMotorConfig.Slot0.kI,
      (value) -> {
        m_driveMotorConfig.Slot0.withKI(value);
        m_driveMotor.applyConfigs(m_driveMotorConfig);
      }
    );
    // Configure drive kD
    builder.addDoubleProperty(
      "Drive kD",
      () -> m_driveMotorConfig.Slot0.kD,
      (value) -> {
        m_driveMotorConfig.Slot0.withKD(value);
        m_driveMotor.applyConfigs(m_driveMotorConfig);
      }
    );
    // Configure drive kS
    builder.addDoubleProperty(
      "Drive kS",
      () -> m_driveMotorConfig.Slot0.kS,
      (value) -> {
        m_driveMotorConfig.Slot0.withKS(value);
        m_driveMotor.applyConfigs(m_driveMotorConfig);
      }
    );
    // Configure drive kV
    builder.addDoubleProperty(
      "Drive kV",
      () -> m_driveMotorConfig.Slot0.kV,
      (value) -> {
        m_driveMotorConfig.Slot0.withKV(value);
        m_driveMotor.applyConfigs(m_driveMotorConfig);
      }
    );
    // Configure drive kA
    builder.addDoubleProperty(
      "Drive kA",
      () -> m_driveMotorConfig.Slot0.kA,
      (value) -> {
        m_driveMotorConfig.Slot0.withKA(value);
        m_driveMotor.applyConfigs(m_driveMotorConfig);
      }
    );
    // Configure rotate kP
    builder.addDoubleProperty(
      "Rotate kP",
      () -> m_rotateMotorConfig.Slot0.kP,
      (value) -> {
        m_rotateMotorConfig.Slot0.withKP(value);
        m_rotateMotor.applyConfigs(m_rotateMotorConfig);
      }
    );
    // Configure rotate kI
    builder.addDoubleProperty(
      "Rotate kI",
      () -> m_rotateMotorConfig.Slot0.kI,
      (value) -> {
        m_rotateMotorConfig.Slot0.withKI(value);
        m_rotateMotor.applyConfigs(m_rotateMotorConfig);
      }
    );
    // Configure rotate kD
    builder.addDoubleProperty(
      "Rotate kD",
      () -> m_rotateMotorConfig.Slot0.kD,
      (value) -> {
        m_rotateMotorConfig.Slot0.withKD(value);
        m_rotateMotor.applyConfigs(m_rotateMotorConfig);
      }
    );
  }

  @Override
  public Frequency getUpdateRate() {
    return UPDATE_RATE;
  }

  @Override
  public void enableBrakeMode(boolean enable) {
    m_driveMotorConfig.MotorOutput.withNeutralMode((enable) ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setDriveSysID(Voltage volts) {
    m_rotateMotor.setControl(m_rotatePositionSetter.withPosition(m_zeroOffset.getMeasure()));
    m_driveMotor.setControl(m_driveVoltageSetter.withOutput(volts));
  }

  @Override
  public void setRotateSysID(Voltage volts) {
    m_rotateMotor.setControl(m_rotateVoltageSetter.withOutput(volts));
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

    // Get desired state
    m_desiredState = getDesiredState(state, Rotation2d.fromRadians(m_rotateMotor.getInputs().selectedSensorPosition.in(Units.Radians)));

    // Set rotate motor position
    m_rotateMotor.setControl(m_rotatePositionSetter.withPosition(m_desiredState.angle.getMeasure()));

    // Set drive motor speed
    m_driveMotor.setControl(
      m_driveVelocitySetter.withVelocity(linearToAngularVelocity(Units.MetersPerSecond.of(m_desiredState.speedMetersPerSecond)))
    );

    // Save rotate position
    m_previousRotatePosition = m_desiredState.angle;

    // Increment odometer
    super.incrementOdometer(Math.abs(m_desiredState.speedMetersPerSecond) * GlobalConstants.ROBOT_LOOP_FREQUENCY.asPeriod().in(Units.Seconds));
  }

  @Override
  public LinearVelocity getDriveVelocity() {
    return angularToLinearVelocity(m_driveMotor.getInputs().selectedSensorVelocity);
  }

  @Override
  public SwerveModuleState getState() {
    return new SwerveModuleState(
      getDriveVelocity(),
      Rotation2d.fromRadians(m_rotateMotor.getInputs().selectedSensorPosition.in(Units.Radians)).minus(m_zeroOffset)
    );
  }

  @Override
  public SwerveModulePosition getPosition() {
    if (RobotBase.isSimulation()) return m_simModulePosition;
    return new SwerveModulePosition(
      angleToDistance(m_driveMotor.getInputs().selectedSensorPosition),
      Rotation2d.fromRadians(m_rotateMotor.getInputs().selectedSensorPosition.in(Units.Radians)).minus(m_zeroOffset)
    );
  }

  @Override
  public void resetDriveEncoder() {
    m_driveMotor.resetPosition();
    m_simDrivePosition.mut_replace(Units.Meters.zero());
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