// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.drive.swerve.parent;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.time.Duration;
import java.time.Instant;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.lasarobotics.drive.TractionControlController;
import org.lasarobotics.drive.swerve.AdvancedSwerveKinematics;
import org.lasarobotics.drive.swerve.DriveWheel;
import org.lasarobotics.drive.swerve.SwerveModule;
import org.lasarobotics.drive.swerve.SwerveModuleSim;
import org.lasarobotics.hardware.PurpleManager;
import org.lasarobotics.hardware.ctre.CANCoder;
import org.lasarobotics.hardware.ctre.TalonFX;
import org.lasarobotics.utils.FFConstants;
import org.lasarobotics.utils.GlobalConstants;
import org.lasarobotics.utils.PIDConstants;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

/** REV MAXSwerve module */
public class CTRESwerveModule implements SwerveModule, Sendable, AutoCloseable {
  /**
   * CTRE swerve module hardware
   */
  public static class Hardware {
    public final TalonFX driveMotor;
    public final TalonFX rotateMotor;
    public final CANCoder canCoder;

    public Hardware(TalonFX driveMotor, TalonFX rotateMotor, CANCoder canCoder) {
      this.driveMotor = driveMotor;
      this.rotateMotor = rotateMotor;
      this.canCoder = canCoder;
    }
  }

  public static final Time DEFAULT_SIGNAL_PERIOD = Units.Milliseconds.of(10.0);
  public static final Time ALTERNATE_SIGNAL_PERIOD = Units.Seconds.of(10.0);
  public static final double DRIVETRAIN_EFFICIENCY = 0.90;

  private final double EPSILON = 5e-3;
  private final Current ROTATE_MOTOR_CURRENT_LIMIT = Units.Amps.of(20.0);
  private final Rotation2d LOCK_POSITION = Rotation2d.fromRadians(Math.PI / 4);

  private static final String IS_SLIPPING_LOG_ENTRY = "/IsSlipping";
  private static final String ODOMETER_LOG_ENTRY = "/Odometer";
  private static final String ROTATE_ERROR_LOG_ENTRY = "/RotateError";
  private static final double MAX_AUTO_LOCK_TIME = 10.0;
  private final double DRIVE_TICKS_PER_METER;
  private final double DRIVE_METERS_PER_TICK;
  private final double DRIVE_METERS_PER_ROTATION;
  private final double DRIVE_MAX_LINEAR_SPEED;
  private final int COSINE_CORRECTION;

  private TalonFX m_driveMotor;
  private TalonFX m_rotateMotor;
  private CANCoder m_canCoder;
  private SwerveModuleSim m_moduleSim;
  private TalonFXConfiguration m_driveMotorConfig;
  private TalonFXConfiguration m_rotateMotorConfig;
  private CANcoderConfiguration m_canCoderConfig;
  private Translation2d m_moduleCoordinate;
  private Rotation2d m_zeroOffset;

  private SwerveModule.Location m_location;
  private Rotation2d m_previousRotatePosition;

  private volatile SwerveModuleState m_desiredState;

  private SwerveModule.GearRatio m_gearRatio;
  private double m_driveConversionFactor;
  private double m_autoLockTime;
  private boolean m_autoLock;
  private double m_runningOdometer;
  private String m_odometerOutputPath;
  private final boolean m_isPhoenixPro;

  private TractionControlController m_tractionControlController;
  private Instant m_autoLockTimer;

  /**
   * Create an instance of a CTRE swerve module
   * @param swerveHardware Hardware devices required by swerve module
   * @param orientation Motor mount orientation
   * @param location Module location
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
   * @param driveCurrentLimit Desired current limit for the drive motor
   */
  public CTRESwerveModule(Hardware swerveHardware,
                          SwerveModule.MountOrientation orientation, SwerveModule.Location location,
                          SwerveModule.GearRatio gearRatio,
                          DriveWheel driveWheel, Angle zeroOffset,
                          PIDConstants drivePID, FFConstants driveFF,
                          PIDConstants rotatePID, FFConstants rotateFF,
                          Dimensionless slipRatio, Mass mass,
                          Distance wheelbase, Distance trackWidth,
                          Time autoLockTime, Current driveCurrentLimit) {
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
    COSINE_CORRECTION = RobotBase.isReal() ? 1 : 0;

    this.m_driveMotor = swerveHardware.driveMotor;
    this.m_rotateMotor = swerveHardware.rotateMotor;
    this.m_canCoder = swerveHardware.canCoder;
    this.m_moduleSim = new SwerveModuleSim(DCMotor.getKrakenX60(1), driveFF, DCMotor.getKrakenX60(1), rotateFF);
    this.m_location = location;
    this.m_gearRatio = gearRatio;
    this.m_zeroOffset = Rotation2d.fromRadians(zeroOffset.in(Units.Radians));
    this.m_autoLock = true;
    this.m_desiredState = new SwerveModuleState(Units.MetersPerSecond.of(0.0), LOCK_POSITION);
    this.m_autoLockTime = MathUtil.clamp(autoLockTime.in(Units.Milliseconds), 0.0, MAX_AUTO_LOCK_TIME * 1000);
    this.m_previousRotatePosition = LOCK_POSITION;
    this.m_tractionControlController =  new TractionControlController(driveWheel, slipRatio, mass, Units.MetersPerSecond.of(DRIVE_MAX_LINEAR_SPEED));
    this.m_autoLockTimer = Instant.now();
    this.m_runningOdometer = 0.0;

    m_driveMotorConfig = new TalonFXConfiguration();
    m_rotateMotorConfig = new TalonFXConfiguration();
    m_canCoderConfig = new CANcoderConfiguration();

    // Set drive encoder config
    m_driveConversionFactor = driveWheel.diameter.in(Units.Meters) * Math.PI / m_gearRatio.getDriveRatio();
    m_driveMotorConfig.Feedback.withSensorToMechanismRatio(m_driveConversionFactor);

    // Set rotate encoder config
    m_canCoder.getConfigurator()
      .apply(m_canCoderConfig.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Units.Rotations.of(1))
      .withSensorDirection(orientation.equals(MountOrientation.INVERTED) ? SensorDirectionValue.Clockwise_Positive
                                                                         : SensorDirectionValue.CounterClockwise_Positive));

    // Set sensor to use for closed loop control
    m_driveMotorConfig.Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);
    m_rotateMotorConfig.Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                                .withRotorToSensorRatio(m_gearRatio.getRotateRatio())
                                .withFeedbackRemoteSensorID(m_canCoder.getID().deviceID);

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
    m_driveMotorConfig.CurrentLimits.withStatorCurrentLimit(driveCurrentLimit);
    m_rotateMotorConfig.CurrentLimits.withStatorCurrentLimit(ROTATE_MOTOR_CURRENT_LIMIT);

    // Reset encoder
    resetDriveEncoder();

    // Configure motors with desired config
    m_driveMotor.applyConfigs(m_driveMotorConfig);
    m_rotateMotor.applyConfigs(m_rotateMotorConfig);

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

    // Read odometer file if exists
    m_odometerOutputPath = (m_driveMotor.getID().name + "-odometer.txt").replace('/', '-');
    File file = new File(m_odometerOutputPath);
    Distance previousDistanceTraveled = Units.Meters.of(0.0);
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
   * @param canCoderID CANCoder ID
   * @return Hardware object containing all necessary objects for a MAXSwerve module
   */
  public static Hardware initializeHardware(TalonFX.ID driveMotorID,
                                            TalonFX.ID rotateMotorID,
                                            CANCoder.ID canCoderID) {
    var period = RobotBase.isReal() ? DEFAULT_SIGNAL_PERIOD : Units.Seconds.of(GlobalConstants.ROBOT_LOOP_PERIOD);
    Hardware swerveModuleHardware = new Hardware(
      new TalonFX(driveMotorID, period),
      new TalonFX(rotateMotorID, period),
      new CANCoder(canCoderID)
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
      requestedState.angle.plus(m_zeroOffset)
    );

    // Get current module angle
    var currentAngle = Rotation2d.fromRadians(m_rotateMotor.getInputs().selectedSensorPosition.in(Units.Radians));

    // Optimize swerve module rotation state
    // REV encoder returns an angle in radians
    desiredState.optimize(currentAngle);

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother driving.
    desiredState.speedMetersPerSecond *= Math.pow(desiredState.angle.minus(currentAngle).getCos(), COSINE_CORRECTION);

    // Return corrected desired swerve module state
    return desiredState;
  }

  /**
   * Get velocity of module parallel to the desired orientation
   * @param state Desired swerve module state representing desired speed and orientation
   * @param realSpeeds Real speeds of robot from IMU
   * @return Velocity of module parallel to desired module orientation
   */
  private LinearVelocity getParallelVelocity(SwerveModuleState state, ChassisSpeeds realSpeeds) {
    // Get physical velocity of module through space
    var moduleInertialVelocityState = AdvancedSwerveKinematics.getRealModuleVelocity(realSpeeds, m_moduleCoordinate);
    // Get vector of physical velocity
    var moduleInertialVelocityVector = new Vector2D(
      moduleInertialVelocityState.speedMetersPerSecond * moduleInertialVelocityState.angle.getCos(),
      moduleInertialVelocityState.speedMetersPerSecond * moduleInertialVelocityState.angle.getSin()
    );
    // Get vector of desired velocity
    var moduleDesiredVelocityVector = new Vector2D(
      state.speedMetersPerSecond * state.angle.getCos(),
      state.speedMetersPerSecond * state.angle.getSin()
    );

    // Get angle between vectors
    var vectorAngle = Units.Radians.of(Math.acos(
      moduleDesiredVelocityVector.dotProduct(moduleInertialVelocityVector) / (moduleDesiredVelocityVector.getNorm() * moduleInertialVelocityVector.getNorm())
    ));

    // Calculate portion of inertial velocity that is in direction of desired
    var parallelModuleVelocityVector = moduleDesiredVelocityVector.scalarMultiply(
      moduleDesiredVelocityVector.getNorm() != 0.0
        ? moduleInertialVelocityVector.dotProduct(moduleDesiredVelocityVector) / moduleDesiredVelocityVector.getNormSq()
        : 1.0
    );

    // Get velocity
    var parallelModuleVelocity = Units.MetersPerSecond.of(parallelModuleVelocityVector.getNorm());

    // If requested state is generally in the opposite direction of inertial, negate the velocity to be reported
    if (vectorAngle.gt(Units.Radians.of(Math.PI / 2))) parallelModuleVelocity = parallelModuleVelocity.times(-1);

    // Return velocity
    return parallelModuleVelocity;
  }

  /**
   * Call this method periodically
   */
  private void periodic() {
    Logger.recordOutput(m_driveMotor.getID().name + IS_SLIPPING_LOG_ENTRY, isSlipping());
    Logger.recordOutput(m_driveMotor.getID().name + ODOMETER_LOG_ENTRY, m_runningOdometer);
    Logger.recordOutput(m_rotateMotor.getID().name + ROTATE_ERROR_LOG_ENTRY, m_desiredState.angle.minus(Rotation2d.fromRadians(m_rotateMotor.getInputs().selectedSensorPosition.in(Units.Radians))));
  }

 /**
   * Call this method periodically in simulation
   */
  private void simulationPeriodic() {
    var vbus = Units.Volts.of(RobotController.getBatteryVoltage());

    m_driveMotor.getSimState().setSupplyVoltage(vbus);
    m_rotateMotor.getSimState().setSupplyVoltage(vbus);

    m_driveMotor.getSimState().setRotorVelocity(m_moduleSim.getDriveMotorVelocity());
    m_rotateMotor.getSimState().setRotorVelocity(m_moduleSim.getRotateMotorVelocity());

    m_moduleSim.update(
      m_driveMotor.getSimState().getMotorVoltageMeasure(),
      m_rotateMotor.getSimState().getMotorVoltageMeasure()
    );

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_moduleSim.getTotalCurrentDraw().in(Units.Amps)));
  }

  /**
   * Allow for adding MAXSwerveModule as a sendable object for dashboard interactivity
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

  /**
   * Call method during initialization of disabled mode to set drive motor to brake mode and log odometry
   */
  private void disabledInit() {
    m_driveMotorConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    m_driveMotor.applyConfigs(m_driveMotorConfig);

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
    m_driveMotorConfig.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
    m_driveMotor.applyConfigs(m_driveMotorConfig);
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
        state.angle = LOCK_POSITION.minus(m_zeroOffset);
      // Waiting to lock...
      else state.angle = m_previousRotatePosition.minus(m_zeroOffset);
    } else {
      // Not locking this loop, restart timer...
      m_autoLockTimer = Instant.now();
    }

    // Get desired state
    m_desiredState = getDesiredState(state);

    // Set rotate motor position
    m_rotateMotor.setControl(new PositionVoltage(Units.Radians.of(m_desiredState.angle.getRadians())));

    // Set drive motor speed
    m_driveMotor.setControl(new VelocityVoltage(m_desiredState.speedMetersPerSecond));

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
    // Apply traction control
    state.speedMetersPerSecond = m_tractionControlController.calculate(
      Units.MetersPerSecond.of(state.speedMetersPerSecond),
      getParallelVelocity(state, realSpeeds),
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
    set(states[m_location.ordinal()]);
  }

  /**
   * Set swerve module direction and speed, automatically applying traction control
   * @param states Array of states for all swerve modules representing desired speed
   * @param realSpeeds Real speeds of robot from IMU
   */
  public void set(SwerveModuleState[] states, ChassisSpeeds realSpeeds) {
    set(states[m_location.ordinal()], realSpeeds);
  }

  /**
   * Get velocity of drive wheel
   * @return velocity of drive wheel in m/s
   */
  public LinearVelocity getDriveVelocity() {
    return Units.MetersPerSecond.of(m_driveMotor.getInputs().selectedSensorVelocity.magnitude());
  }

  /**
   * Get current module state
   * @return Current module state
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
      getDriveVelocity(),
      Rotation2d.fromRadians(m_canCoder.getInputs().absolutePosition.in(Units.Radians)).minus(m_zeroOffset)
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
    return new SwerveModulePosition(
      Units.Meters.of(m_driveMotor.getInputs().selectedSensorPosition.magnitude()),
      Rotation2d.fromRadians(m_rotateMotor.getInputs().selectedSensorPosition.in(Units.Radians)).minus(m_zeroOffset)
    );
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
    m_driveMotor.resetPosition();
  }

  /**
   * Lock swerve module
   */
  public void lock() {
    set(new SwerveModuleState(0.0, LOCK_POSITION.minus(m_zeroOffset)));
  }

  /**
   * Reset swerve module to 0 degrees
   */
  public void reset() {
    set(new SwerveModuleState(0.0, m_zeroOffset));
  }

  /**
   * Toggle traction control
   */
  public void toggleTractionControl() {
    m_tractionControlController.toggleTractionControl();
    if (m_tractionControlController.isEnabled()) m_driveMotorConfig.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
    else m_driveMotorConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
  }

  /**
   * Enable traction control
   */
  public void enableTractionControl() {
    m_tractionControlController.enableTractionControl();
    m_driveMotorConfig.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
    m_driveMotor.applyConfigs(m_driveMotorConfig);
  }

  /**
   * Disable traction control
   */
  public void disableTractionControl() {
    m_tractionControlController.disableTractionControl();
    m_driveMotorConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    m_driveMotor.applyConfigs(m_driveMotorConfig);
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
  public LinearVelocity getMaxLinearSpeed() {
    return Units.MetersPerSecond.of(DRIVE_MAX_LINEAR_SPEED);
  }

  /**
   * Get location of module on robot chassis
   * @return Location of module
   */
  public SwerveModule.Location getModuleLocation() {
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
  public SwerveModule.GearRatio getGearRatio() {
    return m_gearRatio;
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
  public Distance getRunningOdometer() {
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