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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
public class REVSwerveModule implements SwerveModule, Sendable, AutoCloseable {
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

  public static final Time DEFAULT_SIGNAL_PERIOD = Units.Milliseconds.of(10.0);
  public static final Time SLOW_SIGNAL_PERIOD = Units.Seconds.of(10.0);
  public static final double DRIVETRAIN_EFFICIENCY = 0.90;

  private final double EPSILON = 5e-3;
  private final Current DRIVE_MOTOR_CURRENT_LIMIT;
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

  private Spark m_driveMotor;
  private Spark m_rotateMotor;
  private SwerveModuleSim m_moduleSim;
  private SimpleMotorFeedforward m_driveFF;
  private SparkBaseConfig m_driveMotorConfig;
  private SparkBaseConfig m_rotateMotorConfig;
  private Translation2d m_moduleCoordinate;
  private Rotation2d m_zeroOffset;

  private SwerveModule.Location m_location;
  private Rotation2d m_previousRotatePosition;

  private volatile double m_simDrivePosition;
  private volatile SwerveModuleState m_desiredState;

  private SwerveModule.GearRatio m_gearRatio;
  private double m_driveConversionFactor;
  private double m_rotateConversionFactor;
  private double m_autoLockTime;
  private boolean m_autoLock;
  private double m_runningOdometer;
  private String m_odometerOutputPath;

  private TractionControlController m_tractionControlController;
  private Instant m_autoLockTimer;

  /**
   * Create an instance of a REV swerve module
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
  public REVSwerveModule(Hardware swerveHardware,
                         SwerveModule.MountOrientation orientation, SwerveModule.Location location,
                         SwerveModule.GearRatio gearRatio,
                         DriveWheel driveWheel, Angle zeroOffset,
                         PIDConstants drivePID, FFConstants driveFF,
                         PIDConstants rotatePID, FFConstants rotateFF,
                         Dimensionless slipRatio, Mass mass,
                         Distance wheelbase, Distance trackWidth,
                         Time autoLockTime, Current driveCurrentLimit) {
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
    COSINE_CORRECTION = RobotBase.isReal() ? 1 : 0;

    this.m_driveMotor = swerveHardware.driveMotor;
    this.m_rotateMotor = swerveHardware.rotateMotor;
    this.m_moduleSim = new SwerveModuleSim(m_driveMotor.getKind().motor, driveFF, m_rotateMotor.getKind().motor, rotateFF);
    this.m_driveFF = new SimpleMotorFeedforward(driveFF.kS, driveFF.kV, driveFF.kA);
    this.m_location = location;
    this.m_gearRatio = gearRatio;
    this.m_zeroOffset = Rotation2d.fromRadians(zeroOffset.in(Units.Radians));
    this.m_autoLock = true;
    this.m_simDrivePosition = 0.0;
    this.m_desiredState = new SwerveModuleState(Units.MetersPerSecond.of(0.0), LOCK_POSITION);
    this.m_autoLockTime = MathUtil.clamp(autoLockTime.in(Units.Milliseconds), 0.0, MAX_AUTO_LOCK_TIME * 1000);
    this.m_previousRotatePosition = LOCK_POSITION;
    this.m_tractionControlController =  new TractionControlController(driveWheel, slipRatio, mass, Units.MetersPerSecond.of(DRIVE_MAX_LINEAR_SPEED));
    this.m_autoLockTimer = Instant.now();
    this.m_runningOdometer = 0.0;

    m_driveMotorConfig = (m_driveMotor.getKind().equals(MotorKind.NEO_VORTEX)) ? new SparkFlexConfig() : new SparkMaxConfig();
    m_rotateMotorConfig = (m_rotateMotor.getKind().equals(MotorKind.NEO_VORTEX)) ? new SparkFlexConfig() : new SparkMaxConfig();

    // Set drive encoder config
    m_driveConversionFactor = driveWheel.diameter.in(Units.Meters) * Math.PI / m_gearRatio.getDriveRatio();
    m_driveMotorConfig.encoder.positionConversionFactor(m_driveConversionFactor);
    m_driveMotorConfig.encoder.velocityConversionFactor(m_driveConversionFactor / 60);

    // Set rotate encoder config
    m_rotateConversionFactor = 2 * Math.PI;
    m_rotateMotorConfig.absoluteEncoder.positionConversionFactor(m_rotateConversionFactor);
    m_rotateMotorConfig.absoluteEncoder.velocityConversionFactor(m_rotateConversionFactor / 60);
    m_rotateMotorConfig.absoluteEncoder.inverted(orientation.equals(MountOrientation.INVERTED));

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
    m_driveMotorConfig.signals.absoluteEncoderPositionPeriodMs((int)SLOW_SIGNAL_PERIOD.in(Units.Seconds));
    m_driveMotorConfig.signals.absoluteEncoderVelocityPeriodMs((int)SLOW_SIGNAL_PERIOD.in(Units.Seconds));
    m_driveMotorConfig.signals.analogPositionPeriodMs((int)SLOW_SIGNAL_PERIOD.in(Units.Milliseconds));
    m_driveMotorConfig.signals.analogVelocityPeriodMs((int)SLOW_SIGNAL_PERIOD.in(Units.Milliseconds));
    m_driveMotorConfig.signals.limitsPeriodMs((int)SLOW_SIGNAL_PERIOD.in(Units.Milliseconds));
    m_rotateMotorConfig.signals.primaryEncoderPositionPeriodMs((int)SLOW_SIGNAL_PERIOD.in(Units.Seconds));
    m_rotateMotorConfig.signals.primaryEncoderVelocityPeriodMs((int)SLOW_SIGNAL_PERIOD.in(Units.Seconds));
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
   * @param driveMotorKind Kind of drive motor
   * @return Hardware object containing all necessary objects for a MAXSwerve module
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
    var period = RobotBase.isReal() ? DEFAULT_SIGNAL_PERIOD : Units.Seconds.of(GlobalConstants.ROBOT_LOOP_PERIOD);
    Hardware swerveModuleHardware = new Hardware(
      new Spark(driveMotorID, driveMotorKind, period),
      new Spark(rotateMotorID, rotateMotorKind, period)
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
    var currentAngle = Rotation2d.fromRadians(m_rotateMotor.getInputs().absoluteEncoderPosition);

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
    Logger.recordOutput(m_rotateMotor.getID().name + ROTATE_ERROR_LOG_ENTRY, m_desiredState.angle.minus(Rotation2d.fromRadians(m_rotateMotor.getInputs().absoluteEncoderPosition)));
  }

 /**
   * Call this method periodically in simulation
   */
  private void simulationPeriodic() {
    var vbus = Units.Volts.of(RobotController.getBatteryVoltage());
    m_driveMotor.getSim().enable();
    m_rotateMotor.getSim().enable();

    m_driveMotor.getSim().iterate(m_moduleSim.getDriveMotorVelocity().in(Units.RPM), vbus.in(Units.Volts), GlobalConstants.ROBOT_LOOP_PERIOD);
    m_rotateMotor.getSim().iterate(m_moduleSim.getRotateMotorVelocity().in(Units.RPM), vbus.in(Units.Volts), GlobalConstants.ROBOT_LOOP_PERIOD);

    m_moduleSim.update(
      vbus.times(m_driveMotor.getSim().getAppliedOutput()),
      vbus.times(m_rotateMotor.getSim().getAppliedOutput())
    );

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_moduleSim.getTotalCurrentDraw().in(Units.Amps)));

    m_driveMotor.getSim().setMotorCurrent(m_moduleSim.getDriveMotorCurrentDraw().in(Units.Amps));
    m_rotateMotor.getSim().setMotorCurrent(m_moduleSim.getRotateMotorCurrentDraw().in(Units.Amps));
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
        state.angle = LOCK_POSITION.minus(m_zeroOffset);
      // Waiting to lock...
      else state.angle = m_previousRotatePosition.minus(m_zeroOffset);
    } else {
      // Not locking this loop, restart timer...
      m_autoLockTimer = Instant.now();
    }

    // Save previous state
    var oldState = m_desiredState;

    // Get desired state
    m_desiredState = getDesiredState(state);

    // Set rotate motor position
    m_rotateMotor.set(m_desiredState.angle.getRadians(), ControlType.kPosition);

    // Calculate drive FF
    var driveFF = m_driveFF.calculate(
      Units.MetersPerSecond.of(oldState.speedMetersPerSecond),
      Units.MetersPerSecond.of(m_desiredState.speedMetersPerSecond)
    );

    // Set drive motor speed
    m_driveMotor.set(
      m_desiredState.speedMetersPerSecond, ControlType.kVelocity,
      driveFF.in(Units.Volts), ArbFFUnits.kVoltage
    );

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
    return Units.MetersPerSecond.of(m_driveMotor.getInputs().encoderVelocity);
  }

  /**
   * Get current module state
   * @return Current module state
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
      getDriveVelocity(),
      Rotation2d.fromRadians(m_rotateMotor.getInputs().absoluteEncoderPosition).minus(m_zeroOffset)
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
        Rotation2d.fromRadians(m_rotateMotor.getInputs().absoluteEncoderPosition).minus(m_zeroOffset)
      );
    }

    m_simDrivePosition += m_desiredState.speedMetersPerSecond * DEFAULT_SIGNAL_PERIOD.in(Units.Seconds);
    synchronized (m_driveMotor.getInputs()) {
      m_driveMotor.getInputs().encoderPosition = m_simDrivePosition;
      synchronized (m_rotateMotor.getInputs()) {
        m_rotateMotor.getInputs().absoluteEncoderPosition = m_desiredState.angle.getRadians();
        return new SwerveModulePosition(
          m_driveMotor.getInputs().encoderPosition,
          Rotation2d.fromRadians(m_rotateMotor.getInputs().absoluteEncoderPosition).minus(m_zeroOffset)
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