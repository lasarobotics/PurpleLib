// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.hardware.revrobotics;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import org.apache.commons.math3.util.Precision;
import org.lasarobotics.hardware.LoggableHardware;
import org.lasarobotics.hardware.PurpleManager;
import org.lasarobotics.utils.GlobalConstants;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.ExternalFollower;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.REVLibError;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.SparkHelpers;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;

/** REV Spark */
public class Spark extends LoggableHardware {
  /** Spark ID */
  public static class ID {
    public final String name;
    public final int deviceID;

    /**
     * Spark ID
     * @param name Device name for logging
     * @param deviceID CAN ID
     */
    public ID(String name, int deviceID) {
      this.name = name;
      this.deviceID = deviceID;
    }
  }

  /** Kinds of motors */
  public enum MotorKind {
    NEO(MotorType.kBrushless, DCMotor.getNEO(1)),
    NEO_550(MotorType.kBrushless, DCMotor.getNeo550(1)),
    NEO_VORTEX(MotorType.kBrushless, DCMotor.getNeoVortex(1)),
    CIM(MotorType.kBrushed, DCMotor.getCIM(1)),
    MINI_CIM(MotorType.kBrushed, DCMotor.getMiniCIM(1)),
    BAG(MotorType.kBrushed, DCMotor.getBag(1)),
    VEX_775Pro(MotorType.kBrushed, DCMotor.getVex775Pro(1));

    public final MotorType type;
    public final DCMotor motor;
    private MotorKind(MotorType type, DCMotor motor) {
      this.type = type;
      this.motor = motor;
    }

    /**
     * Get max RPM of motor
     * @return Max RPM
     */
    public double getMaxRPM() {
      return edu.wpi.first.math.util.Units.radiansPerSecondToRotationsPerMinute(motor.freeSpeedRadPerSec);
    }
  }

  /** Feedback sensor */
  public enum FeedbackSensor {
    NEO_ENCODER, ANALOG, THROUGH_BORE_ENCODER;
  }

  /**
   * Spark sensor inputs
   */
  @AutoLog
  public static class SparkInputs {
    public double encoderPosition = 0.0;
    public double encoderVelocity = 0.0;
    public double analogPosition = 0.0;
    public double analogVelocity = 0.0;
    public double absoluteEncoderPosition = 0.0;
    public double absoluteEncoderVelocity = 0.0;
    public boolean forwardLimitSwitch = false;
    public boolean reverseLimitSwitch = false;
  }

  private static final int CAN_TIMEOUT_MS = 50;
  private static final int PID_SLOT = 0;
  private static final int MAX_ATTEMPTS = 20;
  private static final int SPARK_MAX_MEASUREMENT_PERIOD = 16;
  private static final int SPARK_FLEX_MEASUREMENT_PERIOD = 32;
  private static final int SPARK_MAX_AVERAGE_DEPTH = 2;
  private static final int SPARK_FLEX_AVERAGE_DEPTH = 8;
  private static final double EPSILON = 2e-8;
  private static final double MAX_VOLTAGE = 12.0;
  private static final double BURN_FLASH_WAIT_TIME = 0.5;
  private static final double APPLY_PARAMETER_WAIT_TIME = 0.1;
  private static final double SMOOTH_MOTION_DEBOUNCE_TIME = 0.1;
  private static final String VALUE_LOG_ENTRY = "/OutputValue";
  private static final String MODE_LOG_ENTRY = "/OutputMode";
  private static final String IDLE_MODE_LOG_ENTRY = "/IdleMode";
  private static final String CURRENT_LOG_ENTRY = "/Current";
  private static final String TEMPERATURE_LOG_ENTRY = "/Temperature";
  private static final String MOTION_LOG_ENTRY = "/SmoothMotion";
  private static final Measure<Time> DEFAULT_STATUS_FRAME_PERIOD = Units.Milliseconds.of(20.0);
  private static final Measure<Time> SLAVE_STATUS_FRAME_PERIOD = Units.Milliseconds.of(100.0);


  private CANSparkBase m_spark;

  private ID m_id;
  private MotorKind m_kind;
  private AtomicReference<SparkInputsAutoLogged> m_inputs;
  private Notifier m_inputsThread;

  private boolean m_isSmoothMotionEnabled;
  private Debouncer m_smoothMotionFinishedDebouncer;
  private TrapezoidProfile.State m_desiredState;
  private TrapezoidProfile.State m_smoothMotionState;
  private Supplier<TrapezoidProfile.State> m_currentStateSupplier;
  private Function<TrapezoidProfile.State, Double> m_feedforwardSupplier;

  private TrapezoidProfile m_motionProfile;
  private TrapezoidProfile.Constraints m_motionConstraint;
  private SparkPIDConfig m_config;
  private FeedbackSensor m_feedbackSensor;
  private SparkLimitSwitch.Type m_limitSwitchType = SparkLimitSwitch.Type.kNormallyOpen;
  private RelativeEncoder m_encoder;

  /**
   * Create a Spark that is unit-testing friendly with built-in logging
   * @param id Spark ID
   * @param kind The kind of motor connected to the controller
   * @param limitSwitchType Polarity of connected limit switches
   */
  public Spark(ID id, MotorKind kind, SparkLimitSwitch.Type limitSwitchType) {
    if (kind.equals(MotorKind.NEO_VORTEX)) {
      this.m_spark = new CANSparkFlex(id.deviceID, kind.type);
      this.m_encoder = m_spark.getEncoder(SparkRelativeEncoder.Type.kQuadrature, GlobalConstants.VORTEX_ENCODER_TICKS_PER_ROTATION);
    } else {
      this.m_spark = new CANSparkMax(id.deviceID, kind.type);
      this.m_encoder = m_spark.getEncoder(SparkRelativeEncoder.Type.kHallSensor, GlobalConstants.NEO_ENCODER_TICKS_PER_ROTATION);
      REVPhysicsSim.getInstance().addSparkMax((CANSparkMax)m_spark, kind.motor);
    }
    this.m_id = id;
    this.m_kind = kind;
    this.m_inputs = new AtomicReference<SparkInputsAutoLogged>();
    this.m_inputsThread = new Notifier(this::updateInputs);
    this.m_isSmoothMotionEnabled = false;
    this.m_limitSwitchType = limitSwitchType;

    // Set CAN timeout
    m_spark.setCANTimeout(CAN_TIMEOUT_MS);

    // Restore defaults
    restoreFactoryDefaults();
    enableVoltageCompensation(Units.Volts.of(MAX_VOLTAGE));

    // Fix velocity measurements
    if (getMotorType() == MotorType.kBrushless) {
      setMeasurementPeriod();
      setAverageDepth();
    }

    // Set status frame rates
    setPeriodicFrameRate(PeriodicFrame.kStatus0, DEFAULT_STATUS_FRAME_PERIOD);
    setPeriodicFrameRate(PeriodicFrame.kStatus1, DEFAULT_STATUS_FRAME_PERIOD);
    setPeriodicFrameRate(PeriodicFrame.kStatus2, DEFAULT_STATUS_FRAME_PERIOD);
    setPeriodicFrameRate(PeriodicFrame.kStatus3, DEFAULT_STATUS_FRAME_PERIOD);
    setPeriodicFrameRate(PeriodicFrame.kStatus4, DEFAULT_STATUS_FRAME_PERIOD);
    setPeriodicFrameRate(PeriodicFrame.kStatus5, DEFAULT_STATUS_FRAME_PERIOD);
    setPeriodicFrameRate(PeriodicFrame.kStatus6, DEFAULT_STATUS_FRAME_PERIOD);

    // Update inputs on init
    periodic();

    // Register device with manager
    PurpleManager.add(this);

    // Start sensor input thread
    m_inputsThread.startPeriodic(GlobalConstants.ROBOT_LOOP_PERIOD);
  }

  /**
   * Create a Spark that is unit-testing friendly with built-in logging
   * <p>
   * Defaults to normally-open limit switches
   * @param id Spark ID
   * @param kind The kind of motor connected to the controller
   */
  public Spark(ID id, MotorKind kind) {
    this(id, kind, SparkLimitSwitch.Type.kNormallyOpen);
  }

  /**
   * Create a Spark that is unit-testing friendly with built-in logging and configure PID
   * @param id Spark ID
   * @param kind The kind of motor connected to the controller
   * @param limitSwitchType Polarity of connected limit switches
   * @param config PID config for Spark
   * @param feedbackSensor Feedback device to use for Spark PID
   */
  public Spark(ID id, MotorKind kind, SparkLimitSwitch.Type limitSwitchType, SparkPIDConfig config, FeedbackSensor feedbackSensor) {
    this(id, kind);

    this.m_config = config;
    this.m_smoothMotionFinishedDebouncer = new Debouncer(SMOOTH_MOTION_DEBOUNCE_TIME);
    this.m_desiredState = new TrapezoidProfile.State();
    this.m_smoothMotionState = new TrapezoidProfile.State();
    this.m_feedforwardSupplier = (motionProfileState) -> 0.0;
    this.m_currentStateSupplier = () -> new TrapezoidProfile.State(getInputs().encoderPosition, getInputs().encoderVelocity);
    initializeSparkPID(m_config, feedbackSensor);
  }

  /**
   * Create a Spark that is unit-testing friendly with built-in logging and configure PID
   * <p>
   * Defaults to normally-open limit switches
   * @param id Spark ID
   * @param kind The kind of motor connected to the controller
   * @param config PID config for Spark
   * @param feedbackSensor Feedback device to use for Spark PID
   */
  public Spark(ID id, MotorKind kind, SparkPIDConfig config, FeedbackSensor feedbackSensor) {
    this(id, kind, SparkLimitSwitch.Type.kNormallyOpen, config, feedbackSensor);
  }

  /**
   * Attempt to apply parameter and check if specified parameter is set correctly
   * @param parameterSetter Method to set desired parameter
   * @param parameterCheckSupplier Method to check for parameter in question
   * @return {@link REVLibError#kOk} if successful
   */
  private REVLibError applyParameter(Supplier<REVLibError> parameterSetter, BooleanSupplier parameterCheckSupplier, String errorMessage) {
    if (RobotBase.isSimulation()) return parameterSetter.get();
    if (parameterCheckSupplier.getAsBoolean()) return REVLibError.kOk;

    REVLibError status = REVLibError.kError;
    for (int i = 0; i < MAX_ATTEMPTS; i++) {
      status = parameterSetter.get();
      if (parameterCheckSupplier.getAsBoolean() && status == REVLibError.kOk) break;
      Timer.delay(APPLY_PARAMETER_WAIT_TIME);
    }

    checkStatus(status, errorMessage);
    return status;
  }

  /**
   * Attempt to apply parameter and check if specified parameter is set correctly, for void setters
   * @param parameterSetter Method to set desired parameter
   * @param parameterCheckSupplier Method to check for parameter in question
   */
  private void applyParameter(Runnable parameterSetter, BooleanSupplier parameterCheckSupplier, String errorMessage) {
    if (RobotBase.isSimulation()) return;
    if (parameterCheckSupplier.getAsBoolean()) return;

    for (int i = 0; i < MAX_ATTEMPTS; i++) {
      parameterSetter.run();
      if (parameterCheckSupplier.getAsBoolean()) return;
      Timer.delay(APPLY_PARAMETER_WAIT_TIME);
    }

    System.err.println(String.join(" ", m_id.name, errorMessage));
  }

  /**
   * Check status and print error message if necessary
   * @param status Status to check
   * @param errorMessage Error message to print
   */
  private void checkStatus(REVLibError status, String errorMessage) {
    if (status != REVLibError.kOk)
      System.err.println(String.join(" ", m_id.name, errorMessage, "-", status.toString()));
  }

  /**
   * Log output values
   * @param value Value that was set
   * @param ctrl Control mode that was used
   */
  private void logOutputs(double value, ControlType ctrl) {
    Logger.recordOutput(m_id.name + VALUE_LOG_ENTRY, value);
    Logger.recordOutput(m_id.name + MODE_LOG_ENTRY, ctrl.toString());
  }

  /**
   * Returns an object for interfacing with the built-in encoder
   * @return
   */
  private RelativeEncoder getEncoder() {
    return m_encoder;
  }

  /**
   * Get the position of the motor encoder. This returns the native units of 'rotations' by default, and can
   * be changed by a scale factor using setPositionConversionFactor().
   * @return Number of rotations of the motor
   */
  private double getEncoderPosition() {
    return getEncoder().getPosition();
  }

  /**
   * Get the velocity of the motor encoder. This returns the native units of 'RPM' by default, and can be
   * changed by a scale factor using setVelocityConversionFactor().
   * @return Number the RPM of the motor
   */
  private double getEncoderVelocity() {
    return getEncoder().getVelocity();
  }

  /**
   * Returns an object for interfacing with a connected analog sensor.
   * @return An object for interfacing with a connected analog sensor
   */
  private SparkAnalogSensor getAnalog() {
    return m_spark.getAnalog(SparkAnalogSensor.Mode.kAbsolute);
  }

  /**
   * Get position of the analog sensor. This returns the native units 'volt' by default, and can
   * be changed by a scale factor using setPositionConversionFactor().
   * @return Volts on the sensor
   */
  private double getAnalogPosition() {
    return getAnalog().getPosition();
  }

  /**
   * Get the velocity of the analog sensor. This returns the native units of 'volts per second' by default, and can be
   * changed by a scale factor using setVelocityConversionFactor().
   * @return Volts per second on the sensor
   */
  private double getAnalogVelocity() {
    return getAnalog().getVelocity();
  }

  /**
   * Returns an object for interfacing with a connected absolute encoder.
   * @return An object for interfacing with a connected absolute encoder
   */
  private AbsoluteEncoder getAbsoluteEncoder() {
    return m_spark.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
  }

  /**
   * Get position of the absolute encoder. This returns the native units 'rotations' by default, and can
   * be changed by a scale factor using setPositionConversionFactor().
   * @return Number of rotations of the motor
   */
  private double getAbsoluteEncoderPosition() {
    return getAbsoluteEncoder().getPosition();
  }

  /**
   * Get the velocity of the absolute encoder. This returns the n`ative units of 'RPM' by default, and can be
   * changed by a scale factor using setVelocityConversionFactor().
   * @return Number the RPM of the motor
   */
  private double getAbsoluteEncoderVelocity() {
    return getAbsoluteEncoder().getVelocity();
  }

  /**
   * Returns an object for interfacing with the forward limit switch connected to the appropriate
   * pins on the data port.
   *
   * <p>This call will disable support for the alternate encoder.
   *
   * @param switchType Whether the limit switch is normally open or normally closed.
   * @return An object for interfacing with the forward limit switch.
   */
  private SparkLimitSwitch getForwardLimitSwitch() {
    return m_spark.getForwardLimitSwitch(m_limitSwitchType);
  }

  /**
   * Returns an object for interfacing with the reverse limit switch connected to the appropriate
   * pins on the data port.
   *
   * <p>This call will disable support for the alternate encoder.
   *
   * @param switchType Whether the limit switch is normally open or normally closed.
   * @return An object for interfacing with the reverse limit switch.
   */
  private SparkLimitSwitch getReverseLimitSwitch() {
    return m_spark.getReverseLimitSwitch(m_limitSwitchType);
  }

  /**
   * Set encoder velocity measurement period
   * <p>
   * Sets to {@value Spark#SPARK_MAX_MEASUREMENT_PERIOD} for Spark Max, {@value Spark#SPARK_FLEX_MEASUREMENT_PERIOD} for Spark Flex
   * @return {@link REVLibError#kOk} if successful
   */
  private REVLibError setMeasurementPeriod() {
    REVLibError status;
    int period = getKind().equals(MotorKind.NEO_VORTEX) ? SPARK_FLEX_MEASUREMENT_PERIOD : SPARK_MAX_MEASUREMENT_PERIOD;
    status = applyParameter(
      () -> getEncoder().setMeasurementPeriod(period),
      () -> getEncoder().getMeasurementPeriod() == period,
      "Set encoder measurement period failure!"
    );
    return status;
  }

  /**
   * Set encoder velocity measurement average depth
   * <p>
   * Sets to {@value Spark#SPARK_MAX_AVERAGE_DEPTH} for Spark Max, {@value Spark#SPARK_FLEX_AVERAGE_DEPTH} for Spark Flex
   * @return {@link REVLibError#kOk} if successful
   */
  private REVLibError setAverageDepth() {
    REVLibError status;
    int averageDepth = getKind().equals(MotorKind.NEO_VORTEX) ? SPARK_FLEX_AVERAGE_DEPTH : SPARK_MAX_AVERAGE_DEPTH;
    status = applyParameter(
      () -> getEncoder().setAverageDepth(averageDepth),
      () -> getEncoder().getAverageDepth() == averageDepth,
      "Set encoder average depth failure!"
    );
    return status;
  }

  /**
   * Enable voltage compensation
   * @param nominalVoltage Nominal voltage to compensate output to
   * @return {@link REVLibError#kOk} if successful
   */
  private REVLibError enableVoltageCompensation(Measure<Voltage> nominalVoltage) {
    REVLibError status;
    status = applyParameter(
      () -> m_spark.enableVoltageCompensation(nominalVoltage.in(Units.Volts)),
      () -> Precision.equals(m_spark.getVoltageCompensationNominalVoltage(), nominalVoltage.in(Units.Volts), EPSILON),
      "Enable voltage compensation failure!"
    );

    return status;
  }

  /**
   * Update sensor input readings
   */
  private void updateInputs() {
    var inputs = new SparkInputsAutoLogged();
    inputs.analogPosition = getAnalogPosition();
    inputs.analogVelocity = getAnalogVelocity();
    inputs.absoluteEncoderPosition = getAbsoluteEncoderPosition();
    inputs.absoluteEncoderVelocity = getAbsoluteEncoderVelocity();
    inputs.forwardLimitSwitch = getForwardLimitSwitch().isPressed();
    inputs.reverseLimitSwitch = getReverseLimitSwitch().isPressed();

    if (!getMotorType().equals(MotorType.kBrushed)) {
      inputs.encoderPosition = getEncoderPosition();
      inputs.encoderVelocity = getEncoderVelocity();
    }

    m_inputs.set(inputs);
  }

  /**
   * Handle smooth motion
   */
  private void handleSmoothMotion() {
    if (!m_isSmoothMotionEnabled) return;

    m_smoothMotionState = m_motionProfile.calculate(GlobalConstants.ROBOT_LOOP_PERIOD, m_smoothMotionState, m_desiredState);
    set(
      m_smoothMotionState.position,
      ControlType.kPosition,
      m_feedforwardSupplier.apply(m_smoothMotionState),
      SparkPIDController.ArbFFUnits.kVoltage
    );

    m_isSmoothMotionEnabled = !isSmoothMotionFinished();
  }

  /**
   * Call this method periodically
   */
  @Override
  protected void periodic() {
    Logger.processInputs(m_id.name, m_inputs.get());

    handleSmoothMotion();

    Logger.recordOutput(m_id.name + CURRENT_LOG_ENTRY, getOutputCurrent());
    Logger.recordOutput(m_id.name + MOTION_LOG_ENTRY, m_isSmoothMotionEnabled);

    if (getMotorType() == MotorType.kBrushed) return;
    Logger.recordOutput(m_id.name + TEMPERATURE_LOG_ENTRY, m_spark.getMotorTemperature());
  }

  /**
   * Get latest sensor input data
   * @return Latest sensor data
   */
  @Override
  public SparkInputsAutoLogged getInputs() {
    return m_inputs.get();
  }

  /**
   * Writes all settings to flash
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError burnFlash() {
    if (RobotBase.isSimulation()) return REVLibError.kOk;

    Timer.delay(BURN_FLASH_WAIT_TIME);
    REVLibError status = m_spark.burnFlash();
    Timer.delay(BURN_FLASH_WAIT_TIME);

    return status;
  }

  /**
   * Restore motor controller parameters to factory defaults until the next controller reboot
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError restoreFactoryDefaults() {
    REVLibError status;
    status = applyParameter(
      () -> m_spark.restoreFactoryDefaults(),
      () -> true,
      "Restore factory defaults failure!"
    );

    return status;
  }

  /**
   * Get device ID
   * @return Device ID
   */
  public ID getID() {
    return m_id;
  }

  /**
   * Get connected motor kind
   * @return Kind of connected motor
   */
  public MotorKind getKind() {
    return m_kind;
  }

  /**
   * Get motor type
   * @return Motor type
   */
  public MotorType getMotorType() {
    return m_spark.getMotorType();
  }

  /**
   * Check if motion is complete
   * @return True if smooth motion is complete
   */
  public boolean isSmoothMotionFinished() {
    return m_smoothMotionFinishedDebouncer.calculate(
      Precision.equals(m_currentStateSupplier.get().position, m_desiredState.position, m_config.getTolerance())
    );
  }

  /**
   * Initializes Spark PID
   * @param config Configuration to apply
   * @param feedbackSensor Feedback device to use for Spark PID
   * @param forwardLimitSwitch Enable forward limit switch
   * @param reverseLimitSwitch Enable reverse limit switch
   */
  public void initializeSparkPID(SparkPIDConfig config, FeedbackSensor feedbackSensor,
                                 boolean forwardLimitSwitch, boolean reverseLimitSwitch) {
    if (getMotorType().equals(MotorType.kBrushed) && feedbackSensor.equals(FeedbackSensor.NEO_ENCODER))
      throw new IllegalArgumentException("NEO encoder cannot be used with a brushed motor!");

    m_config = config;
    m_feedbackSensor = feedbackSensor;
    m_smoothMotionFinishedDebouncer = new Debouncer(SMOOTH_MOTION_DEBOUNCE_TIME);
    m_desiredState = new TrapezoidProfile.State();
    m_smoothMotionState = new TrapezoidProfile.State();
    m_feedforwardSupplier = (motionProfileState) -> 0.0;

    MotorFeedbackSensor selectedSensor;
    switch (m_feedbackSensor) {
      case ANALOG:
        selectedSensor = getAnalog();
        m_currentStateSupplier = () -> new TrapezoidProfile.State(getInputs().analogPosition, getInputs().analogVelocity);
        break;
      case THROUGH_BORE_ENCODER:
        selectedSensor = getAbsoluteEncoder();
        m_currentStateSupplier = () -> new TrapezoidProfile.State(getInputs().absoluteEncoderPosition, getInputs().absoluteEncoderVelocity);
        break;
      case NEO_ENCODER:
      default:
        selectedSensor = getEncoder();
        m_currentStateSupplier = () -> new TrapezoidProfile.State(getInputs().encoderPosition, getInputs().encoderVelocity);
        break;
    }

    // Configure feedback sensor and set sensor phase
    m_spark.getPIDController().setFeedbackDevice(selectedSensor);
    if (!m_feedbackSensor.equals(FeedbackSensor.NEO_ENCODER)) {
      applyParameter(
        () -> selectedSensor.setInverted(m_config.getSensorPhase()),
        () -> selectedSensor.getInverted() == m_config.getSensorPhase(),
      "Set sensor phase failure!"
      );
    }

    // Configure forward and reverse soft limits
    if (config.isSoftLimitEnabled()) {
      setForwardSoftLimit(config.getUpperLimit());
      enableForwardSoftLimit();
      setReverseSoftLimit(config.getLowerLimit());
      enableReverseSoftLimit();
    }

    // Configure forward and reverse limit switches if required
    if (forwardLimitSwitch) enableForwardLimitSwitch();
    if (reverseLimitSwitch) enableReverseLimitSwitch();

    // Invert motor if required
    setInverted(config.getInverted());

    // Configure PID values
    setP(config.getP());
    setI(config.getI());
    setD(config.getD());
    setF(config.getF());
    setIZone(config.getIZone());
  }

  /**
   * Initializes Spark PID
   * <p>
   * Calls {@link Spark#initializeSparkPID(SparkPIDConfig, FeedbackSensor, boolean, boolean)} with no limit switches
   * @param config Configuration to apply
   * @param feedbackSensor Feedback device to use for Spark PID
   */
  public void initializeSparkPID(SparkPIDConfig config, FeedbackSensor feedbackSensor) {
    initializeSparkPID(config, feedbackSensor, false, false);
  }

  /**
   * Slave Spark to another
   * @param master Spark to follow
   * @param invert Set slave to output opposite of the master
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError follow(Spark master, boolean invert) {
    REVLibError status;
    status = applyParameter(
      () -> m_spark.follow(ExternalFollower.kFollowerSpark, master.getID().deviceID, invert),
      () -> m_spark.isFollower(),
      "Set motor master failure!"
    );
    // Increase master output status frame rate
    master.setPeriodicFrameRate(PeriodicFrame.kStatus0, DEFAULT_STATUS_FRAME_PERIOD.divide(4));
    // Decrease all slave status frame rates
    setPeriodicFrameRate(PeriodicFrame.kStatus0, SLAVE_STATUS_FRAME_PERIOD);
    setPeriodicFrameRate(PeriodicFrame.kStatus1, SLAVE_STATUS_FRAME_PERIOD);
    setPeriodicFrameRate(PeriodicFrame.kStatus2, SLAVE_STATUS_FRAME_PERIOD);
    setPeriodicFrameRate(PeriodicFrame.kStatus3, SLAVE_STATUS_FRAME_PERIOD);
    setPeriodicFrameRate(PeriodicFrame.kStatus4, SLAVE_STATUS_FRAME_PERIOD);
    setPeriodicFrameRate(PeriodicFrame.kStatus5, SLAVE_STATUS_FRAME_PERIOD);
    setPeriodicFrameRate(PeriodicFrame.kStatus6, SLAVE_STATUS_FRAME_PERIOD);
    return status;
  }

  /**
   * Slave Spark to another
   * @param master Spark to follow
   */
  public void follow(Spark master) {
    follow(master, false);
  }

  /**
   * Common interface for inverting direction of a speed controller.
   *
   * <p>This call has no effect if the controller is a slave. To invert a slave, see the
   * {@link Spark#follow(Spark, boolean)} method.
   *
   * @param isInverted The state of inversion, true is inverted.
   */
  public void setInverted(boolean isInverted) {
    applyParameter(
      () -> m_spark.setInverted(isInverted),
      () -> m_spark.getInverted() == isInverted,
      "Set motor inverted failure!"
    );
  }

  /**
   * Set motor output duty cycle
   * @param value Value to set [-1.0, +1.0]
   */
  public void set(double value) {
    set(value, ControlType.kDutyCycle);
  }

  /**
   * Set motor output value
   * @param value Value to set
   * @param ctrl Desired control mode
   */
  public void set(double value, ControlType ctrl) {
    m_spark.getPIDController().setReference(value, ctrl);
    logOutputs(value, ctrl);
  }

  /**
   * Set motor output value with arbitrary feed forward
   * @param value Value to set
   * @param ctrl Desired control mode
   * @param arbFeedforward Feed forward value
   * @param arbFFUnits Feed forward units
   */
  public void set(double value, ControlType ctrl, double arbFeedforward, SparkPIDController.ArbFFUnits arbFFUnits) {
    m_spark.getPIDController().setReference(value, ctrl, PID_SLOT, arbFeedforward, arbFFUnits);
    logOutputs(value, ctrl);
  }

  /**
   * Set the conversion factor for position of the encoder. Multiplied by the native output units to
   * give you position.
   * @param sensor Sensor to set conversion factor for
   * @param factor The conversion factor to multiply the native units by
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setPositionConversionFactor(FeedbackSensor sensor, double factor) {
    REVLibError status;
    Supplier<REVLibError> parameterSetter;
    BooleanSupplier parameterCheckSupplier;
    switch (sensor) {
      case NEO_ENCODER:
        parameterSetter = () -> getEncoder().setPositionConversionFactor(factor);
        parameterCheckSupplier = () -> Precision.equals(getEncoder().getPositionConversionFactor(), factor, EPSILON);
        break;
      case ANALOG:
        parameterSetter = () -> getAnalog().setPositionConversionFactor(factor);
        parameterCheckSupplier = () -> Precision.equals(getAnalog().getPositionConversionFactor(), factor, EPSILON);
        break;
      case THROUGH_BORE_ENCODER:
        parameterSetter = () -> getAbsoluteEncoder().setPositionConversionFactor(factor);
        parameterCheckSupplier = () -> Precision.equals(getAbsoluteEncoder().getPositionConversionFactor(), factor, EPSILON);
        break;
      default:
        parameterSetter = () -> REVLibError.kOk;
        parameterCheckSupplier = () -> true;
        break;
    }

    status = applyParameter(parameterSetter, parameterCheckSupplier, "Set position conversion factor failure!");
    return status;
  }

  /**
   * Set the conversion factor for velocity of the encoder. Multiplied by the native output units to
   * give you velocity.
   * @param sensor Sensor to set conversion factor for
   * @param factor The conversion factor to multiply the native units by
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setVelocityConversionFactor(FeedbackSensor sensor, double factor) {
    REVLibError status;
    Supplier<REVLibError> parameterSetter;
    BooleanSupplier parameterCheckSupplier;
    switch (sensor) {
      case NEO_ENCODER:
        parameterSetter = () -> getEncoder().setVelocityConversionFactor(factor);
        parameterCheckSupplier = () -> Precision.equals(getEncoder().getVelocityConversionFactor(), factor, EPSILON);
        break;
      case ANALOG:
        parameterSetter = () -> getAnalog().setVelocityConversionFactor(factor);
        parameterCheckSupplier = () -> Precision.equals(getAnalog().getVelocityConversionFactor(), factor, EPSILON);
        break;
      case THROUGH_BORE_ENCODER:
        parameterSetter = () -> getAbsoluteEncoder().setVelocityConversionFactor(factor);
        parameterCheckSupplier = () -> Precision.equals(getAnalog().getVelocityConversionFactor(), factor, EPSILON);
        break;
      default:
        parameterSetter = () -> REVLibError.kOk;
        parameterCheckSupplier = () -> true;
        break;
    }

    status = applyParameter(parameterSetter, parameterCheckSupplier, "Set velocity conversion factor failure!");
    return status;
  }

  /**
   * Set proportional gain for PIDF controller on Spark
   * @param value Value to set
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setP(double value) {
    REVLibError status;
    status = applyParameter(
      () -> m_spark.getPIDController().setP(value),
      () -> Precision.equals(m_spark.getPIDController().getP(), value, EPSILON),
      "Set kP failure!"
    );
    return status;
  }

  /**
   * Set integral gain for PIDF controller on Spark
   * @param value Value to set
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setI(double value) {
    REVLibError status;
    status = applyParameter(
      () -> m_spark.getPIDController().setI(value),
      () -> Precision.equals(m_spark.getPIDController().getI(), value, EPSILON),
      "Set kI failure!"
    );
    return status;
  }

  /**
   * Set derivative gain for PIDF controller on Spark
   * @param value Value to set
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setD(double value) {
    REVLibError status;
    status = applyParameter(
      () -> m_spark.getPIDController().setD(value),
      () -> Precision.equals(m_spark.getPIDController().getD(), value, EPSILON),
      "Set kD failure!"
    );
    return status;
  }

  /**
   * Set feed-forward gain for PIDF controller on Spark
   * @param value Value to set
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setF(double value) {
    REVLibError status;
    status = applyParameter(
      () -> m_spark.getPIDController().setFF(value),
      () -> Precision.equals(m_spark.getPIDController().getFF(), value, EPSILON),
      "Set kF failure!"
    );
    return status;
  }

  /**
   * Set integral zone range for PIDF controller on Spark
   * <p>
   * This value specifies the range the |error| must be within for the integral constant to take effect
   * @param value Value to set
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setIZone(double value) {
    REVLibError status;
    status = applyParameter(
      () -> m_spark.getPIDController().setIZone(value),
      () -> Precision.equals(m_spark.getPIDController().getIZone(), value, EPSILON),
      "Set IZone failure!"
    );
    return status;
  }

  /**
   * Execute a smooth motion to desired position
   * @param value The target value for the motor
   * @param motionConstraint The constraints for the motor
   * @param feedforwardSupplier Lambda function to calculate feed forward
   */
  public void smoothMotion(double value, TrapezoidProfile.Constraints motionConstraint, Function<TrapezoidProfile.State, Double> feedforwardSupplier) {
    m_isSmoothMotionEnabled = true;
    m_feedforwardSupplier = feedforwardSupplier;
    m_motionConstraint = motionConstraint;
    m_desiredState = new TrapezoidProfile.State(value, 0.0);
    m_motionProfile = new TrapezoidProfile(m_motionConstraint);
    m_smoothMotionState = m_currentStateSupplier.get();

    periodic();
  }

  /**
   * Execute a smooth motion to desired position
   * @param value The target value for the motor
   * @param motionConstraint The constraints for the motor
   */
  public void smoothMotion(double value, TrapezoidProfile.Constraints motionConstraint) {
    smoothMotion(value, motionConstraint, (motionProfileState) -> 0.0);
  }


  /**
   * Reset NEO built-in encoder to desired value
   * @param value Desired encoder value
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError resetEncoder(double value) {
    REVLibError status;
    status = applyParameter(
      () -> getEncoder().setPosition(value),
      () -> Precision.equals(getEncoderPosition(), value, EPSILON),
      "Set encoder failure!"
    );
    if (status.equals(REVLibError.kOk))
      System.out.println(String.join(" ", m_id.name, "Encoder set to", String.valueOf(value), "!"));

    return status;
  }

  /**
   * Reset NEO built-in encoder to zero
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError resetEncoder() {
    return resetEncoder(0.0);
  }

  /**
   * Disable forward limit switch
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError disableForwardLimitSwitch() {
    REVLibError status;
    status = applyParameter(
      () -> getForwardLimitSwitch().enableLimitSwitch(false),
      () -> getForwardLimitSwitch().isLimitSwitchEnabled() == false,
      "Disable forward limit switch failure!"
    );
    return status;
  }

  /**
   * Enable forward limit switch
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError enableForwardLimitSwitch() {
    REVLibError status;
    status = applyParameter(
      () -> getForwardLimitSwitch().enableLimitSwitch(true),
      () -> getForwardLimitSwitch().isLimitSwitchEnabled() == true,
      "Enable forward limit switch failure!"
    );
    return status;
  }

  /**
   * Disable reverse limit switch
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError disableReverseLimitSwitch() {
    REVLibError status;
    status = applyParameter(
      () -> getReverseLimitSwitch().enableLimitSwitch(false),
      () -> getReverseLimitSwitch().isLimitSwitchEnabled() == false,
      "Disable reverse limit switch failure!"
    );
    return status;
  }

  /**
   * Enable reverse limit switch
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError enableReverseLimitSwitch() {
    REVLibError status;
    status = applyParameter(
      () -> getReverseLimitSwitch().enableLimitSwitch(true),
      () -> getReverseLimitSwitch().isLimitSwitchEnabled() == true,
      "Enable reverse limit switch failure!"
    );
    return status;
  }

  /**
   * Set forward soft limit
   * @param limit Value to set
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setForwardSoftLimit(double limit) {
    REVLibError status;
    status = applyParameter(
      () -> m_spark.setSoftLimit(SoftLimitDirection.kForward, (float)limit),
      () -> Precision.equals(m_spark.getSoftLimit(SoftLimitDirection.kForward), limit, EPSILON),
      "Set forward soft limit failure!"
    );
    return status;
  }

  /**
   * Set reverse soft limit
   * @param limit Value to set
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setReverseSoftLimit(double limit) {
    REVLibError status;
    status = applyParameter(
      () -> m_spark.setSoftLimit(SoftLimitDirection.kReverse, (float)limit),
      () -> Precision.equals(m_spark.getSoftLimit(SoftLimitDirection.kReverse), limit, EPSILON),
      "Set reverse soft limit failure!"
    );
    return status;
  }

  /**
   * Enable forward soft limit
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError enableForwardSoftLimit() {
    REVLibError status;
    status = applyParameter(
      () -> m_spark.enableSoftLimit(SoftLimitDirection.kForward, true),
      () -> m_spark.isSoftLimitEnabled(SoftLimitDirection.kForward) == true,
      "Enable forward soft limit failure!"
    );
    return status;
  }

  /**
   * Enable reverse soft limit
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError enableReverseSoftLimit() {
    REVLibError status;
    status = applyParameter(
      () -> m_spark.enableSoftLimit(SoftLimitDirection.kReverse, true),
      () -> m_spark.isSoftLimitEnabled(SoftLimitDirection.kReverse) == true,
      "Enable reverse soft limit failure!"
    );
    return status;
  }

  /**
   * Disable forward soft limit
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError disableForwardSoftLimit() {
    REVLibError status;
    status = applyParameter(
      () -> m_spark.enableSoftLimit(SoftLimitDirection.kForward, false),
      () -> m_spark.isSoftLimitEnabled(SoftLimitDirection.kForward) == false,
      "Disable forward soft limit failure!"
    );
    return status;
  }

  /**
   * Disable reverse soft limit
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError disableReverseSoftLimit() {
    REVLibError status;
    status = applyParameter(
      () -> m_spark.enableSoftLimit(SoftLimitDirection.kReverse, false),
      () -> m_spark.isSoftLimitEnabled(SoftLimitDirection.kReverse) == false,
      "Disable reverse soft limit failure!"
    );
    return status;
  }

  /**
   * Enable PID wrapping for closed loop position control
   * @param minInput Value of min input for position
   * @param maxInput Value of max input for position
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError enablePIDWrapping(double minInput, double maxInput) {
    REVLibError status;
    Supplier<REVLibError> parameterSetter = () -> {
      REVLibError s;
      s = m_spark.getPIDController().setPositionPIDWrappingEnabled(true);
      s = m_spark.getPIDController().setPositionPIDWrappingMinInput(minInput);
      s = m_spark.getPIDController().setPositionPIDWrappingMaxInput(maxInput);
      return s;
    };
    BooleanSupplier parameterCheckSupplier = () ->
      m_spark.getPIDController().getPositionPIDWrappingEnabled() == true &&
      Precision.equals(m_spark.getPIDController().getPositionPIDWrappingMinInput(), minInput, EPSILON) &&
      Precision.equals(m_spark.getPIDController().getPositionPIDWrappingMaxInput(), maxInput, EPSILON);

    status = applyParameter(parameterSetter, parameterCheckSupplier, "Enable position PID wrapping failure!");
    return status;
  }

  /**
   * Disable PID wrapping for close loop position control
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError disablePIDWrapping() {
    REVLibError status;
    status = applyParameter(
      () -> m_spark.getPIDController().setPositionPIDWrappingEnabled(false),
      () -> m_spark.getPIDController().getPositionPIDWrappingEnabled() == false,
      "Disable position PID wrapping failure!"
    );
    return status;
  }

  /**
   * Sets the idle mode setting for the Spark
   * @param mode Idle mode (coast or brake).
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setIdleMode(IdleMode mode) {
    REVLibError status;
    status = applyParameter(
      () -> m_spark.setIdleMode(mode),
      () -> m_spark.getIdleMode() == mode,
      "Set idle mode failure!"
    );
    // Log idle mode
    Logger.recordOutput(m_id.name + IDLE_MODE_LOG_ENTRY, m_spark.getIdleMode() == IdleMode.kCoast);

    return status;
  }

  /**
   * Sets the current limit in Amps.
   *
   * <p>The motor controller will reduce the controller voltage output to avoid surpassing this
   * limit. This limit is enabled by default and used for brushless only. This limit is highly
   * recommended when using the NEO brushless motor.
   *
   * <p>The motor controller has a low internal resistance, which can mean large current spikes
   * that could be enough to cause damage to the motor and controller. This current limit provides a
   * smarter strategy to deal with high current draws and keep the motor and controller operating in
   * a safe region.
   *
   * @param limit The desired current limit
   */
  public REVLibError setSmartCurrentLimit(Measure<Current> limit) {
    REVLibError status;
    status = applyParameter(
      () -> m_spark.setSmartCurrentLimit((int)limit.in(Units.Amps)),
      () -> SparkHelpers.getSmartCurrentLimit(m_spark) == (int)limit.in(Units.Amps),
      "Set current limit failure!"
    );
    return status;
  }

  /**
   * Set the rate of transmission for periodic frames from the SPARK
   *
   * <p>Each motor controller sends back status frames with different data at set rates. Use this
   * function to change the default rates.
   *
   * <p>Defaults: Status0 - 10ms Status1 - 20ms Status2 - 20ms Status3 - 50ms Status4 - 20ms Status5
   * - 200ms Status6 - 200ms Status7 - 250ms
   *
   * <p>This value is not stored in the FLASH after calling burnFlash() and is reset on powerup.
   *
   * <p>Refer to the SPARK reference manual on details for how and when to configure this parameter.
   *
   * @param frame Which type of periodic frame to change the period of
   * @param period The rate the controller sends the frame to the controller.
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setPeriodicFrameRate(PeriodicFrame frame, Measure<Time> period) {
    REVLibError status;
    status = applyParameter(
      () -> m_spark.setPeriodicFramePeriod(frame, (int)period.in(Units.Milliseconds)),
      () -> true,
      "Set " + frame.name() + " rate failure!"
    );
    return status;
  }

  /**
   * Sets the ramp rate for open loop control modes.
   *
   * <p>This is the maximum rate at which the motor controller's output is allowed to change.
   *
   * @param rampTime Time to go from 0 to full throttle.
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setOpenLoopRampRate(Measure<Time> rampTime) {
    return m_spark.setOpenLoopRampRate(rampTime.in(Units.Seconds));
  }

  /**
   * Sets the ramp rate for closed loop control modes.
   *
   * <p>This is the maximum rate at which the motor controller's output is allowed to change.
   *
   * @param rampTime Time to go from 0 to full throttle.
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setClosedLoopRampRate(Measure<Time> rampTime) {
    return m_spark.setClosedLoopRampRate(rampTime.in(Units.Seconds));
  }

  /**
   * Spark output current
   * @return Output current
   */
  public Measure<Current> getOutputCurrent() {
    return Units.Amps.of(m_spark.getOutputCurrent());
  }

  /**
   * Stops motor movement. Motor can be moved again by calling set without having to re-enable the
   * motor.
   */
  public void stopMotor() {
    m_spark.stopMotor();
    logOutputs(0.0, ControlType.kDutyCycle);
  }

  /**
   * Closes the Spark motor controller
   */
  @Override
  public void close() {
    PurpleManager.remove(this);
    m_spark.close();
  }
}
