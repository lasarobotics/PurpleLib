// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.hardware.revrobotics;

import java.util.function.BooleanSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import org.lasarobotics.hardware.LoggableHardware;
import org.lasarobotics.utils.GlobalConstants;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.ExternalFollower;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.REVLibError;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;

/** REV Spark Max */
public class SparkMax implements LoggableHardware, AutoCloseable {
  /** Spark Max ID */
  public static class ID {
    public final String name;
    public final int deviceID;

    /**
     * Spark Max ID
     * @param name Device name for logging
     * @param deviceID CAN ID
     */
    public ID(String name, int deviceID) {
      this.name = name;
      this.deviceID = deviceID;
    }
  }

  /** Feedback sensor */
  public enum FeedbackSensor {
    NEO_ENCODER, ANALOG, THROUGH_BORE_ENCODER;
  }

  /**
   * Spark Max sensor inputs
   */
  @AutoLog
  public static class SparkMaxInputs {
    public double encoderPosition = 0.0;
    public double encoderVelocity = 0.0;
    public double analogPosition = 0.0;
    public double analogVelocity = 0.0;
    public double absoluteEncoderPosition = 0.0;
    public double absoluteEncoderVelocity = 0.0;
    public boolean forwardLimitSwitch = false;
    public boolean reverseLimitSwitch = false;
  }

  private static final int PID_SLOT = 0;
  private static final int MAX_ATTEMPTS = 5;
  private static final double MAX_VOLTAGE = 12.0;
  private static final double SMOOTH_MOTION_DEBOUNCE_TIME = 0.1;
  private static final String VALUE_LOG_ENTRY = "/OutputValue";
  private static final String MODE_LOG_ENTRY = "/OutputMode";
  private static final String CURRENT_LOG_ENTRY = "/Current";
  private static final String MOTION_LOG_ENTRY = "/SmoothMotion";


  private CANSparkMax m_spark;

  private ID m_id;
  private SparkMaxInputsAutoLogged m_inputs;

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
  private SparkMaxLimitSwitch.Type m_limitSwitchType = SparkMaxLimitSwitch.Type.kNormallyOpen;

  /**
   * Create a Spark Max with built-in logging and is unit-testing friendly
   * @param id Spark Max ID
   * @param motorType The motor type connected to the controller
   */
  public SparkMax(ID id, MotorType motorType) {
    this.m_id = id;
    this.m_spark = new CANSparkMax(id.deviceID, motorType);
    this.m_inputs = new SparkMaxInputsAutoLogged();
    this.m_isSmoothMotionEnabled = false;

    m_spark.restoreFactoryDefaults();
    m_spark.enableVoltageCompensation(MAX_VOLTAGE);
  }

  /**
   * Create a Spark Max with built-in logging, is unit-testing friendly and configure PID
   * @param id Spark Max ID
   * @param motorType The motor type connected to the controller
   * @param config PID config for Spark Max
   * @param feedbackSensor Feedback device to use for Spark PID
   */
  public SparkMax(ID id, MotorType motorType, SparkPIDConfig config, FeedbackSensor feedbackSensor) {
    this.m_id = id;
    this.m_spark = new CANSparkMax(id.deviceID, motorType);
    this.m_inputs = new SparkMaxInputsAutoLogged();
    this.m_isSmoothMotionEnabled = false;

    m_spark.restoreFactoryDefaults();
    m_spark.enableVoltageCompensation(MAX_VOLTAGE);

    this.m_config = config;
    this.m_smoothMotionFinishedDebouncer = new Debouncer(SMOOTH_MOTION_DEBOUNCE_TIME);
    this.m_desiredState = new TrapezoidProfile.State();
    this.m_smoothMotionState = new TrapezoidProfile.State();
    this.m_feedforwardSupplier = (motionProfileState) -> 0.0;
    this.m_currentStateSupplier = () -> new TrapezoidProfile.State(getInputs().encoderPosition, getInputs().encoderVelocity);
    initializeSparkPID(m_config, feedbackSensor);
  }

  /**
   * Get internal Spark Max object
   * @return Raw internal Spark Max object
   */
  CANSparkMax getMotorController() {
    return m_spark;
  }

  /**
   * Attempt to apply parameter and check if specified parameter is set correctly
   * @param parameterSetter Method to set desired parameter
   * @param parameterCheckSupplier Method to check for parameter in question
   * @return {@link REVLibError#kOk} if successful
   */
  REVLibError applyParameter(Supplier<REVLibError> parameterSetter, BooleanSupplier parameterCheckSupplier, String errorMessage) {
    if (RobotBase.isSimulation()) return parameterSetter.get();

    REVLibError status = REVLibError.kError;
    for (int i = 0; i < MAX_ATTEMPTS; i++) {
      status = parameterSetter.get();
      if (parameterCheckSupplier.getAsBoolean() && status == REVLibError.kOk) break;
    }

    checkStatus(status, errorMessage);
    return status;
  }

  /**
   * Check status and print error message if necessary
   * @param status Status to check
   * @param errorMessage Error message to print
   */
  private void checkStatus(REVLibError status, String errorMessage) {
    if (status != REVLibError.kOk)
      System.out.println(String.join(" ", m_id.name, errorMessage, "-", status.toString()));
  }

  /**
   * Log output values
   * @param value Value that was set
   * @param ctrl Control mode that was used
   */
  private void logOutputs(double value, ControlType ctrl) {
    Logger.recordOutput(m_id.name + VALUE_LOG_ENTRY, value);
    Logger.recordOutput(m_id.name + MODE_LOG_ENTRY, ctrl.toString());
    Logger.recordOutput(m_id.name + CURRENT_LOG_ENTRY, m_spark.getOutputCurrent());
    Logger.recordOutput(m_id.name + MOTION_LOG_ENTRY, m_isSmoothMotionEnabled);
  }

  /**
   * Get the position of the motor encoder. This returns the native units of 'rotations' by default, and can
   * be changed by a scale factor using setPositionConversionFactor().
   * @return Number of rotations of the motor
   */
  private double getEncoderPosition() {
    return m_spark.getEncoder().getPosition();
  }

  /**
   * Get the velocity of the motor encoder. This returns the native units of 'RPM' by default, and can be
   * changed by a scale factor using setVelocityConversionFactor().
   * @return Number the RPM of the motor
   */
  private double getEncoderVelocity() {
    return m_spark.getEncoder().getVelocity();
  }

  /**
   * Returns an object for interfacing with a connected analog sensor.
   * @return An object for interfacing with a connected analog sensor
   */
  private SparkMaxAnalogSensor getAnalog() {
    return m_spark.getAnalog(SparkMaxAnalogSensor.Mode.kAbsolute);
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
    return m_spark.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
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
   * Get the velocity of the absolute encoder. This returns the native units of 'RPM' by default, and can be
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
  private SparkMaxLimitSwitch getForwardLimitSwitch() {
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
  private SparkMaxLimitSwitch getReverseLimitSwitch() {
    return m_spark.getReverseLimitSwitch(m_limitSwitchType);
  }

  /**
   * Update sensor input readings
   */
  private void updateInputs() {
    m_inputs.analogPosition = getAnalogPosition();
    m_inputs.analogVelocity = getAnalogVelocity();
    m_inputs.absoluteEncoderPosition = getAbsoluteEncoderPosition();
    m_inputs.absoluteEncoderVelocity = getAbsoluteEncoderVelocity();
    m_inputs.forwardLimitSwitch = getForwardLimitSwitch().isPressed();
    m_inputs.reverseLimitSwitch = getReverseLimitSwitch().isPressed();

    if (getMotorType() == MotorType.kBrushed) return;
    m_inputs.encoderPosition = getEncoderPosition();
    m_inputs.encoderVelocity = getEncoderVelocity();
  }

  /**
   * Handle smooth motion
   */
  private void handleSmoothMotion() {
    if (!m_isSmoothMotionEnabled) return;

    m_smoothMotionState = m_motionProfile.calculate(GlobalConstants.ROBOT_LOOP_PERIOD, m_currentStateSupplier.get(), m_smoothMotionState);
    set(
      m_smoothMotionState.position,
      ControlType.kPosition,
      m_feedforwardSupplier.apply(m_smoothMotionState),
      SparkMaxPIDController.ArbFFUnits.kVoltage
    );

    m_isSmoothMotionEnabled = !isSmoothMotionFinished();
  }

  /**
   * Writes all settings to flash
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError burnFlash() {
    if (RobotBase.isSimulation()) return REVLibError.kOk;

    Timer.delay(0.5);
    REVLibError status = m_spark.burnFlash();
    Timer.delay(0.5);

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
      "Restore factory defaults failure"
    );

    return status;
  }

  /**
   * Add motor to simulation
   * @param motor Motor that is connected to this Spark Max
   */
  public void addToSimulation(DCMotor motor) {
    REVPhysicsSim.getInstance().addSparkMax(m_spark, motor);
  }

  /**
   * Call this method periodically
   */
  @Override
  public void periodic() {
    updateInputs();
    Logger.processInputs(m_id.name, m_inputs);

    handleSmoothMotion();
  }

  /**
   * Get latest sensor input data
   * @return Latest sensor data
   */
  @Override
  public SparkMaxInputsAutoLogged getInputs() {
    return m_inputs;
  }

  /**
   * Get device ID
   * @return Device ID
   */
  public ID getID() {
    return m_id;
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
      Math.abs(m_currentStateSupplier.get().position - m_desiredState.position) < m_config.getTolerance()
    );
  }

  /**
   * Initializes Spark Max PID
   * @param config Configuration to apply
   * @param feedbackSensor Feedback device to use for Spark PID
   * @param forwardLimitSwitch Enable forward limit switch
   * @param reverseLimitSwitch Enable reverse limit switch
   */
  public void initializeSparkPID(SparkPIDConfig config, FeedbackSensor feedbackSensor,
                                 boolean forwardLimitSwitch, boolean reverseLimitSwitch) {
    if (getMotorType() == MotorType.kBrushed && feedbackSensor == FeedbackSensor.NEO_ENCODER)
      throw new IllegalArgumentException("NEO encoder cannot be used in brushed motor mode!");

    m_config = config;
    m_feedbackSensor = feedbackSensor;
    m_smoothMotionFinishedDebouncer = new Debouncer(SMOOTH_MOTION_DEBOUNCE_TIME);
    m_desiredState = new TrapezoidProfile.State();
    m_smoothMotionState = new TrapezoidProfile.State();
    m_feedforwardSupplier = (motionProfileState) -> 0.0;

    MotorFeedbackSensor selectedSensor;
    switch (m_feedbackSensor) {
      case ANALOG:
        selectedSensor = m_spark.getAnalog(SparkMaxAnalogSensor.Mode.kAbsolute);
        m_currentStateSupplier = () -> new TrapezoidProfile.State(getInputs().analogPosition, getInputs().analogVelocity);
        break;
      case THROUGH_BORE_ENCODER:
        selectedSensor = m_spark.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        m_currentStateSupplier = () -> new TrapezoidProfile.State(getInputs().absoluteEncoderPosition, getInputs().absoluteEncoderVelocity);
        break;
      case NEO_ENCODER:
      default:
        selectedSensor = m_spark.getEncoder();
        m_currentStateSupplier = () -> new TrapezoidProfile.State(getInputs().encoderPosition, getInputs().encoderVelocity);
        break;
    }

    // Configure feedback sensor and set sensor phase
    try {
      m_spark.getPIDController().setFeedbackDevice(selectedSensor);
      selectedSensor.setInverted(m_config.getSensorPhase());
    } catch (IllegalArgumentException e) {}

    // Configure forward and reverse soft limits
    if (config.isSoftLimitEnabled()) {
      setForwardSoftLimit(config.getUpperLimit());
      enableForwardSoftLimit();
      setReverseSoftLimit(config.getLowerLimit());
      enableReverseSoftLimit();
    }

    // Configure forward and reverse limit switches if required, and disable soft limit
    if (forwardLimitSwitch) {
      enableForwardLimitSwitch();
      disableForwardSoftLimit();
    }
    if (reverseLimitSwitch) {
      enableReverseLimitSwitch();
      disableReverseSoftLimit();
    }

    // Invert motor if required
    setInverted(config.getInverted());

    // Configure PID values
    setP(config.getP());
    setI(config.getI());
    setD(config.getD());
    setF(config.getF());
    setIzone(config.getI() != 0.0 ? config.getTolerance() * 2 : 0.0);
  }

  /**
   * Initializes Spark Max PID
   * <p>
   * Calls {@link SparkMax#initializeSparkPID(SparkPIDConfig, FeedbackSensor, boolean, boolean)} with no limit switches
   * @param config Configuration to apply
   * @param feedbackSensor Feedback device to use for Spark PID
   */
  public void initializeSparkPID(SparkPIDConfig config, FeedbackSensor feedbackSensor) {
    initializeSparkPID(config, feedbackSensor, false, false);
  }

  /**
   * Slave Spark Max to another
   * @param master Spark Max to follow
   * @param invert Set slave to output opposite of the master
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError follow(SparkMax master, boolean invert) {
    REVLibError status;
    status = applyParameter(
      () -> m_spark.follow(ExternalFollower.kFollowerSparkMax, master.getID().deviceID, invert),
      () -> m_spark.isFollower(),
      "Set motor master failure"
    );
    return status;
  }

  /**
   * Slave Spark Max to another
   * @param master Spark Max to follow
   */
  public void follow(SparkMax master) {
    follow(master, false);
  }

  /**
   * Common interface for inverting direction of a speed controller.
   *
   * <p>This call has no effect if the controller is a slave. To invert a slave, see the
   * {@link SparkMax#follow(SparkMax, boolean)} method.
   *
   * @param isInverted The state of inversion, true is inverted.
   */
  public void setInverted(boolean isInverted) {
    m_spark.setInverted(isInverted);
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
  public void set(double value, ControlType ctrl, double arbFeedforward, SparkMaxPIDController.ArbFFUnits arbFFUnits) {
    m_spark.getPIDController().setReference(value, ctrl, PID_SLOT, arbFeedforward, arbFFUnits);
    logOutputs(value, ctrl);
  }

  /**
   * Change the limit switch type
   * @param type The desired limit switch type
   */
  public void setLimitSwitchType(SparkMaxLimitSwitch.Type type) {
    m_limitSwitchType = type;
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
        parameterSetter = () -> m_spark.getEncoder().setPositionConversionFactor(factor);
        parameterCheckSupplier = () -> m_spark.getEncoder().getPositionConversionFactor() == factor;
        break;
      case ANALOG:
        parameterSetter = () -> getAnalog().setPositionConversionFactor(factor);
        parameterCheckSupplier = () -> getAnalog().getPositionConversionFactor() == factor;
        break;
      case THROUGH_BORE_ENCODER:
        parameterSetter = () -> getAbsoluteEncoder().setPositionConversionFactor(factor);
        parameterCheckSupplier = () -> getAbsoluteEncoder().getPositionConversionFactor() == factor;
        break;
      default:
        parameterSetter = () -> REVLibError.kOk;
        parameterCheckSupplier = () -> true;
        break;
    }

    status = applyParameter(parameterSetter, parameterCheckSupplier, "Set position conversion factor failure");
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
        parameterSetter = () -> m_spark.getEncoder().setVelocityConversionFactor(factor);
        parameterCheckSupplier = () -> m_spark.getEncoder().getVelocityConversionFactor() == factor;
        break;
      case ANALOG:
        parameterSetter = () -> getAnalog().setVelocityConversionFactor(factor);
        parameterCheckSupplier = () -> getAnalog().getVelocityConversionFactor() == factor;
        break;
      case THROUGH_BORE_ENCODER:
        parameterSetter = () -> getAbsoluteEncoder().setVelocityConversionFactor(factor);
        parameterCheckSupplier = () -> getAnalog().getVelocityConversionFactor() == factor;
        break;
      default:
        parameterSetter = () -> REVLibError.kOk;
        parameterCheckSupplier = () -> true;
        break;
    }

    status = applyParameter(parameterSetter, parameterCheckSupplier, "Set velocity conversion factor failure");
    return status;
  }

  /**
   * Set proportional gain for PIDF controller on Spark Max
   * @param value Value to set
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setP(double value) {
    REVLibError status;
    status = applyParameter(
      () -> m_spark.getPIDController().setP(value),
      () -> m_spark.getPIDController().getP() == value,
      "Set kP failure!"
    );
    return status;
  }

  /**
   * Set integral gain for PIDF controller on Spark Max
   * @param value Value to set
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setI(double value) {
    REVLibError status;
    status = applyParameter(
      () -> m_spark.getPIDController().setI(value),
      () -> m_spark.getPIDController().getI() == value,
      "Set kI failure!"
    );
    return status;
  }

  /**
   * Set derivative gain for PIDF controller on Spark Max
   * @param value Value to set
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setD(double value) {
    REVLibError status;
    status = applyParameter(
      () -> m_spark.getPIDController().setD(value),
      () -> m_spark.getPIDController().getD() == value,
      "Set kD failure!"
    );
    return status;
  }

  /**
   * Set feed-forward gain for PIDF controller on Spark Max
   * @param value Value to set
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setF(double value) {
    REVLibError status;
    status = applyParameter(
      () -> m_spark.getPIDController().setFF(value),
      () -> m_spark.getPIDController().getFF() == value,
      "Set kF failure!"
    );
    return status;
  }

  /**
   * Set integral zone range for PIDF controller on Spark Max
   * <p>
   * This value specifies the range the |error| must be within for the integral constant to take effect
   * @param value Value to set
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setIzone(double value) {
    REVLibError status;
    status = applyParameter(
      () -> m_spark.getPIDController().setIZone(value),
      () -> m_spark.getPIDController().getIZone() == value,
      "Set Izone failure!"
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
    m_smoothMotionState = m_desiredState;
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
   * Reset NEO built-in encoder
   */
  public void resetEncoder() {
    m_spark.getEncoder().setPosition(0.0);
    System.out.println(String.join(" ", m_id.name, "Encoder reset!"));
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
      () -> m_spark.getSoftLimit(SoftLimitDirection.kForward) == limit,
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
      () -> m_spark.getSoftLimit(SoftLimitDirection.kReverse) == limit,
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
      m_spark.getPIDController().getPositionPIDWrappingMinInput() == minInput &&
      m_spark.getPIDController().getPositionPIDWrappingMaxInput() == maxInput;

    status = applyParameter(parameterSetter, parameterCheckSupplier, "Enable position PID wrapping failure");
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
      "Disable position PID wrapping failure"
    );
    return status;
  }

  /**
   * Sets the idle mode setting for the SPARK MAX.
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
    return status;
  }

  /**
   * Sets the current limit in Amps.
   *
   * <p>The motor controller will reduce the controller voltage output to avoid surpassing this
   * limit. This limit is enabled by default and used for brushless only. This limit is highly
   * recommended when using the NEO brushless motor.
   *
   * <p>The NEO Brushless Motor has a low internal resistance, which can mean large current spikes
   * that could be enough to cause damage to the motor and controller. This current limit provides a
   * smarter strategy to deal with high current draws and keep the motor and controller operating in
   * a safe region.
   *
   * @param limit The current limit in Amps.
   */
  public REVLibError setSmartCurrentLimit(int limit) {
    REVLibError status;
    status = applyParameter(
      () -> m_spark.setSmartCurrentLimit(limit),
      () -> true,
      "Set current limit failure"
    );
    return status;
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
   * Closes the Spark Max controller
   */
  @Override
  public void close() {
    m_spark.close();
  }
}
