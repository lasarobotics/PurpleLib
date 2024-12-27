// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.hardware.revrobotics;

import java.util.LinkedHashSet;
import java.util.function.Supplier;

import org.lasarobotics.hardware.LoggableHardware;
import org.lasarobotics.hardware.PurpleManager;
import org.lasarobotics.utils.GlobalConstants;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.Warnings;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkHelpers;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkBaseConfigAccessor;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkFlexConfigAccessor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkMaxConfigAccessor;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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

  /** Spark output */
  protected class SparkOutput {
    public final double value;
    public final ControlType ctrlType;
    public final double arbFeedforward;
    public final ArbFFUnits arbFFUnits;

    public SparkOutput(double value, ControlType ctrlType, double arbFeedforward, ArbFFUnits arbFFUnits) {
      this.value = value;
      this.ctrlType = ctrlType;
      this.arbFeedforward = arbFeedforward;
      this.arbFFUnits = arbFFUnits;
    }
  }

  /**
   * Spark sensor inputs
   */
  @AutoLog
  public static class SparkInputs {
    public double timestamp = 0.0;
    public double encoderPosition = 0.0;
    public double encoderVelocity = 0.0;
    public double analogPosition = 0.0;
    public double analogVelocity = 0.0;
    public double absoluteEncoderPosition = 0.0;
    public double absoluteEncoderVelocity = 0.0;
    public boolean forwardLimitSwitch = false;
    public boolean reverseLimitSwitch = false;
  }

  private static final String LOG_TAG = "Spark";
  private static final int CAN_TIMEOUT_MS = 50;
  private static final int PID_SLOT = 0;
  private static final int MAX_ATTEMPTS = 5;
  private static final int SPARK_MAX_MEASUREMENT_PERIOD = 16;
  private static final int SPARK_FLEX_MEASUREMENT_PERIOD = 32;
  private static final int SPARK_MAX_AVERAGE_DEPTH = 2;
  private static final int SPARK_FLEX_AVERAGE_DEPTH = 8;
  private static final double MAX_VOLTAGE = 12.0;
  private static final double APPLY_PARAMETER_WAIT_TIME = 0.1;
  private static final String VALUE_LOG_ENTRY = "/OutputValue";
  private static final String MODE_LOG_ENTRY = "/OutputMode";
  private static final String ARB_FF_LOG_ENTRY = "/ArbitraryFF";
  private static final String ARB_FF_UNITS_LOG_ENTRY = "/ArbitraryFFUnits";
  private static final String IDLE_MODE_LOG_ENTRY = "/IdleMode";
  private static final String INPUT_CURRENT_LOG_ENTRY = "/InputCurrent";
  private static final String OUTPUT_CURRENT_LOG_ENTRY = "/OutputCurrent";
  private static final String HEALTH_STATUS_LOG_ENTRY = "/IsHealthy";
  private static final String TEMPERATURE_LOG_ENTRY = "/Temperature";

  private SparkBase m_spark;
  private SparkSim m_sparkSim;
  private SparkBaseConfigAccessor m_configAccessor;

  private ID m_id;
  private MotorKind m_kind;
  private Notifier m_inputThread;

  private LinkedHashSet<Supplier<REVLibError>> m_configChain;
  private Runnable m_invertedRunner;

  private Trigger m_forwardLimitSwitchTrigger;
  private Trigger m_reverseLimitSwitchTrigger;
  private double m_forwardLimitSwitchResetValue = 1;
  private double m_reverseLimitSwitchResetValue = 0;

  private volatile SparkOutput m_output;
  private volatile SparkInputsAutoLogged m_inputs;

  /**
   * Create a Spark that is unit-testing friendly with built-in logging
   * @param id Spark ID
   * @param kind The kind of motor connected to the controller
   * @param updateRate Update rate of Spark inputs
   */
  public Spark(ID id, MotorKind kind, Frequency updateRate) {
    if (kind.equals(MotorKind.NEO_VORTEX)) {
      this.m_spark = new SparkFlex(id.deviceID, kind.type);
      this.m_configAccessor = new SparkFlexConfigAccessor(SparkHelpers.getSparkHandle(m_spark));
    } else {
      this.m_spark = new SparkMax(id.deviceID, kind.type);
      this.m_configAccessor = new SparkMaxConfigAccessor(SparkHelpers.getSparkHandle(m_spark));
    }
    this.m_sparkSim = new SparkSim(m_spark, kind.motor);
    this.m_id = id;
    this.m_kind = kind;
    this.m_output = new SparkOutput(0.0, ControlType.kDutyCycle, 0.0, ArbFFUnits.kVoltage);
    this.m_inputs = new SparkInputsAutoLogged();
    this.m_inputThread = new Notifier(this::updateInputs);
    this.m_configChain = new LinkedHashSet<>();
    this.m_invertedRunner = () -> {};
    this.m_forwardLimitSwitchTrigger = new Trigger(() -> getInputs().forwardLimitSwitch);
    this.m_reverseLimitSwitchTrigger = new Trigger(() -> getInputs().reverseLimitSwitch);

    // Set CAN timeout
    m_spark.setCANTimeout(CAN_TIMEOUT_MS);

    // Restore defaults
    restoreFactoryDefaults();

    // Set status frame rates
    // setPeriodicFrameRate(PeriodicFrame.kStatus0, DEFAULT_STATUS_FRAME_PERIOD);
    // setPeriodicFrameRate(PeriodicFrame.kStatus1, DEFAULT_STATUS_FRAME_PERIOD);
    // setPeriodicFrameRate(PeriodicFrame.kStatus2, DEFAULT_STATUS_FRAME_PERIOD);
    // setPeriodicFrameRate(PeriodicFrame.kStatus3, DEFAULT_STATUS_FRAME_PERIOD);
    // setPeriodicFrameRate(PeriodicFrame.kStatus4, DEFAULT_STATUS_FRAME_PERIOD);
    // setPeriodicFrameRate(PeriodicFrame.kStatus5, DEFAULT_STATUS_FRAME_PERIOD);
    // setPeriodicFrameRate(PeriodicFrame.kStatus6, DEFAULT_STATUS_FRAME_PERIOD);

    // Update inputs on init
    stopMotor();
    updateInputs();
    periodic();

    // Register device with monitor and manager
    SparkMonitor.getInstance().add(this);
    PurpleManager.add(this);

    // Start sensor input thread
    m_inputThread.setName(m_id.name);
    if (!Logger.hasReplaySource()) m_inputThread.startPeriodic(updateRate.asPeriod().in(Units.Seconds));
  }

  /**
   * Create a Spark that is unit-testing friendly with built-in logging
   * <p>
   * Defaults to normally-open limit switches, 50 Hz input thread frequency
   * @param id Spark ID
   * @param kind The kind of motor connected to the controller
   */
  public Spark(ID id, MotorKind kind) {
    this(id, kind, GlobalConstants.ROBOT_LOOP_HZ);
  }

  /**
   * Get internal Spark PID controller
   * @return
   */
  SparkClosedLoopController getPIDController() {
    return m_spark.getClosedLoopController();
  }

  /**
   * Get latest output
   * @return Latest set output sent to Spark
   */
  SparkOutput getLatestOutput() {
    return m_output;
  }

  /**
   * Attempt to apply config and check if specified config is set correctly
   * @param parameterSetter Method to set desired parameter
   * @param parameterCheckSupplier Method to check for parameter in question
   * @param errorMessage Message to show in case of failure
   * @return {@link REVLibError#kOk} if successful
   */
  private REVLibError applyConfig(Supplier<REVLibError> parameterSetter) {
    if (RobotBase.isSimulation()) return parameterSetter.get();

    REVLibError status = REVLibError.kError;
    for (int i = 0; i < MAX_ATTEMPTS; i++) {
      status = parameterSetter.get();

      Timer.delay(APPLY_PARAMETER_WAIT_TIME);
    }

    // Check if status is okay, add parameter to chain if successful
    if (status != REVLibError.kOk)
      org.tinylog.Logger.tag(LOG_TAG).error(String.join(" ", m_id.name, status.toString(), "-", status.toString()));
    else m_configChain.add(parameterSetter);

    return status;
  }

  /**
   * Log output values
   * @param value Value that was set
   * @param ctrl Control mode that was used
   */
  private void logOutputs(double value, ControlType ctrl, double arbFeedforward, ArbFFUnits arbFFUnits) {
    Logger.recordOutput(m_id.name + VALUE_LOG_ENTRY, value);
    Logger.recordOutput(m_id.name + MODE_LOG_ENTRY, ctrl.toString());
    Logger.recordOutput(m_id.name + ARB_FF_LOG_ENTRY, arbFeedforward);
    Logger.recordOutput(m_id.name + ARB_FF_UNITS_LOG_ENTRY, arbFFUnits.toString());
    m_output = new SparkOutput(value, ctrl, arbFeedforward, arbFFUnits);
  }

  /**
   * Get the position of the motor encoder. This returns the native units of 'rotations' by default, and can
   * be changed by a scale factor using setPositionConversionFactor().
   * @return Number of rotations of the motor
   */
  private double getEncoderPosition() {
    return (RobotBase.isReal()) ? m_spark.getEncoder().getPosition() : m_sparkSim.getPosition();
  }

  /**
   * Get the velocity of the motor encoder. This returns the native units of 'RPM' by default, and can be
   * changed by a scale factor using setVelocityConversionFactor().
   * @return Number the RPM of the motor
   */
  private double getEncoderVelocity() {
    return (RobotBase.isReal()) ? m_spark.getEncoder().getVelocity() : m_sparkSim.getVelocity();
  }

  /**
   * Get position of the analog sensor. This returns the native units 'volt' by default, and can
   * be changed by a scale factor using setPositionConversionFactor().
   * @return Volts on the sensor
   */
  private double getAnalogPosition() {
    return (RobotBase.isReal()) ? m_spark.getAnalog().getPosition() : m_sparkSim.getAnalogSensorSim().getPosition();
  }

  /**
   * Get the velocity of the analog sensor. This returns the native units of 'volts per second' by default, and can be
   * changed by a scale factor using setVelocityConversionFactor().
   * @return Volts per second on the sensor
   */
  private double getAnalogVelocity() {
    return (RobotBase.isReal()) ? m_spark.getAnalog().getVelocity() : m_sparkSim.getAnalogSensorSim().getVelocity();
  }

  /**
   * Get position of the absolute encoder. This returns the native units 'rotations' by default, and can
   * be changed by a scale factor using setPositionConversionFactor().
   * @return Number of rotations of the motor
   */
  private double getAbsoluteEncoderPosition() {
    return (RobotBase.isReal()) ? m_spark.getAbsoluteEncoder().getPosition() : m_sparkSim.getAbsoluteEncoderSim().getPosition();
  }

  /**
   * Get the velocity of the absolute encoder. This returns the n`ative units of 'RPM' by default, and can be
   * changed by a scale factor using setVelocityConversionFactor().
   * @return Number the RPM of the motor
   */
  private double getAbsoluteEncoderVelocity() {
    return (RobotBase.isReal()) ? m_spark.getAbsoluteEncoder().getVelocity() : m_sparkSim.getAbsoluteEncoderSim().getVelocity();
  }

  /**
   * Get if forward limit switch is activated
   * @return True if activated
   */
  private boolean getForwardLimitSwitch() {
    return (RobotBase.isReal()) ? m_spark.getForwardLimitSwitch().isPressed() : m_sparkSim.getForwardLimitSwitchSim().getPressed();
  }

  /**
   * Get if reverse limit switch is activated
   * @return True if activated
   */
  private boolean getReverseLimitSwitch() {
    return (RobotBase.isReal()) ? m_spark.getReverseLimitSwitch().isPressed() : m_sparkSim.getReverseLimitSwitchSim().getPressed();
  }

  /**
   * Update sensor input readings
   */
  private void updateInputs() {
    synchronized (m_inputs) {
      // Get sensor inputs
      m_inputs.timestamp = Logger.getRealTimestamp();
      m_inputs.analogPosition = getAnalogPosition();
      m_inputs.analogVelocity = getAnalogVelocity();
      m_inputs.absoluteEncoderPosition = getAbsoluteEncoderPosition();
      m_inputs.absoluteEncoderVelocity = getAbsoluteEncoderVelocity();
      m_inputs.forwardLimitSwitch = getForwardLimitSwitch();
      m_inputs.reverseLimitSwitch = getReverseLimitSwitch();

      // Get motor encoder
      if (!getMotorType().equals(MotorType.kBrushed)) {
        m_inputs.encoderPosition = getEncoderPosition();
        m_inputs.encoderVelocity = getEncoderVelocity();
      }
    }
  }

  @Override
  protected void periodic() {
    synchronized (m_inputs) { Logger.processInputs(m_id.name, m_inputs); }

    Logger.recordOutput(m_id.name + INPUT_CURRENT_LOG_ENTRY, getInputCurrent());
    Logger.recordOutput(m_id.name + OUTPUT_CURRENT_LOG_ENTRY, getOutputCurrent());
    Logger.recordOutput(m_id.name + HEALTH_STATUS_LOG_ENTRY, isHealthy());

    if (getMotorType() == MotorType.kBrushed) return;
    Logger.recordOutput(m_id.name + TEMPERATURE_LOG_ENTRY, m_spark.getMotorTemperature());
  }

  @Override
  public SparkInputsAutoLogged getInputs() {
    synchronized (m_inputs) { return m_inputs; }
  }

  @Override
  public boolean isHealthy() {
    return !m_spark.getWarnings().hasReset;
  }

  @Override
  public boolean reinit() {
    boolean success = true;
    if (m_invertedRunner != null) m_invertedRunner.run();
    for (var parameterApplier : m_configChain) {
      var status = parameterApplier.get();
      success &= status.equals(REVLibError.kOk);
    }
    return success;
  }

  @Override
  public int getMaxRetries() {
    return MAX_ATTEMPTS;
  }

  /**
   * Set the configuration for the SPARK.
   *
   * <p>If {@code resetMode} is {@link ResetMode#kResetSafeParameters}, this method will reset safe
   * writable parameters to their default values before setting the given configuration. The
   * following parameters will not be reset by this action: CAN ID, Motor Type, Idle Mode, PWM Input
   * Deadband, and Duty Cycle Offset.
   *
   * <p>If {@code persistMode} is {@link PersistMode#kPersistParameters}, this method will save all
   * parameters to the SPARK's non-volatile memory after setting the given configuration. This will
   * allow parameters to persist across power cycles.
   *
   * @param config The desired SPARK configuration
   * @param resetMode Whether to reset safe parameters before setting the configuration
   * @param persistMode Whether to persist the parameters after setting the configuration
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError configure(SparkBaseConfig config, SparkBase.ResetMode resetMode, SparkBase.PersistMode persistMode) {
    if (SparkHelpers.getSparkModel(m_spark).equals(SparkHelpers.SparkModel.SparkFlex)) {
      config.encoder.quadratureAverageDepth(SPARK_FLEX_AVERAGE_DEPTH);
      config.encoder.quadratureMeasurementPeriod(SPARK_FLEX_MEASUREMENT_PERIOD);
    } else {
      config.encoder.uvwAverageDepth(SPARK_MAX_AVERAGE_DEPTH);
      config.encoder.uvwMeasurementPeriod(SPARK_MAX_MEASUREMENT_PERIOD);
    }
    config.voltageCompensation(MAX_VOLTAGE);

    var status = m_spark.configure(config, resetMode, persistMode);

     // Log idle mode
     Logger.recordOutput(m_id.name + IDLE_MODE_LOG_ENTRY, m_configAccessor.getIdleMode() == IdleMode.kCoast);

     return status;
  }

  /**
   * Get config accessor
   * @return Object to access Spark config
   */
  public SparkBaseConfigAccessor getConfigAccessor() {
    return m_configAccessor;
  }

  /**
   * Restore motor controller parameters to factory defaults until the next controller reboot
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError restoreFactoryDefaults() {
    REVLibError status;
    SparkBaseConfig config;
    if (SparkHelpers.getSparkModel(m_spark).equals(SparkHelpers.SparkModel.SparkFlex)) {
      config = new SparkFlexConfig();
    } else config = new SparkMaxConfig();

    status = applyConfig(
      () -> m_spark.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters)
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
   * Get simulated Spark
   * @return Simulated Spark object, for simulation purposes
   */
  public SparkSim getSim() {
    return m_sparkSim;
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
   * @param ctrl Desired control type
   */
  public void set(double value, ControlType ctrl) {
    set(value, ctrl, 0.0, ArbFFUnits.kVoltage);
  }

  /**
   * Set motor output value with arbitrary feed forward
   * @param value Value to set
   * @param ctrl Desired control type
   * @param arbFeedforward Feed forward value
   * @param arbFFUnits Feed forward units
   */
  public void set(double value, ControlType ctrl, double arbFeedforward, SparkClosedLoopController.ArbFFUnits arbFFUnits) {
    m_spark.getClosedLoopController().setReference(value, ctrl, PID_SLOT, arbFeedforward, arbFFUnits);
    logOutputs(value, ctrl, arbFeedforward, arbFFUnits);
  }

  /**
   * Reset NEO built-in encoder to desired value
   * @param value Desired encoder value
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError resetEncoder(double value) {
    REVLibError status;
    status = applyConfig(() -> m_spark.getEncoder().setPosition(value));

    if (status.equals(REVLibError.kOk)) System.out.println(String.join(" ", m_id.name, "Encoder set to", String.valueOf(value), "!"));

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
   * Sets whether the NEO encoder should be reset when the forward limit switch is hit
   * The value to set it to can be configured using {@link Spark#setForwardLimitSwitchResetValue(double)}
   * @param shouldReset Whether the encoder should be reset
   */
  public void setForwardLimitSwitchShouldReset(boolean shouldReset) {
    m_forwardLimitSwitchTrigger.onTrue(shouldReset ?
      Commands.runOnce(() -> resetEncoder(m_forwardLimitSwitchResetValue)) :
      Commands.none()
    );
  }

  /**
   * Change what value the encoder should be reset to once the forward limit switch is hit
   * @param value Desired encoder value
   */
  public void setForwardLimitSwitchResetValue(double value) {
    m_forwardLimitSwitchResetValue = value;
  }

  /**
   * Sets whether the NEO encoder should be reset when the reverse limit switch is hit
   * The value to set it to can be configured using {@link Spark#setReverseLimitSwitchResetValue(double)}
   * @param shouldReset Whether the encoder should be reset
   */
  public void setReverseLimitSwitchShouldReset(boolean shouldReset) {
    m_reverseLimitSwitchTrigger.onTrue(shouldReset ?
      Commands.runOnce(() -> resetEncoder(m_reverseLimitSwitchResetValue)) :
      Commands.none()
    );
  }

  /**
   * Change what value the encoder should be reset to once the reverse limit switch is hit
   * @param value Desired encoder value
   */
  public void setReverseLimitSwitchResetValue(double value) {
    m_reverseLimitSwitchResetValue = value;
  }

  /**
   * Disable forward limit switch
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError disableForwardLimitSwitch() {
    if (getKind().equals(MotorKind.NEO_VORTEX)) {
      return configure(
        new SparkFlexConfig().apply(new LimitSwitchConfig().forwardLimitSwitchEnabled(false)),
        SparkBase.ResetMode.kNoResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters
      );
    } else {
      return configure(
        new SparkMaxConfig().apply(new LimitSwitchConfig().forwardLimitSwitchEnabled(false)),
        SparkBase.ResetMode.kNoResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters
      );
    }
  }

  /**
   * Enable forward limit switch
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError enableForwardLimitSwitch() {
    if (getKind().equals(MotorKind.NEO_VORTEX)) {
      return configure(
        new SparkFlexConfig().apply(new LimitSwitchConfig().forwardLimitSwitchEnabled(true)),
        SparkBase.ResetMode.kNoResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters
      );
    } else {
      return configure(
        new SparkMaxConfig().apply(new LimitSwitchConfig().forwardLimitSwitchEnabled(true)),
        SparkBase.ResetMode.kNoResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters
      );
    }
  }

  /**
   * Disable reverse limit switch
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError disableReverseLimitSwitch() {
    if (getKind().equals(MotorKind.NEO_VORTEX)) {
      return configure(
        new SparkFlexConfig().apply(new LimitSwitchConfig().reverseLimitSwitchEnabled(false)),
        SparkBase.ResetMode.kNoResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters
      );
    } else {
      return configure(
        new SparkMaxConfig().apply(new LimitSwitchConfig().reverseLimitSwitchEnabled(false)),
        SparkBase.ResetMode.kNoResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters
      );
    }
  }

  /**
   * Enable reverse limit switch
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError enableReverseLimitSwitch() {
    if (getKind().equals(MotorKind.NEO_VORTEX)) {
      return configure(
        new SparkFlexConfig().apply(new LimitSwitchConfig().reverseLimitSwitchEnabled(true)),
        SparkBase.ResetMode.kNoResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters
      );
    } else {
      return configure(
        new SparkMaxConfig().apply(new LimitSwitchConfig().reverseLimitSwitchEnabled(true)),
        SparkBase.ResetMode.kNoResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters
      );
    }
  }

  /**
   * Sets the idle mode setting for the Spark
   * @param mode Idle mode (coast or brake).
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setIdleMode(IdleMode mode) {
    if (getKind().equals(MotorKind.NEO_VORTEX)) {
      return configure(
        new SparkFlexConfig().idleMode(mode),
        SparkBase.ResetMode.kNoResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters
      );
    } else {
      return configure(
        new SparkMaxConfig().idleMode(mode),
        SparkBase.ResetMode.kNoResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters
      );
    }
  }

  /**
   * Spark approximate input current
   * @return
   */
  public Current getInputCurrent() {
    return getOutputCurrent().times(m_spark.getAppliedOutput());
  }

  /**
   * Spark output current
   * @return Output current
   */
  public Current getOutputCurrent() {
  return Units.Amps.of(m_spark.getOutputCurrent());
  }

  /**
   * Get applied output of Spark
   * @return The Spark's applied output duty cycle.
   */
  public double getAppliedOutput() {
    return m_spark.getAppliedOutput();
  }

  /**
   * Get current sticky faults
   * @return All sticky fault bits as a short
   */
  public Faults getStickyFaults() {
    return m_spark.getStickyFaults();
  }

  /**
   * Get the active warnings that are currently present on the SPARK. Warnings are non-fatal errors.
   * @return A struct with each warning and their active value
   */
  public Warnings getWarnings() {
    return m_spark.getWarnings();
  }

  /**
   * Stops motor movement. Motor can be moved again by calling set without having to re-enable the
   * motor.
   */
  public void stopMotor() {
    m_spark.stopMotor();
    logOutputs(0.0, ControlType.kDutyCycle, 0.0, ArbFFUnits.kVoltage);
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
