// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.hardware.ctre;

import org.lasarobotics.hardware.LoggableHardware;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;

/** TalonSRX */
public class TalonSRX implements LoggableHardware, AutoCloseable {
  /** TalonSRX ID */
  public static class ID {
    public final String name;
    public final int deviceID;

    /**
     * TalonSRX ID
     * @param name Device name for logging
     * @param deviceID CAN ID
     */
    public ID(String name, int deviceID) {
      this.name = name;
      this.deviceID = deviceID;
    }
  }

  /**
   * TalonSRX sensor inputs
   */
  @AutoLog
  public static class TalonSRXInputs {
    public double selectedSensorPosition = 0.0;
    public double selectedSensorVelocity = 0.0;
  }

  private static final double MAX_VOLTAGE = 12.0;
  private static final double MOTOR_DEADBAND = 0.01;
  private static final int PID_SLOT = 0;

  private static final String VALUE_LOG_ENTRY = "/OutputValue";
  private static final String MODE_LOG_ENTRY = "/OutputMode";
  private static final String CURRENT_LOG_ENTRY = "/Current";

  private com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX m_talon;

  private ID m_id;
  private TalonSRXInputsAutoLogged m_inputs;

  private TalonPIDConfig m_config;

  /**
   * Create a TalonSRX object with built-in logging
   * @param id TalonSRX ID
   */
  public TalonSRX(TalonSRX.ID id)  {
    this.m_id = id;
    this.m_talon = new com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX(id.deviceID);
    this.m_inputs = new TalonSRXInputsAutoLogged();
  }

  /**
   * Log output values
   * @param value Value that was set
   * @param mode The output mode to apply
   */
  private void logOutputs(ControlMode mode, double value) {
    Logger.recordOutput(m_id.name + VALUE_LOG_ENTRY, value);
    Logger.recordOutput(m_id.name + MODE_LOG_ENTRY, mode.toString());
    Logger.recordOutput(m_id.name + CURRENT_LOG_ENTRY, m_talon.getStatorCurrent());
  }

  /**
   * Get the selected sensor position (in raw sensor units).
   *
   * @return Position of selected sensor (in raw sensor units).
   */
  private double getSelectedSensorPosition() {
    return m_talon.getSelectedSensorPosition();
  }

  /**
   * Get the selected sensor velocity.
   *
   * @return selected sensor (in raw sensor units) per 100ms.
   * See Phoenix-Documentation for how to interpret.
   */
  private double getSelectedSensorVelocity() {
    return m_talon.getSelectedSensorVelocity();
  }

  /**
   * Update sensor input readings
   */
  private void updateInputs() {
    m_inputs.selectedSensorPosition = getSelectedSensorPosition();
    m_inputs.selectedSensorVelocity = getSelectedSensorVelocity();
  }

  @Override
  public void periodic() {
    updateInputs();
    Logger.processInputs(m_id.name, m_inputs);
  }

  /**
   * Get latest sensor input data
   * @return Latest sensor data
   */
  @Override
  public LoggableInputs getInputs() {
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
   * Initializes Talon PID and MotionMagic parameters
   * @param config Configuration to apply
   * @param feedbackDevice Feedback device to use for Talon PID
   * @param forwardLimitSwitch Enable forward limit switch
   * @param reverseLimitSwitch Enable reverse limit switch
   */
  public void initializeTalonPID(TalonPIDConfig config, FeedbackDevice feedbackDevice,
                                 boolean forwardLimitSwitch, boolean reverseLimitSwitch) {
    m_config = config;

    // Reset Talon to default
    m_talon.configFactoryDefault();

    // Configure feedback sensor
    m_talon.configSelectedFeedbackSensor(feedbackDevice);

    // Configure forward and reverse soft limits
    if (m_config.getSoftLimitsEnabled()) {
      m_talon.configForwardSoftLimitThreshold(m_config.getUpperLimit());
      m_talon.configForwardSoftLimitEnable(true);
      m_talon.configReverseSoftLimitThreshold(m_config.getLowerLimit());
      m_talon.configReverseSoftLimitEnable(true);
    }

    // Configure forward and reverse limit switches if required
    if (forwardLimitSwitch) {
      m_talon.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    }
    if (reverseLimitSwitch) {
      m_talon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    }

    // Set sensor phase and invert motor if required
    m_talon.setSensorPhase(config.getSensorPhase());
    m_talon.setInverted(config.getInvertMotor());

    // Configure PID values
    m_talon.config_kP(PID_SLOT, m_config.getkP());
    m_talon.config_kI(PID_SLOT, m_config.getkI());
    m_talon.config_kD(PID_SLOT, m_config.getkD());
    m_talon.config_kF(PID_SLOT, m_config.getkF());
    m_talon.configAllowableClosedloopError(PID_SLOT, m_config.getTolerance());
    m_talon.configClosedLoopPeakOutput(PID_SLOT, 1.0);
    m_talon.config_IntegralZone(PID_SLOT, m_config.getTolerance() * 2);

    // Configure motor deadband
    m_talon.configNeutralDeadband(MOTOR_DEADBAND);

    // Enable voltage compensation
    m_talon.configVoltageCompSaturation(MAX_VOLTAGE);
    m_talon.enableVoltageCompensation(true);

    // Configure MotionMagic values
    if (m_config.getMotionMagic()) {
      m_talon.configMotionCruiseVelocity(m_config.rpmToTicksPer100ms(m_config.getVelocityRPM()));
      m_talon.configMotionAcceleration(m_config.rpmToTicksPer100ms(m_config.getAccelerationRPMPerSec()));
      m_talon.configMotionSCurveStrength(m_config.getMotionSmoothing());
    }
  }

  /**
   * Initializes Talon PID and MotionMagic parameters
   * <p>
   * Calls {@link TalonSRX#initializeTalonPID(TalonPIDConfig, FeedbackDevice, boolean, boolean)} with no limit switches
   * @param config Configuration to apply
   * @param feedbackDevice Feedback device to use for Talon PID
   */
  public void initializeTalonPID(TalonPIDConfig config, FeedbackDevice feedbackDevice) {
    initializeTalonPID(config, feedbackDevice, false, false);
  }

  /**
   * Common interface for setting the speed of a simple speed controller.
   *
   * @param speed The speed to set. Value should be between -1.0 and 1.0.
   *                   Value is also saved for Get().
   */
  public void set(double speed) {
    set(ControlMode.PercentOutput, speed);
    logOutputs(ControlMode.PercentOutput, speed);
  }

  /**
   * Sets the appropriate output on the talon, depending on the mode.
   * @param mode The output mode to apply.
   * In PercentOutput, the output is between -1.0 and 1.0, with 0.0 as stopped.
   * In Current mode, output value is in amperes.
   * In Velocity mode, output value is in position change / 100ms.
   * In Position mode, output value is in encoder ticks or an analog value,
   *   depending on the sensor.
   * In Follower mode, the output value is the integer device ID of the talon to
   * duplicate.
   *
   * @param value The setpoint value, as described above.
   */
  public void set(ControlMode mode, double value) {
    m_talon.set(mode, value);
    logOutputs(mode, value);
  }

  /**
   * Common interface for inverting direction of a speed controller.
   *
   * @param isInverted The state of inversion, true is inverted.
   */
  public void setInverted(boolean isInverted) {
    m_talon.setInverted(isInverted);
  }

  /**
   * Sets the voltage output of the MotorController.  Compensates for the current bus
   * voltage to ensure that the desired voltage is output even if the battery voltage is below
   * 12V - highly useful when the voltage outputs are "meaningful" (e.g. they come from a
   * feedforward calculation).
   *
   * <p>NOTE: This function *must* be called regularly in order for voltage compensation to work
   * properly - unlike the ordinary set function, it is not "set it and forget it."
   *
   * @param outputVolts The voltage to output.
   */
  public void setVoltage(double outputVolts) {
    m_talon.setVoltage(outputVolts);
  }

  /**
   * Common interface to stop the motor until Set is called again.
   */
  public void stopMotor() {
    m_talon.stopMotor();
    logOutputs(ControlMode.PercentOutput, 0.0);
  }

  /**
   * Closes the TalonSRX motor controller
   */
  @Override
  public void close() {
    m_talon.close();
  }
}
