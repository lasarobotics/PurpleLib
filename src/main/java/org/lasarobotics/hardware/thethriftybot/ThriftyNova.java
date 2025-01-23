package org.lasarobotics.hardware.thethriftybot;

import java.util.List;

import org.lasarobotics.hardware.LoggableHardware;
import org.lasarobotics.hardware.PurpleManager;
import org.lasarobotics.utils.GlobalConstants;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;


public class ThriftyNova extends LoggableHardware {

  public static class ID {
    public final String name;
    public final int deviceID;


    public ID(String name, int deviceID) {
      this.name = name;
      this.deviceID = deviceID;
    }
  }

  private enum ControlType {
    PERCENT,
    POSITION,
    VELOCITY,
    VOLTAGE,
    BOOLEAN
  }

  @AutoLog
  public static class ThriftyNovaInputs{
    public double position = 0.0;
    public double velocity = 0.0;
  }

  private com.thethriftybot.ThriftyNova m_thrifty;
  private static final String VALUE_LOG_ENTRY = "/OutputValue";
  private static final String MODE_LOG_ENTRY = "/OutputMode";
  private static final String CURRENT_LOG_ENTRY = "/Current";

  private ID m_id;
  private ThriftyNovaInputsAutoLogged m_inputs;
  private Frequency m_updateRate;

  /**
   * Create a ThriftyNova object with built-in logging
   * @param id ThriftyNova ID
   * @param updateRate Update rate of ThriftyNova inputs
   */
  public ThriftyNova(ID id, Frequency updateRate) {
    this.m_id = id;
    this.m_thrifty = new com.thethriftybot.ThriftyNova(id.deviceID);
    this.m_inputs = new ThriftyNovaInputsAutoLogged();
    this.m_updateRate = updateRate;

    // Update inputs on init
    updateInputs();

    // Register device with manager
    PurpleManager.add(this);
  }

  /**
   * Create a ThriftyNova object with built-in logging
   * <p>
   * Update rate is set to {@link GlobalConstants#ROBOT_LOOP_HZ}
   * @param id ThriftyNova ID
   */
  public ThriftyNova(ID id) {
    this(id, GlobalConstants.ROBOT_LOOP_HZ);
  }

  /**
   * Log output values
   * @param value Value that was set
   * @param type value types
   */
  private void logOutputs(double value, ControlType type) {
    Logger.recordOutput(m_id.name + VALUE_LOG_ENTRY, value);
    Logger.recordOutput(m_id.name + MODE_LOG_ENTRY, type.name());
    Logger.recordOutput(m_id.name + CURRENT_LOG_ENTRY, getStatorCurrent());
  }

  /**
   * Get the selected sensor position (in raw sensor units).
   *
   * @return Position of selected sensor (in raw sensor units).
   */
  private double getPosition() {
    return m_thrifty.getPosition();
  }

  /**
   * Sets position of PID controller
   * @param targetPosition
   */
  public void setPosition(double targetPosition) {
    m_thrifty.setPosition(targetPosition);
    logOutputs(targetPosition, ControlType.POSITION);
  }

  /**
   * Drives the motor using the parameters of the configured followed motor controller. Follower mode is dependant on the fault status frame.
   * The slower that frame is sent, the slower the follower motor will update.
   * @param followerID - The Target ID to follow
   */
  public void follow(int followerID) {
    m_thrifty.follow(followerID);
  }

  /**
   * Calls every status/feedback getter that has a network table entry
   */
  public void updateStatusNT() {
    m_thrifty.updateStatusNT();
  }

  /**
   * Updates the network tables for all ThriftyNova motors.
   */
  public static void updateStatusNTGlobal() {
    com.thethriftybot.ThriftyNova.updateStatusNTGlobal();
  }

  /**
   * Get the selected sensor velocity.
   *
   * @return selected sensor (in raw sensor units) per 100ms.
   * See Thriftybot-Documentation for how to interpret.
   */
  private double getVelocity() {
    return m_thrifty.getVelocity();
  }

  /**
   * Update sensor input readings
   */
   @Override
  protected void updateInputs() {
    m_inputs.position = getPosition();
    m_inputs.velocity = getVelocity();
  }

  @Override
  protected void periodic() {
    synchronized (m_inputs) { Logger.processInputs(m_id.name, m_inputs); }
  }

  @Override
  public Frequency getUpdateRate() {
    return m_updateRate;
  }

  /**
   * Get latest sensor input data
   * @return Latest sensor data
   */
  @Override
  public ThriftyNovaInputsAutoLogged getInputs() {
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
   * Common interface for setting the speed of a simple speed controller.
   *
   * @param speed The speed to set. Value should be between -1.0 and 1.0.
   *                   Value is also saved for Get().
   *
   */
  public void setVelocity(double speed) {
    m_thrifty.setVelocity(speed);
    logOutputs(speed, ControlType.VELOCITY);
  }



    /**
   * Sets the appropriate output on the talon, depending on the mode.
   * @param value The percent output to apply.
   * In PercentOutput, the output is between -1.0 and 1.0, with 0.0 as stopped.
   *
   * @param value The setpoint value, as described above.
   */
  public void set(double value) {
    m_thrifty.set(value);
    logOutputs(value, ControlType.PERCENT);
  }

  /**
   * Returns the last set point specified by the user in any control method. Percent output set point becomes zero.
   * @return the last set point specified by the user in any control method
   */
  public double get() {
    return m_thrifty.get();
  }

  /**
   * @return the error to the last set point that was input, based on last used control mode (position or velocity), and encoder type (internal or quaderature).
   */
  public double getClosedLoopError() {
    return m_thrifty.getClosedLoopError();
  }

  /**
   * sets percent output form between 1.0 (full forward) to -1.0 (full backward), 0.0 = full stop
   * @param percentOutput
   */
  public void setPercent(double percentOutput) {
    m_thrifty.setPercent(percentOutput);
    logOutputs(percentOutput, ControlType.PERCENT);
  }

  /**
   *
   * @param value the maximum forward output, always between 0.0 and 1.0
   */
  public void setMaxOutput(double value) {
    m_thrifty.setMaxOutput(value);
    logOutputs(value, ControlType.PERCENT);
  }

  /**
   *
   * @param fwdValue maximum forward output (range of 0.0 to 1.0)
   * @param bwdValue maximum backward output (range of -1.0 to 1.0)
   */
  public void setMaxOutput(double fwdValue, double bwdValue) {
    m_thrifty.setMaxOutput(fwdValue, bwdValue);
    logOutputs(fwdValue, ControlType.PERCENT);
    logOutputs(bwdValue, ControlType.PERCENT);
  }

  /**
   *
   * @return an int that denotes Stator current which is the output current of the motor
   *            measured in amps
   */
  public Current getStatorCurrent() {
    return Units.Amps.of(m_thrifty.getStatorCurrent());
  }

  /**
   *
   * @return an int that denotes the amount of current moving from supply
   *        measured in amps
   */
  public Current getSupplyCurrent() {
    return Units.Amps.of(m_thrifty.getSupplyCurrent());
  }

  /**
   *
   * @param encoderType encoder type to use
   *                consult Thriftybot documentation for additional information on encoder types
   */
  public void useEncoderType(com.thethriftybot.ThriftyNova.EncoderType encoderType) {
    m_thrifty.useEncoderType(encoderType);
  }

  /**
   * @param rampDownTime time motor controller will take to go from 100% to idle
   *                input of 0.5 means the motor will 0.5 seconds to go from 100% to idle
   */
  public void setRampDown(Time rampDownTime) {
    m_thrifty.setRampDown(rampDownTime.in(Units.Seconds));
  }

  /**
   * @param rampUpTime time motor controller will take to go from idle to 100%
   *                input of 0.5 means the motor will 0.5 seconds to go from idle to 100%
   */
  public void setRampUp(Time rampUpTime) {
    m_thrifty.setRampUp(rampUpTime.in(Units.Seconds));
  }

  /**
   * @param pidSlot Sets the pidslot to use for feedback control
   */
  public void usePidSlot(com.thethriftybot.ThriftyNova.PIDSlot pidSlot) {
    m_thrifty.usePIDSlot(pidSlot);
  }

  /**
   * Common interface for inverting direction of a speed controller.
   *
   * @param isInverted The state of inversion, true is inverted.
   */
  public void setInverted(boolean isInverted) {
    m_thrifty.setInverted(isInverted);
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
  public void setVoltage(Voltage outputVolts) {
    m_thrifty.setVoltage(outputVolts);
    logOutputs(outputVolts.in(Units.Volts), ControlType.VOLTAGE);
  }

  /**
   * gets voltage measurement
   */
  public Voltage getVoltage() {
    return Units.Volts.of(m_thrifty.getVoltage());
  }

  /**
   * Configure the peak allowable current (when current limit is enabled).
   * @param value
   *            value to limit.
   */
  public void configPeakCurrentLimit(com.thethriftybot.ThriftyNova.CurrentType currentType, Current value) {
    m_thrifty.setMaxCurrent(currentType, value.in(Units.Amps));
  }

  /**
   * @return arraylist of errors stored in error buffer
   */
  public List<com.thethriftybot.ThriftyNova.Error> getErrors() {
    return m_thrifty.getErrors();
  }

  /**
   * @param errorType
   *            error type to check for
   * @return a boolean denoting if motor controller has incurred an error of specified type
   *        True = Error, False = No Error
   */
  public boolean hasErrors(com.thethriftybot.ThriftyNova.Error errorType) {
    return m_thrifty.hasError(errorType);
  }

  /**
   * gets ramp up time in seconds
   */
  public Time getRampUp() {
    return Units.Seconds.of(m_thrifty.getRampUp());
  }

  /**
   * gets ramp down time in seconds
   */
  public Time getRampDown(){
    return Units.Seconds.of(m_thrifty.getRampDown());
  }

  /**
   * Common interface to stop the motor until Set is called again.
   */
  public void stopMotor() {
    m_thrifty.stopMotor();
    logOutputs(0.0, ControlType.PERCENT);
  }

  /**
   * disables motor controller
   */
  public void disable() {
    m_thrifty.disable();
  }

  /**
   * Gets break mode status of motor
   */
  public boolean getBrakeMode() {
    return m_thrifty.getBrakeMode();
  }

  /**
   * gets max forward output
   *
   */
  public double getMaxForwardOutput() {
    return m_thrifty.getMaxForwardOutput();
  }

  /**
   * gets max reverse output
   */
  public double getMaxReverseOutput() {
    return m_thrifty.getMaxReverseOutput();
  }

  /**
   * Sets the soft limits that the motor will not cross if soft limits are enabled.
   * @param revLimit - reverse position the motor will not go past
   * @param fwdLimit - forward position the motor will not go past
   */
  public void setSoftLimits(double revLimit, double fwdLimit) {
    m_thrifty.setSoftLimits(revLimit, fwdLimit);
  }

  /**
   * enables soft limits on motor
   * @param enable - boolean variable if true, soft limits are enabled, disabled otherwise
   */
  public void enableSoftLimits(boolean enable) {
    m_thrifty.enableSoftLimits(enable);
  }

  /**
   * enable hard limits on motor
   * @param enable - boolean variable if true, soft limits are enabled, disabled otherwise
   */
  public void enableHardLimits(boolean enable) {
    m_thrifty.enableHardLimits(enable);
  }

  /**
   * Gets maximum current limit
   */
  public Current getMaxCurrent() {
    return Units.Amps.of(m_thrifty.getMaxCurrent());
  }

  /**
   * gets current type being used for limiting
   */
  public com.thethriftybot.ThriftyNova.CurrentType getCurrentType() {
    return m_thrifty.getCurrentType();
  }

  /**
   * gets forward soft limit
   */
  public double getForwardSoftLimit() {
    return m_thrifty.getForwardSoftLimit();
  }

  /**
   * gets reverse soft limit
   */
  public double getReverseSoftLimit() {
    return m_thrifty.getReverseSoftLimit();
  }

  /**
   * gets whether soft limits are enabled
   * @return boolean indicating whether or not soft limits are enabled
   */
  public boolean areSoftLimitsEnabled() {
    return m_thrifty.areSoftLimitsEnabled();
  }

  /**
   * gets whether hard limits are enabled
   * @return boolean indicating whether or not hard limits are enabled
   */
  public boolean areHardLimitsEnabled() {
    return m_thrifty.areHardLimitsEnabled();
  }

  /**
   * Sets the absolute encoder offset.
   * @param offset integer that specifies the amount of offset
   */
  public void setAbsOffset(int offset) {
    m_thrifty.setAbsOffset(offset);
    logOutputs(offset, ControlType.POSITION);
  }

  /**
   * gets absolute encoder offset
   */
  public int getAbsOffset() {
    return m_thrifty.getAbsOffset();
  }

  /**
   * Zeros the absolute encoder reading at the current position
   */
  public void zeroAbsEncoder() {
    m_thrifty.zeroAbsEncoder();
  }

  /**
   * Sets the temperature throttle enable state.
   * @param enable - boolean indicating whether or not this setting is enabled or not
   */
  public void setTempThrottleEnable(boolean enable) {
    m_thrifty.setTempThrottleEnable(enable);
  }

  /**
   * gets the temperature throttle enable state
   */
  public boolean getTemptThrottleEnable() {
    return m_thrifty.getTempThrottleEnable();
  }

  /**
   * gets temperature of motor controller
   * @return integer that specifies temperature of motor controller
   */
  public int getTemperature() {
    return m_thrifty.getTemperature();
  }

  /**
   * sets motor type
   * @param motorType specifies the motor type to be used
   */
  public void setMotorType(com.thethriftybot.ThriftyNova.MotorType motorType) {
    m_thrifty.setMotorType(motorType);
  }

  /**
   * Returns the last set point specified by the user in any control method. Percent output set point becomes zero.
   * @return double that encodes the last set point specified by user in any controler method
   */
  public double getSetPoint() {
    return m_thrifty.getSetPoint();
  }

  /**
   * gets motor type
   */
  public com.thethriftybot.ThriftyNova.MotorType getMotorType() {
    return m_thrifty.getMotorType();
  }

  /**
   * sets voltage compensation value
   * @param vcomp the voltage compensation value
   */
  public void setVoltageCompensation(Voltage vcomp) {
    m_thrifty.setVoltageCompensation(vcomp.in(Units.Volt));
    logOutputs(vcomp.in(Units.Volts), ControlType.VOLTAGE);
  }

  /**
   * gets the voltage compensation value
   */
  public Voltage getVoltageCompensationEnabled() {
    return Units.Volts.of(m_thrifty.getVoltageCompensationEnabled());
  }

  /**
   * sets external encoder
   * @param external is the external encoder to use
   */
  public void setExternalEncoder(com.thethriftybot.ThriftyNova.ExternalEncoder external) {
    m_thrifty.setExternalEncoder(external);
  }

  /**
   * sets encoder position
   * @param position
   */
  public void setEncoderPosition(double position) {
    m_thrifty.setEncoderPosition(position);
  }

  /**
   * resets motor controller to default state
   */
  public void factoryReset() {
    m_thrifty.factoryReset();
  }



  /**
   * Sets NT logging
   * @param enable specifies whether this setting is enabled or not
   */
  public void setNTLogging(boolean enable) {
    m_thrifty.setNTLogging(enable);
  }

  /**
   * Closes the ThriftyNova motor controller
   */
  @Override
  public void close() {
    PurpleManager.remove(this);
  }
}