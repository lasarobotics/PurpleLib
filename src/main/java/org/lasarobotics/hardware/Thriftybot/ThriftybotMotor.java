package org.lasarobotics.hardware.Thriftybot;

import org.lasarobotics.hardware.LoggableHardware;
import org.lasarobotics.hardware.PurpleManager;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.ErrorCode;
import com.thethriftybot.ThriftyNova;

import java.util.*;


public class ThriftybotMotor extends LoggableHardware {

  public static class ID {
    public final String name;
    public final int deviceID;


    public ID(String name, int deviceID) {
      this.name = name;
      this.deviceID = deviceID;
    }
  }

  @AutoLog
  public static class ThriftybotInputs{
    public double position = 0.0;
    public double velocity = 0.0;
  }

  private ThriftyNova m_thrifty;
  private static final String VALUE_LOG_ENTRY = "/OutputValue";
  private static final String MODE_LOG_ENTRY = "/OutputMode";
  private static final String CURRENT_LOG_ENTRY = "/Current";


  private ID m_id;
  private ThriftybotInputsAutoLogged m_inputs;

  public ThriftybotMotor(ID id){
    this.m_id = id;
    this.m_thrifty = new com.thethriftybot.ThriftyNova(id.deviceID);
    this.m_inputs = new ThriftybotInputsAutoLogged();

    periodic();

    PurpleManager.add(this);
  }

  /**
   * Log output values
   * @param value Value that was set
   * @param mode The output mode to apply
   */
  private void logOutputs(double value) {
    Logger.recordOutput(m_id.name + VALUE_LOG_ENTRY, value);
    Logger.recordOutput(m_id.name + MODE_LOG_ENTRY, m_thrifty.toString());
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
  private void updateInputs() {
    m_inputs.position = getPosition();
    m_inputs.velocity = getVelocity();
  }

  @Override
  protected void periodic() {
    updateInputs();
    Logger.processInputs(m_id.name, m_inputs);
  }

  /**
   * Get latest sensor input data
   * @return Latest sensor data
   */
  @Override
  public ThriftybotInputsAutoLogged getInputs() {
    return m_inputs;
  }

  /**
   * Get device ID
   * @return Device ID
   */
  public ID getID(){
    return m_id;
  }

  /**
   * Common interface for setting the speed of a simple speed controller.
   *
   * @param speed The speed to set. Value should be between -1.0 and 1.0.
   *                   Value is also saved for Get().
   * 
   */
  public void setTargetVelocity(double speed) {
    m_thrifty.setVelocity(speed);
    logOutputs(speed);
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
  }

  /**
   * 
   * @param value the maximum forward output, always between 0.0 and 1.0
   */
  public void setMaxOutput(double value){
    m_thrifty.setMaxOutput(value);
  }

  /**
   * 
   * @param fwdValue maximum forward output (range of 0.0 to 1.0)
   * @param bwdValue maximum backward output (range of -1.0 to 1.0)
   */
  public void setMaxOutput(double fwdValue, double bwdValue){
    m_thrifty.setMaxOutput(fwdValue, bwdValue);
  }

  /**
   * 
   * @returns an int that denotes Stator current which is the output current of the motor
   *            measured in amps
   */           
  public int getStatorCurrent(){
    return m_thrifty.getStatorCurrent();
  }

  /**
   * 
   * @return an int that denotes the amount of current moving from supply
   *        measured in amps
   */
  public int getSupplyCurrent(){
    return m_thrifty.getSupplyCurrent();
  }

  /**
   * 
   * @param encoderType encoder type to use
   *                consult Thriftybot documentation for additional information on encoder types
   */
  public void useEncoderType(ThriftyNova.EncoderType encoderType){
    m_thrifty.useEncoderType(encoderType);
  }

  /**
   * @param rampDownTime time motor controller will take to go from 100% to idle
   *                input of 0.5 means the motor will 0.5 seconds to go from 100% to idle
   */
  public void setRampDown(double rampDownTime){
    m_thrifty.setRampDown(rampDownTime);
  }

  /**
   * @param rampUpTime time motor controller will take to go from idle to 100%
   *                input of 0.5 means the motor will 0.5 seconds to go from idle to 100%
   */
  public void setRampUp(double rampUpTime){
    m_thrifty.setRampUp(rampUpTime);
  }
  /**
   * @param pidSlot Sets the pidslot to use for feedback control
   */
  public void usePidSlot(ThriftyNova.PIDSlot pidSlot){
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
  public void setVoltage(double outputVolts) {
    m_thrifty.setVoltage(outputVolts);
  }

  /**
   * Configure the peak allowable current (when current limit is enabled).
   * @param value
   *            value to limit.
   */
  public void configPeakCurrentLimit(ThriftyNova.CurrentType currentType, double value) {
    m_thrifty.setMaxCurrent(currentType, value); 
  }

  /**
   * @returns arraylist of errors stored in error buffer
   */
  public List<ThriftyNova.Error> getErrors(){
    return m_thrifty.getErrors();
  }

  /**
   * @param errorType
   *            error type to check for
   * @returns a boolean denoting if motor controller has incurred an error of specified type
   *        True = Error, False = No Error
   */
  public boolean hasErrors(ThriftyNova.Error errorType){
    return m_thrifty.hasError(errorType);
  }



  /**
   * Common interface to stop the motor until Set is called again.
   */
  public void stopMotor() {
    m_thrifty.stopMotor();
    logOutputs(0.0);
  }

  /**
   * Closes the TalonSRX motor controller
   */
  @Override
  public void close() {
    PurpleManager.remove(this);
  }
  
}


