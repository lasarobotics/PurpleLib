package org.lasarobotics.hardware.thethriftybot;

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

  private static ThriftyNova m_thrifty;
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
  public void logOutputs(double value) {
    Logger.recordOutput(m_id.name + VALUE_LOG_ENTRY, value);
    Logger.recordOutput(m_id.name + MODE_LOG_ENTRY, m_thrifty.toString());
    Logger.recordOutput(m_id.name + CURRENT_LOG_ENTRY, getStatorCurrent());
  }

  /**
   * Get the selected sensor position (in raw sensor units).
   *
   * @return Position of selected sensor (in raw sensor units).
   */
  public double getPosition() {
    return m_thrifty.getPosition();
  }
  
  /**
   * Sets position of PID controller
   * @param targetPosition
   */
  public void setPosition(double targetPosition){
    m_thrifty.setPosition(targetPosition);
  }

  /**
   * Drives the motor using the parameters of the configured followed motor controller. Follower mode is dependant on the fault status frame.
   * The slower that frame is sent, the slower the follower motor will update.
   * @param followerID - The Target ID to follow
   */
  public void follow(int followerID){
    m_thrifty.follow(followerID);
  }

  /**
   * Calls every status/feedback getter that has a network table entry
   */
  public void updateStatusNT(){
    m_thrifty.updateStatusNT();
  }

  /**
   * Updates the network tables for all ThriftyNova motors.
   */
  public static void updateStatusNTGlobal(){
    m_thrifty.updateStatusNTGlobal();
  }


  /**
   * Get the selected sensor velocity.
   *
   * @return selected sensor (in raw sensor units) per 100ms.
   * See Thriftybot-Documentation for how to interpret.
   */
  public double getVelocity() {
    return m_thrifty.getVelocity();
  }

  /**
   * Update sensor input readings
   */
  public void updateInputs() {
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
  public void setVelocity(double speed) {
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
   * Returns the last set point specified by the user in any control method. Percent output set point becomes zero.
   * @return the last set point specified by the user in any control method
   */
  public double get(){
    return m_thrifty.get();
  }

  /**
   * @return the error to the last set point that was input, based on last used control mode (position or velocity), and encoder type (internal or quaderature).
   */
  public double getClosedLoopError(){
    return m_thrifty.getClosedLoopError();
  }



  /**
   * sets percent output form between 1.0 (full forward) to -1.0 (full backward), 0.0 = full stop
   * @param percentOutput
   */
  public void setPercent(double percentOutput){
    m_thrifty.setPercent(percentOutput);
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
   * @return an int that denotes Stator current which is the output current of the motor
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
   * gets voltage measurement
   */
  public double getVoltage(){
    return m_thrifty.getVoltage();
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
   * @return arraylist of errors stored in error buffer
   */
  public List<ThriftyNova.Error> getErrors(){
    return m_thrifty.getErrors();
  }

  /**
   * @param errorType
   *            error type to check for
   * @return a boolean denoting if motor controller has incurred an error of specified type
   *        True = Error, False = No Error
   */
  public boolean hasErrors(ThriftyNova.Error errorType){
    return m_thrifty.hasError(errorType);
  }

  /**
   * gets ramp up time in seconds
   */
  public double getRampUp(){
    return m_thrifty.getRampUp();
  }

  /**
   * gets ramp down time in seconds
   */
  public double getRampDown(){
    return m_thrifty.getRampDown();
  }

  /**
   * Common interface to stop the motor until Set is called again.
   */
  public void stopMotor() {
    m_thrifty.stopMotor();
    logOutputs(0.0);
  }

  /**
   * disables motor controller
   */
  public void disable(){
    m_thrifty.disable();
  }

  /**
   * Gets break mode status of motor
   */
  public boolean getBrakeMode(){
    return m_thrifty.getBrakeMode();
  }

  /**
   * gets max forward output
   * 
   */
  public double getMaxForwardOutput(){
    return m_thrifty.getMaxForwardOutput();
  }

  /**
   * gets max reverse output
   */
  public double getMaxReverseOutput(){
    return m_thrifty.getMaxReverseOutput();
  }

  /**
   * Sets the soft limits that the motor will not cross if soft limits are enabled.
   * @param revLimit - reverse position the motor will not go past
   * @param fwdLimit - forward position the motor will not go past
   */
  public ThriftyNova setSoftLimits(double revLimit, double fwdLimit){
    m_thrifty.setSoftLimits(revLimit, fwdLimit);
    return m_thrifty;
  }

  /**
   * enables soft limits on motor
   * @param enable - boolean variable if true, soft limits are enabled, disabled otherwise
   */
  public ThriftyNova enableSoftLimits(boolean enable){
    m_thrifty.enableSoftLimits(enable);
    return m_thrifty;
  }

  /**
   * enable hard limits on motor
   * @param enable - boolean variable if true, soft limits are enabled, disabled otherwise
   */
  public ThriftyNova enableHardLimits(boolean enable){
    m_thrifty.enableHardLimits(enable);
    return m_thrifty;
  }

  /**
   * Gets maximum current limit
   */
  public double getMaxCurrent(){
    return m_thrifty.getMaxCurrent();
  }

  /**
   * gets current type being used for limiting
   */
  public ThriftyNova.CurrentType getCurrentType(){
    return m_thrifty.getCurrentType();
  }

  /**
   * gets forward soft limit
   */
  public double getForwardSoftLimit(){
    return m_thrifty.getForwardSoftLimit();
  }

  /**
   * gets reverse soft limit
   */
  public double getReverseSoftLimit(){
    return m_thrifty.getReverseSoftLimit();
  }

  /**
   * gets whether soft limits are enabled
   * @return boolean indicating whether or not soft limits are enabled
   */
  public boolean areSoftLimitsEnabled(){
    return m_thrifty.areSoftLimitsEnabled();
  }

  /**
   * gets whether hard limits are enabled
   * @return boolean indicating whether or not hard limits are enabled
   */
  public boolean areHardLimitsEnabled(){
    return m_thrifty.areHardLimitsEnabled();
  }

  /**
   * Sets the absolute encoder offset.
   * @param offset integer that specifies the amount of offset 
   * @return thriftynova object
   */
  public ThriftyNova setAbsOffset(int offset){
    m_thrifty.setAbsOffset(offset);
    return m_thrifty;
  }

  /**
   * gets absolute encoder offset
   */
  public int getAbsOffset(){
    return m_thrifty.getAbsOffset();
  }

  /**
   * Zeros the absolute encoder reading at the current position
   */
  public ThriftyNova zeroAbsEncoder(){
    m_thrifty.zeroAbsEncoder();
    return m_thrifty;
  }

  /**
   * Sets the temperature throttle enable state.
   * @param enable - boolean indicating whether or not this setting is enabled or not
   */
  public ThriftyNova setTempThrottleEnable(boolean enable){
    m_thrifty.setTempThrottleEnable(enable);
    return m_thrifty;
  }

  /**
   * gets the temperature throttle enable state
   */
  public boolean getTemptThrottleEnable(){
    return m_thrifty.getTempThrottleEnable();
  }
  
  /**
   * gets temperature of motor controller
   * @return integer that specifies temperature of motor controller
   */
  public int getTemperature(){
    return m_thrifty.getTemperature();
  }



  /**
   * sets motor type
   * @param motorType specifies the motor type to be used
   */
  public ThriftyNova setMotorType(ThriftyNova.MotorType motorType){
    m_thrifty.setMotorType(motorType);
    return m_thrifty;
  }

  /**
   * Returns the last set point specified by the user in any control method. Percent output set point becomes zero.
   * @return double that encodes the last set point specified by user in any controler method
   */
  public double getSetPoint(){
    return m_thrifty.getSetPoint();
  }



  /**
   * gets motor type
   */
  public ThriftyNova.MotorType getMotorType(){
    return m_thrifty.getMotorType();
  }

  /**
   * sets voltage compensation value
   * @param vcomp the voltage compensation value
   */
  public ThriftyNova setVoltageCompensation(double vcomp){
    m_thrifty.setVoltageCompensation(vcomp);
    return m_thrifty;
  }

  /**
   * gets the voltage compensation value
   */
  public double getVoltageCompensation(){
    return m_thrifty.getVoltageCompensationEnabled();
  }

  /**
   * sets external encoder
   * @param external is the external encoder to use
   */
  public ThriftyNova setExternalEncoder(ThriftyNova.ExternalEncoder external){
    m_thrifty.setExternalEncoder(external);
    return m_thrifty;
  }
  
  /**
   * sets encoder position
   * @param position
   */
  public void setEncoderPosition(double position){
    m_thrifty.setEncoderPosition(position);
  }

  /**
   * resets motor controller to default state
   */
  public void factoryReset(){
    m_thrifty.factoryReset();
  }



  /**
   * Sets NT logging
   * @param enable specifies whether this setting is enabled or not
   */
  public void setNTLogging(boolean enable){
    m_thrifty.setNTLogging(enable);
  }




  /**
   * Closes the TalonSRX motor controller
   */
  @Override
  public void close() {
    PurpleManager.remove(this);
  }

  
}