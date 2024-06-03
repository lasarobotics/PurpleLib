// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.


package org.lasarobotics.hardware.ctre;


import org.lasarobotics.hardware.LoggableHardware;
import org.lasarobotics.hardware.PurpleManager;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import org.lasarobotics.hardware.ctre.CANCoder;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;


/** TalonFX */
public class TalonFX extends LoggableHardware {
  /** TalonFX ID */
  public static class ID {
    public final String name;
    public final PhoenixCANBus bus;
    public final int deviceID;

    /**
     * TalonFX ID
     * @param name Device name for logging
     * @param bus CAN bus
     * @param deviceID CAN ID
     */
    public ID(String name, PhoenixCANBus bus, int deviceID) {
      this.name = name;
      this.bus = bus;
      this.deviceID = deviceID;
    }
  }

  /**
   * TalonFX sensor inputs
   */
  @AutoLog
  public static class TalonFXInputs {
    public double selectedSensorPosition = 0.0;
    public double selectedSensorVelocity = 0.0;
  }
  private static final double MAX_VOLTAGE = 12.0;
  private static final double MOTOR_DEADBAND = 0.01;


  private static final String VALUE_LOG_ENTRY = "/OutputValue";
  private static final String MODE_LOG_ENTRY = "/OutputMode";
  private static final String CURRENT_LOG_ENTRY = "/Current";


  private com.ctre.phoenix6.hardware.TalonFX m_talon;

  private ID m_id;
  private TalonFXInputsAutoLogged m_inputs;

  private TalonFXConfiguration m_TalonFXConfiguration;
 
  /**
   * Create a TalonFX object with built-in logging
   * @param id TalonFX ID
   */
  public TalonFX(TalonFX.ID id) {
    this.m_id = id;
    this.m_talon = new com.ctre.phoenix6.hardware.TalonFX(id.deviceID);
    this.m_inputs = new TalonFXInputsAutoLogged();


    // Disable motor safety
    m_talon.setSafetyEnabled(false);


    periodic();
  }

  /**
   * Log output values
   * @param value Value that was set
   * @param mode The output mode to apply
   */
  private void logOutputs(ControlMode mode, double value) {
    Logger.recordOutput(m_id.name + VALUE_LOG_ENTRY, value);
    Logger.recordOutput(m_id.name + MODE_LOG_ENTRY, mode.toString());
    Logger.recordOutput(m_id.name + CURRENT_LOG_ENTRY, m_talon.getStatorCurrent().getValue());
  }

  /**
   * Get the selected sensor position (in raw sensor units).
   *
   * @return Position of selected sensor (in raw sensor units).
   */
  private double getSelectedSensorPosition() {
    return m_talon.getPosition().getValue();
  }

  /**
   * Get the selected sensor velocity.
   *
   * @return selected sensor (in raw sensor units) per 100ms.
   * See Phoenix-Documentation for how to interpret.
   */
  private double getSelectedSensorVelocity() {
    return m_talon.getVelocity().getValue();
  }
 
  private void updateInputs() {
    m_inputs.selectedSensorPosition = getSelectedSensorPosition();
    m_inputs.selectedSensorVelocity = getSelectedSensorVelocity();
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
  public TalonFXInputsAutoLogged getInputs() {
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
   * Initiialize remote limit switches with RemoteTalonFX as the sensor
   * @param talonfx Sensor for limit switches
   * @param forward Boolean to choose whether to init forward limit switch
   * @param reverse Boolean to choose whether to init reverse limit switch
   */
  public void initializeRemoteLimits(boolean forward, boolean reverse) {
   HardwareLimitSwitchConfigs limitConfigs = m_TalonFXConfiguration.HardwareLimitSwitch;
   if (forward) {
    limitConfigs.ForwardLimitSource = ForwardLimitSourceValue.RemoteTalonFX;
    limitConfigs.ForwardLimitRemoteSensorID = m_talon.getDeviceID();
   }
   if (reverse) {
    limitConfigs.ReverseLimitSource = ReverseLimitSourceValue.RemoteTalonFX;
    limitConfigs.ReverseLimitRemoteSensorID = m_talon.getDeviceID();
   }

   m_talon.getConfigurator().apply(limitConfigs);
  }
  
  /**
   * Initialize remote limit switches with CANcoder as the sensor
   * <p> 
   * Take note of the fact that we are using our own version of CANCoder
   * @param cancoder Sensor for limit switches
   * @param forward Boolean to choose whether to init forward limit switch
   * @param reverse Boolean to choose whether to init reverse limit switch
   */
  public void initializeRemoteLimitSwitches(CANCoder cancoder, boolean forward, boolean reverse) {
   HardwareLimitSwitchConfigs limitConfigs = m_TalonFXConfiguration.HardwareLimitSwitch;
   if (forward) {
    limitConfigs.ForwardLimitSource = ForwardLimitSourceValue.RemoteCANcoder;
    limitConfigs.ForwardLimitRemoteSensorID = cancoder.getID().deviceID;  
   }
   if (reverse) {
    limitConfigs.ReverseLimitSource = ReverseLimitSourceValue.RemoteCANcoder;
    limitConfigs.ReverseLimitRemoteSensorID = cancoder.getID().deviceID;
   }

   m_talon.getConfigurator().apply(limitConfigs);
  }

  /**
   * Initialize current limits
   * Take note supply current limit will never exceed stator and is usually much lower
   * @param statorCurrentLimit Current limit to be applied for stator 
   */
  public void initializeCurrentLimits(int statorCurrentLimit) {
    var limitConfigs = m_TalonFXConfiguration.CurrentLimits;
      limitConfigs.StatorCurrentLimit = statorCurrentLimit;
      limitConfigs.StatorCurrentLimitEnable = true;

    m_talon.getConfigurator().apply(limitConfigs);
  }

   /**
    * Closes the TalonFX motor controller
    */
   @Override
    public void close() {
      PurpleManager.remove(this);
      m_talon.close();
    }
}
