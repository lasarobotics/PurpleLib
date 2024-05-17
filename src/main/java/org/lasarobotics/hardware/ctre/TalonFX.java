// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.


package org.lasarobotics.hardware.ctre;


import org.lasarobotics.hardware.LoggableHardware;
import org.lasarobotics.hardware.PurpleManager;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;


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

  private static final String VALUE_LOG_ENTRY = "/OutputValue";
  private static final String MODE_LOG_ENTRY = "/OutputMode";
  private static final String CURRENT_LOG_ENTRY = "/Current";


  private com.ctre.phoenix6.hardware.TalonFX m_talon;

  private ID m_id;
  private TalonFXInputsAutoLogged m_inputs;

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
    * Closes the TalonFX motor controller
    */
   @Override
    public void close() {
      PurpleManager.remove(this);
      m_talon.close();
    }
}









