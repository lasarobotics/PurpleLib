// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.hardware.generic;

import org.lasarobotics.hardware.LoggableHardware;
import org.lasarobotics.hardware.PurpleManager;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.AnalogInput;

/**
 * Analog sensor
 */
public class Analog extends LoggableHardware {
  /** Analog sensor ID */
  public static class ID {
    public final String name;
    public final int port;

    /**
     * Analog sensor ID
     * @param name Device name for logging
     * @param port Port number
     */
    public ID(String name, int port) {
      this.name = name;
      this.port = port;
    }
  }

  /**
   * Analog sensor inputs
   */
  @AutoLog
  public static class AnalogInputs {
    public MutVoltage voltage = Units.Volts.of(0.0).mutableCopy();
  }

  private AnalogInput m_analogInput;

  private ID m_id;
  private Frequency m_updateRate;
  private AnalogInputsAutoLogged m_inputs;

  /**
   * Create an Analog object
   * @param id Analog ID
   */
  public Analog(Analog.ID id, Frequency updateRate) {
    this.m_id = id;
    this.m_updateRate = updateRate;
    this.m_analogInput = new AnalogInput(id.port);
    this.m_inputs = new AnalogInputsAutoLogged();

    // Update inputs on init
    periodic();

    // Register device with manager
    PurpleManager.add(this);
  }

  /**
   * Update sensor input readings
   */
  protected void updateInputs() {
    m_inputs.voltage.mut_replace(Units.Volts.of(m_analogInput.getVoltage()));
  }

  @Override
  public Frequency getUpdateRate() {
    return m_updateRate;
  }

  /**
   * Call this method periodically
   */
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
  public AnalogInputsAutoLogged getInputs() {
    return m_inputs;
  }

  @Override
  public void close() {
    PurpleManager.remove(this);
    m_analogInput.close();
  }

}
