// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.hardware.generic;

import org.lasarobotics.hardware.LoggableHardware;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.AnalogInput;

/**
 * Analog sensor
 */
public class Analog implements LoggableHardware, AutoCloseable {
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
    public Measure<Voltage> voltage = Units.Volts.of(0.0);
  }

  private AnalogInput m_analogInput;

  private ID m_id;
  private AnalogInputsAutoLogged m_inputs;

  /**
   * Create an Analog object
   * @param id Analog ID
   */
  public Analog(Analog.ID id) {
    this.m_id = id;
    this.m_analogInput = new AnalogInput(id.port);
    this.m_inputs = new AnalogInputsAutoLogged();
  }

  /**
   * Update sensor input readings
   */
  private void updateInputs() {
    m_inputs.voltage = Units.Volts.of(m_analogInput.getVoltage());
  }

  /**
   * Call this method periodically
   */
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
  public AnalogInputsAutoLogged getInputs() {
    return m_inputs;
  }

  @Override
  public void close() {
    m_analogInput.close();
  }

}
