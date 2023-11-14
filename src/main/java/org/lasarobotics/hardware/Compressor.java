// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project

package org.lasarobotics.hardware;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Compressor implements LoggableHardware, AutoCloseable {
  /** Compressor ID */
  public static class ID {
    public final String name;
    public final PneumaticsModuleType moduleType;

    /**
     * Compressor ID
     * @param name Device name for logging
     * @param moduleType Specified module type
     */
    public ID(String name, PneumaticsModuleType moduleType) {
      this.name = name;
      this.moduleType = moduleType;
    }
  }

  @AutoLog
  public static class CompressorInputs {
    boolean isEnabled = false;
    boolean digitalPressure = false;
    double analogPressure = 0.0;
  }

  private edu.wpi.first.wpilibj.Compressor m_compressor;

  private ID m_id;
  private CompressorInputsAutoLogged m_inputs;

  /**
   * Create a Compressor object with built-in logging
   * @param id Compressor ID
   * @param module Specified module
   */
  public Compressor(Compressor.ID id, int module) {
    this.m_id = id;
    this.m_compressor = new edu.wpi.first.wpilibj.Compressor(module, m_id.moduleType);
  }

  /**
   * Create a Compressor object with built-in logging
   * @param id Compressor ID
   */
  public Compressor(Compressor.ID id) {
    this.m_id = id;
    this.m_compressor = new edu.wpi.first.wpilibj.Compressor(m_id.moduleType);
  }

  /**
   * If supported by the device, returns the pressure (in PSI) read by the analog pressure sensor
   * (on channel 0).
   *
   * This function is only supported by the REV PH with the REV Analog Pressure Sensor. On CTRE
   * PCM, this will return 0.
   *
   * @return The pressure (in PSI) read by the analog pressure sensor.
   */
  public double getPressure() {
    return m_compressor.getPressure();
  }

  /**
   * Returns the state of the pressure switch.
   *
   * @return True if pressure switch indicates that the system is not full, otherwise false.
   */
  public boolean getPressureSwitchValue() {
    return m_compressor.getPressureSwitchValue();
  }

  /**
   * Returns whether the compressor is active or not.
   *
   * @return true if the compressor is on - otherwise false.
   */
  public boolean isEnabled() {
    return m_compressor.isEnabled();
  }

  /**
   * Update sensor input readings
   */
  private void updateInputs() {
    m_inputs.isEnabled = isEnabled();
    m_inputs.digitalPressure = getPressureSwitchValue();
    m_inputs.analogPressure = getPressure();
  }

  /**
   * Call this method periodically
   */
  @Override
  public void periodic() {
    updateInputs();
  }

  /**
   * Get latest sensor input data
   * @return Latest sensor data
   */
  @Override
  public CompressorInputsAutoLogged getInputs() {
    return m_inputs;
  }

  /**
   * Enables the compressor in digital mode using the digital pressure switch. The compressor will
   * turn on when the pressure switch indicates that the system is not full, and will turn off when
   * the pressure switch indicates that the system is full.
   */
  public void enableDigital() {
    m_compressor.enableDigital();
  }

  /**
   * If supported by the device, enables the compressor in analog mode. This mode uses an analog
   * pressure sensor connected to analog channel 0 to cycle the compressor. The compressor will turn
   * on when the pressure drops below {@code minPressure} and will turn off when the pressure
   * reaches {@code maxPressure}. This mode is only supported by the REV PH with the REV Analog
   * Pressure Sensor connected to analog channel 0.
   *
   * On CTRE PCM, this will enable digital control.
   *
   * @param minPressure The minimum pressure in PSI. The compressor will turn on when the pressure
   *     drops below this value.
   * @param maxPressure The maximum pressure in PSI. The compressor will turn off when the pressure
   *     reaches this value.
   */
  public void enableAnalog(double minPressure, double maxPressure) {
    m_compressor.enableAnalog(minPressure, maxPressure);
  }

  /**
   * If supported by the device, enables the compressor in hybrid mode. This mode uses both a
   * digital pressure switch and an analog pressure sensor connected to analog channel 0 to cycle
   * the compressor. This mode is only supported by the REV PH with the REV Analog Pressure Sensor
   * connected to analog channel 0.
   *
   * On CTRE PCM, this will enable digital control.
   *
   * @param minPressure The minimum pressure in PSI. The compressor will turn on when the pressure
   *     drops below this value and the pressure switch indicates that the system is not full.
   * @param maxPressure The maximum pressure in PSI. The compressor will turn off when the pressure
   *     reaches this value or the pressure switch is disconnected or indicates that the system is
   *     full.
   */
  public void enableHybrid(double minPressure, double maxPressure) {
    m_compressor.enableHybrid(minPressure, maxPressure);
  }

  /** Disable the compressor. */
  public void disable() {
    m_compressor.disable();
  }

  @Override
  public void close() {
    m_compressor.close();
  }
}
