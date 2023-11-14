// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project

package org.lasarobotics.hardware;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Solenoid implements LoggableHardware, AutoCloseable {
  /** Solenoid ID */
  public static class ID {
    public final String name;
    public final PneumaticsModuleType moduleType;
    public final int channel;

    /**
     * Solenoid ID
     * @param name Device name for logging
     * @param moduleType Specified module type
     * @param channel Channel
     */
    public ID(String name, PneumaticsModuleType moduleType, int channel) {
      this.name = name;
      this.moduleType = moduleType;
      this.channel = channel;
    }
  }

  private static final String VALUE_LOG_ENTRY = "/OutputValue";

  private edu.wpi.first.wpilibj.Solenoid m_solenoid;
  
  private ID m_id;

  /**
   * Create a Solenoid object with built-in logging
   * @param id Solenoid ID
   * @param module Specified module
   */
  public Solenoid(Solenoid.ID id, int module) {
    this.m_id = id;
    this.m_solenoid = new edu.wpi.first.wpilibj.Solenoid(module, m_id.moduleType, m_id.channel);
  }

  /**
   * Create a Solenoid object with built-in logging
   * @param id Solenoid ID
   */
  public Solenoid(Solenoid.ID id) {
    this.m_id = id;
    this.m_solenoid = new edu.wpi.first.wpilibj.Solenoid(m_id.moduleType, m_id.channel);
  }

  private void logOutputs(boolean value) {
    Logger.recordOutput(m_id.name + VALUE_LOG_ENTRY, value);
  }

  @Override
  public void periodic() {}

  @Override
  public LoggableInputs getInputs() {
    return null;
  }

  /**
   * Set the value of a solenoid.
   *
   * @param value True will turn the solenoid output on. False will turn the solenoid output off.
   */
  public void set(boolean value) {
    m_solenoid.set(value);
    logOutputs(value);
  }

  /**
   * Toggle the value of the solenoid.
   *
   * If the solenoid is set to on, it'll be turned off. If the solenoid is set to off, it'll be
   * turned on.
   */
  public void toggle() {
    m_solenoid.toggle();
    logOutputs(m_solenoid.get());
  }

  @Override
  public void close() {
    m_solenoid.close();
  }
}
