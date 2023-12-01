// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project

package org.lasarobotics.hardware;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

/** Double solenoid */
public class DoubleSolenoid implements LoggableHardware, AutoCloseable {
  /** DoubleSolenoid ID */
  public static class ID {
    public final String name;
    public final PneumaticsModuleType moduleType;
    public final int forwardChannel;
    public final int reverseChannel;

    /**
     * DoubleSolenoid ID
     * @param name Device name for logging
     * @param moduleType Specified module type
     * @param forwardChannel Forward channel
     * @param reverseChannel Reverse channel
     */
    public ID(String name, PneumaticsModuleType moduleType, int forwardChannel, int reverseChannel) {
      this.name = name;
      this.moduleType = moduleType;
      this.forwardChannel = forwardChannel;
      this.reverseChannel = reverseChannel;
    }
  }

  private static final String VALUE_LOG_ENTRY = "/OutputValue";

  private edu.wpi.first.wpilibj.DoubleSolenoid m_doubleSolenoid;

  private ID m_id;

  /**
   * Create a DoubleSolenoid object with built-in logging
   * @param id DoubleSolenoid ID
   * @param module Specified module
   */
  public DoubleSolenoid(DoubleSolenoid.ID id, int module) {
    this.m_id = id;
    this.m_doubleSolenoid = new edu.wpi.first.wpilibj.DoubleSolenoid(module, m_id.moduleType, m_id.forwardChannel, m_id.reverseChannel);
  }

  /**
   * Create a DoubleSolenoid object with built-in logging
   * @param id DoubleSolenoid ID
   */
  public DoubleSolenoid(DoubleSolenoid.ID id) {
    this.m_id = id;
    this.m_doubleSolenoid = new edu.wpi.first.wpilibj.DoubleSolenoid(m_id.moduleType, m_id.forwardChannel, m_id.reverseChannel);
  }

  private void logOutputs(String value) {
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
  public void set(edu.wpi.first.wpilibj.DoubleSolenoid.Value value) {
    m_doubleSolenoid.set(value);
    logOutputs(value.name());
  }

  @Override
  public void close() {
    m_doubleSolenoid.close();
  }
}
