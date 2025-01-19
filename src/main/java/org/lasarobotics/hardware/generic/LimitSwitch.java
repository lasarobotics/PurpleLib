// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.hardware.generic;

import org.lasarobotics.hardware.LoggableHardware;
import org.lasarobotics.hardware.PurpleManager;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.DigitalInput;

/** Limit switch */
public class LimitSwitch extends LoggableHardware {
  /** Limit switch ID */
  public static class ID {
    public final String name;
    public final int port;

    /**
     * Limit switch ID
     * @param name Device name for logging
     * @param port Port number
     */
    public ID(String name, int port) {
      this.name = name;
      this.port = port;
    }
  }

  /**
   * Switch polarity
   */
  public enum SwitchPolarity {
    /** Normally open switch */
    NORMALLY_OPEN {
      @Override
      public boolean getValue(boolean value) { return value; }
    },
    /** Normally closed switch */
    NORMALLY_CLOSED {
      @Override
      public boolean getValue(boolean value) { return !value; }
    };

    public abstract boolean getValue(boolean value);
  }

  /**
   * Limit switch sensor inputs
   */
  @AutoLog
  public static class LimitSwitchInputs {
    public boolean value = false;
  }

  private DigitalInput m_limitSwitch;

  private ID m_id;
  private SwitchPolarity m_switchPolarity;
  private Frequency m_updateRate;
  private LimitSwitchInputsAutoLogged m_inputs;

  /**
   * Create a limit switch object with built-in logging
   * @param id Limit switch ID
   * @param switchPolarity Switch polarity
   * @param updateRate Update rate of input from limit switch
   */
  public LimitSwitch(LimitSwitch.ID id, SwitchPolarity switchPolarity, Frequency updateRate) {
    this.m_id = id;
    this.m_switchPolarity = switchPolarity;
    this.m_updateRate = updateRate;
    this.m_limitSwitch = new DigitalInput(m_id.port);
    this.m_inputs = new LimitSwitchInputsAutoLogged();

    // Update inputs on init
    periodic();

    // Register device with manager
    PurpleManager.add(this);
  }

  /**
   * Update sensor input readings
   */
  protected void updateInputs() {
    m_inputs.value = m_switchPolarity.getValue(m_limitSwitch.get());
  }

  /**
   * Call this method periodically
   */
  @Override
  protected void periodic() {
    updateInputs();
    Logger.processInputs(m_id.name, m_inputs);
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
  public LimitSwitchInputsAutoLogged getInputs() {
    return m_inputs;
  }

  @Override
  public void close() {
    PurpleManager.remove(this);
    m_limitSwitch.close();
  }
}
