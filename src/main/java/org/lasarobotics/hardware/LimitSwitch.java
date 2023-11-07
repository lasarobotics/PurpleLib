// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.hardware;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DigitalInput;

public class LimitSwitch implements LoggableHardware, AutoCloseable {
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

  public enum SwitchPolarity {
    NORMALLY_OPEN {
      @Override
      public boolean getValue(boolean value) { return value; }
    },
    NORMALLY_CLOSED {
      @Override
      public boolean getValue(boolean value) { return !value; }
    };

    public abstract boolean getValue(boolean value);
  }

  @AutoLog
  public static class LimitSwitchInputs {
    public boolean value;
  }

  private DigitalInput m_limitSwitch;

  private ID m_id;
  private SwitchPolarity m_switchPolarity;
  private LimitSwitchInputsAutoLogged m_inputs;

  /**
   * Create a limit switch object with built-in logging
   * @param id Limit switch ID
   * @param switchPolarity Switch polarity
   */
  public LimitSwitch(LimitSwitch.ID id, SwitchPolarity switchPolarity) {
    this.m_id = id;
    this.m_switchPolarity = switchPolarity;
    this.m_limitSwitch = new DigitalInput(m_id.port);
    this.m_inputs = new LimitSwitchInputsAutoLogged();
  }

  /**
   * Update sensor input readings
   */
  private void updateInputs() {
    m_inputs.value = m_switchPolarity.getValue(m_limitSwitch.get());
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
  public LimitSwitchInputsAutoLogged getInputs() {
    return m_inputs;
  }

  @Override
  public void close() {
    m_limitSwitch.close();
  }
}
