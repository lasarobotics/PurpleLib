// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.hardware.generic;

import java.util.function.BiConsumer;

import org.lasarobotics.hardware.LoggableHardware;
import org.lasarobotics.hardware.PurpleManager;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.AsynchronousInterrupt;
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
   * Limit switch sensor inputs
   */
  @AutoLog
  public static class LimitSwitchInputs {
    /**
     * True when active for normally open or normally closed
     */
    public boolean value = false;
  }

  private DigitalInput m_limitSwitch;

  private ID m_id;
  private Frequency m_updateRate;
  private volatile LimitSwitchInputsAutoLogged m_inputs;

  /**
   * Create a limit switch object with built-in logging
   * @param id Limit switch ID
   * @param updateRate Update rate of input from limit switch
   */
  public LimitSwitch(LimitSwitch.ID id, Frequency updateRate) {
    this.m_id = id;
    this.m_updateRate = updateRate;
    this.m_limitSwitch = new DigitalInput(m_id.port);
    this.m_inputs = new LimitSwitchInputsAutoLogged();

    // Update inputs on init
    updateInputs();
    periodic();

    // Register device with manager
    PurpleManager.add(this);
  }

  /**
   * Update sensor input readings
   */
  protected void updateInputs() {
    m_inputs.value = m_limitSwitch.get();
  }

  /**
   * Call this method periodically
   */
  @Override
  protected void periodic() {
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
    synchronized (m_inputs) { return m_inputs; }
  }

  /**
   * Bind callback to rising and/or falling edge of limit switch signal as asynchronous interrupt
   * <p>
   * The first bool in the callback indicates the rising edge triggered the interrupt, the second
   * bool is falling edge.
   * <p>
   * Interrupt is enabled by default
   * @param callback Callback to run when interrupt is activated
   * @param risingEdge Trigger on rising edge
   * @param fallingEdge Trigger on falling edge
   * @return Interrupt that was bound
   */
  public AsynchronousInterrupt bindInterrupt(BiConsumer<Boolean, Boolean> callback, boolean risingEdge, boolean fallingEdge) {
    var interrupt = new AsynchronousInterrupt(m_limitSwitch, callback);
    interrupt.setInterruptEdges(risingEdge, fallingEdge);
    interrupt.enable();

    return interrupt;
  }

  @Override
  public void close() {
    PurpleManager.remove(this);
    m_limitSwitch.close();
  }
}
