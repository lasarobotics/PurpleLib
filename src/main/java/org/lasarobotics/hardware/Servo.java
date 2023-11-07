// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.hardware;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class Servo implements LoggableHardware {
  /** Servo ID */
  public static class ID {
    public final String name;
    public final int port;

    /**
     * Servo ID
     * @param name Device name for logging
     * @param port Port number
     */
    public ID(String name, int port) {
      this.name = name;
      this.port = port;
    }
  }

  private static final String VALUE_LOG_ENTRY = "/OutputValue";

  private edu.wpi.first.wpilibj.Servo m_servo;

  private ID m_id;
  private double m_conversionFactor;

  /**
   * Create a servo object with built-in logging
   * @param id Servo ID
   * @param conversionFactor Output conversion factor
   */
  public Servo(Servo.ID id, double conversionFactor) {
    this.m_id = id;
    this.m_conversionFactor = conversionFactor;
    this.m_servo = new edu.wpi.first.wpilibj.Servo(m_id.port);
  }

  /**
   * Create a servo object with built-in logging
   * @param id Servo ID
   */
  public Servo(Servo.ID id) {
    this(id, 1.0);
  }

  private void logOutputs(double value) {
    Logger.getInstance().recordOutput(m_id.name + VALUE_LOG_ENTRY, value);
  }

  @Override
  public void periodic() {}

  @Override
  public LoggableInputs getInputs() {
    return null;
  }

  /**
   * Set conversion factor for output
   * @param conversionFactor Conversion factor
   */
  public void setConversionFactor(double conversionFactor) {
    m_conversionFactor = conversionFactor;
  }

  /**
   * Set position of servo
   * @param value Position value
   */
  public void setPosition(double value) {
    m_servo.set(value / m_conversionFactor);
    logOutputs(value);
  }
}
