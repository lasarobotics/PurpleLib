// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.hardware.ctre;

import org.lasarobotics.hardware.LoggableHardware;
import org.lasarobotics.hardware.PurpleManager;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.ctre.phoenix.motorcontrol.ControlMode;

/** VictorSPX */
public class VictorSPX extends LoggableHardware {
  /** VictorSPX ID */
  public static class ID {
    public final String name;
    public final int deviceID;

    /**
     * WPI_VictorSPX ID
     * @param name Device name for logging
     * @param deviceID CAN ID
     */
    public ID(String name, int deviceID) {
      this.name = name;
      this.deviceID = deviceID;
    }
  }

  private static final String VALUE_LOG_ENTRY = "/OutputValue";
  private static final String MODE_LOG_ENTRY = "/OutputMode";

  private com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX m_victor;

  private ID m_id;

  /**
   * Create a VictorSPX object with built-in logging
   * @param id VictorSPX ID
   */
  public VictorSPX(VictorSPX.ID id)  {
    this.m_id = id;
    this.m_victor = new com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX(id.deviceID);

    // Disable motor safety
    m_victor.setSafetyEnabled(false);

    // Update inputs on init
    periodic();

    // Register device with manager
    PurpleManager.add(this);
  }

  /**
   * Log output values
   * @param value Value that was set
   * @param mode The output mode to apply
   */
  private void logOutputs(ControlMode mode, double value) {
    Logger.recordOutput(m_id.name + VALUE_LOG_ENTRY, value);
    Logger.recordOutput(m_id.name + MODE_LOG_ENTRY, mode.toString());
  }

  @Override
  protected void periodic() {}

  @Override
  public LoggableInputs getInputs() {
    return null;
  }

  /**
   * Get device ID
   * @return Device ID
   */
  public ID getID() {
    return m_id;
  }

  /**
   * Common interface for setting the speed of a simple speed controller.
   *
   * @param speed The speed to set. Value should be between -1.0 and 1.0.
   *                   Value is also saved for Get().
   */
  public void set(double speed) {
    set(ControlMode.PercentOutput, speed);
    logOutputs(ControlMode.PercentOutput, speed);
  }

  /**
   * Sets the appropriate output on the talon, depending on the mode.
   * @param mode The output mode to apply.
   * In PercentOutput, the output is between -1.0 and 1.0, with 0.0 as stopped.
   * In Current mode, output value is in amperes.
   * In Velocity mode, output value is in position change / 100ms.
   * In Position mode, output value is in encoder ticks or an analog value,
   *   depending on the sensor.
   * In Follower mode, the output value is the integer device ID of the talon to
   * duplicate.
   *
   * @param value The setpoint value, as described above.
   */
  public void set(ControlMode mode, double value) {
    m_victor.set(mode, value);
    logOutputs(mode, value);
  }

  /**
   * Common interface for inverting direction of a speed controller.
   *
   * @param isInverted The state of inversion, true is inverted.
   */
  public void setInverted(boolean isInverted) {
    m_victor.setInverted(isInverted);
  }

  /**
   * Sets the voltage output of the MotorController.  Compensates for the current bus
   * voltage to ensure that the desired voltage is output even if the battery voltage is below
   * 12V - highly useful when the voltage outputs are "meaningful" (e.g. they come from a
   * feedforward calculation).
   *
   * <p>NOTE: This function *must* be called regularly in order for voltage compensation to work
   * properly - unlike the ordinary set function, it is not "set it and forget it."
   *
   * @param outputVolts The voltage to output.
   */
  public void setVoltage(double outputVolts) {
    m_victor.setVoltage(outputVolts);
  }

  /**
   * Common interface to stop the motor until Set is called again.
   */
  public void stopMotor() {
    m_victor.stopMotor();
    logOutputs(ControlMode.PercentOutput, 0.0);
  }

  /**
   * Closes the VictorSPX motor controller
   */
  @Override
  public void close() {
    PurpleManager.remove(this);
    m_victor.close();
  }
}
