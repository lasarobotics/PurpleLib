// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.hardware.ctre;

import org.lasarobotics.hardware.LoggableHardware;
import org.lasarobotics.hardware.PurpleManager;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.SensorDirectionValue;

/** CTRE CANCoder */
public class CANCoder extends LoggableHardware {
  /** CANCoder ID */
  public static class ID {
    public final String name;
    public final PhoenixCANBus bus;
    public final int deviceID;

    /**
     * CANCoder ID
     * @param name Device name for logging
     * @param bus CAN bus device is connected to
     * @param deviceID CAN ID
     */
    public ID(String name, PhoenixCANBus bus, int deviceID) {
      this.name = name;
      this.bus = bus;
      this.deviceID = deviceID;
    }
  }

  /** CANCoder Status Frame */
  public enum CANCoderFrame {
    ABSOLUTE_POSITION,
    RELATIVE_POSITION,
    VELOCITY
  }

  @AutoLog
  public static class CANCoderInputs {
    public double absolutePosition = 0.0;
    public double relativePosition = 0.0;
    public double velocity = 0.0;
  }

  private com.ctre.phoenix6.hardware.CANcoder m_canCoder;

  private ID m_id;
  private CANCoderInputsAutoLogged m_inputs;

  private double m_absolutePositionCoefficient = 1.0;
  private double m_relativePositionCoefficient = 1.0;
  private double m_velocityCoefficient = 1.0;

  public CANCoder(ID id) {
    this.m_id = id;
    this.m_canCoder = new com.ctre.phoenix6.hardware.CANcoder(m_id.deviceID, m_id.bus.name);
    this.m_inputs = new CANCoderInputsAutoLogged();

    // Update inputs on init
    periodic();

    // Register device with manager
    PurpleManager.add(this);
  }

   /**
     * Gets the absolute position of the sensor.
     * The absolute position may be unsigned (for example: [0,360) deg), or signed (for example: [-180,+180) deg). This is determined by a configuration. The default selection is unsigned.
     * The units are determined by the internal coefficient, default is rotations.
     * Note: this signal is not affected by calls to SetPosition().
     * @return The position of the sensor.
     */
  private double getAbsolutePosition() {
    return m_canCoder.getAbsolutePosition().getValue() * m_absolutePositionCoefficient;
  }

  /**
   * Gets the position of the sensor.  This may be relative or absolute depending on configuration.
   * The units are determined by the internal coefficient, default is rotations.
   * @return The position of the sensor.
   */
  private double getRelativePosition() {
    return m_canCoder.getPosition().getValue() * m_relativePositionCoefficient;
  }

  /**
   * Gets the velocity of the sensor.
   * The units are determined by the internal coefficient, default is rotations per second.
   * @return The velocity of the sensor.
   */
  private double getVelocity() {
    return m_canCoder.getVelocity().getValue() * m_velocityCoefficient;
  }

  /**
   * Gets the velocity of the sensor.
   * The units are determined by the internal coefficient, default is rotations.
   * @return The Velocity of the sensor.
   */
  private void updateInputs() {
    m_inputs.absolutePosition = getAbsolutePosition();
    m_inputs.relativePosition = getRelativePosition();
    m_inputs.velocity = getVelocity();
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
  public CANCoderInputsAutoLogged getInputs() {
    return m_inputs;
  }

  /**
   * Get device ID
   * @return Device ID
   */
  public ID getID() {
    return m_id;
  }

  /**
   * Sets the Absolute Position coefficent to allow control of units.
   * @param absolutePositionCoefficient Scalar to multiply the absolute position by when it is returned. Defaults to 1.0.
   */
  public void setAbsolutePositionCoefficient(double absolutePositionCoefficient) {
    m_absolutePositionCoefficient = absolutePositionCoefficient;
  }

  /**
   * Sets the Relative Position coefficent to allow control of units.
   * @param relativePositionCoefficient Scalar to multiply the relative position by when it is returned. Defaults to 1.0.
   */
  public void setRelativePositionCoefficient(double relativePositionCoefficient) {
    m_relativePositionCoefficient = relativePositionCoefficient;
  }

  /**
   * Sets the Velocity coefficent to allow control of units.
   * @param velocityCoefficient Scalar to multiply the absolute Velocity by when it is returned. Defaults to 1.0.
   */
  public void setVelocityCoefficient(double velocityCoefficient) {
    m_velocityCoefficient = velocityCoefficient;
  }

  /**
   * Sets the position of the sensor to specified value
   * The units are determined by the coefficient and unit-string configuration params, default is rotations.
   * @param position Position to reset to
   * @return StatusCode generated by function. 0 indicates no error.
   */
  public StatusCode resetPosition(double position) {
    return m_canCoder.setPosition(position / m_relativePositionCoefficient);
  }

 /**
   * Sets the position of the sensor to zero
   * The units are determined by the coefficient and unit-string configuration params, default is degrees.
   * @return StatusCode generated by function. 0 indicates no error.
   */
  public StatusCode resetPosition() {
    return resetPosition(0.0);
  }

  /**
   * Configures all persistent settings to defaults (overloaded so timeoutMs is 50 ms).
   *
   * @return Status Code generated by function. 0 indicates no error.
   */
  public StatusCode configFactoryDefault() {
    return m_canCoder.getConfigurator().apply(new CANcoderConfiguration());
  }

  /**
   * Configures direction of the sensor to determine positive facing the LED side of the CANcoder.
   * @param direction The new sensor direction
   * @return StatusCode generated by function. 0 indicates no error.
   */
  public StatusCode configSensorDirection(SensorDirectionValue direction) {
    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.SensorDirection = direction;

    return m_canCoder.getConfigurator().apply(config);
  }

	/**
	 * Sets the period of the given status frame.
	 *
	 * @param statusFrame
	 *            Frame whose period is to be changed.
	 * @param frequencyHz
	 *            Frequency in Hz for the given frame.
	 * @return Status Code generated by function. 0 indicates no error.
	 */
  public StatusCode setStatusFramePeriod(CANCoderFrame statusFrame, int frequencyHz) {
    switch (statusFrame) {
      case ABSOLUTE_POSITION:
        return m_canCoder.getAbsolutePosition().setUpdateFrequency(frequencyHz);
      case RELATIVE_POSITION:
        return m_canCoder.getPosition().setUpdateFrequency(frequencyHz);
      case VELOCITY:
        return m_canCoder.getVelocity().setUpdateFrequency(frequencyHz);
      default:
        return StatusCode.OK;
    }
  }

  @Override
  public void close() {
    PurpleManager.remove(this);
    m_canCoder = null;
  }
}
