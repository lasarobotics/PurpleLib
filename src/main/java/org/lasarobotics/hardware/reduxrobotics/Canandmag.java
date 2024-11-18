package org.lasarobotics.hardware.reduxrobotics;

// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

import org.lasarobotics.hardware.LoggableHardware;
import org.lasarobotics.hardware.PurpleManager;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.SensorDirectionValue;

/** CTRE CANCoder */
public class Canandmag extends LoggableHardware {
  /** CANCoder ID */
  public static class ID {
    public final String name;
    public final int deviceID;

    /**
     * CANCoder ID
     * @param name Device name for logging
     * @param deviceID CAN ID
     */
    public ID(String name, int deviceID) {
      this.name = name;
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
  public static class CanandmagInputs {
    public double absolutePosition = 0.0;
    public double relativePosition = 0.0;
    public double velocity = 0.0;
  }

  private com.reduxrobotics.sensors.canandmag.Canandmag m_canandmag;

  private ID m_id;
  private CanandmagInputsAutoLogged m_inputs;

  private double m_absolutePositionCoefficient = 1.0;
  private double m_relativePositionCoefficient = 1.0;
  private double m_velocityCoefficient = 1.0;

  public Canandmag(ID id) {
    this.m_id = id;
    this.m_inputs = new CanandmagInputsAutoLogged();

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
    return m_canandmag.getAbsPosition() * m_absolutePositionCoefficient;
  }

  /**
   * Gets the position of the sensor.  This may be relative or absolute depending on configuration.
   * The units are determined by the internal coefficient, default is rotations.
   * @return The position of the sensor.
   */
  private double getRelativePosition() {
    return m_canandmag.getPosition() * m_relativePositionCoefficient;
  }

  /**
   * Gets the velocity of the sensor.
   * The units are determined by the internal coefficient, default is rotations per second.
   * @return The velocity of the sensor.
   */
  private double getCanVelocity() {
    return m_canandmag.getVelocity() * m_velocityCoefficient;
  }

  /**
   * Gets the velocity of the sensor.
   * The units are determined by the internal coefficient, default is rotations.
   * @return The Velocity of the sensor.
   */
  private void updateInputs() {
    m_inputs.absolutePosition = getAbsolutePosition();
    m_inputs.relativePosition = getRelativePosition();
    m_inputs.velocity = getCanVelocity();
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
  public CanandmagInputsAutoLogged getInputs() {
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
   * @return boolean generated by function. 0 indicates no error.
   */
  public boolean resetPosition(double position) {
    return m_canandmag.setPosition(position / m_relativePositionCoefficient);
  }

 /**
   * Sets the position of the sensor to zero
   * The units are determined by the coefficient and unit-string configuration params, default is degrees.
   * @return boolean generated by function. 0 indicates no error.
   */
  public boolean resetPosition() {
    return resetPosition(0.0);
  }

  /**
   * Configures all persistent settings to defaults (overloaded so timeoutMs is 50 ms).
   *
   * @return boolean generated by function. 0 indicates no error.
   */
  public boolean configFactoryDefault() {
    try{
      m_canandmag.resetFactoryDefaults(true);
      return false;
    } catch(Exception e) {
      return true;
    } 
  }

  

  @Override
  public void close() {
    PurpleManager.remove(this);
    m_canandmag = null;
  }
}

