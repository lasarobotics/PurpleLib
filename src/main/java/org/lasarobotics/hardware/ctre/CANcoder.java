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
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.sim.CANcoderSimState;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj.Notifier;

/** CTRE CANCoder */
public class CANcoder extends LoggableHardware {
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
  public enum CANcoderFrame {
    ABSOLUTE_POSITION,
    RELATIVE_POSITION,
    VELOCITY
  }

  @AutoLog
  public static class CANcoderInputs {
    public MutAngle absolutePosition = Units.Radians.zero().mutableCopy();
    public MutAngle relativePosition = Units.Radians.zero().mutableCopy();
    public MutAngularVelocity velocity = Units.RadiansPerSecond.zero().mutableCopy();
  }

  private com.ctre.phoenix6.hardware.CANcoder m_canCoder;

  private ID m_id;
  private Notifier m_inputThread;
  private CANcoderInputsAutoLogged m_inputs;

  /**
   * Create a CANcoder object with built-in logging
   * @param id CANcoder ID
   * @param updateRate Update rate of CANcoder inputs
   */
  public CANcoder(ID id, Frequency updateRate) {
    this.m_id = id;
    this.m_canCoder = new com.ctre.phoenix6.hardware.CANcoder(m_id.deviceID, m_id.bus.name);
    this.m_inputs = new CANcoderInputsAutoLogged();
    this.m_inputThread = new Notifier(this::updateInputs);

    // Update inputs on init
    updateInputs();
    periodic();

    // Set update rate of status frames
    m_canCoder.getAbsolutePosition().setUpdateFrequency(updateRate);
    m_canCoder.getPosition().setUpdateFrequency(updateRate);
    m_canCoder.getVelocity().setUpdateFrequency(updateRate);

    // Register device with manager
    PurpleManager.add(this);

    // Start sensor input thread
    m_inputThread.setName(m_id.name);
    if (!Logger.hasReplaySource()) m_inputThread.startPeriodic(updateRate.asPeriod().in(Units.Seconds));
  }

   /**
     * Gets the absolute position of the sensor.
     * The absolute position may be unsigned (for example: [0,360) deg), or signed (for example: [-180,+180) deg). This is determined by a configuration. The default selection is unsigned.
     * The units are determined by the internal coefficient, default is rotations.
     * Note: this signal is not affected by calls to SetPosition().
     * @return The position of the sensor.
     */
  private Angle getAbsolutePosition() {
    return m_canCoder.getAbsolutePosition().getValue();
  }

  /**
   * Gets the position of the sensor.  This may be relative or absolute depending on configuration.
   * The units are determined by the internal coefficient, default is rotations.
   * @return The position of the sensor.
   */
  private Angle getRelativePosition() {
    return m_canCoder.getPosition().getValue();
  }

  /**
   * Gets the velocity of the sensor.
   * The units are determined by the internal coefficient, default is rotations per second.
   * @return The velocity of the sensor.
   */
  private AngularVelocity getVelocity() {
    return m_canCoder.getVelocity().getValue();
  }

  /**
   * Update sensor input readings
   */
  protected void updateInputs() {
    synchronized (m_inputs) {
      m_inputs.absolutePosition.mut_replace(getAbsolutePosition());
      m_inputs.relativePosition.mut_replace(getRelativePosition());
      m_inputs.velocity.mut_replace(getVelocity());
    }
  }

  /**
   * Call this method periodically
   */
  @Override
  protected void periodic() {
    synchronized (m_inputs) { Logger.processInputs(m_id.name, m_inputs); }
  }

  /**
   * Get latest sensor input data
   * @return Latest sensor data
   */
  @Override
  public CANcoderInputsAutoLogged getInputs() {
    synchronized (m_inputs) { return m_inputs; }
  }

  @Override
  public boolean isHealthy() {
    return getMagnetHealth().equals(MagnetHealthValue.Magnet_Green);
  }

  /**
   * Get device ID
   * @return Device ID
   */
  public ID getID() {
    return m_id;
  }

  /**
   * Applies the contents of the specified config to the device.
   * Call to apply the selected configs.
   *
   * @param configs Configs to apply against.
   * @return StatusCode of the set command
   */
  public StatusCode applyConfigs(CANcoderConfiguration configs) {
    return m_canCoder.getConfigurator().apply(configs);
  }

  /**
   * Sets the position of the sensor to specified value
   * The units are determined by the coefficient and unit-string configuration params, default is rotations.
   * @param position Position to reset to
   * @return StatusCode generated by function. 0 indicates no error.
   */
  public StatusCode resetPosition(double position) {
    return m_canCoder.setPosition(position);
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
   * Get the simulation state for this device.
   * <p>
   * This function reuses an allocated simulation state
   * object, so it is safe to call this function multiple
   * times in a robot loop.
   *
   * @return Simulation state
   */
  public CANcoderSimState getSimState() {
    return m_canCoder.getSimState();
  }

  /**
   * Magnet health as measured by CANcoder.
   * <p>
   * Red indicates too close or too far, Orange is adequate but with
   * reduced accuracy, green is ideal. Invalid means the accuracy cannot
   * be determined.
   *
   * This refreshes and returns a cached object.
   *
   * @return MagnetHealth Object
   */
  public MagnetHealthValue getMagnetHealth() {
    return m_canCoder.getMagnetHealth().getValue();
  }

	/**
	 * Sets the period of the given status frame.
	 *
	 * @param statusFrame
	 *            Frame whose period is to be changed.
	 * @param frequency
	 *            Frequency for the given frame.
	 * @return Status Code generated by function. 0 indicates no error.
	 */
  public StatusCode setStatusFramePeriod(CANcoderFrame statusFrame, Frequency frequency) {
    switch (statusFrame) {
      case ABSOLUTE_POSITION:
        return m_canCoder.getAbsolutePosition().setUpdateFrequency(frequency);
      case RELATIVE_POSITION:
        return m_canCoder.getPosition().setUpdateFrequency(null);
      case VELOCITY:
        return m_canCoder.getVelocity().setUpdateFrequency(null);
      default:
        return StatusCode.OK;
    }
  }

  @Override
  public String getName() {
    return this.getID().name;
  }

  @Override
  public void close() {
    PurpleManager.remove(this);
    m_canCoder = null;
  }
}
