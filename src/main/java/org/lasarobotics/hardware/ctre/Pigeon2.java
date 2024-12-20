// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.hardware.ctre;

import org.lasarobotics.hardware.LoggableHardware;
import org.lasarobotics.hardware.PurpleManager;
import org.lasarobotics.utils.GlobalConstants;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.Notifier;

/** CTRE Pigeon 2.0 */
public class Pigeon2 extends LoggableHardware {
  /** Pigeon ID */
  public static class ID {
    public final String name;
    public final PhoenixCANBus bus;
    public final int deviceID;

    /**
     * Pidgeon2 ID
     * @param name Device name for logging
     * @param bus CAN bus
     * @param deviceID CAN ID
     */
    public ID(String name, PhoenixCANBus bus, int deviceID) {
      this.name = name;
      this.bus = bus;
      this.deviceID = deviceID;
    }
  }

  /** Pigeon Status Frame */
  public enum PigeonStatusFrame {
    PITCH,
    YAW,
    ROLL
  }

  /**
   * Pigeon sensor inputs
   */
  @AutoLog
  public static class Pigeon2Inputs {
    public Measure<Angle> pitchAngle = Units.Radians.of(0.0);
    public Measure<Angle> yawAngle = Units.Radians.of(0.0);
    public Measure<Angle> rollAngle = Units.Radians.of(0.0);
    public Measure<Velocity<Angle>> yawRate = Units.RadiansPerSecond.of(0.0);
    public Rotation2d rotation2d = GlobalConstants.ROTATION_ZERO;
  }

  private int DEFAULT_THREAD_PERIOD = 10;

  private Notifier m_inputThread;
  private Measure<Time> m_inputThreadPeriod = Units.Milliseconds.of(DEFAULT_THREAD_PERIOD);

  private com.ctre.phoenix6.hardware.Pigeon2 m_pigeon;

  private ID m_id;
  private Pigeon2InputsAutoLogged m_inputs;

  public Pigeon2(ID id) {
    this.m_id = id;
    this.m_pigeon = new com.ctre.phoenix6.hardware.Pigeon2(id.deviceID, id.bus.name);
    this.m_inputs = new Pigeon2InputsAutoLogged();
    this.m_inputThread = new Notifier(this::updateInputs);

    // Start input thread
    m_inputThread.startPeriodic(m_inputThreadPeriod.in(Units.Seconds));

    // Update inputs on init
    periodic();

    // Register device with manager
    PurpleManager.add(this);
  }

  /**
	 * Get the pitch from the Pigeon
	 * @return Pitch
	 */
  private double getPitch() {
    return m_pigeon.getPitch().getValue();
  }

  /**
   * Returns the heading of the robot in degrees.
   * <p>
   * The angle increases as the Pigeon 2 turns clockwise when looked
   * at from the top. This follows the NED axis convention.
   * <p>
   * The angle is continuous; that is, it will continue from 360 to
   * 361 degrees. This allows for algorithms that wouldn't want to
   * see a discontinuity in the gyro output as it sweeps past from
   * 360 to 0 on the second time around.
   *
   * @return The current heading of the robot in degrees
   */
  private double getAngle() {
    return m_pigeon.getAngle();
  }

  /**
   * Returns the heading of the robot as a {@link Rotation2d}.
   * <p>
   * The angle increases as the Pigeon 2 turns counterclockwise when
   * looked at from the top. This follows the NWU axis convention.
   * <p>
   * The angle is continuous; that is, it will continue from 360 to
   * 361 degrees. This allows for algorithms that wouldn't want to
   * see a discontinuity in the gyro output as it sweeps past from
   * 360 to 0 on the second time around.
   *
   * @return The current heading of the robot as a {@link Rotation2d}
   */
  private Rotation2d getRotation2d() {
    return m_pigeon.getRotation2d();
  }

  /**
	 * Get the roll from the Pigeon
	 * @return Roll
	 */
  private double getRoll() {
    return m_pigeon.getRoll().getValue();
  }

  /**
   * Return the rate of rotation of the yaw (Z-axis) gyro, in degrees per second.
   *<p>
   * The rate is based on the most recent reading of the yaw gyro angle.
   *<p>
   * @return The current rate of change in yaw angle (in degrees per second)
   */
  private double getRate() {
    return m_pigeon.getRate();
  }


  /**
   * Update Pidgeon input readings
   */
  private void updateInputs() {
    m_inputs.pitchAngle = Units.Degrees.of(getPitch());
    m_inputs.yawAngle = Units.Degrees.of(getAngle());
    m_inputs.rollAngle = Units.Degrees.of(getRoll());
    m_inputs.yawRate = Units.DegreesPerSecond.of(getRate());
    m_inputs.rotation2d = getRotation2d();
  }

  /**
   * Call this method periodically
   */
  @Override
  protected void periodic() {
    Logger.processInputs(m_id.name, m_inputs);
  }

  /**
   * Get latest sensor input data
   * @return Latest sensor data
   */
  @Override
  public Pigeon2InputsAutoLogged getInputs() {
    synchronized (m_inputs) { return m_inputs; }
  }

  /**
   * Get device ID
   * @return Device ID
   */
  public ID getID() {
    return m_id;
  }

  /**
   * Set input thread period
   * <p>
   * Defaults to 10ms
   * @param period Period between getting sensor updates
   */
  public void setPeriod(Measure<Time> period) {
    m_inputThreadPeriod = period;
    m_inputThread.stop();
    m_inputThread.startPeriodic(m_inputThreadPeriod.in(Units.Seconds));
  }

  /**
   * Configures all persistent settings to defaults (overloaded so timeoutMs is 50 ms).
   *
   * @return Status Code generated by function. 0 indicates no error.
   */
  public StatusCode configFactoryDefault() {
    return m_pigeon.getConfigurator().apply(new Pigeon2Configuration());
  }

	/**
	 * Configure the Mount Pose using pitch, roll, and yaw.
	 *
	 * @param pitch The mounting calibration pitch-component
	 * @param roll The mounting calibration roll-component
   * @param yaw The mounting calibration yaw-component
	 * @return Status Code of the set command.
	 */
  public StatusCode configMountPose(double pitch, double roll, double yaw) {
    var toApply = new MountPoseConfigs();
    toApply.MountPosePitch = pitch;
    toApply.MountPoseRoll = roll;
    toApply.MountPoseYaw = yaw;
    return m_pigeon.getConfigurator().apply(toApply);
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
  public StatusCode setStatusFramePeriod(PigeonStatusFrame statusFrame, int frequencyHz) {
    switch (statusFrame) {
      case PITCH:
        return m_pigeon.getPitch().setUpdateFrequency(frequencyHz);
      case YAW:
        return m_pigeon.getYaw().setUpdateFrequency(frequencyHz);
      case ROLL:
        return m_pigeon.getRoll().setUpdateFrequency(frequencyHz);
      default:
        return StatusCode.OK;
    }
  }

  /**
   * Resets the Pigeon 2 to a heading of zero.
   * <p>
   * This can be used if there is significant drift in the gyro,
   * and it needs to be recalibrated after it has been running.
   */
  public void reset() {
    m_pigeon.reset();
  }

  @Override
  public void close() {
    PurpleManager.remove(this);
    m_pigeon.close();
  }
}
