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
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.Time;
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
    public MutAngle pitchAngle = Units.Radians.of(0.0).mutableCopy();
    public MutAngle yawAngle = Units.Radians.of(0.0).mutableCopy();
    public MutAngle rollAngle = Units.Radians.of(0.0).mutableCopy();
    public MutAngularVelocity yawRate = Units.RadiansPerSecond.of(0.0).mutableCopy();
    public Rotation2d rotation2d = GlobalConstants.ROTATION_ZERO;
  }

  private static final int DEFAULT_THREAD_PERIOD = 10;

  private Notifier m_inputThread;
  private Time m_inputThreadPeriod;

  private com.ctre.phoenix6.hardware.Pigeon2 m_pigeon;

  private ID m_id;
  private Pigeon2InputsAutoLogged m_inputs;


  /**
   * Create a Pigeon 2.0 object with built-in logging
   * <p>
   * Input thread period of {@value Pigeon2#DEFAULT_THREAD_PERIOD} ms
   * @param id Pigeon 2.0 ID
   */
  public Pigeon2(ID id) {
    this(id, Units.Milliseconds.of(DEFAULT_THREAD_PERIOD));
  }

  /**
   * Create a Pigeon 2.0 object with built-in logging
   * @param id Pigeon 2.0 ID
   * @param inputThreadPeriod Execution period of getting inputs from Pigeon
   */
  public Pigeon2(ID id, Time inputThreadPeriod) {
    this.m_id = id;
    this.m_pigeon = new com.ctre.phoenix6.hardware.Pigeon2(id.deviceID, id.bus.name);
    this.m_inputs = new Pigeon2InputsAutoLogged();
    this.m_inputThread = new Notifier(this::updateInputs);
    this.m_inputThreadPeriod = inputThreadPeriod;

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
  private Angle getPitch() {
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
  private Angle getAngle() {
    return m_pigeon.getYaw().getValue();
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
   * Current reported roll of the Pigeon2.
   *
   * <ul>
   *   <li> <b>Minimum Value:</b> -180.0
   *   <li> <b>Maximum Value:</b> 179.9560546875
   *   <li> <b>Default Value:</b> 0
   *   <li> <b>Units:</b> deg
   * </ul>
   *
   * Default Rates:
   * <ul>
   *   <li> <b>CAN 2.0:</b> 100.0 Hz
   *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * This refreshes and returns a cached StatusSignal object.
   *
   * @return Roll angle
   */
  private Angle getRoll() {
    return m_pigeon.getRoll().getValue();
  }

  /**
   * Angular Velocity Quaternion Z Component
   * <p>
   * This is the Z component of the angular velocity with respect to the
   * world frame and is mount-calibrated.
   *
   * <ul>
   *   <li> <b>Minimum Value:</b> -2048.0
   *   <li> <b>Maximum Value:</b> 2047.99609375
   *   <li> <b>Default Value:</b> 0
   *   <li> <b>Units:</b> dps
   * </ul>
   *
   * Default Rates:
   * <ul>
   *   <li> <b>CAN 2.0:</b> 10.0 Hz
   *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * This refreshes and returns a cached StatusSignal object.
   *
   * @return Yaw angular velocity
   */
  private AngularVelocity getRate() {
    return m_pigeon.getAngularVelocityZWorld().getValue();
  }


  /**
   * Update Pidgeon input readings
   */
  private void updateInputs() {
    m_inputs.pitchAngle.mut_replace(getPitch());
    m_inputs.yawAngle.mut_replace(getAngle());
    m_inputs.rollAngle.mut_replace(getRoll());
    m_inputs.yawRate.mut_replace(getRate());
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
