// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.hardware.kauailabs;

import org.lasarobotics.hardware.LoggableHardware;
import org.lasarobotics.hardware.PurpleManager;
import org.lasarobotics.utils.GlobalConstants;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.SPI;

/** NavX2 */
public class NavX2 extends LoggableHardware {
  /** NavX2 ID */
  public static class ID {
    public final String name;

    /**
     * NavX2 ID
     * @param name Device name for logging
     */
    public ID(String name) {
      this.name = name;
    }
  }

  /**
   * NavX sensor inputs
   */
  @AutoLog
  public static class NavX2Inputs {
    public boolean isConnected = false;
    public Measure<Angle> pitchAngle = Units.Radians.of(0.0);
    public Measure<Angle> yawAngle = Units.Radians.of(0.0);
    public Measure<Angle> rollAngle = Units.Radians.of(0.0);
    public Measure<Velocity<Distance>> xVelocity = Units.MetersPerSecond.of(0.0);
    public Measure<Velocity<Distance>> yVelocity = Units.MetersPerSecond.of(0.0);
    public Measure<Velocity<Angle>> yawRate = Units.RadiansPerSecond.of(0.0);
    public Rotation2d rotation2d = GlobalConstants.ROTATION_ZERO;
  }

  private static final int UPDATE_RATE = 200;

  private AHRS m_navx;
  private SimDouble m_simNavXYaw;
  private Notifier m_inputThread;

  private String m_name;
  private NavX2InputsAutoLogged m_inputs;

  /**
   * Create a NavX2 object with built-in logging
   * @param id NavX2 ID
   * @param updateRate Custom update rate (Hz)
   */
  public NavX2(ID id) {
    this.m_name = id.name;
    this.m_navx = new AHRS(SPI.Port.kMXP, (byte)UPDATE_RATE);
    this.m_inputs = new NavX2InputsAutoLogged();
    this.m_inputThread = new Notifier(this::updateInputs);
    this.m_simNavXYaw = new SimDouble(SimDeviceDataJNI.getSimValueHandle(SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]"), "Yaw"));
    System.out.println();

    // Start input thread
    m_inputThread.startPeriodic(1.0 / UPDATE_RATE);

    // Update inputs on init
    periodic();

    // Register device with manager
    PurpleManager.add(this);
  }

  /**
   * Update NavX input readings
   */
  private void updateInputs() {
    m_inputs.isConnected = m_navx.isConnected();
    m_inputs.pitchAngle = Units.Degrees.of(m_navx.getPitch());
    m_inputs.yawAngle = Units.Degrees.of(m_navx.getAngle());
    m_inputs.rollAngle = Units.Degrees.of(m_navx.getRoll());
    m_inputs.xVelocity = Units.MetersPerSecond.of(m_navx.getVelocityX());
    m_inputs.yVelocity = Units.MetersPerSecond.of(m_navx.getVelocityY());
    m_inputs.yawRate = Units.DegreesPerSecond.of(m_navx.getRate());
    m_inputs.rotation2d = Rotation2d.fromDegrees(m_inputs.yawAngle.negate().in(Units.Radians));
  }

  /**
   * Call this method periodically
   */
  @Override
  protected void periodic() {
    Logger.processInputs(m_name, m_inputs);
  }

  /**
   * Get latest sensor input data
   * @return Latest NavX data
   */
  @Override
  public NavX2InputsAutoLogged getInputs() {
    synchronized (m_inputs) { return m_inputs; }
  }

  /**
   * Returns true if the sensor is currently performing automatic
   * gyro/accelerometer calibration. Automatic calibration occurs when the
   * sensor is initially powered on, during which time the sensor should be
   * held still, with the Z-axis pointing up (perpendicular to the earth).
   * <p>
   * NOTE: During this automatic calibration, the yaw, pitch and roll values
   * returned may not be accurate.
   * <p>
   * Once calibration is complete, the sensor will automatically remove an
   * internal yaw offset value from all reported values.
   * @return Returns true if the sensor is currently automatically calibrating the gyro
   */
  public boolean isCalibrating() {
    return m_navx.isCalibrating();
  }

  /**
   * Reset the Yaw gyro.
   * <p>
   * Resets the Gyro Z (Yaw) axis to a heading of zero. This can be used if
   * there is significant drift in the gyro and it needs to be recalibrated
   * after it has been running.
   */
  public void reset() {
    m_navx.reset();
    m_simNavXYaw.set(0.0);
  }

  /**
   * Set yaw angle for simulator
   * @param angle Angle to set in degrees
   */
  public void setSimAngle(double angle) {
    m_simNavXYaw.set(angle);
  }

  /**
   * Get yaw angle for simulator
   * @return Simulated angle that was set
   */
  public double getSimAngle() {
    return m_simNavXYaw.get();
  }

  /**
   * Closes the NavX
   */
  @Override
  public void close() {
    PurpleManager.remove(this);
    m_navx = null;
  }
}
