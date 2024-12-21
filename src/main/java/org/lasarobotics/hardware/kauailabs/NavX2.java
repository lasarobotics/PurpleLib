// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.hardware.kauailabs;

import org.lasarobotics.hardware.LoggableHardware;
import org.lasarobotics.hardware.PurpleManager;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS.NavXUpdateRate;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutLinearAcceleration;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.wpilibj.Notifier;

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
    public MutAngle rollAngle = Units.Radians.zero().mutableCopy();
    public MutAngle pitchAngle = Units.Radians.zero().mutableCopy();
    public MutAngle yawAngle = Units.Radians.zero().mutableCopy();
    public MutLinearAcceleration xAcceleration = Units.MetersPerSecondPerSecond.zero().mutableCopy();
    public MutLinearAcceleration yAcceleration = Units.MetersPerSecondPerSecond.zero().mutableCopy();
    public MutLinearAcceleration zAcceleration = Units.MetersPerSecondPerSecond.zero().mutableCopy();
    public MutLinearVelocity xVelocity = Units.MetersPerSecond.zero().mutableCopy();
    public MutLinearVelocity yVelocity = Units.MetersPerSecond.zero().mutableCopy();
    public MutLinearVelocity zVelocity = Units.MetersPerSecond.zero().mutableCopy();
    public MutAngularVelocity yawRate = Units.RadiansPerSecond.zero().mutableCopy();
    public Rotation2d rotation2d = Rotation2d.kZero;
  }

  private AHRS m_navx;
  private Notifier m_inputThread;

  private String m_name;
  private NavX2InputsAutoLogged m_inputs;

  private boolean m_fieldCentricVelocities;

  /**
   * Create a NavX2 object with built-in logging
   * @param id NavX2 ID
   */
  public NavX2(ID id) {
    this.m_name = id.name;
    this.m_navx = new AHRS(NavXComType.kMXP_SPI, NavXUpdateRate.k200Hz);
    this.m_inputs = new NavX2InputsAutoLogged();
    this.m_inputThread = new Notifier(this::updateInputs);
    this.m_fieldCentricVelocities = false;
    System.out.println();

    // Update inputs on init
    updateInputs();

    // Start input thread
    m_inputThread.startPeriodic(1.0 / NavXUpdateRate.k200Hz.getValue());

    // Register device with manager
    PurpleManager.add(this);
  }

  /**
   * Get NavX port number
   * @return Port number
   */
  int getPort() {
    return m_navx.getPort();
  }

  /**
   * Update NavX input readings
   */
  private void updateInputs() {
    m_inputs.isConnected = m_navx.isConnected();
    m_inputs.rollAngle.mut_replace(m_navx.getRoll(), Units.Degrees);
    m_inputs.pitchAngle.mut_replace(m_navx.getPitch(), Units.Degrees);
    m_inputs.yawAngle.mut_replace(m_navx.getAngle(), Units.Degrees);
    m_inputs.xAcceleration.mut_replace(m_navx.getWorldLinearAccelX(), Units.Gs);
    m_inputs.yAcceleration.mut_replace(m_navx.getWorldLinearAccelY(), Units.Gs);
    m_inputs.zAcceleration.mut_replace(m_navx.getWorldLinearAccelZ(), Units.Gs);
    //m_inputs.xVelocity.mut_replace((m_fieldCentricVelocities) ? m_navx.getVelocityX() : m_navx.getRobotCentricVelocityX(), Units.MetersPerSecond);
    //m_inputs.yVelocity.mut_replace((m_fieldCentricVelocities) ? m_navx.getVelocityY() : m_navx.getRobotCentricVelocityY(), Units.MetersPerSecond);
    //m_inputs.zVelocity.mut_replace((m_fieldCentricVelocities) ? m_navx.getVelocityZ() : m_navx.getRobotCentricVelocityZ(), Units.MetersPerSecond);
    m_inputs.yawRate.mut_replace(m_navx.getRate(), Units.DegreesPerSecond);
    m_inputs.rotation2d = Rotation2d.fromRadians(m_inputs.yawAngle.times(-1).in(Units.Radians));
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
   * Call this to configure swapable axes for X/Y or to invert an axis. Currently, this will also swap/invert
   * the robot centic values.
   * @param swapAxes Will swap X/Y Axis
   * @param invertX Will invert X
   * @param invertY Will invert Y
   * @param invertZ Will invert Z
   */
  public void configureVelocity(boolean swapAxes, boolean invertX, boolean invertY, boolean invertZ) {
    m_navx.configureVelocity(swapAxes, invertX, invertY, invertZ);
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
