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
import edu.wpi.first.units.MutableMeasure;
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
    public MutableMeasure<Angle> rollAngle = Units.Radians.zero().mutableCopy();
    public MutableMeasure<Angle> pitchAngle = Units.Radians.zero().mutableCopy();
    public MutableMeasure<Angle> yawAngle = Units.Radians.zero().mutableCopy();
    public MutableMeasure<Velocity<Velocity<Distance>>> xAcceleration = Units.MetersPerSecondPerSecond.zero().mutableCopy();
    public MutableMeasure<Velocity<Velocity<Distance>>> yAcceleration = Units.MetersPerSecondPerSecond.zero().mutableCopy();
    public MutableMeasure<Velocity<Velocity<Distance>>> zAcceleration = Units.MetersPerSecondPerSecond.zero().mutableCopy();
    public MutableMeasure<Velocity<Distance>> xVelocity = Units.MetersPerSecond.zero().mutableCopy();
    public MutableMeasure<Velocity<Distance>> yVelocity = Units.MetersPerSecond.zero().mutableCopy();
    public MutableMeasure<Velocity<Distance>> zVelocity = Units.MetersPerSecond.zero().mutableCopy();
    public MutableMeasure<Velocity<Angle>> yawRate = Units.RadiansPerSecond.zero().mutableCopy();
    public Rotation2d rotation2d = GlobalConstants.ROTATION_ZERO;
  }

  private static final int UPDATE_RATE = 200;

  private AHRS m_navx;
  private SimDouble m_simNavXYaw;
  private Notifier m_inputThread;

  private String m_name;
  private NavX2InputsAutoLogged m_inputs;

  private boolean m_swapXYAxes;
  private boolean m_invertXYAxes;

  /**
   * Create a NavX2 object with built-in logging
   * @param id NavX2 ID
   */
  public NavX2(ID id) {
    this.m_name = id.name;
    this.m_navx = new AHRS(SPI.Port.kMXP, (byte)UPDATE_RATE);
    this.m_inputs = new NavX2InputsAutoLogged();
    this.m_inputThread = new Notifier(this::updateInputs);
    this.m_swapXYAxes = false;
    this.m_invertXYAxes = false;
    this.m_simNavXYaw = new SimDouble(SimDeviceDataJNI.getSimValueHandle(SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]"), "Yaw"));
    System.out.println();

    // Update inputs on init
    updateInputs();

    // Start input thread
    m_inputThread.startPeriodic(1.0 / UPDATE_RATE);

    // Register device with manager
    PurpleManager.add(this);
  }

  /**
   * Get roll angle
   * @return Roll angle measurement
   */
  private Measure<Angle> getRoll() {
    var value = Units.Degrees.of(!m_swapXYAxes ? m_navx.getRoll() : m_navx.getPitch());
    return m_invertXYAxes ? value.negate() : value;
  }

  /**
   * Get pitch angle
   * @return Pitch angle measurement
   */
  private Measure<Angle> getPitch() {
    var value = Units.Degrees.of(!m_swapXYAxes ? m_navx.getPitch() : m_navx.getRoll());
    return m_invertXYAxes ? value.negate() : value;
  }

  /**
   * Get yaw angle
   * @return Yaw angle measurement
   */
  private Measure<Angle> getYaw() {
    return Units.Degrees.of(m_navx.getAngle());
  }

  /**
   * Get X acceleration
   * @return X axis acceleration
   */
  private Measure<Velocity<Velocity<Distance>>> getAccelerationX() {
    var value = Units.MetersPerSecondPerSecond.of(!m_swapXYAxes ? m_navx.getWorldLinearAccelX() : m_navx.getWorldLinearAccelX());
    return m_invertXYAxes ? value.negate() : value;
  }

  /**
   * Get Y acceleration
   * @return Y axis acceleration
   */
  private Measure<Velocity<Velocity<Distance>>> getAccelerationY() {
    var value = Units.MetersPerSecondPerSecond.of(!m_swapXYAxes ? m_navx.getWorldLinearAccelY() : m_navx.getWorldLinearAccelY());
    return m_invertXYAxes ? value.negate() : value;
  }

  /**
   * Get Z acceleration
   * @return Z axis acceleration
   */
  private Measure<Velocity<Velocity<Distance>>> getAccelerationZ() {
    return Units.MetersPerSecondPerSecond.of(m_navx.getWorldLinearAccelZ());
  }

  /**
   * Get X velocity
   * @return X axis velocity
   */
  private Measure<Velocity<Distance>> getVelocityX() {
    var value = Units.MetersPerSecond.of(!m_swapXYAxes ? m_navx.getVelocityX() : m_navx.getVelocityY());
    return m_invertXYAxes ? value.negate() : value;
  }

  /**
   * Get Y velocity
   * @return Y axis velocity
   */
  private Measure<Velocity<Distance>> getVelocityY() {
    var value = Units.MetersPerSecond.of(!m_swapXYAxes ? m_navx.getVelocityY() : m_navx.getVelocityX());
    return m_invertXYAxes ? value.negate() : value;
  }

  /**
   * Get Z velocity
   * @return Z axis velocity
   */
  private Measure<Velocity<Distance>> getVelocityZ() {
    return Units.MetersPerSecond.of(m_navx.getVelocityZ());
  }

  /**
   * Update NavX input readings
   */
  private void updateInputs() {
    m_inputs.isConnected = m_navx.isConnected();
    m_inputs.rollAngle.mut_replace(getRoll());
    m_inputs.pitchAngle.mut_replace(getPitch());
    m_inputs.yawAngle.mut_replace(getYaw());
    m_inputs.xAcceleration.mut_replace(getAccelerationX());
    m_inputs.yAcceleration.mut_replace(getAccelerationY());
    m_inputs.zAcceleration.mut_replace(getAccelerationZ());
    m_inputs.xVelocity.mut_replace(getVelocityX());
    m_inputs.yVelocity.mut_replace(getVelocityY());
    m_inputs.zVelocity.mut_replace(getVelocityZ());
    m_inputs.yawRate.mut_replace(Units.DegreesPerSecond.of(m_navx.getRate()));
    m_inputs.rotation2d = Rotation2d.fromRadians(m_inputs.yawAngle.negate().in(Units.Radians));
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
   * Whether or not to swap X and Y axes on NavX2 when returning values.
   * This swaps X/Y velocities, and roll/pitch angle.
   * <p>
   * Defaults to false
   * @param swap True to swap X and Y axes
   */
  public void swapXYAxes(boolean swap) {
    m_swapXYAxes = swap;
  }

  /**
   * Whether or not to invert X and Y axes on NavX2 when returning values.
   * This inverts the X/Y velocities, and roll/pitch angle.
   * <p>
   * Defaults to false
   * @param invert True to invert X and Y axes
   */
  public void invertXYAxes(boolean invert) {
    m_invertXYAxes = invert;
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
