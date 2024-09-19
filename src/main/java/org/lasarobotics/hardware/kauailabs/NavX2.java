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
    public Measure<Angle> pitchAngle = Units.Radians.of(0.0);
    public Measure<Angle> yawAngle = Units.Radians.of(0.0);
    public Measure<Angle> rollAngle = Units.Radians.of(0.0);
    public Measure<Velocity<Distance>> xVelocity = Units.MetersPerSecond.of(0.0);
    public Measure<Velocity<Distance>> yVelocity = Units.MetersPerSecond.of(0.0);
    public Measure<Velocity<Angle>> yawRate = Units.RadiansPerSecond.of(0.0);
    public Rotation2d rotation2d = GlobalConstants.ROTATION_ZERO;
  }

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
  public NavX2(ID id, int updateRate) {
    this.m_name = id.name;
    this.m_navx = new AHRS(SPI.Port.kMXP, (byte)updateRate);
    this.m_inputs = new NavX2InputsAutoLogged();
    this.m_inputThread = new Notifier(this::updateInputs);
    this.m_simNavXYaw = new SimDouble(SimDeviceDataJNI.getSimValueHandle(SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]"), "Yaw"));
    System.out.println();

    // Start input thread
    m_inputThread.startPeriodic(1.0 / updateRate);

    // Update inputs on init
    periodic();

    // Register device with manager
    PurpleManager.add(this);
  }

  /**
   * Returns the current pitch value (in degrees, from -180 to 180)
   * reported by the sensor.  Pitch is a measure of rotation around
   * the X Axis.
   * @return The current pitch value in degrees (-180 to 180).
   */
  private float getPitch() {
    return m_navx.getPitch();
  }

  /**
   * Returns the total accumulated yaw angle (Z Axis, in degrees)
   * reported by the sensor.
   *<p>
   * NOTE: The angle is continuous, meaning it's range is beyond 360 degrees.
   * This ensures that algorithms that wouldn't want to see a discontinuity
   * in the gyro output as it sweeps past 0 on the second time around.
   *<p>
   * Note that the returned yaw value will be offset by a user-specified
   * offset value; this user-specified offset value is set by
   * invoking the zeroYaw() method.
   *<p>
   * @return The current total accumulated yaw angle (Z axis) of the robot
   * in degrees. This heading is based on integration of the returned rate
   * from the Z-axis (yaw) gyro.
   */
  private double getAngle() {
    return m_navx.getAngle();
  }

  /**
   * Returns the current roll value (in degrees, from -180 to 180)
   * reported by the sensor.  Roll is a measure of rotation around
   * the X Axis.
   * @return The current roll value in degrees (-180 to 180).
   */
  private float getRoll() {
    return m_navx.getRoll();
  }

  /**
   * Returns the velocity (in meters/sec) of the X axis [Experimental].
   *
   * NOTE:  This feature is experimental.  Velocity measures rely on integration
   * of acceleration values from MEMS accelerometers which yield "noisy" values.  The
   * resulting velocities are not known to be very accurate.
   * @return Current Velocity (in meters/squared).
   */
  private float getVelocityX() {
    return m_navx.getVelocityX();
  }

  /**
   * Returns the velocity (in meters/sec) of the Y axis [Experimental].
   *
   * NOTE:  This feature is experimental.  Velocity measures rely on integration
   * of acceleration values from MEMS accelerometers which yield "noisy" values.  The
   * resulting velocities are not known to be very accurate.
   * @return Current Velocity (in meters/squared).
   */
  private float getVelocityY() {
    return m_navx.getVelocityY();
  }

  /**
   * Return the rate of rotation of the yaw (Z-axis) gyro, in degrees per second.
   *<p>
   * The rate is based on the most recent reading of the yaw gyro angle.
   *<p>
   * @return The current rate of change in yaw angle (in degrees per second)
   */
  private double getRate() {
    return m_navx.getRate();
  }

  /**
   * Return the heading of the robot as a {@link edu.wpi.first.math.geometry.Rotation2d}.
   *
   * <p>The angle is continuous, that is it will continue from 360 to 361 degrees. This allows
   * algorithms that wouldn't want to see a discontinuity in the gyro output as it sweeps past from
   * 360 to 0 on the second time around.
   *
   * <p>The angle is expected to increase as the gyro turns counterclockwise when looked at from the
   * top. It needs to follow the NWU axis convention.
   *
   * <p>This heading is based on integration of the returned rate from the gyro.
   *
   * @return the current heading of the robot as a {@link edu.wpi.first.math.geometry.Rotation2d}.
   */
  private Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(-getAngle());
  }

  /**
   * Update NavX input readings
   */
  private void updateInputs() {
    m_inputs.pitchAngle = Units.Degrees.of(getPitch());
    m_inputs.yawAngle = Units.Degrees.of(getAngle());
    m_inputs.rollAngle = Units.Degrees.of(getRoll());
    m_inputs.xVelocity = Units.MetersPerSecond.of(getVelocityX());
    m_inputs.yVelocity = Units.MetersPerSecond.of(getVelocityY());
    m_inputs.yawRate = Units.DegreesPerSecond.of(getRate());
    m_inputs.rotation2d = getRotation2d();
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
    return m_inputs;
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
