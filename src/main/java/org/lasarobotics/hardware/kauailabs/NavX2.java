// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.hardware.kauailabs;

import java.time.Duration;
import java.time.Instant;
import java.util.concurrent.ThreadLocalRandom;

import org.lasarobotics.drive.swerve.AdvancedSwerveKinematics.ControlCentricity;
import org.lasarobotics.hardware.IMU;
import org.lasarobotics.hardware.LoggableHardware;
import org.lasarobotics.hardware.PurpleManager;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS.NavXUpdateRate;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutLinearAcceleration;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

/** NavX2 */
@SuppressWarnings("unused")
public class NavX2 extends LoggableHardware implements IMU {
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

  private static final AngularVelocity NAVX2_YAW_DRIFT_RATE = Units.DegreesPerSecond.of(0.5 / 60);

  private AHRS m_navx;
  private Notifier m_inputThread;
  private ChassisSpeeds m_previousSpeeds;
  private Instant m_lastUpdateTime;

  private final SimDouble m_simPitch;
  private final SimDouble m_simRoll;
  private final SimDouble m_simYaw;
  private final SimDouble m_simAccelX;
  private final SimDouble m_simAccelY;
  private final SimDouble m_simAccelZ;

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

    SimDeviceSim simNavX2 = new SimDeviceSim("navX-Sensor", m_navx.getPort());
    this.m_simPitch = simNavX2.getDouble("Pitch");
    this.m_simRoll = simNavX2.getDouble("Roll");
    this.m_simYaw = simNavX2.getDouble("Yaw");
    this.m_simAccelX = simNavX2.getDouble("LinearWorldAccelX");
    this.m_simAccelY = simNavX2.getDouble("LinearWorldAccelY");
    this.m_simAccelZ = simNavX2.getDouble("LinearWorldAccelZ");
    this.m_previousSpeeds = new ChassisSpeeds();
    this.m_lastUpdateTime = Instant.now();

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
    synchronized (m_inputs) {
      m_inputs.isConnected = m_navx.isConnected();
      m_inputs.rollAngle.mut_replace(m_navx.getRoll(), Units.Degrees);
      m_inputs.pitchAngle.mut_replace(m_navx.getPitch(), Units.Degrees);
      m_inputs.yawAngle.mut_replace(m_navx.getAngle(), Units.Degrees);
      m_inputs.xAcceleration.mut_replace(m_navx.getWorldLinearAccelX(), Units.Gs);
      m_inputs.yAcceleration.mut_replace(m_navx.getWorldLinearAccelY(), Units.Gs);
      m_inputs.zAcceleration.mut_replace(m_navx.getWorldLinearAccelZ(), Units.Gs);
      m_inputs.yawRate.mut_replace(m_navx.getRate(), Units.DegreesPerSecond);
      m_inputs.rotation2d = Rotation2d.fromRadians(m_inputs.yawAngle.times(-1).in(Units.Radians));

      if (RobotBase.isSimulation()) return;
      m_inputs.xVelocity.mut_replace((m_fieldCentricVelocities) ? m_navx.getVelocityX() : m_navx.getRobotCentricVelocityX(), Units.MetersPerSecond);
      m_inputs.yVelocity.mut_replace((m_fieldCentricVelocities) ? m_navx.getVelocityY() : m_navx.getRobotCentricVelocityY(), Units.MetersPerSecond);
      m_inputs.zVelocity.mut_replace((m_fieldCentricVelocities) ? m_navx.getVelocityZ() : m_navx.getRobotCentricVelocityZ(), Units.MetersPerSecond);
    }
  }

  /**
   * Call this method periodically
   */
  @Override
  protected void periodic() {
    synchronized (m_inputs) { Logger.processInputs(m_name, m_inputs); }
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
   * Zeros the displacement integration variables.   Invoke this at the moment when
   * integration begins.
   */
  public void resetDisplacement() {
    m_navx.resetDisplacement();
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

  @Override
  public boolean isConnected() {
    synchronized (m_inputs) { return m_inputs.isConnected; }
  }

  @Override
  public void reset() {
    m_navx.reset();
  }

  @Override
  public Angle getRoll() {
    synchronized (m_inputs) { return m_inputs.rollAngle; }
  }

  @Override
  public Angle getPitch() {
    synchronized (m_inputs) { return m_inputs.pitchAngle; }
  }

  @Override
  public Angle getYaw() {
    synchronized (m_inputs) { return m_inputs.yawAngle; }
  }

  @Override
  public AngularVelocity getYawRate() {
    synchronized (m_inputs) { return m_inputs.yawRate; }
  }

  @Override
  public Rotation2d getRotation2d() {
    synchronized (m_inputs) { return m_inputs.rotation2d; }
  }

  @Override
  public LinearVelocity getVelocityX() {
    synchronized (m_inputs) { return m_inputs.xVelocity; }
  }

  @Override
  public LinearVelocity getVelocityY() {
    synchronized (m_inputs) { return m_inputs.yVelocity; }
  }

  @Override
  public void updateSim(Rotation2d orientation, ChassisSpeeds desiredSpeeds, ControlCentricity controlCentricity) {
    var currentTime = Instant.now();
    double randomNoise = ThreadLocalRandom.current().nextDouble(0.9, 1.0);
    double dt = Duration.between(currentTime, m_lastUpdateTime).toMillis() / 1000.0;

    if (controlCentricity.equals(ControlCentricity.FIELD_CENTRIC))
      desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(desiredSpeeds, orientation);

    m_inputs.xVelocity.mut_replace(desiredSpeeds.vxMetersPerSecond, Units.MetersPerSecond);
    m_inputs.yVelocity.mut_replace(desiredSpeeds.vyMetersPerSecond, Units.MetersPerSecond);

    int yawDriftDirection = ThreadLocalRandom.current().nextDouble(1.0) < 0.5 ? -1 : +1;
    double angle = m_simYaw.get() + Math.toDegrees(desiredSpeeds.omegaRadiansPerSecond * randomNoise) * dt
                   + (NAVX2_YAW_DRIFT_RATE.in(Units.DegreesPerSecond) * dt * yawDriftDirection);
    m_simYaw.set(angle);

    m_simAccelX.set((desiredSpeeds.vxMetersPerSecond - m_previousSpeeds.vxMetersPerSecond) / dt);
    m_simAccelY.set((desiredSpeeds.vyMetersPerSecond - m_previousSpeeds.vyMetersPerSecond) / dt);

    m_previousSpeeds = desiredSpeeds;
    m_lastUpdateTime = currentTime;
  }

  /**
   * Closes the NavX
   */
  @Override
  public void close() {
    PurpleManager.remove(this);
    m_navx.close();
  }
}
