// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.hardware.reduxrobotics;

import java.time.Duration;
import java.time.Instant;
import java.util.concurrent.ThreadLocalRandom;

import org.lasarobotics.drive.swerve.AdvancedSwerveKinematics.ControlCentricity;
import org.lasarobotics.hardware.IMU;
import org.lasarobotics.hardware.LoggableHardware;
import org.lasarobotics.hardware.PurpleManager;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.reduxrobotics.frames.Frame;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutLinearAcceleration;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.Time;

public class Canandgyro extends LoggableHardware implements IMU {
  /** Cananggyro ID */
  public static class ID {
    public final String name;
    public final int deviceID;

    /**
     * Canandgyro ID
     * @param name Device name for logging
     * @param deviceID CAN ID
     */
    public ID(String name, int deviceID) {
      this.name = name;
      this.deviceID = deviceID;
    }
  }

  /**
   * Canandgyro sensor inputs
   */
  @AutoLog
  public static class CanandgyroInputs {
    public boolean isConnected = true;
    public MutAngle pitchAngle = Units.Rotations.zero().mutableCopy();
    public MutAngle rollAngle = Units.Rotations.zero().mutableCopy();
    public MutAngle yawAngle = Units.Rotations.zero().mutableCopy();
    public MutLinearAcceleration xAcceleration = Units.Gs.zero().mutableCopy();
    public MutLinearAcceleration yAcceleration = Units.Gs.zero().mutableCopy();
    public MutLinearAcceleration zAcceleration = Units.Gs.zero().mutableCopy();
    public MutLinearVelocity xVelocity = Units.MetersPerSecond.zero().mutableCopy();
    public MutLinearVelocity yVelocity = Units.MetersPerSecond.zero().mutableCopy();
    public MutLinearVelocity zVelocity = Units.MetersPerSecond.zero().mutableCopy();
    public MutAngularVelocity pitchRate = Units.RotationsPerSecond.zero().mutableCopy();
    public MutAngularVelocity rollRate = Units.RotationsPerSecond.zero().mutableCopy();
    public MutAngularVelocity yawRate = Units.RotationsPerSecond.zero().mutableCopy();
    public Rotation2d rotation2d = Rotation2d.kZero;
  }

  private static final Time INITIAL_FRAME_WAIT_TIME = Units.Seconds.of(5.0);

  private static final AngularVelocity CANANDGYRO_YAW_DRIFT_RATE = Units.DegreesPerSecond.of(0.25 / 60);

  private com.reduxrobotics.sensors.canandgyro.Canandgyro m_gyro;

  private String m_name;
  private CanandgyroInputsAutoLogged m_inputs;

  private Instant m_lastUpdateTime;

  private double m_prevAccelerationTimestamp;

  public Canandgyro(ID id) {
    this.m_name = id.name;
    this.m_gyro = new com.reduxrobotics.sensors.canandgyro.Canandgyro(id.deviceID);
    this.m_inputs = new CanandgyroInputsAutoLogged();
    this.m_prevAccelerationTimestamp = 0.0;
    this.m_lastUpdateTime = Instant.now();

    m_gyro.getAccelerationFrame().addCallback(this::updateAcceleration);
    m_gyro.getMultiturnYawFrame().addCallback(this::updateYaw);
    m_gyro.getAngularVelocityFrame().addCallback(this::updateAngularVelocity);
    m_gyro.getAngularPositionFrame().addCallback(this::updateAngularPosition);


    // Wait up to 5 seconds for initial data frames...
    Frame.waitForFrames(
      INITIAL_FRAME_WAIT_TIME.in(Units.Seconds),
      m_gyro.getAccelerationFrame(),
      m_gyro.getMultiturnYawFrame(),
      m_gyro.getAngularVelocityFrame(),
      m_gyro.getAngularPositionFrame()
    );
    m_prevAccelerationTimestamp = m_gyro.getAccelerationFrame().getTimestamp();

    // Register device with PurpleManager
    PurpleManager.add(this);
  }

  private void updateAcceleration(Frame<Vector<N3>> dataFrame) {
    var dt = Units.Seconds.of(dataFrame.getTimestamp() - m_prevAccelerationTimestamp);
    var vector = dataFrame.getValue();
    m_inputs.xAcceleration.mut_replace(vector.get(0), Units.Gs);
    m_inputs.yAcceleration.mut_replace(vector.get(1), Units.Gs);
    m_inputs.zAcceleration.mut_replace(vector.get(2), Units.Gs);
    m_inputs.xVelocity.mut_plus(m_inputs.xAcceleration.times(dt));
    m_inputs.yVelocity.mut_plus(m_inputs.yAcceleration.times(dt));
    m_inputs.zVelocity.mut_plus(m_inputs.zAcceleration.times(dt));

    m_prevAccelerationTimestamp = dataFrame.getTimestamp();
  }

  private void updateYaw(Frame<Double> dataFrame) {
    m_inputs.isConnected = m_gyro.isConnected();
    m_inputs.yawAngle.mut_replace(dataFrame.getValue(), Units.Rotations);
    m_inputs.rotation2d = Rotation2d.fromRotations(dataFrame.getValue());
  }

  private void updateAngularVelocity(Frame<Vector<N3>> dataFrame) {
    var vector = dataFrame.getValue();
    m_inputs.pitchRate.mut_replace(vector.get(1), Units.RotationsPerSecond);
    m_inputs.rollRate.mut_replace(vector.get(0), Units.RotationsPerSecond);
  }

  private void updateAngularPosition(Frame<Quaternion> dataFrame) {
    var vector = dataFrame.getValue().toRotationVector();
    m_inputs.pitchAngle.mut_replace(vector.get(1), Units.Rotations);
    m_inputs.rollAngle.mut_replace(vector.get(0), Units.Rotations);
  }

  @Override
  protected void updateInputs() {}

  @Override
  protected void periodic() {
    Logger.processInputs(m_name, m_inputs);
  }

  @Override
  public CanandgyroInputsAutoLogged getInputs() {
    synchronized (m_inputs) { return m_inputs; }
  }

  @Override
  public boolean isConnected() {
    synchronized (m_inputs) { return m_inputs.isConnected; }
  }

  @Override
  public void reset() {}

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
  public void updateSim(Rotation2d orientation, ChassisSpeeds desiredSpeeds, ControlCentricity controlCentricity) {
    var currentTime = Instant.now();
    double randomNoise = ThreadLocalRandom.current().nextDouble(0.9, 1.0);
    double dt = Duration.between(currentTime, m_lastUpdateTime).toMillis() / 1000.0;
    if (controlCentricity.equals(ControlCentricity.FIELD_CENTRIC))
    desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(desiredSpeeds, orientation);

    m_inputs.xVelocity.mut_replace(desiredSpeeds.vxMetersPerSecond, Units.MetersPerSecond);
    m_inputs.yVelocity.mut_replace(desiredSpeeds.vyMetersPerSecond, Units.MetersPerSecond);

    int yawDriftDirection = ThreadLocalRandom.current().nextDouble(1.0) < 0.5 ? -1 : +1;
    double angle = m_inputs.yawAngle.in(Units.Degrees) + Math.toDegrees(desiredSpeeds.omegaRadiansPerSecond * randomNoise) * dt
                    + (CANANDGYRO_YAW_DRIFT_RATE.in(Units.DegreesPerSecond) * dt * yawDriftDirection);
    m_inputs.yawAngle.mut_replace(Units.Degrees.of(angle));

    m_inputs.rotation2d = orientation;
  }

  @Override
  public void close() {
    PurpleManager.remove(this);
    m_gyro.close();
  }

}
