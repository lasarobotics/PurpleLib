// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.hardware.reduxrobotics;

import org.lasarobotics.hardware.LoggableHardware;
import org.lasarobotics.hardware.PurpleManager;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.reduxrobotics.frames.Frame;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;

public class Canandgyro extends LoggableHardware {
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
    public MutableMeasure<Angle> pitchAngle = Units.Rotations.zero().mutableCopy();
    public MutableMeasure<Angle> rollAngle = Units.Rotations.zero().mutableCopy();
    public MutableMeasure<Angle> yawAngle = Units.Rotations.zero().mutableCopy();
    public MutableMeasure<Velocity<Velocity<Distance>>> xAcceleration = Units.Gs.zero().mutableCopy();
    public MutableMeasure<Velocity<Velocity<Distance>>> yAcceleration = Units.Gs.zero().mutableCopy();
    public MutableMeasure<Velocity<Velocity<Distance>>> zAcceleration = Units.Gs.zero().mutableCopy();
    public MutableMeasure<Velocity<Distance>> xVelocity = Units.MetersPerSecond.zero().mutableCopy();
    public MutableMeasure<Velocity<Distance>> yVelocity = Units.MetersPerSecond.zero().mutableCopy();
    public MutableMeasure<Velocity<Distance>> zVelocity = Units.MetersPerSecond.zero().mutableCopy();
    public MutableMeasure<Velocity<Angle>> pitchRate = Units.RotationsPerSecond.zero().mutableCopy();
    public MutableMeasure<Velocity<Angle>> rollRate = Units.RotationsPerSecond.zero().mutableCopy();
    public MutableMeasure<Velocity<Angle>> yawRate = Units.RotationsPerSecond.zero().mutableCopy();
    public Rotation2d rotation2d = new Rotation2d();
  }

  private static final Measure<Time> INITIAL_FRAME_WAIT_TIME = Units.Seconds.of(5.0);

  private com.reduxrobotics.sensors.canandgyro.Canandgyro m_gyro;

  private String m_name;
  private CanandgyroInputsAutoLogged m_inputs;

  private double m_prevAccelerationTimestamp;

  public Canandgyro(ID id) {
    this.m_name = id.name;
    this.m_gyro = new com.reduxrobotics.sensors.canandgyro.Canandgyro(id.deviceID);
    this.m_inputs = new CanandgyroInputsAutoLogged();
    this.m_prevAccelerationTimestamp = 0.0;

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
    double dt = dataFrame.getTimestamp() - m_prevAccelerationTimestamp;
    var vector = dataFrame.getValue();
    m_inputs.xAcceleration.mut_replace(vector.get(0), Units.Gs);
    m_inputs.yAcceleration.mut_replace(vector.get(1), Units.Gs);
    m_inputs.zAcceleration.mut_replace(vector.get(2), Units.Gs);
    m_inputs.xVelocity.mut_plus(m_inputs.xAcceleration.in(Units.MetersPerSecondPerSecond) * dt, Units.MetersPerSecond);
    m_inputs.yVelocity.mut_plus(m_inputs.yAcceleration.in(Units.MetersPerSecondPerSecond) * dt, Units.MetersPerSecond);
    m_inputs.zVelocity.mut_plus(m_inputs.zAcceleration.in(Units.MetersPerSecondPerSecond) * dt, Units.MetersPerSecond);

    m_prevAccelerationTimestamp = dataFrame.getTimestamp();
  }

  private void updateYaw(Frame<Double> dataFrame) {
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
  protected void periodic() {
    Logger.processInputs(m_name, m_inputs);
  }

  @Override
  public LoggableInputs getInputs() {
    synchronized (m_inputs) { return m_inputs; }
  }

  @Override
  public void close() {
    PurpleManager.remove(this);
    m_gyro.close();
  }

}
