// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.hardware;

import org.lasarobotics.drive.swerve.AdvancedSwerveKinematics.ControlCentricity;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;

public interface IMU extends AutoCloseable {

  /**
   * Get whether or not IMU is connected
   * @return True if connected
   */
  public default boolean isConnected() {
    return true;
  }

  /**
   * Get update rate for IMU
   * @return Update rate
   */
  public Frequency getUpdateRate();

  /**
   * Reset yaw angle
   */
  public void reset();

  /**
   * Get roll angle
   * @return Roll angle
   */
  public Angle getRoll();

  /**
   * Get pitch angle
   * @return Pitch angle
   */
  public Angle getPitch();

  /**
   * Get yaw angle
   * @return Yaw angle
   */
  public Angle getYaw();

  /**
   * Get yaw rate
   * @return Yaw rate
   */
  public AngularVelocity getYawRate();

  /**
   * Get yaw angle as Rotation2d
   * @return Rotation2d
   */
  public Rotation2d getRotation2d();

  /**
   * Get X axis velocity
   * @return X velocity
   */
  public default LinearVelocity getVelocityX() {
    return Units.MetersPerSecond.zero();
  }

  /**
   * Get Y axis velocity
   * @return Y velocity
   */
  public default LinearVelocity getVelocityY() {
    return Units.MetersPerSecond.zero();
  }

  /**
   * Get Z axis velocity
   * @return Z velocity
   */
  public default LinearVelocity getVelocityZ() {
    return Units.MetersPerSecond.zero();
  }

  /**
   * Update IMU simulation
   * <p>
   * Offers a primitive IMU simulation in order to make swerve drives function correctly in the simulator
   * @param orientation Orientation of robot on the field (typically provided by pose estimator)
   * @param desiredSpeeds Desired speeds commanded by user
   * @param controlCentricity Control centricity of desired
   */
  public void updateSim(Rotation2d orientation, ChassisSpeeds desiredSpeeds, ControlCentricity controlCentricity);

  public Time getTimestamp();

  public void close();
}
