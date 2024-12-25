// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.hardware.kauailabs;

import java.time.Duration;
import java.time.Instant;
import java.util.concurrent.ThreadLocalRandom;

import org.lasarobotics.drive.swerve.AdvancedSwerveKinematics.ControlCentricity;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

/** NavX2 Sim */
@SuppressWarnings("unused")
public class NavX2Sim {
  private static final AngularVelocity NAVX2_YAW_DRIFT_RATE = Units.DegreesPerSecond.of(0.5 / 60);
  private final SimDouble m_pitch;
  private final SimDouble m_roll;
  private final SimDouble m_yaw;
  private final SimDouble m_accelX;
  private final SimDouble m_accelY;
  private final SimDouble m_accelZ;

  private NavX2 m_navx;
  private ChassisSpeeds m_previousSpeeds;
  private Instant m_lastUpdateTime;

  public NavX2Sim(NavX2 navx) {
    this.m_navx = navx;
    SimDeviceSim simNavX2 = new SimDeviceSim("navX-Sensor", navx.getPort());
    this.m_pitch = simNavX2.getDouble("Pitch");
    this.m_roll = simNavX2.getDouble("Roll");
    this.m_yaw = simNavX2.getDouble("Yaw");
    this.m_accelX = simNavX2.getDouble("LinearWorldAccelX");
    this.m_accelY = simNavX2.getDouble("LinearWorldAccelY");
    this.m_accelZ = simNavX2.getDouble("LinearWorldAccelZ");

    this.m_previousSpeeds = new ChassisSpeeds();
    this.m_lastUpdateTime = Instant.now();
  }

  public void update(Rotation2d orientation, ChassisSpeeds desiredSpeeds, ControlCentricity controlCentricity) {
    var currentTime = Instant.now();
    double randomNoise = ThreadLocalRandom.current().nextDouble(0.9, 1.0);
    double dt = Duration.between(currentTime, m_lastUpdateTime).toMillis() / 1000.0;

    if (controlCentricity.equals(ControlCentricity.FIELD_CENTRIC))
      desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(desiredSpeeds, orientation);

    m_navx.getInputs().xVelocity.mut_replace(desiredSpeeds.vxMetersPerSecond, Units.MetersPerSecond);
    m_navx.getInputs().yVelocity.mut_replace(desiredSpeeds.vyMetersPerSecond, Units.MetersPerSecond);

    int yawDriftDirection = ThreadLocalRandom.current().nextDouble(1.0) < 0.5 ? -1 : +1;
    double angle = m_yaw.get() + Math.toDegrees(desiredSpeeds.omegaRadiansPerSecond * randomNoise) * dt
                   + (NAVX2_YAW_DRIFT_RATE.in(Units.DegreesPerSecond) * dt * yawDriftDirection);
    m_yaw.set(angle);

    m_accelX.set((desiredSpeeds.vxMetersPerSecond - m_previousSpeeds.vxMetersPerSecond) / dt);
    m_accelY.set((desiredSpeeds.vyMetersPerSecond - m_previousSpeeds.vyMetersPerSecond) / dt);

    m_previousSpeeds = desiredSpeeds;
    m_lastUpdateTime = currentTime;
  }
}
