// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

public abstract class SwerveModule {

  protected abstract void periodic();

  protected abstract void simulationPeriodic();

  public abstract void set(SwerveModuleState state);

  public abstract SwerveModuleState getState();

  public abstract SwerveModulePosition getPosition();

  public abstract ModuleLocation getModuleLocation();

  public abstract Translation2d getModuleCoordinate();

  public abstract Measure<Velocity<Distance>> getDriveVelocity();
}
