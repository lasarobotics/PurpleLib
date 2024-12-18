// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.drive.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.LinearVelocity;

public interface SwerveModule {

  /** Mount orientation */
  public enum MountOrientation {
    /** Motors are right side up */
    STANDARD,
    /** Motors are upside down */
    INVERTED;
  }

  /** Swerve Ratio for Swerve Modules */
  public interface GearRatio {
    /**
     * Get drive gear ratio
     * @return Gear ratio for driving the wheel
     */
    public double getDriveRatio();

    /**
     * Get rotate gear ratio
     * @return Gear ratio for rotating the wheel
     */
    public double getRotateRatio();
  }

  /** Module location */
  public enum Location {
    LeftFront,
    RightFront,
    LeftRear,
    RightRear;
  }

  public abstract void set(SwerveModuleState state);

  public abstract SwerveModuleState getState();

  public abstract SwerveModulePosition getPosition();

  public abstract SwerveModule.Location getModuleLocation();

  public abstract Translation2d getModuleCoordinate();

  public abstract LinearVelocity getDriveVelocity();

  public abstract SwerveModule.GearRatio getGearRatio();
}
