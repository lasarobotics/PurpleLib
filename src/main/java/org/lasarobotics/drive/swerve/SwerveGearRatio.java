// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.drive.swerve;

/** Swerve Ratio for Swerve Modules */
public interface SwerveGearRatio {
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
