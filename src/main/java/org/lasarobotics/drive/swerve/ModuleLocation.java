// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.drive.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

/** Module location */
public enum ModuleLocation {
  LeftFront(0, Rotation2d.fromRadians(-Math.PI / 2)),
  RightFront(1, Rotation2d.fromRadians(+0.0)),
  LeftRear(2, Rotation2d.fromRadians(+Math.PI)),
  RightRear(3, Rotation2d.fromRadians(+Math.PI / 2));

  /** Module index */
  public final int index;
  /** Module orientation chassis offset */
  public final Rotation2d offset;
  private ModuleLocation(int index, Rotation2d offset) {
    this.index = index;
    this.offset = offset;
  }
}
