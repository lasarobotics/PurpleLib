// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.drive.swerve;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;

public class DriveWheel {
  public final Distance diameter;
  public final Dimensionless staticCoF;
  public final Dimensionless dynamicCoF;

  public DriveWheel(Distance diameter, Dimensionless staticCoF, Dimensionless dynamicCoF) {
    this.diameter = diameter;
    this.staticCoF = staticCoF;
    this.dynamicCoF = dynamicCoF;
  }
}
