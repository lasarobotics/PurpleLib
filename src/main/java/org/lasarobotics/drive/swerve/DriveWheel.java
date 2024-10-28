// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.drive.swerve;

import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;

public class DriveWheel {
  public final Measure<Distance> diameter;
  public final Measure<Dimensionless> staticCoF;
  public final Measure<Dimensionless> dynamicCoF;

  public DriveWheel(Measure<Distance> diameter, Measure<Dimensionless> staticCoF, Measure<Dimensionless> dynamicCoF) {
    this.diameter = diameter;
    this.staticCoF = staticCoF;
    this.dynamicCoF = dynamicCoF;
  }
}
