// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.drive.swerve;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;

/** Swerve drive wheel */
public class DriveWheel {
  public final Distance radius;
  public final Distance diameter;
  public final Dimensionless staticCoF;
  public final Dimensionless dynamicCoF;

  private DriveWheel(Distance diameter, Dimensionless staticCoF, Dimensionless dynamicCoF) {
    this.radius = diameter.div(2);
    this.diameter = diameter;
    this.staticCoF = staticCoF;
    this.dynamicCoF = dynamicCoF;
  }

  /**
   * Create a swerve drive wheel
   * @param diameter Diameter of wheel
   * @param staticCoF Static coefficient of friction
   * @param dynamicCoF Dynamic coefficient of friction
   * @return
   */
  public static DriveWheel create(Distance diameter, Dimensionless staticCoF, Dimensionless dynamicCoF) {
    if (dynamicCoF.gt(staticCoF))
      throw new IllegalArgumentException("Static CoF must be higher than dynamic CoF!");

    return new DriveWheel(diameter, staticCoF, dynamicCoF);
  }
}
