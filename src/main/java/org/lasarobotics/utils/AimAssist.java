// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.utils;

import edu.wpi.first.math.Pair;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

/** Aim Assist */
public class AimAssist {
  private static final double G = Units.Gs.one().in(Units.MetersPerSecondPerSecond);

  /**
   * Calculate angle and launch velocity for projectile shooter
   * @param targetDistance Distance to target (Use apparent distance if shooting on the move)
   * @param heightDelta Height delta between shooter height and target goal height (Positive is above)
   * @param entryAngle Trajectory entry angle of projectile into target
   * @return Launch angle and velocity respectively
   */
  public static Pair<Angle, LinearVelocity> getShooterValues(Distance targetDistance, Distance heightDelta, Angle entryAngle) {
      double phi = entryAngle.in(Units.Radians);
      double x = targetDistance.in(Units.Meters);
      double y = heightDelta.in(Units.Meters);

      double theta = Math.atan(((2 * y) / x) - Math.tan(phi));
      double v = Math.sqrt((G * x) / ((Math.tan(theta) - Math.tan(phi)) * Math.pow(Math.cos(theta), 2)));

    return new Pair<Angle, LinearVelocity>(Units.Radians.of(theta), Units.MetersPerSecond.of(v));
  }
}
