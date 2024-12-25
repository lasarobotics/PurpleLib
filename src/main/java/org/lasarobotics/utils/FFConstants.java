// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.utils;

import org.apache.commons.math3.util.Precision;

/** Feed forward constants */
public class FFConstants {
  private static final double EPSILON = 1e-8;
  /** Static gain */
  public final double kS;
  /** Gravity gain */
  public final double kG;
  /** Velocity gain */
  public final double kV;
  /** Acceleration gain */
  public final double kA;

  private FFConstants(double kS, double kG, double kV, double kA) {
    this.kS = kS;
    this.kG = kG;
    this.kV = kV;
    this.kA = kA;
  }

  /**
   * Feed forward constants
   * @param kS Static gain
   * @param kG Gravity gain
   * @param kV Velocity gain
   * @param kA Acceleration gain
   */
  public static FFConstants of(double kS, double kG, double kV, double kA) {
    return new FFConstants(kS, kG, kV, kA);
  }

  @Override
  public boolean equals(Object object) {
    if (object.getClass() != this.getClass()) return false;

    var other = (FFConstants)object;
    return (Precision.equals(kS, other.kS, EPSILON) &&
            Precision.equals(kG, other.kG, EPSILON) &&
            Precision.equals(kV, other.kV, EPSILON) &&
            Precision.equals(kA, other.kA, EPSILON));
  }
}