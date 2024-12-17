// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.utils;

import org.apache.commons.math3.util.Precision;

/** PID constants */
public class PIDConstants {
  private static final double EPSILON = 1e-8;
  /** Proprotional gain */
  public final double kP;
  /** Integral gain */
  public final double kI;
  /** Derivative gain */
  public final double kD;
  /** Feed-forward gain */
  public final double kF;
  /** Integral zone */
  public final double kIZone;
  /** Loop period */
  public final double period;

  /**
   * PID Constants
   * @param kP Proportional gain
   * @param kI Integral gain
   * @param kD Derivative gain
   * @param kF Feed-forward gain
   * @param kIZone Integral zone
   * @param period PID loop period
   */
  public PIDConstants(double kP, double kI, double kD, double kF, double kIZone, double period) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.kF = kF;
    this.kIZone = kIZone;
    this.period = period;
  }

  /**
   * PID Constants with default period
   * @param kP Proportional gain
   * @param kI Integral gain
   * @param kD Derivative gain
   * @param kF Feed-forward gain
   * @param kIZone Integral zone
   */
  public PIDConstants(double kP, double kI, double kD, double kF, double kIZone) {
    this(kP, kI, kD, kF, kIZone, GlobalConstants.ROBOT_LOOP_PERIOD);
  }

  @Override
  public boolean equals(Object object) {
    if (object.getClass() != this.getClass()) return false;

    var other = (PIDConstants)object;
    return (Precision.equals(kP, other.kP, EPSILON) &&
            Precision.equals(kI, other.kI, EPSILON) &&
            Precision.equals(kD, other.kD, EPSILON) &&
            Precision.equals(kF, other.kF, EPSILON) &&
            Precision.equals(kIZone, other.kIZone, EPSILON) &&
            Precision.equals(period, other.period, EPSILON));
  }
}