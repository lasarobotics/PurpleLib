// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.lasarobotics.utils;

/** Feed forward constants */
public class FFConstants {
  /** Static gain */
  public final double kS;
  /** Gravity gain */
  public final double kG;
  /** Velocity gain */
  public final double kV;
  /** Acceleration gain */
  public final double kA;

  /**
   * Feed forward constants
   * @param kP Proportional gain
   * @param kI Integral gain
   * @param kD Derivative gain
   * @param kF Feed-forward gain
   * @param period PID loop period
   */
  public FFConstants(double kS, double kG, double kV, double kA) {
    this.kS = kS;
    this.kG = kG;
    this.kV = kV;
    this.kA = kA;
  }
}