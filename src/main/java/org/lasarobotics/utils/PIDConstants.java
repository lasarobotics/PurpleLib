// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.utils;

/** PID constants */
public class PIDConstants {
  /** Proprotional gain */
  public final double kP;
  /** Integral gain */
  public final double kI;
  /** Derivative gain */
  public final double kD;
  /** Feed-forward gain */
  public final double kF;
  /** Loop period */
  public final double period;

  /**
   * PID Constants
   * @param kP Proportional gain
   * @param kI Integral gain
   * @param kD Derivative gain
   * @param kF Feed-forward gain
   * @param period PID loop period
   */
  public PIDConstants(double kP, double kI, double kD, double kF, double period) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.kF = kF;
    this.period = period;
  }

  /**
   * PID Constants with default period
   * @param kP Proportional gain
   * @param kI Integral gain
   * @param kD Derivative gain
   * @param kF Feed-forward gain
   */
  public PIDConstants(double kP, double kI, double kD, double kF) {
    this(kP, kI, kD, kF, GlobalConstants.ROBOT_LOOP_PERIOD);
  }
}