// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.utils;

public class PIDConstants {
  public final double kP;
  public final double kI;
  public final double kD;
  public final double kF;
  public final double period;

  public PIDConstants(double kP, double kI, double kD, double kF, double period) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.kF = kF;
    this.period = period;
  }

  public PIDConstants(double kP, double kI, double kD, double kF) {
    this(kP, kI, kD, kF, GlobalConstants.ROBOT_LOOP_PERIOD);
  }
}