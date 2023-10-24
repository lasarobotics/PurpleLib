// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.utils;

public class GlobalConstants {
  public static final int ROBOT_LOOP_HZ = 50;
  public static final double ROBOT_LOOP_PERIOD = 1.0 / ROBOT_LOOP_HZ;

  // Motor RPMs, encoder values, and gear ratios
  public static final int NEO_MAX_RPM = 5676;
  public static final int NEO_ENCODER_TICKS_PER_ROTATION = 42;
  public static final int REV_ENCODER_TICKS_PER_ROTATION = 8192;

  public static final String REPLAY_ENVIRONMENT_VAR = "ROBOT_REPLAY";
}
