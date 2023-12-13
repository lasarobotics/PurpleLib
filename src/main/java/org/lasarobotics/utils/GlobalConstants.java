// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.utils;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Global Constants
 * <p>
 * Contains useful, commonly used constants
 */
public class GlobalConstants {
  /** Robot loop frequency */
  public static final int ROBOT_LOOP_HZ = 50;
  /** Robot loop period */
  public static final double ROBOT_LOOP_PERIOD = 1.0 / ROBOT_LOOP_HZ;

  /** Max RPM of NEO */
  public static final int NEO_MAX_RPM = 5676;
  /** Ticks per revolution of NEO built-in encoder */
  public static final int NEO_ENCODER_TICKS_PER_ROTATION = 42;
  /** Ticks per revolution of REV through bore encoder */
  public static final int REV_ENCODER_TICKS_PER_ROTATION = 8192;

  /** Environment variable to toggle robot replay */
  public static final String REPLAY_ENVIRONMENT_VAR = "ROBOT_REPLAY";

  /** Zero radians rotation */
  public static final Rotation2d ROTATION_ZERO = Rotation2d.fromRadians(0.0);
  /** Pi radians rotation */
  public static final Rotation2d ROTATION_PI = Rotation2d.fromRadians(Math.PI);
}
