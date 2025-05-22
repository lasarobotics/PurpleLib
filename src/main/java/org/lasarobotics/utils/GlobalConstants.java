// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.utils;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Frequency;

/**
 * Global Constants
 * <p>
 * Contains useful, commonly used constants
 */
public class GlobalConstants {
  /** Robot loop frequency */
  public static final Frequency ROBOT_LOOP_FREQUENCY = Units.Hertz.of(50);

  /** Max RPM of NEO */
  public static final int NEO_MAX_RPM = 5676;
  /** Max RPM of Vortex */
  public static final int VORTEX_MAX_RPM = 6784;
  /** Ticks per revolution of NEO built-in encoder */
  public static final int NEO_ENCODER_TICKS_PER_ROTATION = 42;
  /** Ticks per rotation of Vortex built-in encoder */
  public static final int VORTEX_ENCODER_TICKS_PER_ROTATION = 7168;
  /** Ticks per revolution of REV through bore encoder */
  public static final int REV_ENCODER_TICKS_PER_ROTATION = 8192;
  /** Ticks per revolution of WCP/CTRE Kraken X60 */
  public static final int KRAKEN_X60_ENCODER_TICKS_PER_ROTATION = 2048;

  /** Environment variable to toggle robot replay */
  public static final String REPLAY_ENVIRONMENT_VAR = "ROBOT_REPLAY";
}
