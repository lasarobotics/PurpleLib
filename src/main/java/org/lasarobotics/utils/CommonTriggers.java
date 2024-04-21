// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Common useful triggers */
public class CommonTriggers {

  /**
   * Returns a trigger that is true when the Driver Station is attached.
   * @return A trigger that is true when the Driver Station is attached.
   */
  public static Trigger isDSAttached() {
    return new Trigger(DriverStation::isDSAttached);
  }
}
