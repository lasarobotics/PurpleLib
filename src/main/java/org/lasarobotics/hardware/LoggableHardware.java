// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.hardware;

import org.lasarobotics.utils.GlobalConstants;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.units.measure.Frequency;

public abstract class LoggableHardware extends Monitorable implements AutoCloseable {
  protected abstract void updateInputs();
  /**
   * Call this method periodically
   */
  protected abstract void periodic();

  /**
   * Get latest sensor input data
   * @return Latest sensor data
   */
  public abstract LoggableInputs getInputs();

  /**
   * Closes the hardware device
   */
  public abstract void close();

  /**
   * Get update rate for device
   * <p>
   * Defaults to {@link GlobalConstants#ROBOT_LOOP_FREQUENCY}
   * @return Desired update rate for this device
   */
  public Frequency getUpdateRate() {
    return GlobalConstants.ROBOT_LOOP_FREQUENCY;
  }
}
