// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.hardware;

import org.littletonrobotics.junction.inputs.LoggableInputs;

public abstract class LoggableHardware extends Monitorable implements AutoCloseable {
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
}
