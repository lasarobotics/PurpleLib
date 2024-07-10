// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.hardware;

public interface Monitorable {
  static final int DEFAULT_RETRIES = 3;

  /**
   * Return true if the component is healthy. Some components may not have this capability, so a
   * default of returning true is used.
   * @return True if the component is connected.
   */
  public default boolean isHealthy() {
    return true;
  }


  /**
   * Method to call to re-initialize component
   * @return True if component re-initialized successfully
   */
  public default boolean reinit() {
    return true;
  }

  /**
   * Get maximum number of times to retry re-initializing component
   * <p>
   * Defaults to {@value Monitorable#DEFAULT_RETRIES}
   * @return Number of retries
   */
  public default int getMaxRetries() {
    return DEFAULT_RETRIES;
  }

  /**
   * Save number of errors that have occured
   */
  public default void setErrorCount(int num) {};

  /**
   * Get number of failures that have occured
   * @return Number of failures
   */
  public default int getErrorCount() {
    return 0;
  }
}
