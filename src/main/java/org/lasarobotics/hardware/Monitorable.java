// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.hardware;

public abstract class Monitorable {
  private static final int DEFAULT_RETRIES = 3;
  private int m_errorCount = 0;

  /**
   * Return true if the component is healthy. Some components may not have this capability, so a
   * default of returning true is used.
   * @return True if the component is connected.
   */
  public boolean isHealthy() {
    return true;
  }


  /**
   * Method to call to re-initialize component
   * @return True if component re-initialized successfully
   */
  public boolean reinit() {
    return true;
  }

  /**
   * Get maximum number of times to retry re-initializing component
   * <p>
   * Defaults to {@value Monitorable#DEFAULT_RETRIES}
   * @return Number of retries
   */
  public int getMaxRetries() {
    return DEFAULT_RETRIES;
  }

  /**
   * Save number of errors that have occured
   * @param num Number of errors
   */
  public void setErrorCount(int num) {
    if (num < 0) num = 0;
    m_errorCount = num;
  };

  /**
   * Get number of failures that have occured
   * @return Number of failures
   */
  public int getErrorCount() {
    return m_errorCount;
  }
}
