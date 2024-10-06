// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.hardware;

public abstract class Monitorable {
  private static final int DEFAULT_RETRIES = 3;
  private int m_errorCount = 0;

  /**
   * Return true if the component is healthy.
   * <p>
   * Some components may not have this capability, so a
   * default of returning true is used.
   * @return True if the component is connected.
   */
  public boolean isHealthy() {
    return true;
  }


  /**
   * Get if component is dead.
   * Component is declared "dead" when the number of re-initialization attempts
   * exceeds the value returned by {@link Monitorable#getMaxRetries()}
   * <p>
   * HE'S DEAD JIM!
   * @return True if component is dead
   */
  public boolean isDead() {
    return m_errorCount >= getMaxRetries();
  }

  /**
   * Method to call to re-initialize component
   * <p>
   * A default of returning false is used to allow for components that only require monitoring
   * with no attempt to re-initialize
   * @return True if component re-initialized successfully
   */
  public boolean reinit() {
    return false;
  }

  /**
   * Get maximum number of times to try re-initializing component
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
