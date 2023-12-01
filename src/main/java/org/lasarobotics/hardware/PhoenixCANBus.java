package org.lasarobotics.hardware;

/**
 * Select CAN bus when using CTRE devices
 * <p>
 * Only a single CANivore is supported, and it MUST be named "canivore"
 */
public enum PhoenixCANBus {
  RIO("rio"),
  CANIVORE("canivore");

  public final String busName;
  private PhoenixCANBus(String busName) {
    this.busName = busName;
  }

}
