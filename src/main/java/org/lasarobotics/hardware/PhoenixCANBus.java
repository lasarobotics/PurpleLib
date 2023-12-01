package org.lasarobotics.hardware;

/** CTRE Phoenix CAN bus */
public enum PhoenixCANBus {
  /** roboRIO CAN bus */
  RIO("rio"),
  /**
   * CANivore CAN bus
   * <p>
   * Only a single CANivore is supported, and MUST be named "canivore"
   */
  CANIVORE("canivore");

  /** CAN bus name */
  public final String name;
  private PhoenixCANBus(String name) {
    this.name = name;
  }
}
