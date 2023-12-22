// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.hardware.ctre;

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
