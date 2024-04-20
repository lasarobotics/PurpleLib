// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.hardware;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.wpilibj.RobotBase;

/** PurpleLib Hardware Logging Manager */
public class PurpleManager {
  private static ArrayList<LoggableHardware> m_hardware = new ArrayList<>();
  private static ArrayList<Runnable> m_callbacks = new ArrayList<>();
  private static ArrayList<Runnable> m_simCallbacks = new ArrayList<>();

  /**
   * Add hardware device to PurpleLib hardware logging manager
   * <p>
   * Should not be necessary to call this manually, PurpleLib devices register themselves when instantiated
   * @param devices Devices to add
   */
  public static void add(LoggableHardware... devices) {
    m_hardware.addAll(Arrays.asList(devices));
  }

  /**
   * Remove hardware devices from PurpleLib hardware logging manager
   * <p>
   * Should not be necessary to call this manually, PurpleLib devices remove themselves when closed
   * @param devices Devices to remove
   */
  public static void remove(LoggableHardware... devices) {
    m_hardware.removeAll(Arrays.asList(devices));
  }

  /**
   * Add custom callback to PurpleLib hardware logging manager to be called every loop
   * @param callback Desired callback
   */
  public static void addCallback(Runnable callback) {
    m_callbacks.add(callback);
  }

  /**
   * Add custom callback to PurpleLib hardware logging manager
   * <p>
   * Callbacks added using this method will only be called in simulation
   * @param callback
   */
  public static void addCallbackSim(Runnable callback) {
    m_simCallbacks.add(callback);
  }

  /**
   * Update all hardware devices
   * <p>
   * Call this peridically, preferably in the beginning of <code>robotPeriodic()</code> every loop
   */
  public static void update() {
    m_hardware.stream().forEach((device) -> device.periodic());
    m_callbacks.stream().forEach(Runnable::run);

    if (RobotBase.isReal()) return;
    m_simCallbacks.stream().forEach(Runnable::run);
  }

  /**
   * Remove all hardware devices from logging manager
   */
  public static void clear() {
    m_hardware.stream().forEach((device) -> device.close());
    m_hardware.clear();
  }
}
