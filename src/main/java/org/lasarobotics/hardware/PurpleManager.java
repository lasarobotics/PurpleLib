// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.hardware;

import java.util.ArrayList;
import java.util.Arrays;

import org.lasarobotics.battery.BatteryTracker;
import org.lasarobotics.utils.GlobalConstants;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;

/** PurpleLib Hardware Logging Manager */
public class PurpleManager {
  private static ArrayList<LoggableHardware> m_hardware = new ArrayList<>();
  private static ArrayList<Runnable> m_callbacks = new ArrayList<>();
  private static ArrayList<Runnable> m_simCallbacks = new ArrayList<>();

  /**
   * Initialize and start logging
   * <p>
   * Call this at the beginning of <code>robotInit()</code>.
   * <p>
   * To enable replay, set the environment variable <code>ROBOT_REPLAY=1</code>, otherwise simply unset this variable.
   * @param robot Robot object
   * @param projectName Project name
   * @param gitSHA Git SHA
   * @param buildDate Build date string
   * @param logPath Path for log file
   * @param batteryTrackingEnabled True to enable battery tracking
   */
  @SuppressWarnings("resource")
  public static void initialize(LoggedRobot robot,
                                String projectName,
                                String gitSHA,
                                String buildDate,
                                String logPath,
                                boolean batteryTrackingEnabled) {
    // AdvantageKit Logging
    Logger.recordMetadata("ProjectName", projectName);
    Logger.recordMetadata("gitSHA", gitSHA);
    Logger.recordMetadata("BuildDate", buildDate);

    if (RobotBase.isReal()) {
      // If robot is real, log to USB drive and publish data to NetworkTables
      Logger.addDataReceiver(new WPILOGWriter(logPath));
      Logger.addDataReceiver(new NT4Publisher());
      new PowerDistribution();

      // Battery Tracking
      if (batteryTrackingEnabled) {
        BatteryTracker batteryTracker = new BatteryTracker(BatteryTracker.initializeHardware());
        Logger.recordMetadata("BatteryName", batteryTracker.scanBattery());
        if (batteryTracker.isBatteryReused())
          DriverStation.reportError(batteryTracker.scanBattery() + " is being reused!", false);
        else batteryTracker.writeCurrentBattery();
      }
    } else {
      // Else just publish to NetworkTables for simulation or replay log file if var is set
      String replay = System.getenv(GlobalConstants.REPLAY_ENVIRONMENT_VAR);
      if (replay == null || replay.isBlank()) Logger.addDataReceiver(new NT4Publisher());
      else {
        // Run as fast as possible
        robot.setUseTiming(false);
        // Pull the replay log from AdvantageScope (or prompt the user)
        String replayLogPath = LogFileUtil.findReplayLog();
        // Read replay log
        Logger.setReplaySource(new WPILOGReader(replayLogPath));
        // Save outputs to a new log
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(replayLogPath, "_sim")));
      }
    }

    // Start logging! No more data receivers, replay sources, or metadata values may be added.
    Logger.start();
  }

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
   * Only update all sim callbacks
   */
  public static void updateSim() {
    m_simCallbacks.stream().forEach(Runnable::run);
  }

  /**
   * Remove all hardware devices and callbacks from logging manager
   */
  public static void clear() {
    m_hardware.stream().forEach((device) -> device.close());
    m_hardware.clear();
    m_callbacks.clear();
    m_simCallbacks.clear();
  }
}
