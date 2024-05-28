// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.hardware;

import java.nio.file.Path;
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
import org.littletonrobotics.urcl.URCL;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

import com.ctre.phoenix6.SignalLogger;
import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;

/** PurpleLib Hardware Logging Manager */
public class PurpleManager {
  private static ArrayList<LoggableHardware> m_hardware = new ArrayList<>();
  private static ArrayList<Runnable> m_callbacks = new ArrayList<>();
  private static ArrayList<Runnable> m_simCallbacks = new ArrayList<>();
  private static VisionSystemSim m_visionSim = new VisionSystemSim("PurpleManager");

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
                                AprilTagFieldLayout fieldLayout,
                                Path logPath,
                                String projectName,
                                String gitSHA,
                                String buildDate,
                                boolean batteryTrackingEnabled) {
    // AdvantageKit Logging
    Logger.recordMetadata("ProjectName", projectName);
    Logger.recordMetadata("gitSHA", gitSHA);
    Logger.recordMetadata("BuildDate", buildDate);

    if (RobotBase.isReal()) {
      // If robot is real, log to USB drive and publish data to NetworkTables
      Logger.addDataReceiver(new WPILOGWriter(logPath.toAbsolutePath().toString()));
      Logger.addDataReceiver(new NT4Publisher());
      new PowerDistribution();

      // Set CTRE log path
      SignalLogger.setPath(Path.of(logPath.toAbsolutePath().toString(), "ctre-logs").toAbsolutePath().toString());

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
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(replayLogPath, "_replay_sim")));
      }
    }

    // Initialize vision sim
    fieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
    m_visionSim.addAprilTags(fieldLayout);

    // Run REV physics sim
    addCallbackSim(() -> REVPhysicsSim.getInstance().run());

    // Register URCL for logging REV devices
    Logger.registerURCL(URCL.startExternal());

    // Start CTRE signal logging
    SignalLogger.start();

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
   * Adds a simulated camera with a specified robot-to-camera transformation
   * <p>
   * The vision targets registered will be observed by the camera
   * @param cameraSim The camera simulation
   * @param robotToCamera The transform from the robot pose to the camera pose
   */
  public static void addCameraSim(PhotonCameraSim cameraSim, Transform3d robotToCamera) {
    m_visionSim.addCamera(cameraSim, robotToCamera);
  }

  /**
   * Adds targets on the field which your vision system is designed to detect. The
   * cameras simulated from this simulation will report the location of the camera relative to
   * the subset of these targets which are visible from the given camera position.
   * <p>
   * By default these are added under the type "targets".
   * @param targets Targets to add to the simulated field
   */
  public static void addVisionTargetSims(VisionTargetSim... targets) {
    m_visionSim.addVisionTargets(targets);
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
