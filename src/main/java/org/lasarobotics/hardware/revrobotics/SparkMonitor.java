// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.hardware.revrobotics;

import java.util.HashSet;

import org.lasarobotics.hardware.PurpleManager;
import org.lasarobotics.utils.GlobalConstants;
import org.tinylog.Logger;

import edu.wpi.first.units.Units;

/** This is a basic monitor class separate from the HealthMonitor setup. */
public class SparkMonitor {
  private static SparkMonitor s_instance = new SparkMonitor();
  private static HashSet<Spark> s_sparks = new HashSet<>();
  private static int s_runCount = 0;

  /** Creates a new SparkMonitor. */
  private SparkMonitor() {
    PurpleManager.addCallback(() -> s_instance.periodic());
  }

  /**
   * Get Spark monitor
   * @return Instance of Spark monitor
   */
  public static SparkMonitor getInstance() {
    return s_instance;
  }

  /**
   * Add Spark to be monitored for faults
   * @param spark Spark to monitor
   * @return True if added successfully
   */
  public boolean add(Spark spark) {
    return s_sparks.add(spark);
  }

  /**
   * Remove Spark from monitor
   * @param spark Spark to be removed
   * @return True if removed successfully
   */
  public boolean remove(Spark spark) {
    return s_sparks.remove(spark);
  }

  public void periodic() {
    long sTime = System.currentTimeMillis();
    // Run at 1 second
    if (s_runCount++ < GlobalConstants.ROBOT_LOOP_HZ.in(Units.Hertz)) return;

    s_runCount = 0;

    s_sparks.forEach((spark) -> {
      var faults = spark.getStickyFaults();
      var warnings = spark.getWarnings();
      if (faults.rawBits != 0) {
        Logger.tag("Spark Monitor").warn(
          "{} faults: {}",
          spark.getID().name,
          "Other: " + faults.other +
          " Motor Type: " + faults.motorType +
          " Sensor: " + faults.sensor +
          " CAN: " + faults.can +
          " Temperature: " + faults.temperature +
          " Gate driver: " + faults.gateDriver +
          " ESC EEprom: " + faults.escEeprom +
          " Firmware: " + faults.firmware
        );
      }
      org.littletonrobotics.junction.Logger.recordOutput("[purpleLibUpdate] Spark Monitor Periodic Checkpoint 1 [" + spark.getID().name + "]", System.currentTimeMillis() - sTime);
      if (warnings.rawBits != 0) {
        Logger.tag("Spark Monitor").warn(
          "{} warnings: {}",
          spark.getID().name,
          "Brownout: " + warnings.brownout +
          " Overcurrent: " + warnings.overcurrent +
          " ESC EEprom: " + warnings.escEeprom +
          " Ext EEprom: " + warnings.extEeprom +
          " Sensor: " + warnings.sensor +
          " Stall: " + warnings.stall +
          " Has reset: " + warnings.hasReset +
          " Other: " + warnings.other
        );
      }
      org.littletonrobotics.junction.Logger.recordOutput("[purpleLibUpdate] Spark Monitor Periodic Checkpoint 2 [" + spark.getID().name + "]", System.currentTimeMillis() - sTime);
    });
    org.littletonrobotics.junction.Logger.recordOutput("[purpleLibUpdate] Spark Monitor Periodic End", System.currentTimeMillis() - sTime);
  }
}
