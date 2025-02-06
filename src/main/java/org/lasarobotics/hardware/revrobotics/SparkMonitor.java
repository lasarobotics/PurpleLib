// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.hardware.revrobotics;

import java.util.HashMap;

import org.lasarobotics.hardware.PurpleManager;
import org.lasarobotics.utils.GlobalConstants;
import org.tinylog.Logger;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.Faults;

import edu.wpi.first.units.Units;

/** This is a basic monitor class separate from the HealthMonitor setup. */
public class SparkMonitor {
  private static SparkMonitor s_instance = new SparkMonitor();
  private static HashMap<Spark, Faults> s_sparks = new HashMap<>();
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
    if (s_sparks.containsKey(spark)) return false;
    s_sparks.put(spark, new SparkBase.Faults(0));
    return true;
  }

  /**
   * Remove Spark from monitor
   * @param spark Spark to be removed
   * @return True if removed successfully
   */
  public boolean remove(Spark spark) {
    if (s_sparks.containsKey(spark)) {
      s_sparks.remove(spark);
      return true;
    } else return false;
  }

  public void periodic() {
    // Run at 1 second
    if (s_runCount++ < GlobalConstants.ROBOT_LOOP_HZ.in(Units.Hertz)) return;

    s_runCount = 0;

    s_sparks.forEach((sparkMax, prevFaults) -> {
      Faults faults = sparkMax.getStickyFaults();
      if (faults.rawBits != prevFaults.rawBits) {
        Logger.tag("Spark Monitor")
            .warn(
                "{} faults: {}",
                sparkMax.getID().name,
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
      s_sparks.put(sparkMax, faults);
    });
  }
}
