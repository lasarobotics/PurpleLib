// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.hardware.revrobotics;

import java.util.HashMap;

import org.lasarobotics.hardware.PurpleManager;
import org.lasarobotics.utils.GlobalConstants;
import org.tinylog.Logger;

import com.revrobotics.SparkHelpers;

/** This is a basic monitor class separate from the HealthMonitor setup. */
public class SparkMonitor {
  private static SparkMonitor s_instance = new SparkMonitor();
  private static HashMap<Spark, Short> s_sparks = new HashMap<>();
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
    s_sparks.put(spark, (short) 0);
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
    if (s_runCount++ < GlobalConstants.ROBOT_LOOP_HZ) return;

    s_runCount = 0;

    s_sparks.forEach((sparkMax, prevFault) -> {
      short faults = sparkMax.getStickyFaults();
      if (faults != prevFault.shortValue()) {
        Logger.tag("Spark Monitor")
            .warn(
                "{} faults: {}",
                sparkMax.getID().name,
                SparkHelpers.faultWordToString(faults));
      }
      s_sparks.put(sparkMax, faults);
    });
  }
}
