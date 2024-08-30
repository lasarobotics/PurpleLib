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
  private static SparkMonitor m_instance = new SparkMonitor();
  private static HashMap<Spark, Short> m_sparks = new HashMap<>();
  private static int m_runCount = 0;

  /** Creates a new SparkMonitor. */
  private SparkMonitor() {
    PurpleManager.addCallback(this::periodic);
  }

  /**
   * Get Spark monitor
   * @return Instance of Spark monitor
   */
  public static SparkMonitor getInstance() {
    return m_instance;
  }

  /**
   * Add Spark to be monitored for faults
   * @param spark Spark to monitor
   * @return True if added successfully
   */
  public boolean add(Spark spark) {
    if (m_sparks.containsKey(spark)) return false;
    m_sparks.put(spark, (short) 0);
    return true;
  }

  /**
   * Remove Spark from monitor
   * @param spark Spark to be removed
   * @return True if removed successfully
   */
  public boolean remove(Spark spark) {
    if (m_sparks.containsKey(spark)) {
      m_sparks.remove(spark);
      return true;
    } else return false;
  }

  public void periodic() {
    // Run at 1 second
    if (m_runCount++ < GlobalConstants.ROBOT_LOOP_HZ) return;

    m_runCount = 0;

    m_sparks.forEach((sparkMax, prevFault) -> {
      short faults = sparkMax.getStickyFaults();
      if (faults != prevFault.shortValue()) {
        Logger.tag("Spark Monitor")
            .warn(
                "{} faults: {}",
                sparkMax.getID().name,
                SparkHelpers.faultWordToString(faults));
      }
      m_sparks.put(sparkMax, faults);
    });
  }
}
