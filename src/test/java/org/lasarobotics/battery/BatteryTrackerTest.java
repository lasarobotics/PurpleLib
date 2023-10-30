// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.battery;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import java.io.File;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;

@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
public class BatteryTrackerTest {
  private static final String BATTERY_1 = "00000001";
  private static final String BATTERY_2 = "00000002";

  private static BatteryTracker m_batteryTracker;
  private static BatteryTracker.Hardware m_batteryHardware;
  private static BatteryScanner m_batteryScanner;

  @BeforeAll
  public static void setup() {
    // Create mock hardware device
    m_batteryScanner = mock(BatteryScanner.class);

    // Create hardware object using mock device
    m_batteryHardware = new BatteryTracker.Hardware(m_batteryScanner);

    // Create battery tracker
    m_batteryTracker = new BatteryTracker(m_batteryHardware);

    // Delete existing battery file if exists
    File file = new File(BatteryTracker.PREVIOUS_BATTERY_PATH);
    if (file.exists()) file.delete();
  }

  @Test
  @Order(1)
  @DisplayName("Test if tracker works when previous battery is unknown")
  public void unknownPrevious() {
    // Check if battery is reused
    assertEquals(false, m_batteryTracker.isBatteryReused());
  }

  @Test
  @Order(2)
  @DisplayName("Test if tracker can recognise reused battery")
  public void usedBattery() {
    // Hardcode sensor values
    when(m_batteryScanner.scanBattery()).thenReturn(BATTERY_1);

    // Write current battery
    m_batteryTracker.writeCurrentBattery();

    // Check if tracker reports battery as reused
    assertEquals(true, m_batteryTracker.isBatteryReused());
  }

  @Test
  @Order(3)
  @DisplayName("Test if tracker can recognise new battery")
  public void newBattery() {
    when(m_batteryScanner.scanBattery()).thenReturn(BATTERY_1);
    m_batteryTracker.writeCurrentBattery();

    when(m_batteryScanner.scanBattery()).thenReturn(BATTERY_2);
    assertEquals(false, m_batteryTracker.isBatteryReused());
  }
}
