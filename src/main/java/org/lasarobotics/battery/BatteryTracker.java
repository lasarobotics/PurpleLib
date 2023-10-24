// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.battery;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;

public class BatteryTracker {
  private static final String PREVIOUS_BATTERY_PATH = "previous_battery.txt";

  /**
   * Check if current battery is being reused
   * @return True if battery is the same as previous
   */
  public static boolean isBatteryReused() {
    File file = new File(PREVIOUS_BATTERY_PATH);

    if (!file.exists()) return false;
    
    // Read previous battery name
    String previousBatteryID = "";
    try {
        previousBatteryID =
          new String(Files.readAllBytes(Paths.get(PREVIOUS_BATTERY_PATH)), StandardCharsets.UTF_8);
    } catch (IOException e) {
      e.printStackTrace();
    }

    if (previousBatteryID.equals(BatteryScanner.scanBattery())) {
      return true;
    } else {
      // New battery, delete file
      file.delete();
      return false;
    }
  }

  /**
   * Write current battery ID to file
   */
  public static void writeCurrentBattery() {
    try {
      FileWriter fileWriter = new FileWriter(PREVIOUS_BATTERY_PATH);
      fileWriter.write(BatteryScanner.scanBattery());
      fileWriter.close();
    } catch (IOException e) {
      e.printStackTrace();
    }
  }
}
