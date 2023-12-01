// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.led;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

import org.lasarobotics.led.LEDStrip.Pattern;
import org.lasarobotics.led.LEDStrip.Section;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** LED Subsystem */
public class LEDSubsystem extends SubsystemBase implements AutoCloseable {
  private static Pattern m_overridePattern = Pattern.TEAM_COLOR_SOLID;

  private static LEDSubsystem m_subsystem;

  private Set<LEDStrip> m_ledStrips;

  /** Creates a new LEDSubsystem. */
  private LEDSubsystem() {
    m_ledStrips = new HashSet<LEDStrip>();
  }

  public static LEDSubsystem getInstance() {
    if (m_subsystem == null) m_subsystem = new LEDSubsystem();
    return m_subsystem;
  }

  private void setLEDs() {
    for (LEDStrip ledStrip : m_ledStrips) ledStrip.update();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setLEDs();
  }

  /**
   * Add LED strips
   * @param ledStrips LED strips to add
   */
  public void add(LEDStrip... ledStrips) {
    m_ledStrips.addAll(Arrays.asList(ledStrips));
  }

  /**
   * Start override of all LEDs
   * @param pattern Pattern to override with
   */
  public void startOverride(Pattern pattern) {
    m_overridePattern = pattern;
    for (LEDStrip ledStrip : m_ledStrips) {
      ledStrip.startOverride();
      ledStrip.set(m_overridePattern, Section.FULL);
    }
  }

  /**
   * End override of LEDs, resume previous patterns
   */
  public void endOverride() {
    m_overridePattern = Pattern.TEAM_COLOR_SOLID;
    for (LEDStrip ledStrip : m_ledStrips) ledStrip.endOverride();
  }

  @Override
  public void close() {
    for (LEDStrip ledStrip : m_ledStrips) ledStrip.close();
  }
}
