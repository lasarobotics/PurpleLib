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
public class LEDSubsystem extends SubsystemBase implements AutoCloseable {
  private static boolean m_override = false;
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

  private void overrideLEDs() {
    for (LEDStrip ledStrip : m_ledStrips) ledStrip.set(m_overridePattern, Section.FULL);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (m_override) overrideLEDs();
    setLEDs();
  }

  public void add(LEDStrip... ledStrips) {
    m_ledStrips.addAll(Arrays.asList(ledStrips));
  }

  public void startOverride(Pattern pattern) {
    m_override = true;
    m_overridePattern = pattern;
    for (LEDStrip ledStrip : m_ledStrips) ledStrip.startOverride();
  }

  public void endOverride() {
    m_override = false;
    m_overridePattern = Pattern.TEAM_COLOR_SOLID;
    for (LEDStrip ledStrip : m_ledStrips) ledStrip.endOverride();
  }

  @Override
  public void close() {
    for (LEDStrip ledStrip : m_ledStrips) ledStrip.close();
  }
}
