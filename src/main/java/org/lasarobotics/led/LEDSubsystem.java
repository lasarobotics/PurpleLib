// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.led;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

import org.lasarobotics.led.LEDStrip.Pattern;
import org.lasarobotics.led.LEDStrip.Section;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** LED Subsystem */
public class LEDSubsystem extends SubsystemBase implements AutoCloseable {
  private static Pattern m_overridePattern = Pattern.TEAM_COLOR_SOLID;

  private static LEDSubsystem m_subsystem;

  private LEDStrip m_ledStrips;

  /** Creates a new LEDSubsystem. */
  private LEDSubsystem() {
    setDefaultCommand(m_ledStrips.runAnimation());
  }

  public static LEDSubsystem getInstance() {
    if (m_subsystem == null) m_subsystem = new LEDSubsystem();
    return m_subsystem;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  /**
   * Add LED strips
   * @param ledStrips LED strips to add
   */
  public void add(LEDStrip ledStrips) {
    m_ledStrips = ledStrips;
  }

  /**
   * Start override of all LEDs
   * @param pattern Pattern to override with
   */
  public void startOverride(Pattern pattern) {
    m_overridePattern = pattern;
    m_ledStrips.startOverride();
    m_ledStrips.set(m_overridePattern, Section.FULL);
  }

  /**
   * End override of LEDs, resume previous patterns
   */
  public void endOverride() {
    m_overridePattern = Pattern.TEAM_COLOR_SOLID;
    m_ledStrips.endOverride();
  }

  @Override
  public void close() {
    m_ledStrips.close();
  }
}

