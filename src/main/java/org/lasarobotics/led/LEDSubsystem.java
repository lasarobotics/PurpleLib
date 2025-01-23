// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.led;

import org.lasarobotics.led.LEDStrip.Section;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** LED Subsystem */
public class LEDSubsystem extends SubsystemBase implements AutoCloseable {
  private static LEDSubsystem s_instance;
  private static LEDPattern m_overridePattern = LEDPattern.solid(LEDStrip.TEAM_COLOR);
  private static LEDStrip s_ledStrip;

  /** Creates a new LEDSubsystem. */
  private LEDSubsystem() {
    setDefaultCommand(run(() -> {
      if (s_ledStrip != null) s_ledStrip.runAnimation();
    }));
  }

  public static LEDSubsystem getInstance() {
    if (s_instance == null) s_instance = new LEDSubsystem();
    return s_instance;
  }

  public void setLEDStrip(LEDStrip ledStrip) {
    s_ledStrip = ledStrip;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Start override of all LEDs
   * @param pattern Pattern to override with
   */
  public void startOverride(LEDPattern pattern) {
    m_overridePattern = pattern;
    s_ledStrip.startOverride();
    s_ledStrip.set(m_overridePattern, Section.FULL);
  }

  /**
   * End override of LEDs, resume previous patterns
   */
  public void endOverride() {
    m_overridePattern = LEDPattern.solid(LEDStrip.TEAM_COLOR);
    s_ledStrip.endOverride();
  }

  /**
   * Set pattern and color of LED strip sections
   * @param pattern Desired pattern
   * @param sections LED strip sections to set
   */
  public void set(LEDPattern pattern, Section... sections) {
    s_ledStrip.set(pattern, sections);
  }

  /**
   * Set pattern and color of LED strip
   * @param pattern Desired pattern
   */
  public void set(LEDPattern pattern) {
    s_ledStrip.set(pattern, Section.FULL);
  }

  /**
   * Turn off LED strip sections
   * @param sections LED strip sections
   */
  public void off(Section... sections) {
    s_ledStrip.set(LEDPattern.kOff, sections);
  }

  /**
   * Turn off LED strip
   */
  public void off() {
    s_ledStrip.set(LEDPattern.kOff, Section.FULL);
  }

  @Override
  public void close() {
    s_ledStrip.close();
  }
}

