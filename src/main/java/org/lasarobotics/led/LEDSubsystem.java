// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.led;

import org.lasarobotics.led.LEDStrip.Section;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** LED Subsystem */
public class LEDSubsystem extends SubsystemBase implements AutoCloseable {
  private static LEDPattern m_overridePattern = LEDPattern.solid(LEDStrip.TEAM_COLOR);

  private LEDStrip m_ledStrip;

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem(LEDStrip ledStrip) {
    this.m_ledStrip = ledStrip;
    setDefaultCommand(run(m_ledStrip.runAnimation()));
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
    m_ledStrip = ledStrips;
  }

  /**
   * Start override of all LEDs
   * @param pattern Pattern to override with
   */
  public void startOverride(LEDPattern pattern) {
    m_overridePattern = pattern;
    m_ledStrip.startOverride();
    m_ledStrip.set(m_overridePattern, Section.FULL);
  }

  /**
   * End override of LEDs, resume previous patterns
   */
  public void endOverride() {
    m_overridePattern = LEDPattern.solid(LEDStrip.TEAM_COLOR);
    m_ledStrip.endOverride();
  }

  @Override
  public void close() {
    m_ledStrip.close();
  }
}

