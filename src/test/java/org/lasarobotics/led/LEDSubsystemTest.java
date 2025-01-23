// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.led;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;
import org.lasarobotics.led.LEDStrip.AddressableLED;
import org.lasarobotics.led.LEDStrip.Section;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
public class LEDSubsystemTest {
  private final int LENGTH = 31;
  private final int MIDDLE_START = 11;
  private final int MIDDLE_END = 21;

  private LEDStrip m_ledStrip1;

  private AddressableLED m_leds1;

  @BeforeEach
  public void setup() {
    HAL.initialize(100, 0);

    // Create mock hardware devices
    m_leds1 = mock(AddressableLED.class);

    // Return length for unit tests
    when(m_leds1.getLength()).thenReturn(LENGTH);

    // Create LED strip objects
    m_ledStrip1 = new LEDStrip(new LEDStrip.Hardware(m_leds1));

    // Set LED strip for subsystem
    LEDSubsystem.getInstance().setLEDStrip(m_ledStrip1);
  }

  @AfterEach
  public void close() {
    LEDSubsystem.getInstance().close();
  }

  @Test
  @Order(1)
  @DisplayName("Test if robot can set LED strip to single static solid color")
  public void solidFull() {
    // Set LED pattern
    LEDSubsystem.getInstance().set(LEDPattern.solid(LEDStrip.TEAM_COLOR), LEDStrip.Section.FULL);

    // Run LED subsystem loop
    LEDSubsystem.getInstance().getDefaultCommand().execute();

    Color ledBuffer[] = new Color[LENGTH];
    for (int i = 0; i < ledBuffer.length; i++) ledBuffer[i] = m_ledStrip1.getBuffer().getLED(i);

    // Verify LED pattern
    for (int i = 0; i < LENGTH; i++)
      assertEquals(LEDStrip.TEAM_COLOR, m_ledStrip1.getBuffer().getLED(i));
  }

  @Test
  @Order(2)
  @DisplayName("Test if robot can set LED strip start section independently")
  public void startSection() {
    // Set LED pattern
    LEDSubsystem.getInstance().set(LEDPattern.solid(Color.kRed), Section.START);
    LEDSubsystem.getInstance().set(LEDPattern.solid(LEDStrip.TEAM_COLOR), Section.MIDDLE, Section.END);

    // Run LED subsystem loop
    LEDSubsystem.getInstance().getDefaultCommand().execute();

    // Verify LED pattern
    for (int i = 0; i < MIDDLE_START; i++)
      assertEquals(Color.kRed, m_ledStrip1.getBuffer().getLED(i));
    for (int i = MIDDLE_START; i < LENGTH; i++)
      assertEquals(LEDStrip.TEAM_COLOR, m_ledStrip1.getBuffer().getLED(i));
  }

  @Test
  @Order(3)
  @DisplayName("Test if robot can set LED strip middle section independently")
  public void middleSection() {
    // Set LED pattern
    LEDSubsystem.getInstance().set(LEDPattern.solid(Color.kRed), Section.MIDDLE);
    LEDSubsystem.getInstance().set(LEDPattern.solid(LEDStrip.TEAM_COLOR), Section.START, Section.END);

    // Run LED subsystem loop
    LEDSubsystem.getInstance().getDefaultCommand().execute();

    Color ledBuffer[] = new Color[LENGTH];
    for (int i = 0; i < ledBuffer.length; i++) ledBuffer[i] = m_ledStrip1.getBuffer().getLED(i);

    // Verify LED pattern
    for (int i = 0; i < MIDDLE_START; i++)
      assertEquals(LEDStrip.TEAM_COLOR, m_ledStrip1.getBuffer().getLED(i));
    for (int i = MIDDLE_START; i < MIDDLE_END; i++)
      assertEquals(Color.kRed, m_ledStrip1.getBuffer().getLED(i));
    for (int i = MIDDLE_END; i < LENGTH; i++)
      assertEquals(LEDStrip.TEAM_COLOR, m_ledStrip1.getBuffer().getLED(i));
  }

  @Test
  @Order(4)
  @DisplayName("Test if robot can set LED strip end section independently")
  public void endSection() {
    // Set LED pattern
    LEDSubsystem.getInstance().set(LEDPattern.solid(Color.kRed), Section.END);
    LEDSubsystem.getInstance().set(LEDPattern.solid(LEDStrip.TEAM_COLOR), Section.START, Section.MIDDLE);

    // Run LED subsystem loop
    LEDSubsystem.getInstance().getDefaultCommand().execute();

    // Verify LED pattern
    for (int i = 0; i < MIDDLE_END; i++)
      assertEquals(LEDStrip.TEAM_COLOR, m_ledStrip1.getBuffer().getLED(i));
    for (int i = MIDDLE_END; i < LENGTH; i++)
      assertEquals(Color.kRed, m_ledStrip1.getBuffer().getLED(i));
  }

  @Test
  @Order(6)
  @DisplayName("Test if robot can override subsystem LED control")
  public void ledOverride() {
    // Set LED pattern
    LEDSubsystem.getInstance().set(LEDPattern.solid(Color.kBlue), Section.FULL);

    // Request LED override
    LEDSubsystem.getInstance().startOverride(LEDPattern.solid(LEDStrip.TEAM_COLOR));

    // Run LED subsystem loop
    LEDSubsystem.getInstance().getDefaultCommand().execute();

    // Verify LED pattern
    for (int i = 0; i < LENGTH; i++)
      assertEquals(LEDStrip.TEAM_COLOR, m_ledStrip1.getBuffer().getLED(i));
    for (int i = 0; i < LENGTH; i++)
      assertEquals(LEDStrip.TEAM_COLOR, m_ledStrip1.getBuffer().getLED(i));

    // End LED override
    LEDSubsystem.getInstance().endOverride();

    // Run LED subsystem loop
    LEDSubsystem.getInstance().getDefaultCommand().execute();

    // Verify LED pattern
    for (int i = 0; i < LENGTH; i++)
      assertEquals(Color.kBlue, m_ledStrip1.getBuffer().getLED(i));
  }
}
