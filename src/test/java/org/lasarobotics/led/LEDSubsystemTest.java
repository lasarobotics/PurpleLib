// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.led;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;
import org.lasarobotics.led.LEDStrip.AddressableLED;
import org.lasarobotics.led.LEDStrip.Pattern;
import org.lasarobotics.led.LEDStrip.Section;
import org.mockito.ArgumentCaptor;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
public class LEDSubsystemTest {
  private final int LENGTH = 30;
  private final int MIDDLE_START = 10;
  private final int MIDDLE_END = 20;
  private LEDSubsystem m_ledSubsystem;

  private LEDStrip m_ledStrip1;
  private LEDStrip m_ledStrip2;

  private AddressableLED m_leds1;
  private AddressableLED m_leds2;

  @BeforeEach
  public void setup() {
    HAL.initialize(100, 0);

    // Create mock hardware devices
    m_leds1 = mock(AddressableLED.class);
    m_leds2 = mock(AddressableLED.class);

    // Return length for unit tests
    when(m_leds1.getLength()).thenReturn(LENGTH);
    when(m_leds2.getLength()).thenReturn(LENGTH);

    // Create LED strip objects
    m_ledStrip1 = new LEDStrip(new LEDStrip.Hardware(m_leds1));
    m_ledStrip2 = new LEDStrip(new LEDStrip.Hardware(m_leds2));

    // Create LEDSubsystem object
    m_ledSubsystem = LEDSubsystem.getInstance();

    // Add LED strips to LED subsystem
    m_ledSubsystem.add(m_ledStrip1);
    m_ledSubsystem.add(m_ledStrip2);
  }

  @AfterEach
  public void close() {
    m_ledSubsystem.close();
    m_ledSubsystem = null;
  }

  @Test
  @Order(1)
  @DisplayName("Test if robot can set LED strip to single static solid color")
  public void solidFull() {
    // Initialize buffer captor
    ArgumentCaptor<AddressableLEDBuffer> bufferCaptor = ArgumentCaptor.forClass(AddressableLEDBuffer.class);

    // Set LED pattern
    m_ledStrip1.set(Pattern.TEAM_COLOR_SOLID, Section.FULL);

    // Run LED subsystem loop
    m_ledSubsystem.periodic();

    // Verify LEDs are being set
    verify(m_leds1, times(1)).setData(bufferCaptor.capture());

    // Verify LED pattern
    for (int i = 0; i < LENGTH; i++)
      assertEquals(LEDStrip.TEAM_COLOR, bufferCaptor.getValue().getLED(i));
  }

  @Test
  @Order(2)
  @DisplayName("Test if robot can set LED strip start section independently")
  public void startSection() {
    // Initialize buffer captor
    ArgumentCaptor<AddressableLEDBuffer> bufferCaptor = ArgumentCaptor.forClass(AddressableLEDBuffer.class);

    // Set LED pattern
    m_ledStrip1.set(Pattern.RED_SOLID, Section.START);
    m_ledStrip1.set(Pattern.TEAM_COLOR_SOLID, Section.MIDDLE, Section.END);

    // Run LED subsystem loop
    m_ledSubsystem.periodic();

    // Verify LEDs are being set
    verify(m_leds1, times(1)).setData(bufferCaptor.capture());

    // Verify LED pattern
    for (int i = 0; i < MIDDLE_START; i++)
      assertEquals(Color.kRed, bufferCaptor.getValue().getLED(i));
    for (int i = MIDDLE_START; i < LENGTH; i++)
      assertEquals(LEDStrip.TEAM_COLOR, bufferCaptor.getValue().getLED(i));
  }

  @Test
  @Order(3)
  @DisplayName("Test if robot can set LED strip middle section independently")
  public void middleSection() {
    // Initialize buffer captor
    ArgumentCaptor<AddressableLEDBuffer> bufferCaptor = ArgumentCaptor.forClass(AddressableLEDBuffer.class);

    // Set LED pattern
    m_ledStrip1.set(Pattern.RED_SOLID, Section.MIDDLE);
    m_ledStrip1.set(Pattern.TEAM_COLOR_SOLID, Section.START, Section.END);

    // Run LED subsystem loop
    m_ledSubsystem.periodic();

    // Verify LEDs are being set
    verify(m_leds1, times(1)).setData(bufferCaptor.capture());

    // Verify LED pattern
    for (int i = 0; i < MIDDLE_START; i++)
      assertEquals(LEDStrip.TEAM_COLOR, bufferCaptor.getValue().getLED(i));
    for (int i = MIDDLE_START; i < MIDDLE_END; i++)
      assertEquals(Color.kRed, bufferCaptor.getValue().getLED(i));
    for (int i = MIDDLE_END; i < LENGTH; i++)
      assertEquals(LEDStrip.TEAM_COLOR, bufferCaptor.getValue().getLED(i));
  }

  @Test
  @Order(4)
  @DisplayName("Test if robot can set LED strip end section independently")
  public void endSection() {
    // Initialize buffer captor
    ArgumentCaptor<AddressableLEDBuffer> bufferCaptor = ArgumentCaptor.forClass(AddressableLEDBuffer.class);

    // Set LED pattern
    m_ledStrip1.set(Pattern.RED_SOLID, Section.END);
    m_ledStrip1.set(Pattern.TEAM_COLOR_SOLID, Section.START, Section.MIDDLE);

    // Run LED subsystem loop
    m_ledSubsystem.periodic();

    // Verify LEDs are being set
    verify(m_leds1, times(1)).setData(bufferCaptor.capture());

    // Verify LED pattern
    for (int i = 0; i < MIDDLE_END; i++)
      assertEquals(LEDStrip.TEAM_COLOR, bufferCaptor.getValue().getLED(i));
    for (int i = MIDDLE_END; i < LENGTH; i++)
      assertEquals(Color.kRed, bufferCaptor.getValue().getLED(i));
  }

  @Test
  @Order(5)
  @DisplayName("Test if robot can control multiple LED strips")
  public void multipleStrips() {
    // Initialize buffer captors
    ArgumentCaptor<AddressableLEDBuffer> bufferCaptor1 = ArgumentCaptor.forClass(AddressableLEDBuffer.class);
    ArgumentCaptor<AddressableLEDBuffer> bufferCaptor2 = ArgumentCaptor.forClass(AddressableLEDBuffer.class);

    // Set LED pattern
    m_ledStrip1.set(Pattern.RED_SOLID, Section.FULL);
    m_ledStrip2.set(Pattern.BLUE_SOLID, Section.FULL);

    // Run LED subsystem loop
    m_ledSubsystem.periodic();

    // Verify LEDs are being set
    verify(m_leds1, times(1)).setData(bufferCaptor1.capture());
    verify(m_leds2, times(1)).setData(bufferCaptor2.capture());

    // Verify LED pattern
    for (int i = 0; i < LENGTH; i++)
      assertEquals(Color.kRed, bufferCaptor1.getValue().getLED(i));
    for (int i = 0; i < LENGTH; i++)
      assertEquals(Color.kBlue, bufferCaptor2.getValue().getLED(i));
  }

  @Test
  @Order(6)
  @DisplayName("Test if robot can override subsystem LED control")
  public void ledOverride() {
    // Initialize buffer captors
    ArgumentCaptor<AddressableLEDBuffer> bufferCaptor1 = ArgumentCaptor.forClass(AddressableLEDBuffer.class);
    ArgumentCaptor<AddressableLEDBuffer> bufferCaptor2 = ArgumentCaptor.forClass(AddressableLEDBuffer.class);

    // Set LED pattern
    m_ledStrip1.set(Pattern.RED_SOLID, Section.FULL);
    m_ledStrip2.set(Pattern.BLUE_SOLID, Section.FULL);

    // Request LED override
    m_ledSubsystem.startOverride(Pattern.TEAM_COLOR_SOLID);

    // Run LED subsystem loop
    m_ledSubsystem.periodic();

    // Verify LEDs are being set
    verify(m_leds1, times(1)).setData(bufferCaptor1.capture());
    verify(m_leds2, times(1)).setData(bufferCaptor2.capture());

    // Verify LED pattern
    for (int i = 0; i < LENGTH; i++)
      assertEquals(LEDStrip.TEAM_COLOR, bufferCaptor1.getValue().getLED(i));
    for (int i = 0; i < LENGTH; i++)
      assertEquals(LEDStrip.TEAM_COLOR, bufferCaptor2.getValue().getLED(i));

    // End LED override
    m_ledSubsystem.endOverride();

    // Run LED subsystem loop
    m_ledSubsystem.periodic();

    // Verify LEDs are being set
    verify(m_leds1, times(2)).setData(bufferCaptor1.capture());
    verify(m_leds2, times(2)).setData(bufferCaptor2.capture());

    // Verify LED pattern
    for (int i = 0; i < LENGTH; i++)
      assertEquals(Color.kRed, bufferCaptor1.getValue().getLED(i));
    for (int i = 0; i < LENGTH; i++)
      assertEquals(Color.kBlue, bufferCaptor2.getValue().getLED(i));
  }
}
