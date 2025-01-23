// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.led;

import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

/** LED Strip */
public class LEDStrip implements AutoCloseable {
  public static final Color TEAM_COLOR = new Color(0x66, 0x33, 0x99);

  /** LED strip ID */
  public static class ID {
    public final String name;
    public final int port;
    public final int length;

    /**
     * LED strip ID
     * @param name Device name for logging
     * @param port PWM output port
     * @param length Strip length
     */
    public ID(String name, int port, int length) {
      this.name = name;
      this.port = port;
      this.length = length;
    }
  }

  /**
   * LED strip hardware
   */
  public static class Hardware {
    AddressableLED ledStrip;

    public Hardware(AddressableLED ledStrip) {
      this.ledStrip = ledStrip;
    }
  }

  /**
   * LED strip sections
   */
  public static enum Section {
    START,
    MIDDLE,
    END;

    public static final Section[] FULL = { START, MIDDLE, END };

    private static final int SMALL_SECTION_LENGTH = 10;
  }
  /** Addressable LEDs */
  public static class AddressableLED extends edu.wpi.first.wpilibj.AddressableLED {
    private final String name;
    private final int length;


    /**
     * Addressable LED strip
     * @param name Name
     * @param port PWM port (Must be on RIO)
     * @param length Length of LED strip
     */
    public AddressableLED(String name, int port, int length) {
      super(port);
      this.name = name;
      this.length = length;

      if (length < Section.SMALL_SECTION_LENGTH * 2 + 1)
        throw new IllegalArgumentException("Length is too short, must be at least " + Section.SMALL_SECTION_LENGTH * 2 + 1);

      setLength(length);
    }

    /**
     * Get name
     * @return name
     */
    public String getName() { return name; }

    /**
     * Get length
     * @return length
     */
    public int getLength() { return length; }
  }

  private AddressableLED m_leds;
  private AddressableLEDBuffer m_ledBuffer;
  private HashMap<Section[], LEDPattern> m_sectionLEDPatterns, m_tempLEDPatterns;

  private final Map<Section, AddressableLEDBufferView> SECTION_MAP;

  /**
   * Create an instance of an LED strip
   * @param ledStripHardware Hardware devices required by LED strip
   */
  public LEDStrip(Hardware ledStripHardware) {
    this.m_leds = ledStripHardware.ledStrip;
    this.m_sectionLEDPatterns = new HashMap<>();
    this.m_tempLEDPatterns = new HashMap<>();
    this.m_ledBuffer = new AddressableLEDBuffer(m_leds.getLength());

    var startBufferView = m_ledBuffer.createView(0, Section.SMALL_SECTION_LENGTH);
    var middleBufferView = m_ledBuffer.createView(Section.SMALL_SECTION_LENGTH + 1, m_leds.getLength() - Section.SMALL_SECTION_LENGTH - 1);
    var endBufferView = m_ledBuffer.createView(m_leds.getLength() - Section.SMALL_SECTION_LENGTH, m_leds.getLength() - 1);

    SECTION_MAP = Map.ofEntries(
      Map.entry(Section.START, startBufferView),
      Map.entry(Section.MIDDLE, middleBufferView),
      Map.entry(Section.END, endBufferView)
    );

    m_sectionLEDPatterns.put(Section.FULL, LEDPattern.solid(TEAM_COLOR));
    m_leds.start();
  }

  /**
   * Initialize hardware devices for LED strip
   *
   * @param id LED strip ID
   * @return Hardware object containing all necessary devices
   */
  public static Hardware initializeHardware(ID id) {
    Hardware ledStripHardware = new Hardware(new AddressableLED(id.name, id.port, id.length));

    return ledStripHardware;
  }

  Runnable runAnimation() {
    return () -> {
      m_sectionLEDPatterns.entrySet().stream().forEach(entry -> {
        for (var section : entry.getKey()) entry.getValue().applyTo(SECTION_MAP.get(section));
      });
      m_leds.setData(m_ledBuffer);
    };
  }

  /**
   * Prepare for LED override
   */
  protected void startOverride() {
    // Save LED patterns
    m_tempLEDPatterns.clear();
    m_tempLEDPatterns.putAll(m_sectionLEDPatterns);
  }

  /**
   * Restore LED patterns after override
   */
  protected void endOverride() {
    m_sectionLEDPatterns.clear();
    m_sectionLEDPatterns.putAll(m_tempLEDPatterns);
  }

  /**
   * Get latest LED buffer
   * @return Addressable LED buffer
   */
  public AddressableLEDBuffer getBuffer() {
    return m_ledBuffer;
  }

  /**
   * Set pattern and color of LED strip
   * @param pattern Desired pattern
   */
  public void set(LEDPattern pattern) {
    set(pattern, Section.FULL);
  }

  /**
   * Set pattern and color of LED strip sections
   * @param pattern Desired pattern
   * @param sections LED strip sections to set
   */
  public void set(LEDPattern pattern, Section... sections) {
    // Remove all conflicting scheduled LED patterns
    m_sectionLEDPatterns.entrySet().removeIf(
      (entry) -> !Collections.disjoint(Arrays.asList(entry.getKey()), Arrays.asList(sections))
    );

    // Schedule LED pattern
    m_sectionLEDPatterns.put(sections, pattern);

    // Log patterns
    for (Section section : sections)
      Logger.recordOutput(String.join("/", m_leds.getName(), section.name()), pattern.toString());
  }

  /**
   * Turn off LED strip
   */
  public void off() {
    set(LEDPattern.kOff, Section.FULL);
  }

  /**
   * Turn off LED strip sections
   * @param sections LED strip sections
   */
  public void off(Section... sections) {
    set(LEDPattern.kOff, sections);
  }

  @Override
  public void close() {
    m_leds.close();
  }
}
