// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.led;

import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

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

    /**
     * Get start index of LED strip sections
     * @param buffer LED buffer
     * @param sections Desired sections
     * @return Start index
     */
    private static int start(AddressableLEDBuffer buffer, Section... sections) {
      return start(
        buffer,
        Collections.min(
          Arrays.asList(sections),
          (a, b) -> Integer.compare(a.ordinal(), b.ordinal())
        )
      );
    }

    /**
     * Get start index of LED strip section
     * @param buffer LED buffer
     * @param section Desired section
     * @return Start index
     */
    private static int start(AddressableLEDBuffer buffer, Section section) {
      switch (section) {
        case START:
          return 0;
        case MIDDLE:
          return SMALL_SECTION_LENGTH;
        case END:
          return buffer.getLength() - SMALL_SECTION_LENGTH;
        default:
          return 0;
      }
    }

    /**
     * Get end index of LED strip sections
     * @param buffer LED buffer
     * @param sections Desired sections
     * @return End index
     */
    private static int end(AddressableLEDBuffer buffer, Section... sections) {
      return end(
        buffer,
        Collections.max(
          Arrays.asList(sections),
          (a, b) -> Integer.compare(a.ordinal(), b.ordinal())
        )
      );
    }

    /**
     * Get end index of LED strip section
     * @param buffer LED buffer
     * @param sections Desired section
     * @return end index
     */
    private static int end(AddressableLEDBuffer buffer, Section section) {
      switch (section) {
        case START:
          return SMALL_SECTION_LENGTH;
        case MIDDLE:
          return buffer.getLength() - SMALL_SECTION_LENGTH;
        case END:
          return buffer.getLength();
        default:
          return buffer.getLength();
      }
    }

    /**
     * Check if index is within LED strip sections
     * @param i Index
     * @param buffer LED buffer
     * @param sections Desired sections
     * @return True if index falls within specified sections
     */
    private static boolean contains(int i, AddressableLEDBuffer buffer, Section... sections) {
      boolean contains = false;
      for (Section section : sections) {
        contains |= Section.start(buffer, section) <= i && i < Section.end(buffer, section);
        if (contains) break;
      }

      return contains;
    }
  }

  /**
   * Valid LED patterns
   */
  public enum Pattern {
    // Team color patterns
    TEAM_COLOR_SOLID(PatternType.SOLID, TEAM_COLOR),
    TEAM_COLOR_STROBE(PatternType.STROBE, TEAM_COLOR),
    TEAM_COLOR_BREATHE(PatternType.BREATHE, TEAM_COLOR),
    TEAM_COLOR_WAVE(PatternType.WAVE, TEAM_COLOR),
    // White patterns
    WHITE_SOLID(PatternType.SOLID, Color.kWhite),
    WHITE_STROBE(PatternType.STROBE, Color.kWhite),
    WHITE_BREATHE(PatternType.BREATHE, Color.kWhite),
    WHITE_WAVE(PatternType.WAVE, Color.kWhite),
    // Red patterns
    RED_SOLID(PatternType.SOLID, Color.kRed),
    RED_STROBE(PatternType.STROBE, Color.kRed),
    RED_BREATHE(PatternType.BREATHE, Color.kRed),
    RED_WAVE(PatternType.WAVE, Color.kRed),
    // Orange patterns
    ORANGE_SOLID(PatternType.SOLID, Color.kOrange),
    ORANGE_STROBE(PatternType.STROBE, Color.kOrange),
    ORANGE_BREATHE(PatternType.BREATHE, Color.kOrange),
    ORANGE_WAVE(PatternType.WAVE, Color.kOrange),
    // Yellow patterns
    YELLOW_SOLID(PatternType.SOLID, Color.kYellow),
    YELLOW_STROBE(PatternType.STROBE, Color.kYellow),
    YELLOW_BREATHE(PatternType.BREATHE, Color.kYellow),
    YELLOW_WAVE(PatternType.WAVE, Color.kYellow),
    // Green patterns
    GREEN_SOLID(PatternType.SOLID, Color.kGreen),
    GREEN_STROBE(PatternType.STROBE, Color.kGreen),
    GREEN_BREATHE(PatternType.BREATHE, Color.kGreen),
    GREEN_WAVE(PatternType.WAVE, Color.kGreen),
    // Blue patterns
    BLUE_SOLID(PatternType.SOLID, Color.kBlue),
    BLUE_STROBE(PatternType.STROBE, Color.kBlue),
    BLUE_BREATHE(PatternType.BREATHE, Color.kBlue),
    BLUE_WAVE(PatternType.WAVE, Color.kBlue),
    // Indigo patterns
    INDIGO_SOLID(PatternType.SOLID, Color.kIndigo),
    INDIGO_STROBE(PatternType.STROBE, Color.kIndigo),
    INDIGO_BREATHE(PatternType.BREATHE, Color.kIndigo),
    INDIGO_WAVE(PatternType.WAVE, Color.kIndigo),
    // Violet patterns
    VIOLET_SOLID(PatternType.SOLID, Color.kViolet),
    VIOLET_STROBE(PatternType.STROBE, Color.kViolet),
    VIOLET_BREATHE(PatternType.BREATHE, Color.kViolet),
    VIOLET_WAVE(PatternType.WAVE, Color.kViolet),
    // Pink patterns
    PINK_SOLID(PatternType.SOLID, Color.kPink),
    PINK_STROBE(PatternType.STROBE, Color.kPink),
    PINK_BREATHE(PatternType.BREATHE, Color.kPink),
    PINK_WAVE(PatternType.WAVE, Color.kPink),
    // Special patterns
    OFF(PatternType.SOLID, Color.kBlack),
    RAINBOW(PatternType.RAINBOW, Color.kBlack);

    /** Pattern type */
    public final PatternType type;
    /** Pattern color */
    public final Color color;
    private Pattern(PatternType type, Color color) {
      this.type = type;
      this.color = color;
    }
  }

  private enum PatternType {
    SOLID, STROBE, BREATHE, WAVE, RAINBOW;
  }

  /** Addressable LEDs */
  public static class AddressableLED extends edu.wpi.first.wpilibj.AddressableLED {
    private final String m_name;
    private final int m_length;
    private AddressableLEDBuffer m_ledBuffer;
    private AddressableLEDBufferView m_startLeds;
    private AddressableLEDBufferView m_middleLeds;
    private AddressableLEDBufferView m_endLeds;
    private LEDPattern m_startAnimation;
    private LEDPattern m_middleAnimation;
    private LEDPattern m_endAnimation;


    /**
     * Addressable LED strip
     * @param name Name
     * @param port PWM port (Must be on RIO)
     * @param length Length of LED strip
     */
    public AddressableLED(String name, int port, int length) {
      super(port);
      this.m_ledBuffer = new AddressableLEDBuffer(length);
      this.m_startLeds = m_ledBuffer.createView(0, 9);
      this.m_middleLeds = m_ledBuffer.createView(10, length-10);
      this.m_endLeds = m_ledBuffer.createView(length-9, length);
      this.m_name = name;
      this.m_length = length;
      this.m_startAnimation = LEDPattern.solid(TEAM_COLOR);
      this.m_middleAnimation = LEDPattern.solid(TEAM_COLOR);
      this.m_endAnimation = LEDPattern.solid(TEAM_COLOR);



      if (length < Section.SMALL_SECTION_LENGTH * 2 + 1)
        throw new IllegalArgumentException("Length is too short, must be at least " + Section.SMALL_SECTION_LENGTH * 2 + 1);

      setLength(length);
    }

    /**
     * Get name
     * @return name
     */
    public String getName() { return m_name; }

    /**
     * Get length
     * @return length
     */
    public int getLength() { return m_length; }
  }

  private AddressableLED m_leds;
  private AddressableLEDBuffer m_buffer;
  private HashMap<Section[], Pattern> m_sectionLEDPatterns, m_tempLEDPatterns;


  private static final double STROBE_DURATION = 0.2;
  private static final double BREATHE_DURATION = 4.0;
  private static final double RAINBOW_CYCLE_LENGTH = 25.0;
  private static final double RAINBOW_DURATION = 2.0;
  private static final double WAVE_EXPONENT = 0.4;
  private static final double WAVE_CYCLE_LENGTH = 25.0;
  private static final double WAVE_DURATION = 2.0;

  /**
   * Create an instance of an LED strip
   * @param ledStripHardware Hardware devices required by LED strip
   */
  public LEDStrip(Hardware ledStripHardware) {
    this.m_leds = ledStripHardware.ledStrip;
    this.m_buffer = new AddressableLEDBuffer(m_leds.getLength());
    this.m_sectionLEDPatterns = new HashMap<Section[], Pattern>();
    this.m_tempLEDPatterns = new HashMap<Section[], Pattern>();

    m_sectionLEDPatterns.put(Section.FULL, Pattern.TEAM_COLOR_SOLID);
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

  public Command runAnimation() {
    return Commands.run(() -> {
      m_leds.m_startAnimation.applyTo(m_leds.m_startLeds);
      m_leds.m_middleAnimation.applyTo(m_leds.m_middleLeds);
      m_leds.m_endAnimation.applyTo(m_leds.m_endLeds);
    });
  }

public void setAnimation(LEDPattern animation, Section... sections){
  for(Section i : sections){
    switch (i) {
      case START:
        m_leds.m_startAnimation = animation;
      case MIDDLE:
        m_leds.m_middleAnimation = animation;
      case END:
        m_leds.m_endAnimation = animation;
      default:
        continue;
    }
  }
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
    return m_buffer;
  }

  /**
   * Set pattern and color of LED strip
   * @param pattern Desired pattern
   */
  public void set(Pattern pattern) {
    set(pattern, Section.FULL);
  }

  /**
   * Set pattern and color of LED strip sections
   * @param pattern Desired pattern
   * @param sections LED strip sections to set
   */
  public void set(Pattern pattern, Section... sections) {
    // Remove all conflicting scheduled LED patterns
    m_sectionLEDPatterns.entrySet().removeIf(
      (entry) -> !Collections.disjoint(Arrays.asList(entry.getKey()), Arrays.asList(sections))
    );

    // Schedule LED pattern
    m_sectionLEDPatterns.put(sections, pattern);

    // Log patterns
    for (Section section : sections)
      Logger.recordOutput(String.join("/", m_leds.getName(), section.name()), pattern.name());
  }

  /**
   * Turn off LED strip
   */
  public void off() {
    set(Pattern.OFF, Section.FULL);
  }

  /**
   * Turn off LED strip sections
   * @param sections LED strip sections
   */
  public void off(Section... sections) {
    set(Pattern.OFF, sections);
  }

  @Override
  public void close() {
    m_leds.close();
  }
}
