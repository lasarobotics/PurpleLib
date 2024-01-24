// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.drive;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;
import org.lasarobotics.utils.GlobalConstants;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.Timer;

@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
public class TractionControlTest {
  private final Measure<Velocity<Distance>> MAX_LINEAR_SPEED = Units.MetersPerSecond.of(4.30);
  private final double DRIVE_SLIP_RATIO = 0.08;

  private TractionControlController m_tractionControlController;

  @BeforeEach
  public void setup() {
    m_tractionControlController = new TractionControlController(MAX_LINEAR_SPEED, DRIVE_SLIP_RATIO);
  }

  @Test
  @Order(1)
  @DisplayName("Test if traction control controller detects and limits slip")
  public void limitSlip() {
    // Simulate scenario
    var outputSpeed = m_tractionControlController.calculate(MAX_LINEAR_SPEED, Units.MetersPerSecond.of(0.0), MAX_LINEAR_SPEED.divide(2));

    for (int i = 0; i < 5; i++) {
      Timer.delay(GlobalConstants.ROBOT_LOOP_PERIOD);
      outputSpeed = m_tractionControlController.calculate(MAX_LINEAR_SPEED, Units.MetersPerSecond.of(0.0), MAX_LINEAR_SPEED.divide(2));
    }

    // Verify behavior
    assertTrue(m_tractionControlController.isSlipping());
    assertTrue(outputSpeed.lt(MAX_LINEAR_SPEED));
  }

  @Test
  @Order(2)
  @DisplayName("Test if traction control controller can be disabled")
  public void disable() {
    // Disable traction control
    m_tractionControlController.disableTractionControl();

    // Simulate scenario
    var outputSpeed = m_tractionControlController.calculate(MAX_LINEAR_SPEED, Units.MetersPerSecond.of(0.0), MAX_LINEAR_SPEED.divide(2));

    // Verify behavior
    assertFalse(m_tractionControlController.isSlipping());
    assertTrue(outputSpeed.isEquivalent(MAX_LINEAR_SPEED));
  }
}
