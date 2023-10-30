// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;

@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
public class TractionControlTest {
  private final double DELTA = 1e-5;

  private final double DRIVE_SLIP_RATIO = 0.08;
  private final double MAX_LINEAR_SPEED = 4.30;

  private TractionControlController m_tractionControlController;

  @BeforeEach
  public void setup() {
    m_tractionControlController = new TractionControlController(DRIVE_SLIP_RATIO, MAX_LINEAR_SPEED);
  }

  @Test
  @Order(1)
  @DisplayName("Test if traction control controller detects and limits slip")
  public void limitSlip() {
    // Simulate scenario
    double outputSpeed = m_tractionControlController.calculate(MAX_LINEAR_SPEED, 0.0, MAX_LINEAR_SPEED / 2);

    // Verify behavior
    assertTrue(m_tractionControlController.isSlipping());
    assertTrue(outputSpeed < MAX_LINEAR_SPEED);
  }

  @Test
  @Order(2)
  @DisplayName("Test if traction control controller can be disabled")
  public void disable() {
    // Disable traction control
    m_tractionControlController.disableTractionControl();

    // Simulate scenario
    double outputSpeed = m_tractionControlController.calculate(MAX_LINEAR_SPEED, 0.0, MAX_LINEAR_SPEED / 2);

    // Verify behavior
    assertFalse(m_tractionControlController.isSlipping());
    assertEquals(MAX_LINEAR_SPEED, outputSpeed, DELTA);
  }
}
