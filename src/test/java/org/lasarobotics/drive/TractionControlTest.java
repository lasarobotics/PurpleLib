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

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.Timer;

@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
public class TractionControlTest {
  private final Dimensionless SLIP_RATIO = Units.Percent.of(8.0);
  private final Dimensionless STATIC_FRICTION_COEFFICIENT = Units.Value.of(0.9);
  private final Dimensionless DYNAMIC_FRICTION_COEFFICIENT = Units.Value.of(0.8);
  private final Mass MASS = Units.Pounds.of(135.0);
  private final LinearVelocity MAX_LINEAR_SPEED = Units.MetersPerSecond.of(5.2);
  private final double THRESHOLD = 0.05;

  private TractionControlController m_tractionControlController;

  @BeforeEach
  public void setup() {
    m_tractionControlController = new TractionControlController(STATIC_FRICTION_COEFFICIENT, DYNAMIC_FRICTION_COEFFICIENT, SLIP_RATIO, MASS, MAX_LINEAR_SPEED);
  }

  @Test
  @Order(1)
  @DisplayName("Test if traction control controller detects and limits slip")
  public void limitSlip() {
    // Simulate scenario
    var outputSpeed = Units.MetersPerSecond.of(100.0);
    for (int i = 0; i < GlobalConstants.ROBOT_LOOP_HZ.times(2).in(Units.Hertz); i++) {
      Timer.delay(GlobalConstants.ROBOT_LOOP_HZ.asPeriod().in(Units.Seconds));
      outputSpeed = m_tractionControlController.calculate(MAX_LINEAR_SPEED, Units.MetersPerSecond.of(0.0), MAX_LINEAR_SPEED.div(2));
    }

    // Verify behavior
    assertTrue(m_tractionControlController.isSlipping());
    assertTrue(outputSpeed.lte(MAX_LINEAR_SPEED));
  }

  @Test
  @Order(2)
  @DisplayName("Test if traction control controller allows robot to accelerate")
  public void accelerate() {
    // Simulate scenario
    var outputSpeed = Units.MetersPerSecond.of(0.0);
    var inertialVelocity = Units.MetersPerSecond.of(0.0);
    var wheelSpeed = Units.MetersPerSecond.of(0.0);

    while (inertialVelocity.lt(MAX_LINEAR_SPEED)) {
      Timer.delay(GlobalConstants.ROBOT_LOOP_HZ.asPeriod().in(Units.Seconds));
      outputSpeed = m_tractionControlController.calculate(MAX_LINEAR_SPEED, inertialVelocity, wheelSpeed);

      // Verify behavior
      assertTrue(outputSpeed.gte(inertialVelocity) & outputSpeed.lte(MAX_LINEAR_SPEED));

      // Update values
      inertialVelocity = outputSpeed;
      wheelSpeed = outputSpeed;

      // Exit if test is complete
      if (inertialVelocity.isNear(MAX_LINEAR_SPEED, THRESHOLD)) break;
    }
  }

  @Test
  @Order(3)
  @DisplayName("Test if traction control controller can be disabled")
  public void disable() {
    // Disable traction control
    m_tractionControlController.disableTractionControl();

    // Simulate scenario
    var outputSpeed = m_tractionControlController.calculate(MAX_LINEAR_SPEED, Units.MetersPerSecond.of(0.0), MAX_LINEAR_SPEED.div(2));

    // Verify behavior
    assertFalse(m_tractionControlController.isSlipping());
    assertTrue(outputSpeed.isEquivalent(MAX_LINEAR_SPEED));
  }
}