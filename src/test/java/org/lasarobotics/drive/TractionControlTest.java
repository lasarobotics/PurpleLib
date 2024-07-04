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

import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Mass;
import edu.wpi.first.units.Mass;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.Timer;

@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
public class TractionControlTest {
  private final Measure<Dimensionless> SLIP_RATIO = Units.Percent.of(8.0);
  private final Measure<Dimensionless> COEFFICIENT_FRICTION = Units.Value.of(1.1);
  private final Measure<Mass> MASS = Units.Pounds.of(110.0);
  private final Measure<Velocity<Distance>> MAX_LINEAR_SPEED = Units.MetersPerSecond.of(4.30);
  private final double THRESHOLD = 0.05;

  private TractionControlController m_tractionControlController;

  @BeforeEach
  public void setup() {
    m_tractionControlController = new TractionControlController(SLIP_RATIO, COEFFICIENT_FRICTION, MASS, MAX_LINEAR_SPEED);
  }

  @Test
  @Order(1)
  @DisplayName("Test if traction control controller detects and limits slip")
  public void limitSlip() {
    // Simulate scenario
    var outputSpeed = Units.MetersPerSecond.of(100.0);
    for (int i = 0; i < 50; i++) {
      Timer.delay(GlobalConstants.ROBOT_LOOP_PERIOD);
      outputSpeed = m_tractionControlController.calculate(MAX_LINEAR_SPEED, Units.MetersPerSecond.of(0.0), MAX_LINEAR_SPEED.divide(2));
    }

    // Verify behavior
    assertTrue(m_tractionControlController.isSlipping());
    assertTrue(outputSpeed.lte(MAX_LINEAR_SPEED.times(SLIP_RATIO.in(Units.Value))));
  }

  @Test
  @Order(2)
  @DisplayName("Test if traction control controller allows robot to accelerate")
  public void accelerate() {
    // Simulate scenario
    var outputSpeed = Units.MetersPerSecond.of(100.0);
    var inertialVelocity = Units.MetersPerSecond.of(0.0);
    while (inertialVelocity.lte(MAX_LINEAR_SPEED)) {
      Timer.delay(GlobalConstants.ROBOT_LOOP_PERIOD);
      outputSpeed = m_tractionControlController.calculate(MAX_LINEAR_SPEED, inertialVelocity, Units.MetersPerSecond.of(0.0));

      // Verify behavior
      assertTrue(outputSpeed.gte(inertialVelocity) & outputSpeed.lt(MAX_LINEAR_SPEED));

      // Update values
      inertialVelocity = outputSpeed;

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
    var outputSpeed = m_tractionControlController.calculate(MAX_LINEAR_SPEED, Units.MetersPerSecond.of(0.0), MAX_LINEAR_SPEED.divide(2));

    // Verify behavior
    assertFalse(m_tractionControlController.isSlipping());
    assertTrue(outputSpeed.isEquivalent(MAX_LINEAR_SPEED));
  }
}
