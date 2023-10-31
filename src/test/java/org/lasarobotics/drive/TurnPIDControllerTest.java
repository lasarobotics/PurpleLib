// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;
import org.lasarobotics.utils.GlobalConstants;
import org.lasarobotics.utils.PIDConstants;

@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
public class TurnPIDControllerTest {
  private final double DELTA = 1e-5;

  private static final double CONTROLLER_DEADBAND = 0.10;
  private static final double DRIVE_TURN_INPUT_CURVE_X[] = { 0.0, 0.100, 0.200, 0.300, 0.400, 0.500, 0.600, 0.700, 0.800, 0.900, 1.0 };
  private static final double DRIVE_TURN_INPUT_CURVE_Y[] = { 0.0, 0.008, 0.032, 0.072, 0.128, 0.200, 0.288, 0.392, 0.512, 0.768, 1.0 };
  private static final PolynomialSplineFunction DRIVE_TURN_INPUT_CURVE = new SplineInterpolator().interpolate(DRIVE_TURN_INPUT_CURVE_X, DRIVE_TURN_INPUT_CURVE_Y);
  private static final PIDConstants DRIVE_TURN_PID = new PIDConstants(30.0, 0.0, 0.3, 0.0, GlobalConstants.ROBOT_LOOP_PERIOD);
  private static final double DRIVE_TURN_SCALAR = 30.0;
  private static final double DRIVE_LOOKAHEAD = 3;

  private TurnPIDController m_turnPIDController;

  @BeforeEach
  public void setup() {
    m_turnPIDController = new TurnPIDController(DRIVE_TURN_INPUT_CURVE, DRIVE_TURN_PID, DRIVE_TURN_SCALAR, CONTROLLER_DEADBAND, DRIVE_LOOKAHEAD);
  }

  @Test
  @Order(1)
  @DisplayName("Test if turn PID controller returns zero")
  public void zero() {
    assertEquals(0.0, m_turnPIDController.calculate(0.0, 0.0, 0.0), DELTA);
  }

  @Test
  @Order(2)
  @DisplayName("Test if turn PID controller ignores values within deadband")
  public void deadband() {
    assertEquals(0.0,m_turnPIDController.calculate(0.0, 0.0, 0.09), DELTA);
  }

  @Test
  @Order(3)
  @DisplayName("Test if turn PID controller accepts negative input")
  public void negative() {
    assertTrue(m_turnPIDController.calculate(0.0, 0.0, -1.0) < 0.0);
  }

  @Test
  @Order(4)
  @DisplayName("Test if turn PID controller accepts positive input")
  public void positive() {
    assertTrue(m_turnPIDController.calculate(0.0, 0.0, +1.0) > 0.0);
  }

  @Test
  @Order(5)
  @DisplayName("Test if turn PID controller detects turns")
  public void isTurning() {
    m_turnPIDController.calculate(0.0, 0.0, +1.0);
    assertTrue(m_turnPIDController.isTurning());
  }
}
