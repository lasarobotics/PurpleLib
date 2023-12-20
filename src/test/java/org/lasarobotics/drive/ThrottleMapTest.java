// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;

@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
public class ThrottleMapTest {
  private final double DELTA = 1e-5;
  private final double CONTROLLER_DEADBAND = 0.10;
  private final double ACCELERATION_TIME = 0.1;
  private final double[] DRIVE_THROTTLE_INPUT_CURVE_X = { 0.0, 0.100, 0.200, 0.300, 0.400, 0.500, 0.600, 0.700, 0.800, 0.900, 1.000 };
  private final double[] DRIVE_THROTTLE_INPUT_CURVE_Y = { 0.0, 0.200, 0.400, 0.600, 0.800, 1.000, 1.200, 1.400, 1.600, 1.800, 2.000 };
  private final SplineInterpolator SPLINE_INTERPOLATOR = new SplineInterpolator();
  private final PolynomialSplineFunction DRIVE_THROTTLE_INPUT_CURVE = SPLINE_INTERPOLATOR.interpolate(DRIVE_THROTTLE_INPUT_CURVE_X, DRIVE_THROTTLE_INPUT_CURVE_Y);

  private ThrottleMap m_throttleMap;

  @BeforeEach
  public void setup() {
    m_throttleMap = new ThrottleMap(
      DRIVE_THROTTLE_INPUT_CURVE,
      CONTROLLER_DEADBAND,
      ACCELERATION_TIME,
      DRIVE_THROTTLE_INPUT_CURVE_Y[DRIVE_THROTTLE_INPUT_CURVE_Y.length - 1]
    );
  }

  @AfterEach
  public void close() {
    m_throttleMap = null;
  }

  @Test
  @Order(1)
  @DisplayName("Test if throttle map returns zero")
  public void returnZero() {
    assertEquals(0.0, m_throttleMap.throttleLookup(0.0), DELTA);
  }

  @Test
  @Order(2)
  @DisplayName("Test if throttle map ignores values within deadband")
  public void deadband() {
    assertEquals(0.0, m_throttleMap.throttleLookup(0.0), DELTA);
  }

  @Test
  @Order(3)
  @DisplayName("Test if throttle map can handle negative input")
  public void negative() {
    assertTrue(m_throttleMap.throttleLookup(-0.5) < 0.0);
  }

  @Test
  @Order(4)
  @DisplayName("Test if throttle map can handle positive input")
  public void positive() {
    assertTrue(m_throttleMap.throttleLookup(+0.5) > 0.0);
  }

  @Test
  @Order(5)
  @DisplayName("Test if throttle map can handle negative illegal input")
  public void illegalNegative() {
    assertTrue(m_throttleMap.throttleLookup(-1.5) < 0.0);
  }

  @Test
  @Order(6)
  @DisplayName("Test if throttle map can handle positive illegal input")
  public void illegalPositive() {
    assertTrue(m_throttleMap.throttleLookup(+1.5) > 0.0);
  }
}
