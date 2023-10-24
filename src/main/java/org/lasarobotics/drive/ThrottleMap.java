// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.drive;

import java.util.HashMap;

import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

import edu.wpi.first.math.MathUtil;

public class ThrottleMap {
  private final double MIN_DEADBAND = 0.001;
  private final double MAX_DEADBAND = 0.2;

  private double m_deadband = 0.0;

  private HashMap<Double, Double> m_throttleInputMap = new HashMap<Double, Double>();
  
  /**
   * Create an instance of ThrottleMap
   * @param throttleInputCurve Spline function characterising throttle input curve
   * @param deadband Deadband for controller input [+0.001, +0.2]
   * @param maxLinearSpeed maximum linear speed of robot (m/s)
   */
  public ThrottleMap(PolynomialSplineFunction throttleInputCurve, double deadband, double maxLinearSpeed) {
    this.m_deadband = MathUtil.clamp(deadband, MIN_DEADBAND, MAX_DEADBAND);

    // Fill throttle input hashmap
    for (int i = 0; i <= 1000; i++) {
      double key = (double)i / 1000;
      double deadbandKey = MathUtil.applyDeadband(key, m_deadband);
      // Evaluate value between [0.0, +MAX_LINEAR_SPEED]
      double value = MathUtil.clamp(throttleInputCurve.value(deadbandKey), 0.0, +maxLinearSpeed);
      // Add both positive and negative values to map
      m_throttleInputMap.put(+key, +value);
      m_throttleInputMap.put(-key, -value);
    }
  }

  /**
   * Lookup velocity given throttle input
   * @param throttleLookup Throttle input [-1.0, +1.0]
   * @return Corresponding velocity
   */
  public double throttleLookup(double throttleLookup) {
    throttleLookup = Math.copySign(Math.floor(Math.abs(throttleLookup) * 1000) / 1000, throttleLookup) + 0.0;
    throttleLookup = MathUtil.clamp(throttleLookup, -1.0, +1.0);
    
    return m_throttleInputMap.get(throttleLookup);
  }
}
