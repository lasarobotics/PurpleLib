// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.drive;

import java.util.HashMap;

import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import org.lasarobotics.utils.GlobalConstants;
import org.lasarobotics.utils.PIDConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

/** Turn PID controller */
public class TurnPIDController extends PIDController {
  private final double MIN_DEADBAND = 0.001;
  private final double MAX_DEADBAND = 0.2;
  private final double FILTER_FACTOR = 1.0 / 3.0;

  private HashMap<Double, Double> m_turnInputMap = new HashMap<Double, Double>();
  private double m_turnScalar;
  private double m_lookAhead;
  private double m_deadband;
  private double m_turnRequest;
  private boolean m_isTurning;

  /**
   * Create an instance of TurnPIDController
   * @param turnInputCurve Turn input curve
   * @param pidf PID constants
   * @param turnScalar Value to turn input by (degrees)
   * @param deadband Controller deadband
   * @param lookAhead Number of loops to look ahead by
   */
  public TurnPIDController(PolynomialSplineFunction turnInputCurve, PIDConstants pidf, double turnScalar, double deadband, double lookAhead) {
    super(pidf.kP, 0.0, pidf.kD, pidf.period);
    this.m_turnScalar = turnScalar;
    this.m_deadband = MathUtil.clamp(deadband, MIN_DEADBAND, MAX_DEADBAND);
    this.m_lookAhead = lookAhead;
    this.m_isTurning = false;

    // Fill turn input hashmap
    for (int i = 0; i <= 1000; i++) {
      double key = (double)i / 1000;
      double deadbandKey = MathUtil.applyDeadband(key, m_deadband);
      // Evaluate and clamp value between [0.0, +1.0]
      double value = MathUtil.clamp(turnInputCurve.value(deadbandKey), 0.0, +1.0);
      // Add both positive and negative values to map
      m_turnInputMap.put(+key, +value);
      m_turnInputMap.put(-key, -value);
    }
  }

  /**
   * Returns next output of TurnPIDController
   * @param currentAngle current yaw angle of robot (degrees)
   * @param rotateRate current yaw rotate rate of robot (degrees/sec)
   * @param rotateRequest rotate request [-1.0, +1.0]
   *
   * @return optimal turn output [-1.0, +1.0]
   */
  public double calculate(double currentAngle, double rotateRate, double rotateRequest) {
    // Filter turnRequest
    m_turnRequest -= (m_turnRequest - rotateRequest) * FILTER_FACTOR;

    // Start turning if input is greater than deadband
    if (Math.abs(m_turnRequest) >= m_deadband) {
      // Get scaled turnRequest
      m_turnRequest = Math.copySign(Math.floor(Math.abs(m_turnRequest) * 1000) / 1000, m_turnRequest) + 0.0;
      double scaledTurnRequest = m_turnInputMap.get(m_turnRequest);
      // Add delta to setpoint scaled by factor
      super.setSetpoint(currentAngle + (scaledTurnRequest * m_turnScalar));
      m_isTurning = true;
    } else {
      // When turning is complete, set setpoint to current angle
      if (m_isTurning) {
        super.setSetpoint(currentAngle + (rotateRate * m_lookAhead * GlobalConstants.ROBOT_LOOP_PERIOD));
        m_isTurning = false;
      }
    }

    return super.calculate(currentAngle);
  }

  /**
   * Get if robot is turning
   * @return true if turning
   */
  public boolean isTurning() {
    return m_isTurning;
  }
}
