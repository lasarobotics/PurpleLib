// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.drive;

import java.util.HashMap;

import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import org.lasarobotics.utils.PIDConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Time;

/** Rotate PID controller */
public class RotatePIDController {
  private final double MIN_DEADBAND = 0.001;
  private final double MAX_DEADBAND = 0.2;
  private final double FILTER_FACTOR = 1.0 / 3.0;
  private final Time MAX_LOOKAHEAD = Units.Seconds.of(2);

  private HashMap<Double, Double> m_rotateInputMap = new HashMap<Double, Double>();
  private PIDController m_pidController;
  private Angle m_rotateScalar;
  private Time m_lookAhead;
  private double m_deadband;
  private double m_rotateRequest;
  private boolean m_isRotating;

  /**
   * Create an instance of RotatePIDController
   * @param rotateInputCurve Rotate input curve
   * @param pidf PID constants
   * @param rotateScalar Value to scale rotate input by
   * @param deadband Controller deadband
   * @param lookAhead Number of loops to look ahead by
   */
  public RotatePIDController(PolynomialSplineFunction rotateInputCurve,
                             PIDConstants pidf,
                             Angle rotateScalar,
                             Dimensionless deadband,
                             Time lookAhead) {
    this.m_pidController = new PIDController(pidf.kP, 0.0, pidf.kD, pidf.period.in(Units.Seconds));
    this.m_rotateScalar = rotateScalar;
    this.m_deadband = MathUtil.clamp(deadband.in(Units.Percent), MIN_DEADBAND, MAX_DEADBAND);
    this.m_lookAhead = Units.Seconds.of(MathUtil.clamp(lookAhead.in(Units.Seconds), 0.0, MAX_LOOKAHEAD.in(Units.Seconds)));
    this.m_isRotating = false;

    // Enable PID position wrapping
    m_pidController.enableContinuousInput(0.0, 360.0);

    // Fill turn input hashmap
    for (int i = 0; i <= 1000; i++) {
      double key = (double)i / 1000;
      double deadbandKey = MathUtil.applyDeadband(key, m_deadband);
      // Evaluate and clamp value between [0.0, +1.0]
      double value = MathUtil.clamp(rotateInputCurve.value(deadbandKey), 0.0, +1.0);
      // Add both positive and negative values to map
      m_rotateInputMap.put(+key, +value);
      m_rotateInputMap.put(-key, -value);
    }
  }

  /**
   * Returns next output of rotation PID controller
   * @param currentAngle current yaw angle of robot
   * @param rotateRate current yaw rotate rate of robot
   * @param rotateRequest rotate request [-1.0, +1.0]
   *
   * @return Optimal rotate output
   */
  public AngularVelocity calculate(Angle currentAngle, AngularVelocity rotateRate, double rotateRequest) {
    // Filter rotate request
    m_rotateRequest -= (m_rotateRequest - rotateRequest) * FILTER_FACTOR;

    // Start turning if input is greater than deadband
    if (Math.abs(m_rotateRequest) >= m_deadband) {
      // Get scaled rotate request
      m_rotateRequest = Math.copySign(Math.floor(Math.abs(m_rotateRequest) * 1000) / 1000, m_rotateRequest) + 0.0;
      double scaledTurnRequest = m_rotateInputMap.get(m_rotateRequest);
      // Add delta to setpoint scaled by factor
      m_pidController.setSetpoint(currentAngle.in(Units.Degrees) + m_rotateScalar.times(scaledTurnRequest).in(Units.Degrees));
      m_isRotating = true;
    } else {
      // When rotation is complete, set setpoint to current angle
      if (m_isRotating) {
        m_pidController.setSetpoint(currentAngle.in(Units.Degrees) + (rotateRate.in(Units.DegreesPerSecond) * m_lookAhead.in(Units.Seconds)));
        m_isRotating = false;
      }
    }

    return Units.DegreesPerSecond.of(m_pidController.calculate(currentAngle.in(Units.Degrees)));
  }

  public AngularVelocity calculate(Angle currentAngle, Angle targetAngle) {
    return Units.DegreesPerSecond.of(m_pidController.calculate(currentAngle.in(Units.Degrees), targetAngle.in(Units.Degrees)));
  }

  /**
   * Set the tolerable error
   * @param positionTolerance Position error that is tolerable
   * @param velocityTolerance Velocity error that is tolerable
   */
  public void setTolerance(Angle positionTolerance, AngularVelocity velocityTolerance) {
    m_pidController.setTolerance(positionTolerance.in(Units.Degrees), velocityTolerance.in(Units.DegreesPerSecond));
  }

  /**
   * Set setpoint of rotation PID controller
   * @param angle
   */
  public void setSetpoint(Angle angle) {
    m_pidController.setSetpoint(angle.in(Units.Degrees));
  }

  /**
   * Reset rotation PID controller
   */
  public void reset() {
    m_pidController.reset();
  }

  /**
   * Get if robot is rotating
   * @return True if rotating
   */
  public boolean isRotating() {
    return m_isRotating;
  }
}
