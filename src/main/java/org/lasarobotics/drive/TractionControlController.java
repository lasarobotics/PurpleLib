// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.drive;

import org.lasarobotics.utils.GlobalConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;

/** Traction control controller */
public class TractionControlController {
  private enum State {
    DISABLED {
      @Override
      public State toggle() { return ENABLED; }
    },
    ENABLED {
      @Override
      public State toggle() { return DISABLED; }
    };

    public abstract State toggle();
  }

  private final double VELOCITY_REQUEST_MIN_THRESHOLD;
  private final double MIN_SLIP_RATIO = 0.01;
  private final double MAX_SLIP_RATIO = 0.40;
  private final double EPSILON = 1e-3;
  private final int WHEEL_FILTER_TIME_CONSTANT_MULTIPLIER = 3;
  private final int INERTIAL_FILTER_TIME_CONSTANT_MULTIPLIER = 6;

  private double m_averageWheelSpeed = 0.0;
  private double m_optimalSlipRatio = 0.0;
  private double m_currentSlipRatio = 0.0;
  private double m_maxLinearSpeed = 0.0;
  private double m_prevVelocityRequest = 0.0;
  private boolean m_isSlipping = false;
  private State m_state = State.ENABLED;

  private LinearFilter m_wheelSpeedFilter;
  private LinearFilter m_inertialVelocityFilter;

  /**
   * Create an instance of TractionControlController
   * @param maxLinearSpeed Maximum linear speed of robot
   * @param optimalSlipRatio Desired slip ratio [+0.01, +0.40]
   */
  public TractionControlController(Measure<Velocity<Distance>> maxLinearSpeed, double optimalSlipRatio) {
    this.m_optimalSlipRatio = MathUtil.clamp(optimalSlipRatio, MIN_SLIP_RATIO, MAX_SLIP_RATIO);
    this.m_maxLinearSpeed = Math.floor(maxLinearSpeed.in(Units.MetersPerSecond) * 1000) / 1000;
    this.m_wheelSpeedFilter = LinearFilter.singlePoleIIR(
      GlobalConstants.ROBOT_LOOP_PERIOD * WHEEL_FILTER_TIME_CONSTANT_MULTIPLIER,
      GlobalConstants.ROBOT_LOOP_PERIOD
    );
    this.m_inertialVelocityFilter = LinearFilter.singlePoleIIR(
      GlobalConstants.ROBOT_LOOP_PERIOD * INERTIAL_FILTER_TIME_CONSTANT_MULTIPLIER,
      GlobalConstants.ROBOT_LOOP_PERIOD
    );

    VELOCITY_REQUEST_MIN_THRESHOLD = m_maxLinearSpeed * m_optimalSlipRatio;
  }

  private void updateSlipRatio(double wheelSpeed, double inertialVelocity) {
    // Calculate average speed using single pole IIR filter
    m_averageWheelSpeed = m_wheelSpeedFilter.calculate(wheelSpeed);

    // Calculate current slip ratio
    m_currentSlipRatio = ((m_averageWheelSpeed - inertialVelocity) / inertialVelocity);

    // Check if wheel is slipping, false if disabled
    m_isSlipping = m_currentSlipRatio > m_optimalSlipRatio & isEnabled();
  }

  /**
   * Returns the next output of the traction control controller
   * @param velocityRequest Velocity request (m/s)
   * @param inertialVelocity Current inertial velocity (m/s)
   * @param wheelSpeed Linear wheel speed (m/s)
   * @return Optimal motor speed output (m/s)
   */
  public Measure<Velocity<Distance>> calculate(Measure<Velocity<Distance>> velocityRequest,
                                               Measure<Velocity<Distance>> inertialVelocity,
                                               Measure<Velocity<Distance>> wheelSpeed) {
    double velocityRequestMetersPerSecond = velocityRequest.in(Units.MetersPerSecond);
    double inertialVelocityMetersPerSecond = inertialVelocity.in(Units.MetersPerSecond);
    double wheelSpeedMetersPerSecond = wheelSpeed.in(Units.MetersPerSecond);

    // Filter inertial velocity
    inertialVelocityMetersPerSecond = m_inertialVelocityFilter.calculate(inertialVelocityMetersPerSecond);

    // If velocity request has changed or is near zero, reset speed filter
    if (Math.abs(velocityRequestMetersPerSecond) < EPSILON || Math.abs(m_prevVelocityRequest - velocityRequestMetersPerSecond) > EPSILON)
      m_wheelSpeedFilter.reset();

    // Initialize velocity output to requested velocity
    double velocityOutput = velocityRequestMetersPerSecond;

    // If input is below treshold, return
    if (velocityRequestMetersPerSecond < VELOCITY_REQUEST_MIN_THRESHOLD) return Units.MetersPerSecond.of(velocityOutput);

    // Make sure wheel speed and inertial velocity are positive
    wheelSpeedMetersPerSecond = Math.abs(wheelSpeedMetersPerSecond);
    inertialVelocityMetersPerSecond = Math.abs(inertialVelocityMetersPerSecond);

    // Apply basic traction control
    // Limit wheel speed if slipping excessively
    updateSlipRatio(wheelSpeedMetersPerSecond, inertialVelocityMetersPerSecond);
    if (isSlipping())
      velocityOutput = Math.copySign(inertialVelocityMetersPerSecond * m_optimalSlipRatio + inertialVelocityMetersPerSecond, velocityRequestMetersPerSecond);

    // Save velocity request
    m_prevVelocityRequest = velocityRequestMetersPerSecond;

    // Return corrected velocity output, clamping to max linear speed
    velocityOutput = MathUtil.clamp(velocityOutput, -m_maxLinearSpeed, +m_maxLinearSpeed);
    return Units.MetersPerSecond.of(velocityOutput);
  }

  /**
   * Is wheel slipping
   * @return True if wheel is slipping
   */
  public boolean isSlipping() {
    return m_isSlipping;
  }

  /**
   * Toggle traction control
   */
  public void toggleTractionControl() {
    m_state = m_state.toggle();
    m_wheelSpeedFilter.reset();
    m_inertialVelocityFilter.reset();
  }

  /**
   * Enable traction control
   */
  public void enableTractionControl() {
    m_state = State.ENABLED;
    m_wheelSpeedFilter.reset();
    m_inertialVelocityFilter.reset();
  }

  /**
   * Disable traction control
   */
  public void disableTractionControl() {
    m_state = State.DISABLED;
    m_wheelSpeedFilter.reset();
    m_inertialVelocityFilter.reset();
  }

  /**
   * Is traction control enabled
   * @return True if enabled
   */
  public boolean isEnabled() {
    return m_state.equals(State.ENABLED);
  }
}
