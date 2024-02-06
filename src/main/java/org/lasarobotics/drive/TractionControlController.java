// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
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

  private double m_optimalSlipRatio = 0.0;
  private double m_currentSlipRatio = 0.0;
  private double m_maxLinearSpeed = 0.0;
  private boolean m_isSlipping = false;
  private State m_state = State.ENABLED;

  private Debouncer m_slippingDebouncer;

  /**
   * Create an instance of TractionControlController
   * @param maxLinearSpeed Maximum linear speed of robot
   * @param maxSlippingTime Maximum time that wheel is allowed to slip
   * @param optimalSlipRatio Desired slip ratio [+0.01, +0.40]
   */
  public TractionControlController(Measure<Velocity<Distance>> maxLinearSpeed, Measure<Time> maxSlippingTime, double optimalSlipRatio) {
    this.m_optimalSlipRatio = MathUtil.clamp(optimalSlipRatio, MIN_SLIP_RATIO, MAX_SLIP_RATIO);
    this.m_maxLinearSpeed = Math.floor(maxLinearSpeed.in(Units.MetersPerSecond) * 1000) / 1000;
    this.m_slippingDebouncer = new Debouncer(maxSlippingTime.in(Units.Seconds), DebounceType.kRising);

    VELOCITY_REQUEST_MIN_THRESHOLD = m_maxLinearSpeed * m_optimalSlipRatio;
  }

  /**
   * Update slip ratio, and check if wheel is slipping
   * @param wheelSpeed Current wheel speed
   * @param inertialVelocity Inertial velocity
   */
  private void updateSlipRatio(double wheelSpeed, double inertialVelocity) {
    // Calculate current slip ratio
    m_currentSlipRatio = ((wheelSpeed - inertialVelocity) / inertialVelocity);

    // Check if wheel is slipping, false if disabled
    m_isSlipping = m_slippingDebouncer.calculate(m_currentSlipRatio > m_optimalSlipRatio) & isEnabled();
  }

  /**
   * Returns the next output of the traction control controller
   * @param velocityRequest Velocity request
   * @param inertialVelocity Current inertial velocity
   * @param wheelSpeed Linear wheel speed
   * @return Optimal motor speed output
   */
  public Measure<Velocity<Distance>> calculate(Measure<Velocity<Distance>> velocityRequest,
                                               Measure<Velocity<Distance>> inertialVelocity,
                                               Measure<Velocity<Distance>> wheelSpeed) {
    double velocityRequestMetersPerSecond = velocityRequest.in(Units.MetersPerSecond);
    double inertialVelocityMetersPerSecond = inertialVelocity.in(Units.MetersPerSecond);
    double wheelSpeedMetersPerSecond = wheelSpeed.in(Units.MetersPerSecond);

    // Initialize velocity output to requested velocity
    double velocityOutputMetersPerSecond = velocityRequestMetersPerSecond;

    // Make sure wheel speed and inertial velocity are positive
    wheelSpeedMetersPerSecond = Math.abs(wheelSpeedMetersPerSecond);
    inertialVelocityMetersPerSecond = Math.abs(inertialVelocityMetersPerSecond);

    // Update slip ratio given current wheel speed and inertial velocity
    updateSlipRatio(wheelSpeedMetersPerSecond, inertialVelocityMetersPerSecond);

    // If input is below threshold, return
    if (Math.abs(velocityRequestMetersPerSecond) < VELOCITY_REQUEST_MIN_THRESHOLD) return Units.MetersPerSecond.of(velocityOutputMetersPerSecond);

    // Limit wheel speed if slipping excessively
    if (isSlipping())
      velocityOutputMetersPerSecond = Math.copySign(inertialVelocityMetersPerSecond * m_optimalSlipRatio + inertialVelocityMetersPerSecond, velocityRequestMetersPerSecond);

    // Return corrected velocity output, clamping to max linear speed
    velocityOutputMetersPerSecond = MathUtil.clamp(velocityOutputMetersPerSecond, -m_maxLinearSpeed, +m_maxLinearSpeed);
    return Units.MetersPerSecond.of(velocityOutputMetersPerSecond);
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
  }

  /**
   * Enable traction control
   */
  public void enableTractionControl() {
    m_state = State.ENABLED;
  }

  /**
   * Disable traction control
   */
  public void disableTractionControl() {
    m_state = State.DISABLED;
  }

  /**
   * Is traction control enabled
   * @return True if enabled
   */
  public boolean isEnabled() {
    return m_state.equals(State.ENABLED);
  }
}
