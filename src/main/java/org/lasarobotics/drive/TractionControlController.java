// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.drive;

import org.lasarobotics.utils.GlobalConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
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
  private final int WHEEL_FILTER_TAPS = 2;
  private final int SLIPPING_DEBOUNCER_MULTIPLIER = 3;

  private double m_filteredWheelSpeed = 0.0;
  private double m_optimalSlipRatio = 0.0;
  private double m_currentSlipRatio = 0.0;
  private double m_maxLinearSpeed = 0.0;
  private boolean m_isSlipping = false;
  private State m_state = State.ENABLED;

  private MedianFilter m_wheelSpeedFilter;
  private Debouncer m_slippingDebouncer;

  /**
   * Create an instance of TractionControlController
   * @param maxLinearSpeed Maximum linear speed of robot
   * @param optimalSlipRatio Desired slip ratio [+0.01, +0.40]
   */
  public TractionControlController(Measure<Velocity<Distance>> maxLinearSpeed, double optimalSlipRatio) {
    this.m_optimalSlipRatio = MathUtil.clamp(optimalSlipRatio, MIN_SLIP_RATIO, MAX_SLIP_RATIO);
    this.m_maxLinearSpeed = Math.floor(maxLinearSpeed.in(Units.MetersPerSecond) * 1000) / 1000;
    this.m_wheelSpeedFilter = new MedianFilter(WHEEL_FILTER_TAPS);
    this.m_slippingDebouncer = new Debouncer(GlobalConstants.ROBOT_LOOP_PERIOD * SLIPPING_DEBOUNCER_MULTIPLIER, DebounceType.kBoth);

    VELOCITY_REQUEST_MIN_THRESHOLD = m_maxLinearSpeed * m_optimalSlipRatio;
  }

  private void updateSlipRatio(double wheelSpeed, double inertialVelocity) {
    // Calculate average speed using median filter
    m_filteredWheelSpeed = m_wheelSpeedFilter.calculate(wheelSpeed);

    // Calculate current slip ratio
    m_currentSlipRatio = ((m_filteredWheelSpeed - inertialVelocity) / inertialVelocity);

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
    m_wheelSpeedFilter.reset();
  }

  /**
   * Enable traction control
   */
  public void enableTractionControl() {
    m_state = State.ENABLED;
    m_wheelSpeedFilter.reset();
  }

  /**
   * Disable traction control
   */
  public void disableTractionControl() {
    m_state = State.DISABLED;
    m_wheelSpeedFilter.reset();
  }

  /**
   * Is traction control enabled
   * @return True if enabled
   */
  public boolean isEnabled() {
    return m_state.equals(State.ENABLED);
  }
}
