// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.drive;

import org.lasarobotics.utils.GlobalConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;

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

  private final double MIN_SLIP_RATIO = 0.01;
  private final double MAX_SLIP_RATIO = 0.40;
  private final double EPSILON = 1e-3;
  private final int FILTER_TIME_CONSTANT_MULTIPLIER = 10;
  private final int DEBOUNCER_TIME_CONSTANT_MULTIPLIER = 5;

  private double m_averageWheelSpeed = 0.0;
  private double m_optimalSlipRatio = 0.0;
  private double m_currentSlipRatio = 0.0;
  private double m_maxLinearSpeed = 0.0;
  private double m_prevVelocityRequest = 0.0;
  private boolean m_isSlipping = false;
  private State m_state = State.ENABLED;

  private LinearFilter m_speedFilter;
  private Debouncer m_slippingDebouncer;

  /**
   * Create an instance of TractionControlController
   * @param optimalSlipRatio Desired slip ratio [+0.01, +0.40]
   * @param maxLinearSpeed maximum linear speed of robot (m/s)
   */
  public TractionControlController(double optimalSlipRatio, double maxLinearSpeed) {
    this.m_optimalSlipRatio = MathUtil.clamp(optimalSlipRatio, MIN_SLIP_RATIO, MAX_SLIP_RATIO);
    this.m_maxLinearSpeed = Math.floor(maxLinearSpeed * 1000) / 1000;
    this.m_speedFilter = LinearFilter.singlePoleIIR(
      GlobalConstants.ROBOT_LOOP_PERIOD * FILTER_TIME_CONSTANT_MULTIPLIER,
      GlobalConstants.ROBOT_LOOP_PERIOD
    );
    this.m_slippingDebouncer = new Debouncer(
      GlobalConstants.ROBOT_LOOP_PERIOD * DEBOUNCER_TIME_CONSTANT_MULTIPLIER,
      Debouncer.DebounceType.kFalling
    );
  }

  private void updateSlipRatio(double wheelSpeed, double inertialVelocity) {
    // Calculate average speed using single pole IIR filter
    m_averageWheelSpeed = m_speedFilter.calculate(wheelSpeed);

    // Calculate current slip ratio
    m_currentSlipRatio = ((m_averageWheelSpeed - inertialVelocity) / inertialVelocity);

    // Check if wheel is slipping, false if disabled
    m_isSlipping = m_slippingDebouncer.calculate(m_currentSlipRatio > m_optimalSlipRatio) & isEnabled();
  }

  /**
   * Returns the next output of the traction control controller
   * @param velocityRequest Velocity request (m/s)
   * @param inertialVelocity Current inertial velocity (m/s)
   * @param wheelSpeed Linear wheel speed (m/s)
   * @return Optimal motor speed output (m/s)
   */
  public double calculate(double velocityRequest, double inertialVelocity, double wheelSpeed) {
    // If velocity request has changed or is near zero, reset speed filter
    if (Math.abs(velocityRequest) < EPSILON || Math.abs(m_prevVelocityRequest - velocityRequest) > EPSILON)
      m_speedFilter.reset();

    // Initialize velocity output to requested velocity
    double velocityOutput = velocityRequest;

    // Make sure wheel speed and inertial velocity are positive
    wheelSpeed = Math.abs(wheelSpeed);
    inertialVelocity = Math.abs(inertialVelocity);
    
    // Apply basic traction control
    // Limit wheel speed if slipping excessively
    updateSlipRatio(wheelSpeed, inertialVelocity);
    if (isSlipping())
      velocityOutput = Math.copySign(m_optimalSlipRatio * inertialVelocity + m_averageWheelSpeed, velocityRequest);

    // Save velocity request
    m_prevVelocityRequest = velocityRequest;

    // Return corrected velocity output, clamping to max linear speed
    return MathUtil.clamp(velocityOutput, -m_maxLinearSpeed, +m_maxLinearSpeed);
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
    m_speedFilter.reset();
  }

  /**
   * Enable traction control
   */
  public void enableTractionControl() {
    m_state = State.ENABLED;
    m_speedFilter.reset();
  }

  /**
   * Disable traction control
   */
  public void disableTractionControl() {
    m_state = State.DISABLED;
    m_speedFilter.reset();
  }

  /**
   * Is traction control enabled
   * @return True if enabled
   */
  public boolean isEnabled() {
    return m_state.equals(State.ENABLED);
  }
}
