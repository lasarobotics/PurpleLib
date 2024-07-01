// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.drive;

import org.lasarobotics.utils.GlobalConstants;

import org.lasarobotics.utils.GlobalConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Mass;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;

/** Traction control controller */
public class TractionControlController {
  private enum State {
    DISABLED(0) {
      @Override
      public State toggle() { return ENABLED; }
    },
    ENABLED(1) {
      @Override
      public State toggle() { return DISABLED; }
    };

    public int value;
    private State(int value) {
      this.value = value;
    }

    public abstract State toggle();
  }

  private final double MIN_SLIP_RATIO = 0.01;
  private final double MAX_SLIP_RATIO = 0.40;

  private double m_optimalSlipRatio;
  private double m_mass;
  private double m_maxLinearSpeed;
  private double m_maxVelocityCorrection;
  private double m_frictionCoefficient;
  private boolean m_isSlipping;
  private State m_state;

  /**
   * Create an instance of TractionControlController
   * @param optimalSlipRatio Desired slip ratio [1%, 40%]
   * @param frictionCoefficient CoF between the wheel and the field surface
   * @param mass Mass of robot
   * @param maxLinearSpeed Maximum linear speed of robot
   */
  public TractionControlController(Measure<Dimensionless> optimalSlipRatio, Measure<Dimensionless> frictionCoefficient, Measure<Mass> mass, Measure<Velocity<Distance>> maxLinearSpeed) {
    this.m_optimalSlipRatio = MathUtil.clamp(optimalSlipRatio.in(Units.Value), MIN_SLIP_RATIO, MAX_SLIP_RATIO);
    this.m_mass = mass.divide(4).in(Units.Kilograms);
    this.m_maxLinearSpeed = Math.floor(maxLinearSpeed.in(Units.MetersPerSecond) * 1000) / 1000;
    this.m_maxVelocityCorrection = m_maxLinearSpeed * (1.0 - m_optimalSlipRatio);
    this.m_frictionCoefficient = frictionCoefficient.in(Units.Value);
    this.m_isSlipping = false;
    this.m_state = State.ENABLED;
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
    var velocityOutput = velocityRequest;

    // Get current slip ratio, and check if slipping
    inertialVelocity = Units.MetersPerSecond.of(Math.abs(inertialVelocity.in(Units.MetersPerSecond)));
    double currentSlipRatio =  (Math.abs(wheelSpeed.in(Units.MetersPerSecond)) - inertialVelocity.in(Units.MetersPerSecond)) / inertialVelocity.in(Units.MetersPerSecond);
    m_isSlipping = currentSlipRatio > m_optimalSlipRatio & isEnabled();

    // Get desired acceleration
    var desiredAcceleration = velocityRequest.minus(inertialVelocity).per(Units.Seconds.of(GlobalConstants.ROBOT_LOOP_PERIOD));

    // Simplified prediction of future slip ratio based on desired acceleration
    double predictedSlipRatio = Math.abs(
      desiredAcceleration.in(Units.MetersPerSecondPerSecond) /
        (inertialVelocity.in(Units.MetersPerSecond) *
          GlobalConstants.GRAVITATIONAL_ACCELERATION.in(Units.MetersPerSecondPerSecond) + m_frictionCoefficient * m_mass * GlobalConstants.GRAVITATIONAL_ACCELERATION.in(Units.MetersPerSecondPerSecond))
    );

    // Calculate correction based on difference between optimal and predicted slip ratio
    double velocityCorrection = MathUtil.clamp(
      velocityOutput.in(Units.MetersPerSecond) * (m_optimalSlipRatio - predictedSlipRatio) * m_state.value,
      -m_maxVelocityCorrection,
      +m_maxVelocityCorrection
    );

    // Update output, clamping to max linear speed
    velocityOutput = Units.MetersPerSecond.of(MathUtil.clamp(
      velocityOutput.plus(Units.MetersPerSecond.of(velocityCorrection)).in(Units.MetersPerSecond),
      -m_maxLinearSpeed,
      +m_maxLinearSpeed
    ));

    return velocityOutput;
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
