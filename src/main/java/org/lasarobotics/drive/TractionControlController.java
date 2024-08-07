// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.drive;

import org.lasarobotics.utils.GlobalConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Mass;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
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

  public static final double VELOCITY_CORRECTION_SCALAR = 0.7;

  private final double MIN_SLIP_RATIO = 0.01;
  private final double MAX_SLIP_RATIO = 0.40;
  private final int SIGMOID_K = 10;
  private final Measure<Velocity<Distance>> INERTIAL_VELOCITY_THRESHOLD = Units.MetersPerSecond.of(0.01);
  private final Measure<Time> MIN_SLIPPING_TIME = Units.Seconds.of(1.1);

  private double m_optimalSlipRatio;
  private double m_mass;
  private double m_maxLinearSpeed;
  private double m_staticCoF;
  private double m_dynamicCoF;
  private double m_maxPredictedSlipRatio;
  private boolean m_isSlipping;
  private Debouncer m_slippingDebouncer;
  private State m_state;

  /**
   * Create an instance of TractionControlController
   * @param driveWheel Wheel that is delivering power to the ground
   * @param optimalSlipRatio Desired slip ratio [1%, 40%]
   * @param mass Mass of robot
   * @param maxLinearSpeed Maximum linear speed of robot
   */
  public TractionControlController(DriveWheel driveWheel, Measure<Dimensionless> optimalSlipRatio, Measure<Mass> mass, Measure<Velocity<Distance>> maxLinearSpeed) {
    this(driveWheel.staticCoF, driveWheel.dynamicCoF, optimalSlipRatio, mass, maxLinearSpeed);
  }

  /**
   * Create an instance of TractionControlController
   * @param staticCoF Static CoF between the wheel and the field surface
   * @param dynamicCoF Dynamic CoF between the wheel and the field surface
   * @param optimalSlipRatio Desired slip ratio [1%, 40%]
   * @param mass Mass of robot
   * @param maxLinearSpeed Maximum linear speed of robot
   */
  public TractionControlController(Measure<Dimensionless> staticCoF,
                                   Measure<Dimensionless> dynamicCoF,
                                   Measure<Dimensionless> optimalSlipRatio,
                                   Measure<Mass> mass,
                                   Measure<Velocity<Distance>> maxLinearSpeed) {
    if (dynamicCoF.gt(staticCoF))
      throw new IllegalArgumentException("Static CoF must be higher than dynamic CoF!");
    this.m_staticCoF = staticCoF.in(Units.Value);
    this.m_dynamicCoF = dynamicCoF.in(Units.Value);
    this.m_optimalSlipRatio = MathUtil.clamp(optimalSlipRatio.in(Units.Value), MIN_SLIP_RATIO, MAX_SLIP_RATIO);
    this.m_mass = mass.divide(4).in(Units.Kilograms);
    this.m_maxLinearSpeed = Math.floor(maxLinearSpeed.in(Units.MetersPerSecond) * 1000) / 1000;
    this.m_maxPredictedSlipRatio = (m_maxLinearSpeed * GlobalConstants.ROBOT_LOOP_HZ)
      / (m_staticCoF * m_mass * GlobalConstants.GRAVITATIONAL_ACCELERATION.in(Units.MetersPerSecondPerSecond));
    this.m_isSlipping = false;
    this.m_slippingDebouncer = new Debouncer(MIN_SLIPPING_TIME.in(Units.Seconds), DebounceType.kRising);
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

    // Passthrough input if disabled
    if (!isEnabled()) return velocityOutput;

    // Get current slip ratio, and check if slipping
    inertialVelocity = Units.MetersPerSecond.of(Math.abs(inertialVelocity.in(Units.MetersPerSecond)));
    double currentSlipRatio = Math.abs(
      inertialVelocity.lte(INERTIAL_VELOCITY_THRESHOLD)
        ? wheelSpeed.in(Units.MetersPerSecond) / m_maxLinearSpeed
        : (Math.abs(wheelSpeed.in(Units.MetersPerSecond)) - inertialVelocity.in(Units.MetersPerSecond)) / inertialVelocity.in(Units.MetersPerSecond)
    );
    m_isSlipping = m_slippingDebouncer.calculate(
      currentSlipRatio > m_optimalSlipRatio
      & Math.abs(wheelSpeed.in(Units.MetersPerSecond)) > m_maxLinearSpeed * m_optimalSlipRatio
      & isEnabled()
    );

    // Get desired acceleration
    var desiredAcceleration = velocityRequest.minus(inertialVelocity).per(Units.Seconds.of(GlobalConstants.ROBOT_LOOP_PERIOD));

    // Get sigmoid value
    double sigmoid = 1 / (1 + Math.exp(-SIGMOID_K * MathUtil.clamp(2 * (currentSlipRatio - m_optimalSlipRatio) - 1, -1.0, +1.0)));

    // Scale CoF based on whether or not robot is currently slipping
    double effectiveCoF =  m_isSlipping ? m_staticCoF * (1 - sigmoid) + m_dynamicCoF * sigmoid : m_staticCoF;

    // Simplified prediction of future slip ratio based on desired acceleration
    double predictedSlipRatio = Math.abs(
      desiredAcceleration.in(Units.MetersPerSecondPerSecond) /
        (inertialVelocity.in(Units.MetersPerSecond) * GlobalConstants.GRAVITATIONAL_ACCELERATION.in(Units.MetersPerSecondPerSecond)
          + effectiveCoF * m_mass * GlobalConstants.GRAVITATIONAL_ACCELERATION.in(Units.MetersPerSecondPerSecond))
    ) / m_maxPredictedSlipRatio;

    // Calculate correction based on difference between optimal and weighted slip ratio, which combines the predicted and current slip ratios
    var velocityCorrection = velocityOutput.times((m_optimalSlipRatio - predictedSlipRatio) * VELOCITY_CORRECTION_SCALAR * m_state.value);

    // Update output, clamping to max linear speed
    velocityOutput = Units.MetersPerSecond.of(MathUtil.clamp(
      velocityOutput.plus(velocityCorrection).in(Units.MetersPerSecond),
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
