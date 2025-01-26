// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.drive;

import org.lasarobotics.drive.swerve.DriveWheel;
import org.lasarobotics.utils.GlobalConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;

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
  private final int SIGMOID_K = 10;
  private final double FORCE_ACCELERATION_MULTIPLIER = 0.7;
  private final LinearVelocity INERTIAL_VELOCITY_THRESHOLD = Units.MetersPerSecond.of(0.01);
  private final LinearVelocity VELOCITY_DIFFERENCE_THRESHOLD = Units.MetersPerSecond.of(1.0);
  private final Time MIN_SLIPPING_TIME = Units.Seconds.of(0.5);
  private final LinearVelocity VELOCITY_REQUEST_THRESHOLD = Units.MetersPerSecond.of(0.05);
  private final LinearVelocity WHEEL_SPEED_THRESHOLD = Units.MetersPerSecond.of(0.05);
  private final Time STATIC_FORCE_ACCELERATE_TRIGGER_TIME = Units.Seconds.of(0.1);
  private final Time DYNAMIC_FORCE_ACCELERATE_TRIGGER_TIME = Units.Seconds.of(0.2);
  private final Time FORCE_ACCELERATE_TIME = Units.Seconds.of(2.0);

  private double m_optimalSlipRatio;
  private double m_mass;
  private double m_maxLinearVelocity;
  private double m_maxAcceleration;
  private double m_staticCoF;
  private double m_dynamicCoF;
  private double m_maxPredictedSlipRatio;
  private boolean m_isSlipping;
  private Debouncer m_slippingDebouncer;
  private Debouncer m_staticForceAccelerationDebouncer;
  private Debouncer m_dynamicForceAccelerationDebouncer;
  private State m_state;
  private Timer m_forceAccelerateTimer;

  /**
   * Create an instance of TractionControlController
   * @param driveWheel Wheel that is delivering power to the ground
   * @param optimalSlipRatio Desired slip ratio [1%, 40%]
   * @param mass Mass of robot
   * @param maxLinearSpeed Maximum linear speed of robot
   */
  public TractionControlController(DriveWheel driveWheel, Dimensionless optimalSlipRatio, Mass mass, LinearVelocity maxLinearSpeed) {
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
  public TractionControlController(Dimensionless staticCoF,
                                   Dimensionless dynamicCoF,
                                   Dimensionless optimalSlipRatio,
                                   Mass mass,
                                   LinearVelocity maxLinearSpeed) {
    if (dynamicCoF.gt(staticCoF))
      throw new IllegalArgumentException("Static CoF must be higher than dynamic CoF!");
    this.m_staticCoF = staticCoF.in(Units.Value);
    this.m_dynamicCoF = dynamicCoF.in(Units.Value);
    this.m_optimalSlipRatio = MathUtil.clamp(optimalSlipRatio.in(Units.Value), MIN_SLIP_RATIO, MAX_SLIP_RATIO);
    this.m_mass = mass.div(4).in(Units.Kilograms);
    this.m_maxLinearVelocity = Math.floor(maxLinearSpeed.in(Units.MetersPerSecond) * 1000) / 1000;
    this.m_maxAcceleration = m_staticCoF * Units.Gs.one().in(Units.MetersPerSecondPerSecond);
    this.m_maxPredictedSlipRatio = (m_maxAcceleration * GlobalConstants.ROBOT_LOOP_HZ.in(Units.Hertz))
      / (m_staticCoF * m_mass * Units.Gs.one().in(Units.MetersPerSecondPerSecond));
    this.m_isSlipping = false;
    this.m_slippingDebouncer = new Debouncer(MIN_SLIPPING_TIME.in(Units.Seconds), DebounceType.kRising);
    this.m_state = State.ENABLED;
    this.m_staticForceAccelerationDebouncer = new Debouncer(STATIC_FORCE_ACCELERATE_TRIGGER_TIME.in(Units.Seconds), DebounceType.kRising);
    this.m_dynamicForceAccelerationDebouncer = new Debouncer(DYNAMIC_FORCE_ACCELERATE_TRIGGER_TIME.in(Units.Seconds), DebounceType.kRising);
    this.m_forceAccelerateTimer = new Timer();

    m_forceAccelerateTimer.reset();
    m_forceAccelerateTimer.start();
  }

  /**
   * Returns the next output of the traction control controller
   * @param velocityRequest Velocity request
   * @param inertialVelocity Current inertial velocity (Negative values indicate inertial velocity is in opposite direction to request)
   * @param wheelSpeed Linear wheel speed
   * @return Optimal motor speed output
   */
  public LinearVelocity calculate(LinearVelocity velocityRequest,
                                  LinearVelocity inertialVelocity,
                                  LinearVelocity wheelSpeed) {
    var velocityOutput = velocityRequest;
    boolean oppositeDirection = inertialVelocity.lt(Units.MetersPerSecond.zero());

    // Make wheel speed and inertial velocity positive
    wheelSpeed = Units.MetersPerSecond.of(Math.abs(wheelSpeed.in(Units.MetersPerSecond)));
    inertialVelocity = Units.MetersPerSecond.of(Math.abs(inertialVelocity.in(Units.MetersPerSecond)));

    // See if user has been trying to accelerate for a while...
    boolean slowWheel = Units.MetersPerSecond.of(Math.abs(velocityRequest.in(Units.MetersPerSecond))).minus(wheelSpeed)
      .gt(VELOCITY_DIFFERENCE_THRESHOLD);
    if (slowWheel) m_forceAccelerateTimer.start();
    else {
      m_forceAccelerateTimer.reset();
      m_forceAccelerateTimer.stop();
    }
    boolean forceAcceleration = m_staticForceAccelerationDebouncer.calculate(
      velocityRequest.gte(VELOCITY_REQUEST_THRESHOLD) && wheelSpeed.lte(WHEEL_SPEED_THRESHOLD)
    ) || m_dynamicForceAccelerationDebouncer.calculate(
      slowWheel && !m_forceAccelerateTimer.advanceIfElapsed(FORCE_ACCELERATE_TIME.in(Units.Seconds)) && !oppositeDirection
    );

    // Get current slip ratio, and check if slipping
    double currentSlipRatio = Math.abs(
      inertialVelocity.lte(INERTIAL_VELOCITY_THRESHOLD)
        ? wheelSpeed.in(Units.MetersPerSecond) / m_maxLinearVelocity
        : (Math.abs(wheelSpeed.in(Units.MetersPerSecond)) - inertialVelocity.in(Units.MetersPerSecond)) / inertialVelocity.in(Units.MetersPerSecond)
    );
    m_isSlipping = m_slippingDebouncer.calculate(
      currentSlipRatio > m_optimalSlipRatio
      & Math.abs(wheelSpeed.in(Units.MetersPerSecond)) > m_maxLinearVelocity * m_optimalSlipRatio
      & isEnabled()
    );

    // Get desired acceleration
    var desiredAcceleration = velocityRequest.minus(inertialVelocity.times(oppositeDirection ? -1 : +1))
      .div(GlobalConstants.ROBOT_LOOP_HZ.asPeriod());

    // Get sigmoid value
    double sigmoid = 1 / (1 + Math.exp(-SIGMOID_K * MathUtil.clamp(2 * (currentSlipRatio - m_optimalSlipRatio) - 1, -1.0, +1.0)));

    // Scale CoF based on whether or not robot is currently slipping
    double effectiveCoF = m_isSlipping ? m_staticCoF * (1 - sigmoid) + m_dynamicCoF * sigmoid : m_staticCoF;

    // Simplified prediction of future slip ratio based on desired acceleration
    double predictedSlipRatio = Math.abs(
      desiredAcceleration.in(Units.MetersPerSecondPerSecond) /
        (inertialVelocity.in(Units.MetersPerSecond) * Units.Gs.one().in(Units.MetersPerSecondPerSecond)
          + effectiveCoF * m_mass * Units.Gs.one().in(Units.MetersPerSecondPerSecond))
    ) / m_maxPredictedSlipRatio;

    // Calculate correction based on difference between optimal and predicted slip ratio
    var velocityCorrection = velocityOutput.times((m_optimalSlipRatio - predictedSlipRatio) * m_state.value);

    // Reduce velocity correction if trying to force acceleration
    if (forceAcceleration) velocityCorrection = velocityCorrection.times(FORCE_ACCELERATION_MULTIPLIER);

    // Update output, clamping to max linear speed
    velocityOutput = Units.MetersPerSecond.of(MathUtil.clamp(
      velocityOutput.plus(velocityCorrection).in(Units.MetersPerSecond),
      -m_maxLinearVelocity,
      +m_maxLinearVelocity
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
   * Get max linear velocity
   * @return Maximum linear velocity
   */
  public LinearVelocity getMaxLinearVelocity() {
    return Units.MetersPerSecond.of(m_maxLinearVelocity);
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