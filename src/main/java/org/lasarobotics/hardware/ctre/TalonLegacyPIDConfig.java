// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.hardware.ctre;

import org.lasarobotics.utils.PIDConstants;

import edu.wpi.first.math.MathUtil;

/**
 * Automates the configuration of Talon Legacy PID and MotionMagic parameters
 */
public class TalonLegacyPIDConfig {
  private static final double MIN_TOLERANCE = 1.0;
  private static final int MIN_MOTION_SMOOTHING = 0;
  private static final int MAX_MOTION_SMOOTHING = 8;

  private boolean m_motionMagic = false;
  private boolean m_enableSoftLimits = true;

  private boolean m_sensorPhase = false;
  private boolean m_invertMotor = false;
  private double m_ticksPerRotation = 0.0;
  private double m_kP = 0.0;
  private double m_kI = 0.0;
  private double m_kD = 0.0;
  private double m_kF = 0.0;
  private double m_tolerance = 1.0;
  private double m_lowerLimit = 0.0;
  private double m_upperLimit = 0.0;

  private double m_velocityRPM = 1.0;
  private double m_accelerationRPMPerSec = 1.0;
  private int m_motionSmoothing = 0;

  /**
   * Create a TalonLegacyPIDConfig, without MotionMagic parameters
   * <p>
   * USE FOR VELOCITY PID ONLY!
   * @param pidf PID constants
   * @param ff Feedforward constants
   * @param sensorPhase set sensor phase of encoder
   * @param invertMotor invert motor or not
   * @param ticksPerRotation number of ticks in one encoder revolution
   * @param tolerance tolerance of PID loop in ticks per 100ms
   */
  public TalonLegacyPIDConfig(PIDConstants pidf, boolean sensorPhase, boolean invertMotor, double ticksPerRotation, double tolerance) {
    this.m_kP = pidf.kP;
    this.m_kI = pidf.kI;
    this.m_kD = pidf.kD;
    this.m_kF = pidf.kF;
    this.m_sensorPhase = sensorPhase;
    this.m_invertMotor = invertMotor;
    this.m_ticksPerRotation = ticksPerRotation;
    this.m_tolerance = Math.max(tolerance, MIN_TOLERANCE);

    this.m_enableSoftLimits = false;
  }

  /**
   * Create a TalonLegacyPIDConfig, with MotionMagic parameters
   * <p>
   * USE FOR POSITION PID ONLY!
   * @param pidf PID constants
   * @param sensorPhase set sensor phase of encoder
   * @param invertMotor invert motor or not
   * @param ticksPerRotation number of ticks in one encoder revolution
   * @param tolerance tolerance of PID loop in ticks
   * @param lowerLimit Lower soft limit
   * @param upperLimit Upper soft limit
   * @param enableSoftLimits True to enable soft limits
   * @param velocity MotionMagic cruise velocity in RPM
   * @param acceleration MotionMagic acceleration in RPM per second
   * @param motionSmoothing MotionMagic smoothing factor [0, 8]
   */
  public TalonLegacyPIDConfig(PIDConstants pidf, boolean sensorPhase, boolean invertMotor, double ticksPerRotation,
                        double tolerance, double lowerLimit, double upperLimit, boolean enableSoftLimits,
                        double velocity, double acceleration, int motionSmoothing) {
    this.m_kP = pidf.kP;
    this.m_kI = pidf.kI;
    this.m_kD = pidf.kD;
    this.m_kF = pidf.kF;
    this.m_sensorPhase = sensorPhase;
    this.m_invertMotor = invertMotor;
    this.m_ticksPerRotation = ticksPerRotation;
    this.m_tolerance = Math.max(tolerance, MIN_TOLERANCE);
    this.m_lowerLimit = lowerLimit;
    this.m_upperLimit = upperLimit;
    this.m_enableSoftLimits = enableSoftLimits;

    this.m_velocityRPM = velocity;
    this.m_accelerationRPMPerSec = acceleration;
    this.m_motionSmoothing = MathUtil.clamp(motionSmoothing, MIN_MOTION_SMOOTHING, MAX_MOTION_SMOOTHING);

    this.m_motionMagic = true;
  }

  /**
   * Convert RPM to ticks per 100ms
   * @param rpm RPM
   * @return Value in ticks per 100ms
   */
  public double rpmToTicksPer100ms(double rpm) {
    return (rpm * m_ticksPerRotation) / 600;
  }

  /**
   * Convert ticks per 100ms to RPM
   * @param ticks Encoder ticks per 100ms
   * @return Value in RPM
   */
  public double ticksPer100msToRPM(double ticks) {
    return (ticks * 600) / m_ticksPerRotation;
  }

  /**
   * @return Sensor phase
   */
  public boolean getSensorPhase() {
    return m_sensorPhase;
  }

  /**
   * @return Whether motor is inverted or not
   */
  public boolean getInvertMotor() {
    return m_invertMotor;
  }

  /**
   * @return Proportional gain
   */
  public double getkP() {
    return m_kP;
  }

  /**
   * @return Integral gain
   */
  public double getkI() {
    return m_kI;
  }

  /**
   * @return Derivative gain
   */
  public double getkD() {
    return m_kD;
  }

  /**
   * @return Feed-forward gain
   */
  public double getkF() {
    return m_kF;
  }

  /**
   * @return PID loop tolerance
   */
  public double getTolerance() {
    return m_tolerance;
  }

  /**
   * @return Lower limit of mechanism
   */
  public double getLowerLimit() {
    return m_lowerLimit;
  }

  /**
   * @return Upper limit of mechanism
   */
  public double getUpperLimit() {
    return m_upperLimit;
  }

  /**
   * @return Whether soft limits are enabled or not
   */
  public boolean getSoftLimitsEnabled() {
    return m_enableSoftLimits;
  }

  /**
   * @return MotionMagic cruise velocity in RPM
   */
  public double getVelocityRPM() {
    return m_velocityRPM;
  }

  /**
   * @return MotionMagic acceleration in RPM per sec
   */
  public double getAccelerationRPMPerSec() {
    return m_accelerationRPMPerSec;
  }

  /**
   * @return Whether motion magic is enabled or not
   */
  public boolean getMotionMagic() {
    return m_motionMagic;
  }

  /**
   * @return MotionMagic smoothing factor
   */
  public int getMotionSmoothing() {
    return m_motionSmoothing;
  }
}
