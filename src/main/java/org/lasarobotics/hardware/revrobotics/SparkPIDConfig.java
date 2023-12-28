// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.hardware.revrobotics;

import org.lasarobotics.utils.PIDConstants;

/**
 * Automates the configuration of Spark PID
 */
public class SparkPIDConfig {
  private boolean m_enableSoftLimits = true;
  private boolean m_sensorPhase = false;
  private boolean m_invertMotor = false;
  private double m_kP = 0.0;
  private double m_kI = 0.0;
  private double m_kD = 0.0;
  private double m_kF = 0.0;
  private double m_tolerance = 1.0;
  private double m_lowerLimit = 0.0;
  private double m_upperLimit = 0.0;

  /**
   * Create a SparkPIDConfig
   * <p>
   * USE FOR VELOCITY PID ONLY!
   * @param pidf PID constants
   * @param sensorPhase set sensor phase of encoder
   * @param invertMotor invert motor or not
   * @param tolerance tolerance of PID loop
   */
  public SparkPIDConfig(PIDConstants pidf, boolean sensorPhase, boolean invertMotor, double tolerance) {
    this.m_kP = pidf.kP;
    this.m_kI = pidf.kI;
    this.m_kD = pidf.kD;
    this.m_kF = pidf.kF;
    this.m_sensorPhase = sensorPhase;
    this.m_invertMotor = invertMotor;
    this.m_tolerance = tolerance;

    this.m_enableSoftLimits = false;
  }

  /**
   * Create a SparkPIDConfig
   * <p>
   * USE FOR POSITION PID ONLY!
   * @param pidf PID constants
   * @param sensorPhase set sensor phase of encoder
   * @param invertMotor invert motor or not
   * @param tolerance tolerance of PID loop
   * @param lowerLimit lower soft limit
   * @param upperLimit upper soft limit
   * @param enableSoftLimits true to enable soft limits
   */
  public SparkPIDConfig(PIDConstants pidf, boolean sensorPhase, boolean invertMotor,
                        double tolerance, double lowerLimit, double upperLimit, boolean enableSoftLimits) {
    this.m_kP = pidf.kP;
    this.m_kI = pidf.kI;
    this.m_kD = pidf.kD;
    this.m_kF = pidf.kF;
    this.m_sensorPhase = sensorPhase;
    this.m_invertMotor = invertMotor;
    this.m_tolerance = tolerance;
    this.m_lowerLimit = lowerLimit;
    this.m_upperLimit = upperLimit;
    this.m_enableSoftLimits = enableSoftLimits;
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
  public boolean getInverted() {
    return m_invertMotor;
  }

  /**
   * @return Proportional gain
   */
  public double getP() {
    return m_kP;
  }

  /**
   * @return Integral gain
   */
  public double getI() {
    return m_kI;
  }

  /**
   * @return Derivative gain
   */
  public double getD() {
    return m_kD;
  }

  /**
   * @return Feed-forward gain
   */
  public double getF() {
    return m_kF;
  }

  /**
   * @return PID loop tolerance
   */
  public double getTolerance() {
    return m_tolerance;
  }

  public boolean isSoftLimitEnabled() {
    return m_enableSoftLimits;
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
}
