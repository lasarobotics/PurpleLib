// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.hardware.rev;

import org.lasarobotics.utils.PIDConstants;

import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.SparkMaxPIDController;

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
   * Initializes Spark Max PID
   * <p>
   * Must call {@link SparkMax#burnFlash()} after configuring all settings
   * @param spark Spark motor controller to apply settings to
   * @param feedbackSensor Feedback device to use for Spark PID
   * @param forwardLimitSwitch Enable forward limit switch
   * @param reverseLimitSwitch Enable reverse limit switch
   */
  public void initializeSparkPID(SparkMax spark, MotorFeedbackSensor feedbackSensor,
                                 boolean forwardLimitSwitch, boolean reverseLimitSwitch) {
    // Get PID controller
    SparkMaxPIDController pidController = spark.getMotorController().getPIDController();

    // Configure feedback sensor and set sensor phase
    try {
      pidController.setFeedbackDevice(feedbackSensor);
      feedbackSensor.setInverted(m_sensorPhase);
    } catch (IllegalArgumentException e) {}

    // Configure forward and reverse soft limits
    if (m_enableSoftLimits) {
      spark.setForwardSoftLimit(m_upperLimit);
      spark.enableForwardSoftLimit();
      spark.setReverseSoftLimit(m_lowerLimit);
      spark.enableReverseSoftLimit();
    }

    // Configure forward and reverse limit switches if required, and disable soft limit
    if (forwardLimitSwitch) {
      spark.enableForwardLimitSwitch();
      spark.disableForwardSoftLimit();
    }
    if (reverseLimitSwitch) {
      spark.enableReverseLimitSwitch();
      spark.disableReverseSoftLimit();
    }

    // Invert motor if required
    spark.setInverted(m_invertMotor);

    // Configure PID values
    spark.applyParameter(() -> pidController.setP(m_kP), () -> pidController.getP() == m_kP, "Set kP failure!");
    spark.applyParameter(() -> pidController.setI(m_kI), () -> pidController.getI() == m_kI, "Set kI failure!");
    spark.applyParameter(() -> pidController.setD(m_kD), () -> pidController.getD() == m_kD, "Set kD failure!");
    spark.applyParameter(() -> pidController.setFF(m_kF), () -> pidController.getFF() == m_kF, "Set kF failure!");
    spark.applyParameter(
      () -> pidController.setIZone(m_kI != 0.0 ? m_tolerance * 2 : 0.0),
      () -> pidController.getIZone() == (m_kI != 0.0 ? m_tolerance * 2 : 0.0),
      "Set Izone failure!"
    );
  }

  /**
   * Initializes Spark PID
   * <p>
   * Calls {@link SparkPIDConfig#initializeSparkPID(SparkMax, MotorFeedbackSensor, boolean, boolean)} with no limit switches
   * <p>
   * Must call {@link SparkMax#burnFlash()} after configuring all settings
   * @param spark Spark motor controller to apply settings to
   * @param feedbackSensor Feedback device to use for Spark PID
   */
  public void initializeSparkPID(SparkMax spark, MotorFeedbackSensor feedbackSensor) {
    initializeSparkPID(spark, feedbackSensor, false, false);
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
}
