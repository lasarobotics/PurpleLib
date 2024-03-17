// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package com.revrobotics;

import com.revrobotics.jni.CANSparkMaxJNI;

/** Spark helpers */
public class SparkHelpers {
  /**
   * Get current limit of Spark
   * @param spark Spark to check
   * @return Current limit that is currently set
   */
  public static int getSmartCurrentLimit(CANSparkBase spark) {
    return CANSparkMaxJNI.c_SparkMax_GetSmartCurrentStallLimit(spark.sparkMaxHandle);
  }

  /** Enable center aligned mode for the duty cycle sensor. */
  public static REVLibError enableCenterAlignedMode(CANSparkBase spark) {
    return REVLibError.fromInt(
        CANSparkMaxJNI.c_SparkMax_SetParameterBool(spark.sparkMaxHandle, 152, true));
  }

  /** Disable center aligned mode for the duty cycle sensor. */
  public static REVLibError disableCenterAlignedMode(CANSparkBase spark) {
    return REVLibError.fromInt(
        CANSparkMaxJNI.c_SparkMax_SetParameterBool(spark.sparkMaxHandle, 152, false));
  }

  /**
   * Enable mode which sets the output of the PID controllers to be voltage instead of duty cycle.
   *
   * <p>To disable, change or disable voltage compensation. Those settings will overwrite this one
   */
  public static REVLibError enablePIDVoltageOutput(CANSparkBase spark) {
    return REVLibError.fromInt(CANSparkMaxJNI.c_SparkMax_SetParameterUint32(spark.sparkMaxHandle, 74, 1));
  }
}
