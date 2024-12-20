// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package com.revrobotics;

import com.revrobotics.jni.CANSparkMaxJNI;

/** Spark helpers */
public class SparkHelpers {
  /**
   * Get current stall limit of Spark
   * @param spark Spark to check
   * @return Current limit that is currently set
   */
  public static int getSmartCurrentStallLimit(CANSparkBase spark) {
    return CANSparkMaxJNI.c_SparkMax_GetSmartCurrentStallLimit(spark.sparkMaxHandle);
  }

  /**
   * Get current free limit of Spark
   * @param spark Spark to check
   * @return Current limit that is currently set
   */
  public static int getSmartCurrentFreeLimit(CANSparkBase spark) {
    return CANSparkMaxJNI.c_SparkMax_GetSmartCurrentFreeLimit(spark.sparkMaxHandle);
  }

  /**
   * Get current limit RPM of Spark
   * @param spark Spark to check
   * @return Current limit that is currently set
   */
  public static int getSmartCurrentLimitRPM(CANSparkBase spark) {
    return CANSparkMaxJNI.c_SparkMax_GetSmartCurrentLimitRPM(spark.sparkMaxHandle);
  }

  /**
   * Get the secondary limit cycles of Spark, which is also the number of chop cycles
   * @param spark Spark to check
   * @return Current limit that is currently set
   */
  public static int getSecondaryCurrentLimitCycles(CANSparkBase spark) {
    return CANSparkMaxJNI.c_SparkMax_GetSecondaryCurrentLimitCycles(spark.sparkMaxHandle);
  }

  /**
   * Get the secondary current limit of Spark
   * @param spark Spark to check
   * @return Current limit that is currently set
   */
  public static float getSecondaryCurrentLimit(CANSparkBase spark) {
    return CANSparkMaxJNI.c_SparkMax_GetSecondaryCurrentLimit(spark.sparkMaxHandle);
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

  /**
   * Convert Spark faults to human-readable strings
   * @param faults Sticky fault bits
   * @return All sticky fault messages
   */
  public static String faultWordToString(short faults) {
    if (faults == 0) {
      return "";
    }

    StringBuilder builder = new StringBuilder();
    int faultsInt = faults;
    for (int i = 0; i < 16; i++) {
      if (((1 << i) & faultsInt) != 0) {
        builder.append(CANSparkMax.FaultID.fromId(i).toString());
        builder.append(" ");
      }
    }
    return builder.toString();
  }
}
