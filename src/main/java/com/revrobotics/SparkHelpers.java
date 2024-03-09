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
}
