// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package com.revrobotics.spark;

/** Spark helpers */
public class SparkHelpers {

  public enum SparkModel {
    SparkMax(0),
    SparkFlex(1),
    Unknown(255);

    final int id;

    SparkModel(int id) {
      this.id = id;
    }

    static SparkModel fromId(int id) {
      for (SparkModel model : values()) {
        if (model.id == id) return model;
      }
      return Unknown;
    }
  }
  /**
   * Enable mode which sets the output of the PID controllers to be voltage instead of duty cycle.
   *
   * <p>To disable, change or disable voltage compensation. Those settings will overwrite this one
   */
  // public static REVLibError enablePIDVoltageOutput(CANSparkBase spark) {
  //   return REVLibError.fromInt(CANSparkMaxJNI.c_SparkMax_SetParameterUint32(spark.sparkMaxHandle, 74, 1));
  // }

  /**
   * Get model of Spark
   * @param spark Spark to get model of
   * @return Model of Spark
   */
  public static SparkModel getSparkModel(SparkBase spark) {
    switch (spark.getSparkModel()) {
      case SparkFlex:
        return SparkModel.SparkFlex;
      case SparkMax:
        return SparkModel.SparkMax;
      default:
        return SparkModel.Unknown;
    }
  }

  public static long getSparkHandle(SparkBase spark) {
    return spark.sparkHandle;
  }
}
