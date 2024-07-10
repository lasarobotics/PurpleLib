// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.utils;

public class NoiseGenerator {

  static java.util.Random m_rand = new java.util.Random();

  public static double whiteNoise(double input, double variance) {
    return m_rand.nextGaussian() * Math.sqrt(variance) + input;
  }

  public static double hallSensorVelocity(double speedRPM) {
    // This isn't so realistic at the moment. Idea is to increase noise with speed
    return whiteNoise(speedRPM, Math.max(speedRPM, 0.0) / 20000.0);
  }
}