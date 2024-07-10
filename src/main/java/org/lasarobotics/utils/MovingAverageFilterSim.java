// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.utils;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.LinearFilter;

/**
 * Simulate a moving average filter where samples are normally at some fixed rate that is different
 * than the rate provided by the simulation.
 *
 * <p>A sum of sample deltas is stored until the cumulative sample delta is greater than or equal to
 * the sample rate. Two cases are possible:
 *
 * <p>1) The cumulative delta between samples is *greater or equal* to the sample rate. In this
 * case, take a simple linear interpolation between the sample before and after the sample time, and
 * add this new sample to the moving average.
 *
 * <p>2) The cumulative delta between the samples is *less* than the sample rate. In this case,
 * store the last value, and accumulate the delta time.
 */
public class MovingAverageFilterSim {
  // Actual filter
  private final LinearFilter m_filter;

  // Current filter value
  private double m_value = 0.0;

  private final double m_sampleRate;

  // Pair <delta, last sample value>
  private Pair<Double, Double> m_state = new Pair<>(0.0, 0.0);

  /**
   * Create a MovingAverageFilterSim object.
   * @param taps number of samples in moving average filter
   * @param sampleRate sample rate of moving average filter to simulate
   */
  public MovingAverageFilterSim(int taps, double sampleRate) {
    m_sampleRate = sampleRate;
    m_filter = LinearFilter.movingAverage(taps);
  }

  private double lerp(Pair<Double, Double> p1, Pair<Double, Double> p2, double x) {
    if (p2.getFirst() == p1.getFirst()) {
      return p2.getSecond();
    }
    return p1.getSecond()
        + (x - p1.getFirst())
            * ((p2.getSecond() - p1.getSecond()) / (p2.getFirst() - p1.getFirst()));
  }

  /**
   * Put a new measurement into the moving average filter. This will add any number of samples (or
   * none) depending on the time delta provided.
   * @param value new measurement value
   * @param delta time delta between last measurement value
   */
  public void put(double value, double delta) {
    double newDelta = m_state.getFirst() + delta;

    while (newDelta >= m_sampleRate) {
      // put new filter value
      m_value = m_filter.calculate(lerp(m_state, new Pair<>(newDelta, value), m_sampleRate));
      newDelta = newDelta - m_sampleRate;
    }
    m_state = new Pair<>(newDelta, value);
  }

  /**
   * Get the current value of the filter
   * @return filtered value
   */
  public double get() {
    return m_value;
  }
}