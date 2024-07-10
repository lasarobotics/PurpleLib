// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.utils;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Function;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

@SuppressWarnings("unused")
public class SimDynamics {
  private Runnable m_update;
  private DoubleSupplier m_getVelocityRPM;
  private DoubleSupplier m_getCurrentDrawAmps;
  private DoubleConsumer m_setInputVoltage;
  private Function<Double, Boolean> m_limits;

  public static SimDynamics fromSim(FlywheelSim flywheel) {
    return new SimDynamics(
      () -> flywheel.update(GlobalConstants.ROBOT_LOOP_PERIOD),
      () -> flywheel.getAngularVelocityRPM(),
      () -> flywheel.getCurrentDrawAmps(),
      (voltage) -> flywheel.setInputVoltage(voltage),
      (position) -> false
    );
  }

  public static SimDynamics fromSim(SingleJointedArmSim singleJointedArm) {
    return new SimDynamics(
      () -> singleJointedArm.update(GlobalConstants.ROBOT_LOOP_PERIOD),
      () -> edu.wpi.first.math.util.Units.radiansPerSecondToRotationsPerMinute(singleJointedArm.getVelocityRadPerSec()),
      () -> singleJointedArm.getCurrentDrawAmps(),
      (voltage) -> singleJointedArm.setInputVoltage(voltage),
      (position) -> false
    );
  }

  public SimDynamics(Runnable update,
                     DoubleSupplier getVelocityRPM,
                     DoubleSupplier getCurrentDrawAmps,
                     DoubleConsumer setInputVoltage,
                     Function<Double, Boolean> hitLimit) {
    m_getVelocityRPM = getVelocityRPM;
    m_getCurrentDrawAmps = getCurrentDrawAmps;
    m_setInputVoltage = setInputVoltage;
    m_update = update;
    m_limits = hitLimit;
  }

  /**
   * Update simulated mechanism
   */
  public void update() {
    m_update.run();
  }

  /**
   * Set input voltage
   * @param vbus Vbus voltage
   */
  public void setInputVoltage(Measure<Voltage> vbus) {
    m_setInputVoltage.accept(vbus.in(Units.Volts));
  }

  /**
   * Get velocity of simulated mechanism
   * @return Angular velocity
   */
  public Measure<Velocity<Angle>> getAngularVelocity() {
    return Units.RPM.of(m_getVelocityRPM.getAsDouble());
  }

  /**
   * Get current draw of simulated mechanism
   * @return Current draw
   */
  public Measure<Current> getCurrentDraw() {
    return Units.Amps.of(m_getCurrentDrawAmps.getAsDouble());
  }
}