package org.lasarobotics.drive.swerve;

import org.lasarobotics.utils.FFConstants;
import org.lasarobotics.utils.GlobalConstants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class SwerveModuleSim {
  private final FlywheelSim m_driveSim;
  private final FlywheelSim m_rotateSim;

  /**
   * Overly simplistic model of swerve module with very simple dynamics using WPILib DCMotorSim
   * classes. The only input is a DC Motor and an inertia. Note that gearing should be factored in
   * using the withReduction method of the DCMotor class.
   *
   * @param driveMotor DCMotor including reductions for the drive motor
   * @param driveFF Feed-forward that takes modeled inertia of the drive motor into account
   * @param rotateMotor DCMotor including reductions for the rotate motor
   * @param rotateFF Feed-forward that takes modeled inertia of the rotate motor into account
   */
  public SwerveModuleSim(DCMotor driveMotor, FFConstants driveFF, DCMotor rotateMotor, FFConstants rotateFF) {
    m_driveSim = new FlywheelSim(LinearSystemId.identifyVelocitySystem(driveFF.kV, driveFF.kA), driveMotor, 1.0);
    m_rotateSim = new FlywheelSim(LinearSystemId.identifyVelocitySystem(rotateFF.kV, rotateFF.kA), rotateMotor, 1.0);
  }

  /**
   * Update swerve module simulation, should be called periodically
   * @param driveVoltage Drive motor input voltage
   * @param rotateVoltage Rotate motor input voltage
   */
  public void update(Voltage driveVoltage, Voltage rotateVoltage) {
    m_driveSim.setInputVoltage(driveVoltage.in(Units.Volts));
    m_rotateSim.setInputVoltage(rotateVoltage.in(Units.Volts));
    m_driveSim.update(GlobalConstants.ROBOT_LOOP_HZ.asPeriod().in(Units.Seconds));
    m_rotateSim.update(GlobalConstants.ROBOT_LOOP_HZ.asPeriod().in(Units.Seconds));
  }

  /**
   * Get drive motor simulation
   * @return Object representing drive motor dynamics
   */
  public FlywheelSim getDriveSim() {
    return m_driveSim;
  }

  /**
   * Get rotate motor simulation
   * @return Object representing rotate motor dynamics
   */
  public FlywheelSim getRotateSim() {
    return m_rotateSim;
  }

  /**
   * Get drive motor velocity
   * @return Drive motor velocity
   */
  public AngularVelocity getDriveMotorVelocity() {
    return Units.RPM.of(m_driveSim.getAngularVelocityRPM());
  }

  /**
   * Get rotate motor velocity
   * @return Rotate motor velocity
   */
  public AngularVelocity getRotateMotorVelocity() {
    return Units.RPM.of(m_rotateSim.getAngularVelocityRPM());
  }

  /**
   * Get drive motor current draw
   * @return Drive motor current draw
   */
  public Current getDriveMotorCurrentDraw() {
    return Units.Amps.of(m_driveSim.getCurrentDrawAmps());
  }

  /**
   * Get rotate motor current draw
   * @return Rotate motor current draw
   */
  public Current getRotateMotorCurrentDraw() {
    return Units.Amps.of(m_rotateSim.getCurrentDrawAmps());
  }

  /**
   * Get total current draw for swerve module
   * @return Total current draw
   */
  public Current getTotalCurrentDraw() {
    return Units.Amps.of(m_driveSim.getCurrentDrawAmps() + m_rotateSim.getCurrentDrawAmps());
  }
}