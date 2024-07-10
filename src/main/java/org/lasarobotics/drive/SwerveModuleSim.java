package org.lasarobotics.drive;

import org.lasarobotics.utils.FFConstants;
import org.lasarobotics.utils.GlobalConstants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class SwerveModuleSim {
  private final FlywheelSim m_driveMotor;
  private final FlywheelSim m_turningMotor;

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
    m_driveMotor = new FlywheelSim(LinearSystemId.identifyVelocitySystem(driveFF.kV, driveFF.kA), driveMotor, 1.0);
    m_turningMotor = new FlywheelSim(LinearSystemId.identifyVelocitySystem(rotateFF.kV, rotateFF.kA), rotateMotor, 1.0);
  }

  /**
   * Update swerve module simulation, should be called periodically
   * @param driveVoltage Drive motor input voltage
   * @param rotateVoltage Rotate motor input voltage
   */
  public void update(double driveVoltage, double rotateVoltage) {
    m_driveMotor.setInputVoltage(driveVoltage);
    m_turningMotor.setInputVoltage(rotateVoltage);
    m_driveMotor.update(GlobalConstants.ROBOT_LOOP_PERIOD);
    m_turningMotor.update(GlobalConstants.ROBOT_LOOP_PERIOD);
  }

  /**
   * Get drive motor velocity
   * @return Drive motor velocity
   */
  public Measure<Velocity<Angle>> getDriveMotorVelocity() {
    return Units.RPM.of(m_driveMotor.getAngularVelocityRPM());
  }

  /**
   * Get rotate motor velocity
   * @return Rotate motor velocity
   */
  public Measure<Velocity<Angle>> getRotateMotorVelocity() {
    return Units.RPM.of(m_turningMotor.getAngularVelocityRPM());
  }

  /**
   * Get drive motor current draw
   * @return Drive motor current draw
   */
  public Measure<Current> getDriveMotorCurrentDraw() {
    return Units.Amps.of(m_driveMotor.getCurrentDrawAmps());
  }

  /**
   * Get rotate motor current draw
   * @return Rotate motor current draw
   */
  public Measure<Current> getRotateMotorCurrentDraw() {
    return Units.Amps.of(m_turningMotor.getCurrentDrawAmps());
  }

  /**
   * Get total current draw for swerve module
   * @return Total current draw
   */
  public Measure<Current> getTotalCurrentDraw() {
    return Units.Amps.of(m_driveMotor.getCurrentDrawAmps() + m_turningMotor.getCurrentDrawAmps());
  }
}