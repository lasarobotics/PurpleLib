// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.drive.swerve.revrobotics;

import org.lasarobotics.drive.swerve.DriveWheel;
import org.lasarobotics.drive.swerve.SwerveGearRatio;
import org.lasarobotics.drive.swerve.SwerveModule;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.utils.FFConstants;
import org.lasarobotics.utils.PIDConstants;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;

/** Helper class to create an instance of MAXSwerveModule */
public class MAXSwerveModule {

  /**
   * MAXSwerve gear ratio
   */
  public enum GearRatio implements SwerveGearRatio {
    /** 5.50:1 */
    L1(5.50),
    /** 5.08:1 */
    L2(5.08),
    /** 4.71:1 */
    L3(4.71),
    /** 4.50:1 */
    L4(4.50),
    /** 4.29:1 */
    L5(4.29),
    /** 4.00:1 */
    L6(4.00),
    /** 3.75:1 */
    L7(3.75),
    /** 3.56:1 */
    L8(3.56);

    private static final double ROTATE_GEAR_RATIO = 9424.0 / 203.0;
    private final double driveGearRatio;
    private GearRatio(double driveGearRatio) {
      this.driveGearRatio = driveGearRatio;
    }

    @Override
    public double getDriveRatio() {
      return driveGearRatio;
    }

    @Override
    public double getRotateRatio() {
      return ROTATE_GEAR_RATIO;
    }
  }

  // Swerve velocity PID settings
  private static final double DRIVE_kP = 0.3;
  private static final double DRIVE_kD = 0.001;

  // Swerve rotate PID settings
  private static final PIDConstants ROTATE_PID = new PIDConstants(2.0, 0.0, 0.1, 0.0, 0.0);

  /**
   * Create an instance of a MAXSwerveModule
   * @param swerveHardware Hardware devices required by swerve module
   * @param location Location of module
   * @param gearRatio Gear ratio for module
   * @param driveWheel Wheel installed in swerve module
   * @param slipRatio Desired slip ratio [1%, 40%]
   * @param mass Robot mass
   * @param wheelbase Robot wheelbase
   * @param trackWidth Robot track width
   * @param autoLockTime Time before automatically rotating module to locked position (10 seconds max)
   * @param driveMotorCurrentLimit Desired current limit for the drive motor
   */
  public static REVSwerveModule create(REVSwerveModule.Hardware swerveHardware,
                                       SwerveModule.Location location,
                                       MAXSwerveModule.GearRatio gearRatio, DriveWheel driveWheel,
                                       Dimensionless slipRatio, Mass mass,
                                       Distance wheelbase, Distance trackWidth,
                                       Time autoLockTime, Current driveMotorCurrentLimit) {
    if (!swerveHardware.rotateMotor.getKind().equals(Spark.MotorKind.NEO_550))
      throw new IllegalArgumentException("MAXSwerve rotate motor MUST be a NEO 550!");

    double driveConversionFactor = driveWheel.diameter.in(Units.Meters) * Math.PI / gearRatio.getDriveRatio();
    double drive_kF = 1 / ((swerveHardware.driveMotor.getKind().getMaxRPM() * REVSwerveModule.DRIVETRAIN_EFFICIENCY / 60) * driveConversionFactor);

    return new REVSwerveModule(swerveHardware,
                               SwerveModule.Vendor.REV, SwerveModule.MountOrientation.STANDARD, location,
                               gearRatio, driveWheel,
                               new PIDConstants(DRIVE_kP, 0.0, DRIVE_kD, drive_kF, 0.0), new FFConstants(0, 0, 0, 0),
                               ROTATE_PID, new FFConstants(0, 0, 0, 0),
                               slipRatio, mass,
                               wheelbase, trackWidth,
                               autoLockTime, driveMotorCurrentLimit);
  }
}
