// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.drive.swerve.child;

import org.lasarobotics.drive.swerve.DriveWheel;
import org.lasarobotics.drive.swerve.SwerveModule;
import org.lasarobotics.drive.swerve.parent.CTRESwerveModule;
import org.lasarobotics.drive.swerve.parent.REVSwerveModule;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.utils.FFConstants;
import org.lasarobotics.utils.PIDConstants;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;

/** Helper class to create an instance of MK4cModule */
public abstract class MK4cModule {

  /**
   * MK4c gear ratios
   */
  public enum GearRatio implements SwerveModule.GearRatio {
    /** 7.13:1 */
    L1(7.13),
    /** 5.90:1 */
    L2(5.90),
    /** 5.36:1 */
    L3(5.36);

    private static final double ROTATE_GEAR_RATIO = 12.8;
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

  private static final Angle ZERO_OFFSET = Units.Radians.zero();

  /**
   * Do not allow direct instantiation
   */
  private MK4cModule() {}

  /**
   * Create an instance of a SDS MK4c module powered by REV Spark motor controllers
   * @param swerveHardware Hardware devices required by swerve module
   * @param location Location of module
   * @param gearRatio Gear ratio for module
   * @param driveWheel Wheel installed in swerve module
   * @param drivePID Drive motor PID gains
   * @param driveFF Drive motor feed forward gains
   * @param rotatePID Rotate motor PID gains
   * @param rotateFF Rotate motor feed forward gains
   * @param slipRatio Desired slip ratio [1%, 40%]
   * @param mass Robot mass
   * @param wheelbase Robot wheelbase
   * @param trackWidth Robot track width
   * @param autoLockTime Time before automatically rotating module to locked position (10 seconds max)
   * @param driveCurrentLimit Desired current limit for the drive motor
   */
  public static REVSwerveModule create(REVSwerveModule.Hardware swerveHardware,
                                       SwerveModule.Location location,
                                       GearRatio gearRatio, DriveWheel driveWheel,
                                       PIDConstants drivePID, FFConstants driveFF,
                                       PIDConstants rotatePID, FFConstants rotateFF,
                                       Dimensionless slipRatio, Mass mass,
                                       Distance wheelbase, Distance trackWidth,
                                       Time autoLockTime, Current driveCurrentLimit) {
    if (swerveHardware.rotateMotor.getKind().equals(Spark.MotorKind.NEO_550))
      throw new IllegalArgumentException("MK4c rotate motor cannot be a NEO 550!");

    return new REVSwerveModule(swerveHardware,
                               location,
                               SwerveModule.MountOrientation.STANDARD,
                               SwerveModule.MountOrientation.STANDARD,
                               gearRatio,
                               driveWheel, ZERO_OFFSET,
                               drivePID, driveFF,
                               rotatePID, rotateFF,
                               slipRatio, mass,
                               wheelbase, trackWidth,
                               autoLockTime, driveCurrentLimit);
  }

  /**
   * Create an instance of a SDS MK4c module powered by CTRE TalonFX motor controllers
   * @param swerveHardware Hardware devices required by swerve module
   * @param location Location of module
   * @param gearRatio Gear ratio for module
   * @param driveWheel Wheel installed in swerve module
   * @param drivePID Drive motor PID gains
   * @param driveFF Drive motor feed forward gains
   * @param rotatePID Rotate motor PID gains
   * @param rotateFF Rotate motor feed forward gains
   * @param slipRatio Desired slip ratio [1%, 40%]
   * @param mass Robot mass
   * @param wheelbase Robot wheelbase
   * @param trackWidth Robot track width
   * @param autoLockTime Time before automatically rotating module to locked position (10 seconds max)
   * @param driveCurrentLimit Desired current limit for the drive motor
   */
  public static CTRESwerveModule create(CTRESwerveModule.Hardware swerveHardware,
                                        SwerveModule.Location location,
                                        GearRatio gearRatio, DriveWheel driveWheel,
                                        PIDConstants drivePID, FFConstants driveFF,
                                        PIDConstants rotatePID, FFConstants rotateFF,
                                        Dimensionless slipRatio, Mass mass,
                                        Distance wheelbase, Distance trackWidth,
                                        Time autoLockTime, Current driveCurrentLimit) {
    return new CTRESwerveModule(swerveHardware,
                                location,
                                SwerveModule.MountOrientation.STANDARD,
                                SwerveModule.MountOrientation.STANDARD,
                                gearRatio,
                                driveWheel, ZERO_OFFSET,
                                drivePID, driveFF,
                                rotatePID, rotateFF,
                                slipRatio, mass,
                                wheelbase, trackWidth,
                                autoLockTime, driveCurrentLimit);
  }
}
