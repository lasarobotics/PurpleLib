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

/** Helper class to create an instance of SwerveXModule */
public abstract class SwerveXModule {

  /**
   * Swerve X gear ratios
   */
  public enum GearRatio implements SwerveModule.GearRatio {
    /** 7.85:1 */
    X1_1(7.85),
    /** 7.13:1 */
    X1_2(7.13),
    /** 6.54:1 */
    X1_3(6.54),
    /** 6.56:1 */
    X2_1(6.56),
    /** 5.96:1 */
    X2_2(5.96),
    /** 5.46:1 */
    X2_3(5.46),
    /** 5.14:1 */
    X3_1(5.14),
    /** 4.75:1 */
    X3_2(4.75),
    /** 4.41:1 */
    X3_3(4.75);

    private static final double ROTATE_GEAR_RATIO = 396.0 / 35.0;
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
  private SwerveXModule() {}

  /**
   * Create an instance of a WCP Swerve X module powered by REV Spark motor controllers
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
      throw new IllegalArgumentException("Swerve X rotate motor cannot be a NEO 550!");

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
   * Create an instance of a WCP Swerve X module powered by CTRE TalonFX motor controllers
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
