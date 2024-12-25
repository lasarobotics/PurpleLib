// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.drive.swerve.child;

import java.util.Map;

import org.lasarobotics.drive.swerve.DriveWheel;
import org.lasarobotics.drive.swerve.SwerveModule;
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

/** Helper class to create an instance of MAXSwerveModule */
public abstract class MAXSwerveModule {

  /**
   * MAXSwerve gear ratios
   */
  public enum GearRatio implements SwerveModule.GearRatio {
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

  public static final Map<SwerveModule.Location, Angle> ZERO_OFFSET = Map.ofEntries(
    Map.entry(SwerveModule.Location.LeftFront, Units.Radians.of(Math.PI / 2)),
    Map.entry(SwerveModule.Location.RightFront, Units.Radians.zero()),
    Map.entry(SwerveModule.Location.LeftRear, Units.Radians.of(Math.PI)),
    Map.entry(SwerveModule.Location.RightRear, Units.Radians.of(Math.PI / 2))
  );

  /**
   * Do not allow direct instantiation
   */
  private MAXSwerveModule() {}

  /**
   * Create an instance of a REV MAXSwerve module
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
                                       MAXSwerveModule.GearRatio gearRatio, DriveWheel driveWheel,
                                       PIDConstants drivePID, FFConstants driveFF,
                                       PIDConstants rotatePID, FFConstants rotateFF,
                                       Dimensionless slipRatio, Mass mass,
                                       Distance wheelbase, Distance trackWidth,
                                       Time autoLockTime, Current driveCurrentLimit) {
    if (!swerveHardware.rotateMotor.getKind().equals(Spark.MotorKind.NEO_550))
      throw new IllegalArgumentException("MAXSwerve rotate motor MUST be a NEO 550!");


    return new REVSwerveModule(swerveHardware,
                               location,
                               SwerveModule.MountOrientation.STANDARD,
                               SwerveModule.MountOrientation.INVERTED,
                               gearRatio,
                               driveWheel, ZERO_OFFSET.get(location),
                               drivePID, driveFF,
                               rotatePID, rotateFF,
                               slipRatio, mass,
                               wheelbase, trackWidth,
                               autoLockTime, driveCurrentLimit);
  }
}
