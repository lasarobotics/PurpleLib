// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.drive.swerve;

import org.lasarobotics.utils.GlobalConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.LinearVelocity;

public interface SwerveModule {

  /** Module vendor */
  public enum Vendor {
    REV(true, true),
    SDS(false, true),
    WCP(false, true),
    TTB(false, false);

    private final boolean standardSensorPhase;
    private final boolean invertedSensorPhase;
    private Vendor(boolean standardSensorPhase, boolean invertedSensorPhase) {
      this.standardSensorPhase = standardSensorPhase;
      this.invertedSensorPhase = invertedSensorPhase;
    }

    public boolean getSensorPhase(MountOrientation orientation) {
      return orientation.equals(MountOrientation.STANDARD) ? standardSensorPhase : invertedSensorPhase;
    }
  }

  /** Mount orientation */
  public enum MountOrientation {
    STANDARD, INVERTED;
  }

  /** Module location */
  public enum Location {
    LeftFront(
      GlobalConstants.ROTATION_PI.div(2).unaryMinus(),
      GlobalConstants.ROTATION_PI.div(2).unaryMinus(),
      GlobalConstants.ROTATION_PI.div(4),
      GlobalConstants.ROTATION_PI.div(2).unaryMinus()
    ),
    RightFront(
      GlobalConstants.ROTATION_ZERO,
      GlobalConstants.ROTATION_ZERO,
      GlobalConstants.ROTATION_PI.div(4),
      GlobalConstants.ROTATION_ZERO
    ),
    LeftRear(
      GlobalConstants.ROTATION_PI,
      GlobalConstants.ROTATION_PI,
      GlobalConstants.ROTATION_PI.div(4),
      GlobalConstants.ROTATION_PI
    ),
    RightRear(
      GlobalConstants.ROTATION_PI.div(2),
      GlobalConstants.ROTATION_PI.div(2),
      GlobalConstants.ROTATION_PI.div(4),
      GlobalConstants.ROTATION_PI.div(2)
    );

    /** Module orientation chassis offset */
    private final Rotation2d m_revOffset;
    private final Rotation2d m_sdsOffset;
    private final Rotation2d m_wcpOffset;
    private final Rotation2d m_ttbOffset;

    private Location(Rotation2d revOffset, Rotation2d sdsOffset, Rotation2d wcpOffset, Rotation2d ttbOffset) {
      this.m_revOffset = revOffset;
      this.m_sdsOffset = sdsOffset;
      this.m_wcpOffset = wcpOffset;
      this.m_ttbOffset = ttbOffset;
    }

    /**
     * Get module offset for vendor
     * @param vendor Vendor of swerve module
     * @return Module calibration offset
     */
    public Rotation2d getOffset(SwerveModule.Vendor vendor) {
      switch (vendor) {
        case REV:
          return m_revOffset;
        case SDS:
          return m_sdsOffset;
        case WCP:
          return m_wcpOffset;
        case TTB:
          return m_ttbOffset;
        default:
          return GlobalConstants.ROTATION_ZERO;
      }
    }
  }

  public abstract void set(SwerveModuleState state);

  public abstract SwerveModuleState getState();

  public abstract SwerveModulePosition getPosition();

  public abstract SwerveModule.Location getModuleLocation();

  public abstract Translation2d getModuleCoordinate();

  public abstract LinearVelocity getDriveVelocity();
}
