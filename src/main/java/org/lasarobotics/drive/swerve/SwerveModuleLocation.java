// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.drive.swerve;

import org.lasarobotics.utils.GlobalConstants;

import edu.wpi.first.math.geometry.Rotation2d;

/** Module location */
public enum SwerveModuleLocation {
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

  private SwerveModuleLocation(Rotation2d revOffset, Rotation2d sdsOffset, Rotation2d wcpOffset, Rotation2d ttbOffset) {
    this.m_revOffset = revOffset;
    this.m_sdsOffset = sdsOffset;
    this.m_wcpOffset = wcpOffset;
    this.m_ttbOffset = ttbOffset;
  }

  /**
   * Get module offset for REV swerve module
   * @return Module orientation offset
   */
  public Rotation2d getREVOffset() {
    return m_revOffset;
  }

  /**
   * Get module offset for SDS swerve module
   * @return Module orientation offset
   */
  public Rotation2d getSDSOffset() {
    return m_sdsOffset;
  }

  /**
   * Get module offset for WCP swerve module
   * @return Module orientation offset
   */
  public Rotation2d getWCPOffset() {
    return m_wcpOffset;
  }

  /**
   * Get module offset for TTB swerve module
   * @return Module orientation offset
   */
  public Rotation2d getTTBOffset() {
    return m_ttbOffset;
  }
}
