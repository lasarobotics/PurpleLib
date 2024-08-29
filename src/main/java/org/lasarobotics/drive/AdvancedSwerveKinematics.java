// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.drive;

import static edu.wpi.first.math.Nat.N1;
import static edu.wpi.first.math.Nat.N2;
import static edu.wpi.first.math.Nat.N3;
import static edu.wpi.first.math.Nat.N4;

import org.lasarobotics.utils.GlobalConstants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;


/**
 * Advanced Swerve Kinematics
 * <p>
 * Provides functionality that can correct for second order kinematics
 */
public class AdvancedSwerveKinematics {

  /** Control centricity */
  public enum ControlCentricity {
    /**
     * Robot centric control
     */
    ROBOT_CENTRIC,
    /**
     * Field centric control
     */
    FIELD_CENTRIC;
  }

  private static final double EPS = 1E-9;
  private Translation2d[] m_moduleLocations;

  /**
    * Create a SecondOrderSwerveKinematics object
    * <p>
    * Corrects for path drift when the robot is rotating
    * @param moduleLocations Location of all 4 swerve modules, LF/RF/LR/RR
    */
  public AdvancedSwerveKinematics(Translation2d... moduleLocations) {
    if (moduleLocations.length < 2) throw new IllegalArgumentException("A swerve drive requires at least two modules");

    m_moduleLocations = moduleLocations;
  }

  /**
   * Obtain a new pose from a constant curvature velocity
   * @param delta Velocity
   * @return Pose
   */
  public static Pose2d exp(final Twist2d delta) {
    double sin_theta = Math.sin(delta.dtheta);
    double cos_theta = Math.cos(delta.dtheta);
    double s, c;
    if (Math.abs(delta.dtheta) < EPS) {
      s = 1.0 - 1.0 / 6.0 * delta.dtheta * delta.dtheta;
      c = .5 * delta.dtheta;
    } else {
      s = sin_theta / delta.dtheta;
      c = (1.0 - cos_theta) / delta.dtheta;
    }
    return new Pose2d(
      new Translation2d(delta.dx * s - delta.dy * c, delta.dx * c + delta.dy * s),
      new Rotation2d(cos_theta, sin_theta)
    );
  }

  /**
   * Obtain constant curvature velocity given pose
   * @param transform Pose
   * @return Velocity
   */
  private static Twist2d log(final Pose2d transform) {
    final double dtheta = transform.getRotation().getRadians();
    final double half_dtheta = 0.5 * dtheta;
    final double cos_minus_one = Math.cos(transform.getRotation().getRadians()) - 1.0;
    double halftheta_by_tan_of_halfdtheta;
    if (Math.abs(cos_minus_one) < EPS) halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
    else halftheta_by_tan_of_halfdtheta = -(half_dtheta * Math.sin(transform.getRotation().getRadians())) / cos_minus_one;

    final Translation2d translation_part =
      transform.getTranslation().rotateBy(new Rotation2d(halftheta_by_tan_of_halfdtheta, -half_dtheta));
    return new Twist2d(translation_part.getX(), translation_part.getY(), dtheta);
  }

  /**
   * Correct chassis speeds for second order kinematics
   * @param requestedSpeeds Requested chassis speeds
   * @return Corrected chassis speeds
   */
  public static ChassisSpeeds correctForDynamics(ChassisSpeeds requestedSpeeds) {
    Pose2d futureRobotPose = new Pose2d(
      requestedSpeeds.vxMetersPerSecond * GlobalConstants.ROBOT_LOOP_PERIOD,
      requestedSpeeds.vyMetersPerSecond * GlobalConstants.ROBOT_LOOP_PERIOD,
      Rotation2d.fromRadians(requestedSpeeds.omegaRadiansPerSecond * GlobalConstants.ROBOT_LOOP_PERIOD)
    );

    Twist2d twistForPose = log(futureRobotPose);

    ChassisSpeeds correctedSpeeds = new ChassisSpeeds(
      twistForPose.dx / GlobalConstants.ROBOT_LOOP_PERIOD,
      twistForPose.dy / GlobalConstants.ROBOT_LOOP_PERIOD,
      twistForPose.dtheta / GlobalConstants.ROBOT_LOOP_PERIOD
    );

    return correctedSpeeds;
  }

  public static SwerveModuleState getRealModuleSpeed(Translation2d moduleLocation, ChassisSpeeds desiredSpeed) {
    Matrix<N3, N1> firstOrderInputMatrix = new Matrix<>(N3(),N1());
    Matrix<N2, N3> firstOrderMatrix = new Matrix<>(N2(),N3());
    Matrix<N2, N2> rotationMatrix = new Matrix<>(N2(),N2());

    firstOrderInputMatrix.set(0, 0, desiredSpeed.vxMetersPerSecond);
    firstOrderInputMatrix.set(1, 0, desiredSpeed.vyMetersPerSecond);
    firstOrderInputMatrix.set(2, 0, desiredSpeed.omegaRadiansPerSecond);

    firstOrderMatrix.set(0, 0, 1);
    firstOrderMatrix.set(1, 1, 1);

    var swerveModuleState = new SwerveModuleState();

    // Angle that the module location vector makes with respect to the robot
    Rotation2d moduleAngle = new Rotation2d(Math.atan2(moduleLocation.getY(), moduleLocation.getX()));
    // Angle that the module location vector makes with respect to the field for field centric if applicable
    moduleAngle = Rotation2d.fromRadians(moduleAngle.getRadians());
    double moduleX = moduleLocation.getNorm() * Math.cos(moduleAngle.getRadians());
    double moduleY = moduleLocation.getNorm() * Math.sin(moduleAngle.getRadians());
    // -r_y
    firstOrderMatrix.set(0, 2, -moduleY);
    // +r_x
    firstOrderMatrix.set(1, 2, +moduleX);

    Matrix<N2, N1> firstOrderOutput = firstOrderMatrix.times(firstOrderInputMatrix);

    double moduleHeading = Math.atan2(firstOrderOutput.get(1, 0), firstOrderOutput.get(0, 0));
    double moduleSpeed = Math.sqrt(firstOrderOutput.elementPower(2).elementSum());

    rotationMatrix.set(0, 0, +Math.cos(moduleHeading));
    rotationMatrix.set(0, 1, +Math.sin(moduleHeading));
    rotationMatrix.set(1, 0, -Math.sin(moduleHeading));
    rotationMatrix.set(1, 1, +Math.cos(moduleHeading));

    // Set swerve module state
    swerveModuleState = new SwerveModuleState(moduleSpeed, Rotation2d.fromRadians(moduleHeading));

    return swerveModuleState;
  }

  /**
    * Convert chassis speed to states of individual modules using second order kinematics
    *
    * @param desiredSpeed Desired translation and rotation speed of the robot
    * @param robotHeading Heading of the robot relative to the field
    * @param controlCentricity Control centricity to use (field or robot centric)
    * @return Array of the speed direction of the swerve modules
    */
  public SwerveModuleState[] toSwerveModuleStates(ChassisSpeeds desiredSpeed, Rotation2d robotHeading, ControlCentricity controlCentricity) {
    Matrix<N3, N1> firstOrderInputMatrix = new Matrix<>(N3(),N1());
    Matrix<N2, N3> firstOrderMatrix = new Matrix<>(N2(),N3());
    Matrix<N4, N1> secondOrderInputMatrix = new Matrix<>(N4(),N1());
    Matrix<N2, N4> secondOrderMatrix = new Matrix<>(N2(),N4());
    Matrix<N2, N2> rotationMatrix = new Matrix<>(N2(),N2());

    firstOrderInputMatrix.set(0, 0, desiredSpeed.vxMetersPerSecond);
    firstOrderInputMatrix.set(1, 0, desiredSpeed.vyMetersPerSecond);
    firstOrderInputMatrix.set(2, 0, desiredSpeed.omegaRadiansPerSecond);

    secondOrderInputMatrix.set(2, 0, Math.pow(desiredSpeed.omegaRadiansPerSecond, 2));

    firstOrderMatrix.set(0, 0, 1);
    firstOrderMatrix.set(1, 1, 1);

    secondOrderMatrix.set(0, 0, 1);
    secondOrderMatrix.set(1, 1, 1);

    SwerveModuleState[] swerveModuleStates = new SwerveModuleState[m_moduleLocations.length];
    double[] moduleTurnSpeeds = new double[m_moduleLocations.length];

    for (int i = 0; i < m_moduleLocations.length; i++) {
      // Angle that the module location vector makes with respect to the robot
      Rotation2d moduleAngle = new Rotation2d(Math.atan2(m_moduleLocations[i].getY(), m_moduleLocations[i].getX()));
      // Angle that the module location vector makes with respect to the field for field centric if applicable
      moduleAngle = Rotation2d.fromRadians(moduleAngle.getRadians() + robotHeading.getRadians() * controlCentricity.ordinal());
      double moduleX = m_moduleLocations[i].getNorm() * Math.cos(moduleAngle.getRadians());
      double moduleY = m_moduleLocations[i].getNorm() * Math.sin(moduleAngle.getRadians());
      // -r_y
      firstOrderMatrix.set(0, 2, -moduleY);
      // +r_x
      firstOrderMatrix.set(1, 2, +moduleX);

      Matrix<N2, N1> firstOrderOutput = firstOrderMatrix.times(firstOrderInputMatrix);

      double moduleHeading = Math.atan2(firstOrderOutput.get(1, 0), firstOrderOutput.get(0, 0));
      double moduleSpeed = Math.sqrt(firstOrderOutput.elementPower(2).elementSum());

      secondOrderMatrix.set(0, 2, -moduleX);
      secondOrderMatrix.set(0, 3, -moduleY);
      secondOrderMatrix.set(1, 2, -moduleY);
      secondOrderMatrix.set(1, 3, +moduleX);

      rotationMatrix.set(0, 0, +Math.cos(moduleHeading));
      rotationMatrix.set(0, 1, +Math.sin(moduleHeading));
      rotationMatrix.set(1, 0, -Math.sin(moduleHeading));
      rotationMatrix.set(1, 1, +Math.cos(moduleHeading));

      Matrix<N2,N1> secondOrderOutput = rotationMatrix.times(secondOrderMatrix.times(secondOrderInputMatrix));

      // Correct module heading for control centricity
      moduleHeading -= robotHeading.getRadians() * controlCentricity.ordinal();
      swerveModuleStates[i] = new SwerveModuleState(moduleSpeed, Rotation2d.fromRadians(moduleHeading));
      moduleTurnSpeeds[i] = secondOrderOutput.get(1, 0) / moduleSpeed - desiredSpeed.omegaRadiansPerSecond;
    }

    return swerveModuleStates;
  }
}