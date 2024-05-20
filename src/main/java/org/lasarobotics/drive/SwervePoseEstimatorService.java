// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.drive;

import java.util.Arrays;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

import org.lasarobotics.drive.MAXSwerveModule.ModuleLocation;
import org.lasarobotics.hardware.ctre.Pigeon2;
import org.lasarobotics.hardware.kauailabs.NavX2;
import org.lasarobotics.vision.AprilTagCameraResult;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Notifier;

/** Swerve Odometry Service */
public class SwervePoseEstimatorService {
  private static final Matrix<N3,N1> VISION_STDDEV = VecBuilder.fill(1.0, 1.0, Math.toRadians(3.0));
  private static final Measure<Time> DEFAULT_THREAD_PERIOD = Units.Milliseconds.of(5.0);

  private Supplier<Rotation2d> m_rotation2dSupplier;
  private Supplier<SwerveModulePosition[]> m_swerveModulePositionSupplier;
  private SwerveDrivePoseEstimator m_poseEstimator;
  private AtomicReference<Pose2d> m_currentPose;
  private AtomicReference<List<AprilTagCameraResult>> m_apriltagCameraResults;
  private Measure<Time> m_threadPeriod;
  private Notifier m_thread;

  /**
   * Create Swerve Pose Estimator Service
   * <p>
   * This service runs in the background and keeps track of where the robot is on the field
   * @param odometryStdDev Standard deviation of wheel odometry measurements
   * @param imu NavX2 MXP IMU
   * @param modules MAXSwerve modules
   */
  public SwervePoseEstimatorService(Matrix<N3,N1> odometryStdDev, NavX2 imu, MAXSwerveModule... modules) {
    this(odometryStdDev, () -> imu.getInputs().rotation2d, modules);
  }

  /**
   * Create Swerve Pose Estimator Service
   * <p>
   * This service runs in the background and keeps track of where the robot is on the field
   * @param odometryStdDev Standard deviation of wheel odometry measurements
   * @param imu CTRE Pidgeon 2.0 IMU
   * @param modules MAXSwerve modules
   */
  public SwervePoseEstimatorService(Matrix<N3,N1> odometryStdDev, Pigeon2 imu, MAXSwerveModule... modules) {
    this(odometryStdDev, () -> imu.getInputs().rotation2d, modules);
  }

  private SwervePoseEstimatorService(Matrix<N3,N1> odometryStdDev, Supplier<Rotation2d> rotation2dSupplier, MAXSwerveModule... modules) {
    if (modules.length != 4) throw new IllegalArgumentException("Four (4) modules must be used!");
    // Remember how to get rotation2d from IMU
    this.m_rotation2dSupplier = rotation2dSupplier;

    // Get each individual module
    var moduleList = Arrays.asList(modules);
    var lFrontModule = moduleList.stream().filter(module -> module.getModuleLocation().equals(ModuleLocation.LeftFront)).findFirst();
    var rFrontModule = moduleList.stream().filter(module -> module.getModuleLocation().equals(ModuleLocation.RightFront)).findFirst();
    var lRearModule = moduleList.stream().filter(module -> module.getModuleLocation().equals(ModuleLocation.LeftRear)).findFirst();
    var rRearModule = moduleList.stream().filter(module -> module.getModuleLocation().equals(ModuleLocation.RightRear)).findFirst();

    // Make sure each module is available
    if (lFrontModule.isEmpty()) throw new IllegalArgumentException("Left front module missing!");
    if (rFrontModule.isEmpty()) throw new IllegalArgumentException("Right front module missing!");
    if (lRearModule.isEmpty()) throw new IllegalArgumentException("Left rear module missing!");
    if (rRearModule.isEmpty()) throw new IllegalArgumentException("Right rear module missing!");

    // Remember how to get swerve module positions from each module
    this.m_swerveModulePositionSupplier = () -> new SwerveModulePosition[] {
      lFrontModule.get().getPosition(),
      rFrontModule.get().getPosition(),
      lRearModule.get().getPosition(),
      rRearModule.get().getPosition()
    };

    // Initialise kinematics
    var kinematics = new SwerveDriveKinematics(
      lFrontModule.get().getModuleCoordinate(),
      rFrontModule.get().getModuleCoordinate(),
      lRearModule.get().getModuleCoordinate(),
      rRearModule.get().getModuleCoordinate()
    );

    // Initialise pose estimator
    this.m_poseEstimator = new SwerveDrivePoseEstimator(
      kinematics,
      m_rotation2dSupplier.get(),
      m_swerveModulePositionSupplier.get(),
      new Pose2d(),
      odometryStdDev,
      VISION_STDDEV
    );

    // Initialise current pose and AprilTag camera results
    this.m_currentPose = new AtomicReference<Pose2d>(new Pose2d());
    this.m_apriltagCameraResults = new AtomicReference<List<AprilTagCameraResult>>(List.of());

    // Initialise pose estimator thread
    this.m_thread = new Notifier(() -> {
      // Add AprilTag pose estimates if available
      var apriltagCameraResults = m_apriltagCameraResults.get();
      if (!apriltagCameraResults.isEmpty()) {
        for (var result : apriltagCameraResults) {
          m_poseEstimator.addVisionMeasurement(
            result.estimatedRobotPose.estimatedPose.toPose2d(),
            result.estimatedRobotPose.timestampSeconds,
            result.standardDeviation
          );
        }
        m_apriltagCameraResults.set(List.of());
      }
      // Update current pose
      m_currentPose.set(m_poseEstimator.update(m_rotation2dSupplier.get(), m_swerveModulePositionSupplier.get()));
    });
    this.m_threadPeriod = DEFAULT_THREAD_PERIOD;
  }

  /**
   * Set how frequently the pose estimator should update
   * <p>
   * Defaults to 5ms if not set (200Hz)
   * @param period Period between pose estimator updates
   */
  public void setPeriod(Measure<Time> period) {
    m_threadPeriod = period;
  }

  /**
   * Add AprilTag camera pose estimate
   * @param result AprilTag camera pose estimate result
   */
  public void addAprilTagResult(AprilTagCameraResult result) {
    m_apriltagCameraResults.getAcquire().add(result);
  }

  /**
   * Start pose estimator thread
   */
  public void start() {
    m_thread.startPeriodic(m_threadPeriod.in(Units.Seconds));
  }

  /**
   * Stop pose estimator thread
   */
  public void stop() {
    m_thread.stop();
  }
}
