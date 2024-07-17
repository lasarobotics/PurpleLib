// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.drive;

import java.time.Duration;
import java.time.Instant;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.lasarobotics.hardware.PurpleManager;
import org.lasarobotics.hardware.ctre.Pigeon2;
import org.lasarobotics.hardware.kauailabs.NavX2;
import org.lasarobotics.utils.GlobalConstants;
import org.lasarobotics.vision.AprilTagCamera;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;

/** Swerve Odometry Service */
public class SwervePoseEstimatorService {

  @AutoLog
  public static class SwervePoseEstimatorServiceInputs {
    Pose2d currentPose = new Pose2d();
  }

  private static final Matrix<N3,N1> VISION_STDDEV = VecBuilder.fill(1.0, 1.0, Math.toRadians(3.0));
  private static final Measure<Time> DEFAULT_THREAD_PERIOD = Units.Milliseconds.of(10.0);
  private static final Measure<Velocity<Angle>> VISION_ANGULAR_VELOCITY_THRESHOLD = Units.DegreesPerSecond.of(720.0);
  private static final String NAME = "SwervePoseEstimatorService";
  private static final String VISIBLE_TAGS_LOG_ENTRY = "/Vision/VisibleTags";
  private static final String ESTIMATED_POSES_LOG_ENTRY = "/Vision/EstimatedPoses";

  private boolean m_running;
  private Supplier<Rotation2d> m_rotation2dSupplier;
  private Supplier<Measure<Velocity<Angle>>> m_yawRateSupplier;
  private Supplier<SwerveModulePosition[]> m_swerveModulePositionSupplier;
  private Consumer<Pose2d> m_poseResetMethod;
  private SwerveDrivePoseEstimator m_poseEstimator;
  private List<AprilTagCamera> m_cameras;
  private Measure<Time> m_threadPeriod;
  private Instant m_lastVisionUpdateTime;
  private Notifier m_thread;

  private volatile List<Pose2d> m_visionEstimatedPoses;
  private volatile List<AprilTag> m_visibleTags;
  private volatile List<Pose3d> m_visibleTagPoses;
  private volatile SwervePoseEstimatorServiceInputsAutoLogged m_pose;

  /**
   * Create Swerve Pose Estimator Service
   * <p>
   * This service runs in the background and keeps track of where the robot is on the field
   * @param odometryStdDev Standard deviation of wheel odometry measurements
   * @param imu NavX2 MXP IMU
   * @param modules MAXSwerve modules
   */
  public SwervePoseEstimatorService(Matrix<N3,N1> odometryStdDev, NavX2 imu, SwerveModule... modules) {
    this(odometryStdDev, () -> imu.getInputs().rotation2d, () -> imu.getInputs().yawRate, modules);
  }

  /**
   * Create Swerve Pose Estimator Service
   * <p>
   * This service runs in the background and keeps track of where the robot is on the field
   * @param odometryStdDev Standard deviation of wheel odometry measurements
   * @param imu CTRE Pigeon 2.0 IMU
   * @param modules MAXSwerve modules
   */
  public SwervePoseEstimatorService(Matrix<N3,N1> odometryStdDev, Pigeon2 imu, SwerveModule... modules) {
    this(odometryStdDev, () -> imu.getInputs().rotation2d, () -> imu.getInputs().yawRate, modules);
  }

  private SwervePoseEstimatorService(Matrix<N3,N1> odometryStdDev,
                                     Supplier<Rotation2d> rotation2dSupplier,
                                     Supplier<Measure<Velocity<Angle>>> yawRateSupplier,
                                     SwerveModule... modules) {
    if (modules.length != 4) throw new IllegalArgumentException("Four (4) modules must be used!");
    this.m_running = false;
    // Remember how to get rotation2d from IMU
    this.m_rotation2dSupplier = rotation2dSupplier;
    // Remember how to get yaw rate from IMU
    this.m_yawRateSupplier = yawRateSupplier;

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

    // Register callback with PurpleManager
    PurpleManager.addCallback(() -> periodic());

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
    this.m_pose = new SwervePoseEstimatorServiceInputsAutoLogged();

    // Initialise camera list
    this.m_cameras = new ArrayList<AprilTagCamera>();

    // Initialise vision variables
    this.m_visibleTags = new ArrayList<AprilTag>();
    this.m_visibleTagPoses = new ArrayList<Pose3d>();
    this.m_visionEstimatedPoses = new ArrayList<Pose2d>();

    // Initialise pose estimator thread
    this.m_thread = new Notifier(() -> {
      // If no cameras or yaw rate is too high, just update pose based on odometry and exit
      if (m_cameras.isEmpty() || m_yawRateSupplier.get().gte(VISION_ANGULAR_VELOCITY_THRESHOLD)) {
        m_pose.currentPose = m_poseEstimator.update(m_rotation2dSupplier.get(), m_swerveModulePositionSupplier.get());
        return;
      }

      // Add AprilTag pose estimates if available
      for (var camera : m_cameras) {
        var result = camera.getLatestEstimatedPose();

        // If no updated vision pose estimate, continue
        if (result == null) continue;

        // Save estimated pose and visible tags for logging on main thread
        m_visionEstimatedPoses.add(result.estimatedRobotPose.estimatedPose.toPose2d());
        result.estimatedRobotPose.targetsUsed.forEach(photonTrackedTarget -> {
          var tag = camera.getTag(photonTrackedTarget.getFiducialId());
          if (tag.isPresent()) {
            m_visibleTags.add(tag.get());
            m_visibleTagPoses.add(tag.get().pose);
          }
        });
        // Add vision measurement
        m_poseEstimator.addVisionMeasurement(
          result.estimatedRobotPose.estimatedPose.toPose2d(),
          result.estimatedRobotPose.timestampSeconds,
          result.standardDeviation
        );

        // Update last vision update time
        m_lastVisionUpdateTime = Instant.now();
      }
      // Update current pose
      m_pose.currentPose = m_poseEstimator.update(m_rotation2dSupplier.get(), m_swerveModulePositionSupplier.get());

      // Clear vision logging variables if its been a while since last update
      if (Duration.between(m_lastVisionUpdateTime, Instant.now()).toMillis() / 1000.0 > GlobalConstants.ROBOT_LOOP_PERIOD) {
        m_visibleTags = new ArrayList<AprilTag>();
        m_visibleTagPoses = new ArrayList<Pose3d>();
        m_visionEstimatedPoses = new ArrayList<Pose2d>();
      }
    });
    m_thread.setName(NAME);

    // Remember how to reset pose
    this.m_poseResetMethod = pose -> m_poseEstimator.resetPosition(m_rotation2dSupplier.get(), m_swerveModulePositionSupplier.get(), pose);

    // Register service as pose supplier with PurpleManager for simulation
    PurpleManager.setPoseSupplier(this::getPose);

    // Set thread period to default
    this.m_threadPeriod = DEFAULT_THREAD_PERIOD;

    // Set last vision update time
    this.m_lastVisionUpdateTime = Instant.now();

    // Set period if sim
    if (RobotBase.isSimulation()) setPeriod(Units.Seconds.of(GlobalConstants.ROBOT_LOOP_PERIOD));
  }

  /**
   * Call this method periodically
   */
  private void periodic() {
    Logger.processInputs(NAME, m_pose);
    Logger.recordOutput(NAME + VISIBLE_TAGS_LOG_ENTRY, m_visibleTagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(NAME + ESTIMATED_POSES_LOG_ENTRY, m_visionEstimatedPoses.toArray(new Pose2d[0]));
  }

  /**
   * Set how frequently the pose estimator should update
   * <p>
   * Defaults to 10ms if not set (100Hz), 20ms for simulation (50Hz)
   * @param period Period between pose estimator updates
   */
  public void setPeriod(Measure<Time> period) {
    m_threadPeriod = period;
  }

  /**
   * Add AprilTag camera to pose estimator service
   * <p>
   * Service will automatically query the camera for vision pose estimates
   * and add them to the pose estimator
   * @param cameras
   */
  public void addAprilTagCamera(AprilTagCamera... cameras) {
    m_cameras.addAll(Arrays.asList(cameras));
  }

  /**
   * Start pose estimator thread
   */
  public void start() {
    m_running = true;
    if (Logger.hasReplaySource()) return;
    m_thread.startPeriodic(m_threadPeriod.in(Units.Seconds));
  }

  /**
   * Stop pose estimator thread
   */
  public void stop() {
    m_running = false;
    if (Logger.hasReplaySource()) return;
    m_thread.stop();
  }

  /**
   * Get latest estimated pose from service
   * @return Latest robot pose estimate
   */
  public Pose2d getPose() {
    return m_pose.currentPose;
  }

  /**
   * Reset pose estimator
   * @param pose Pose to reset to
   */
  public void resetPose(Pose2d pose) {
    // Remember if service was running and stop if it is
    var wasRunning = m_running;
    if (m_running) stop();
    // Reset pose estimator to desired pose
    m_poseResetMethod.accept(pose);
    m_pose.currentPose = pose;
    // Restart service if it was previously running
    if (wasRunning) start();
  }

  /**
   * Get currently visible AprilTags
   * @return List of currently visible tags
   */
  public List<AprilTag> getVisibleTags() {
    return m_visibleTags;
  }
}
