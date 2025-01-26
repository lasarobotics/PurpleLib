// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.drive;

import java.time.Duration;
import java.time.Instant;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.lasarobotics.drive.swerve.SwerveModule;
import org.lasarobotics.hardware.IMU;
import org.lasarobotics.hardware.PurpleManager;
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
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;

/** Swerve Odometry Service */
public class SwervePoseEstimatorService {

  @AutoLog
  public static class SwervePoseEstimatorServiceInputs {
    Pose2d currentPose = new Pose2d();
  }

  private static final ScheduledExecutorService POSE_ESTIMATION_EXECUTOR = Executors.newSingleThreadScheduledExecutor();
  private static final Matrix<N3,N1> VISION_STDDEV = VecBuilder.fill(1.0, 1.0, Math.toRadians(3.0));
  private static final AngularVelocity VISION_ANGULAR_VELOCITY_THRESHOLD = Units.DegreesPerSecond.of(720.0);
  private static final String NAME = "SwervePoseEstimatorService";
  private static final String VISIBLE_TAGS_LOG_ENTRY = "/Vision/VisibleTags";
  private static final String ESTIMATED_POSES_LOG_ENTRY = "/Vision/EstimatedPoses";

  private boolean m_running;
  private IMU m_imu;
  private Supplier<SwerveModulePosition[]> m_swerveModulePositionSupplier;
  private Consumer<Pose2d> m_poseResetMethod;
  private SwerveDriveKinematics m_kinematics;
  private SwerveDrivePoseEstimator m_poseEstimator;
  private List<AprilTagCamera> m_cameras;
  private Time m_threadPeriod;
  private Instant m_lastVisionUpdateTime;
  private Runnable m_task;

  private volatile double m_currentTimestamp;
  private volatile double m_previousTimestamp;
  private volatile List<Pose2d> m_visionEstimatedPoses;
  private volatile List<AprilTag> m_visibleTags;
  private volatile List<Pose3d> m_visibleTagPoses;
  private volatile SwervePoseEstimatorServiceInputsAutoLogged m_pose;
  private volatile Rotation2d m_yawAngle = Rotation2d.fromRadians(0.0);
  private volatile SwerveModulePosition[] m_previousModulePositions = new SwerveModulePosition[4];

  /**
   * Create Swerve Pose Estimator Service
   * <p>
   * This service runs in the background and keeps track of where the robot is on the field
   * @param odometryStdDev Standard deviation of wheel odometry measurements
   * @param imu IMU installed on robot
   * @param modules Swerve modules
   */
  public SwervePoseEstimatorService(Matrix<N3,N1> odometryStdDev, IMU imu, SwerveModule... modules) {
    if (modules.length != 4) throw new IllegalArgumentException("Four (4) modules must be used!");
    this.m_imu = imu;
    this.m_running = false;

    // Get each individual module
    var moduleList = Arrays.asList(modules);
    var lFrontModule = moduleList.stream().filter(module -> module.getModuleLocation().equals(SwerveModule.Location.LeftFront)).findFirst();
    var rFrontModule = moduleList.stream().filter(module -> module.getModuleLocation().equals(SwerveModule.Location.RightFront)).findFirst();
    var lRearModule = moduleList.stream().filter(module -> module.getModuleLocation().equals(SwerveModule.Location.LeftRear)).findFirst();
    var rRearModule = moduleList.stream().filter(module -> module.getModuleLocation().equals(SwerveModule.Location.RightRear)).findFirst();

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

    // Get initial swerve positions
    this.m_previousModulePositions = m_swerveModulePositionSupplier.get();

    // Initialise kinematics
    this.m_kinematics = new SwerveDriveKinematics(
      lFrontModule.get().getModuleCoordinate(),
      rFrontModule.get().getModuleCoordinate(),
      lRearModule.get().getModuleCoordinate(),
      rRearModule.get().getModuleCoordinate()
    );

    // Get update rate
    var updateRate = moduleList.stream()
      .min((module1, module2) -> Double.compare(module1.getUpdateRate().in(Units.Hertz), module2.getUpdateRate().in(Units.Hertz)))
      .get().getUpdateRate();
    updateRate = Units.Hertz.of(Math.min(updateRate.in(Units.Hertz), imu.getUpdateRate().in(Units.Hertz)));

    // Register callback with PurpleManager
    PurpleManager.addCallback(() -> periodic());

    // Initialise pose estimator
    this.m_poseEstimator = new SwerveDrivePoseEstimator(
      m_kinematics,
      m_imu.getRotation2d(),
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
    this.m_task = () -> {
      // Iterate timestamp
      m_previousTimestamp = m_currentTimestamp;
      m_currentTimestamp = RobotController.getFPGATime();

      // Check if IMU is connected
      boolean isIMUConnected = m_imu.isConnected();

      // Get synchronized swerve module positions
      var currentModulePositions = m_swerveModulePositionSupplier.get();

      // Update yaw angle and yaw rate, using module deltas if IMU is not available
      var moduleDeltas = new SwerveModulePosition[4];
      for (int i = 0; i < moduleDeltas.length; i++) {
        moduleDeltas[i] = new SwerveModulePosition(
          currentModulePositions[i].distanceMeters - m_previousModulePositions[i].distanceMeters,
          currentModulePositions[i].angle
        );
        m_previousModulePositions[i] = currentModulePositions[i];
      }
      var previousYawAngle = m_yawAngle;
      m_yawAngle = (isIMUConnected) ? m_imu.getRotation2d() :
        m_yawAngle.plus(new Rotation2d(m_kinematics.toTwist2d(moduleDeltas).dtheta));
      var yawRate = (isIMUConnected) ? m_imu.getYawRate() :
        Units.Radians.of(m_yawAngle.minus(previousYawAngle).getRadians()).div(Units.Microseconds.of(m_currentTimestamp - m_previousTimestamp));

      // If no cameras or yaw rate is too high, just update pose based on odometry and exit
      if (m_cameras.isEmpty() || yawRate.gte(VISION_ANGULAR_VELOCITY_THRESHOLD)) {
        m_pose.currentPose = m_poseEstimator.updateWithTime(m_currentTimestamp / 1e6, m_yawAngle, m_swerveModulePositionSupplier.get());
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
      m_pose.currentPose = m_poseEstimator.updateWithTime(m_currentTimestamp / 1e6, m_yawAngle, m_swerveModulePositionSupplier.get());

      // Clear vision logging variables if its been a while since last update
      if (Duration.between(m_lastVisionUpdateTime, Instant.now()).toMillis() / 1000.0 > GlobalConstants.ROBOT_LOOP_HZ.asPeriod().in(Units.Seconds)) {
        m_visibleTags = new ArrayList<AprilTag>();
        m_visibleTagPoses = new ArrayList<Pose3d>();
        m_visionEstimatedPoses = new ArrayList<Pose2d>();
      }
    };

    // Remember how to reset pose
    this.m_poseResetMethod = pose -> m_poseEstimator.resetPosition(m_imu.getRotation2d(), m_swerveModulePositionSupplier.get(), pose);

    // Register service as pose supplier with PurpleManager for simulation
    PurpleManager.setPoseSupplier(this::getPose);

    // Set thread period to default
    this.m_threadPeriod = updateRate.asPeriod();

    // Set last vision update time
    this.m_lastVisionUpdateTime = Instant.now();

    // Set period if sim
    if (RobotBase.isSimulation()) setPeriod(GlobalConstants.ROBOT_LOOP_HZ.asPeriod());
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
   * Defaults to minimum update rate of swerve modules and IMU
   * @param period Period between pose estimator updates
   */
  public void setPeriod(Time period) {
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
    POSE_ESTIMATION_EXECUTOR.scheduleAtFixedRate(m_task, 0, (long)m_threadPeriod.in(Units.Microseconds), java.util.concurrent.TimeUnit.MICROSECONDS);
  }

  /**
   * Stop pose estimator thread
   */
  public void stop() {
    m_running = false;
    if (Logger.hasReplaySource()) return;
    POSE_ESTIMATION_EXECUTOR.shutdownNow();
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
