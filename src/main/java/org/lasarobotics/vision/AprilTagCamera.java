// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.vision;

import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;

import org.lasarobotics.hardware.PurpleManager;
import org.lasarobotics.utils.GlobalConstants;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Create a camera */
public class AprilTagCamera implements AutoCloseable {
  public static final Object LOCK = new Object();

  private final double APRILTAG_POSE_AMBIGUITY_THRESHOLD = 0.1;
  private final double POSE_MAX_HEIGHT = 0.75;
  private final Measure<Distance> MAX_TAG_DISTANCE = Units.Meters.of(5.0);
  private final Measure<Distance> SINGLE_TO_MULTI_TAG_POSE_DELTA = Units.Meters.of(0.1);

  public static class Result {
    public final EstimatedRobotPose estimatedRobotPose;
    public final Matrix<N3, N1> standardDeviation;

    public Result(EstimatedRobotPose estimatedRobotPose, Matrix<N3, N1> standardDeviation) {
      this.estimatedRobotPose = estimatedRobotPose;
      this.standardDeviation = standardDeviation;
    }
  }

  public enum Resolution {
    RES_320_240(320, 240),
    RES_640_480(640, 480),
    RES_1280_720(1280, 720),
    RES_1280_800(1280, 800),
    RES_1920_1080(1920, 1080);

    public final int width;
    public final int height;

    private Resolution(int width, int height) {
      this.width = width;
      this.height = height;
    }
  }

  private PhotonCamera m_camera;
  private PhotonCameraSim m_cameraSim;
  private PhotonPoseEstimator m_poseEstimator;
  private Transform3d m_transform;
  private AprilTagFieldLayout m_fieldLayout;
  private Notifier m_thread;
  private AtomicReference<AprilTagCamera.Result> m_latestResult;

  /**
   * Create VisionCamera
   * @param name Name of device
   * @param transform Location on robot in meters
   * @param resolution Resolution used by camera
   * @param fovDiag Diagonal FOV of camera
   */
  public AprilTagCamera(String name, Transform3d transform, Resolution resolution, Rotation2d fovDiag, AprilTagFieldLayout fieldLayout) {
    this.m_camera = new PhotonCamera(name);
    this.m_transform = transform;
    this.m_fieldLayout = fieldLayout;
    // PV estimates will always be blue
    m_fieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    this.m_poseEstimator = new PhotonPoseEstimator(m_fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_camera, m_transform);
    m_poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    this.m_latestResult = new AtomicReference<AprilTagCamera.Result>();

    // Create simulated AprilTag camera
    var cameraProperties = SimCameraProperties.PERFECT_90DEG();
    cameraProperties.setCalibration(resolution.width, resolution.height, fovDiag);
    this.m_cameraSim = new PhotonCameraSim(m_camera, cameraProperties);

    // Enable wireframe in sim camera stream
    m_cameraSim.enableDrawWireframe(true);

    // Register camera sim with PurpleManager
    PurpleManager.addCameraSim(m_cameraSim, m_transform);

    // Start camera thread
    this.m_thread = new Notifier(this::run);
    m_thread.startPeriodic(GlobalConstants.ROBOT_LOOP_PERIOD);
  }

  /**
   * Check if pose is valid
   * @param pose Pose to check
   * @return True if pose is valid
   */
  private boolean isPoseValid(Pose3d pose) {
    // Make sure pose is on the field
    if (pose.getX() < 0.0 || pose.getX() > m_fieldLayout.getFieldLength()
     || pose.getY() < 0.0 || pose.getY() > m_fieldLayout.getFieldWidth()) return false;

    // Make sure pose is near the floor
    if (pose.getZ() < 0.0 || pose.getZ() > POSE_MAX_HEIGHT) return false;

    // Pose is acceptable
    return true;
  }

  /**
   * Get standard deviation
   * @param closestTagDistance Distance to closest tag
   * @param numTargetsUsed Number of tags used for pose estimate
   * @return Standard deviation of measurement
   */
  private double getStandardDeviation(Measure<Distance> closestTagDistance, int numTagsUsed) {
    return 0.01 * Math.pow(closestTagDistance.in(Units.Meters), 2.0) / numTagsUsed;
  }

  /**
   * Run main camera logic
   */
  private void run() {
    // Return if camera or field layout failed to load
    if (m_poseEstimator == null || m_camera == null) return;

    // Put camera connected indicator on SmartDashboard
    SmartDashboard.putBoolean(m_camera.getName(), m_camera.isConnected());

    // Return if camera is not connected
    if (!m_camera.isConnected()) return;

    // Update and log inputs
    PhotonPipelineResult pipelineResult = m_camera.getLatestResult();

    // Return if result is non-existent or invalid
    if (!pipelineResult.hasTargets()) return;
    if (pipelineResult.targets.size() == 1
        && pipelineResult.targets.get(0).getPoseAmbiguity() > APRILTAG_POSE_AMBIGUITY_THRESHOLD) return;

    // Update pose estimate
    m_poseEstimator.update(pipelineResult).ifPresent(estimatedRobotPose -> {
      // Make sure the measurement is valid
      if (!isPoseValid(estimatedRobotPose.estimatedPose)) return;

      // Get distance to closest tag
      var closestTagDistance = Units.Meters.of(100.0);
      // Loop through all targets used for this estimate
      for (var target : estimatedRobotPose.targetsUsed) {
        // Get tag
        var tag = getTag(target.getFiducialId());
        // Get distance to tag
        var tagDistance = Units.Meters.of(target.getBestCameraToTarget().getTranslation().getNorm());
        // Get pose estimate based on just this tag
        var singleTargetPose = tag.get().pose
                              .transformBy(target.getBestCameraToTarget().inverse())
                              .transformBy(m_transform.inverse());
        // Ignore if single tag pose estimate is too far from multi-tag estimate
        if (estimatedRobotPose.estimatedPose.relativeTo(singleTargetPose).getTranslation().getNorm() >
            SINGLE_TO_MULTI_TAG_POSE_DELTA.in(Units.Meters)) {
          m_latestResult.set(null);
          return;
        }
        // Check if tag distance is closest yet
        if (tagDistance.lte(closestTagDistance)) closestTagDistance = tagDistance;
      }

      // Ignore if tags are too far
      if (closestTagDistance.gte(MAX_TAG_DISTANCE)) return;

      // Calculate standard deviation
      double xyStdDev = getStandardDeviation(closestTagDistance, estimatedRobotPose.targetsUsed.size());
      double thetaStdDev = RobotState.isDisabled()
        ? xyStdDev
        : Double.MAX_VALUE;

      // Set result
      var result = new AprilTagCamera.Result(
        estimatedRobotPose,
        VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)
      );
      m_latestResult.set(result);
    });
  }

  /**
   * Gets the latest robot pose. Calling this will only return the pose once.
   * If it returns a non-null value, it is a new estimate that hasn't been returned before.
   * This pose will always be for the BLUE alliance.
   * @return Latest estimated pose
   */
  public AprilTagCamera.Result getLatestEstimatedPose() {
    return m_latestResult.getAndSet(null);
  }

/**
  * Get AprilTag
  * @param id Fiducial ID
  * @return AprilTag object
  */
  public Optional<AprilTag> getTag(int id) {
    return m_fieldLayout.getTags().stream().filter((tag) -> tag.ID == id).findFirst();
  }

  /**
   * Allows user to select the active pipeline index
   * @param index The active pipeline index
   */
  public void setPipelineIndex(int index) {
    m_camera.setPipelineIndex(index);
  }

  /**
   * Get camera to robot transform
   * @return Camera to robot transform
   */
  public Transform3d getTransform() {
    return m_transform;
  }

  @Override
  public void close() {
    m_thread.close();
    m_camera.close();
  }
}
