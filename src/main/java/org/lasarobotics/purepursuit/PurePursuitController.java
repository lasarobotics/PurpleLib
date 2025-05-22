package org.lasarobotics.purepursuit;

import java.util.ArrayList;
// Assuming your Pose2d, Rotation2d, Translation2d, ChassisSpeeds, Path, Path.Segment classes exist
import java.util.List;
import java.util.Optional;

import org.lasarobotics.utils.GlobalConstants;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;

public class PurePursuitController {
  private List<Path.Segment> m_remainingSegments; // The dynamic list of segments to follow
  private Pose2d m_finalWaypointOfOriginalPath; // To know the ultimate destination for final slowdown/stop
  private double m_waypointCompletionProximity = 5e-3; // How close to an endpoint to consider it for removal (meters)

  private double m_lookaheadDistance;
  private double m_maxVelocity;
  private double m_acceleration;
  private double m_previousDesiredSpeed;

  private int m_lastLookaheadFoundOnSegmentIndex = 0; // Index relative to the current activeSegments list

  private final double m_segmentPassProximityMultiplier = 2.0; // Multiplier for lookaheadDistance for segment pass check
  // Parameters for slowdown based on angle between upcoming path segments
  private final double pathAngleThresholdForSlowingDegrees = 20.0; // Start slowing if turn angle > this (e.g., 20 degrees)
  private final double pathMaxAngleForMaxSlowdownDegrees = 75.0;   // Angle at which max slowdown is applied (e.g., 75 degrees)
  private final double pathMaxSpeedReductionForTurns = 0.7; // Max speed reduction factor (e.g., speed becomes 1.0 - 0.7 = 30% of current)

  private final ProfiledPIDController m_rotationController = new ProfiledPIDController(5.0, 0.0, 0.0, new TrapezoidProfile.Constraints(Math.PI * 4, Math.PI * 3));

  public PurePursuitController(Path originalPath, double lookaheadDistance, double maxVelocity, double acceleration) {
    if (originalPath == null || originalPath.getSegments().isEmpty()) {
      throw new IllegalArgumentException("Original path cannot be null or empty.");
    }
    // Create a mutable copy of the segments
    this.m_remainingSegments = new ArrayList<>(originalPath.getSegments());
    this.m_finalWaypointOfOriginalPath = originalPath.getEndWaypoint(); // Store the original final waypoint

    this.m_lookaheadDistance = lookaheadDistance;
    this.m_maxVelocity = maxVelocity;
    this.m_acceleration = acceleration;

    this.m_waypointCompletionProximity = Math.min(m_waypointCompletionProximity, m_lookaheadDistance);
    this.m_previousDesiredSpeed = 0.0;
  }

  /**
  * Allows resetting or setting a new path.
  */
  public void setPath(Path newPath) {
    if (newPath == null || newPath.getSegments().isEmpty()) {
      throw new IllegalArgumentException("New path cannot be null or empty.");
    }
    this.m_remainingSegments = new ArrayList<>(newPath.getSegments());
    this.m_finalWaypointOfOriginalPath = newPath.getEndWaypoint();
    this.m_lastLookaheadFoundOnSegmentIndex = 0; // Reset search index
  }

  private boolean hasRobotPassedSegmentEndpoint(Translation2d robotPos, Path.Segment segment) {
    if (segment.length < 1e-6) { // Segment is essentially a point
      return robotPos.getDistance(segment.endTranslation) < m_waypointCompletionProximity;
    }

    Translation2d startToRobotVector = robotPos.minus(segment.startTranslation);
    double dotProduct = startToRobotVector.getX() * segment.segmentVector.getX() +
    startToRobotVector.getY() * segment.segmentVector.getY();

    double projectionFactor = dotProduct / (segment.length * segment.length);

    // Robot's projection is past the endpoint, AND robot is reasonably close to the endpoint's vicinity
    double distanceToEndPoint = robotPos.getDistance(segment.end.getTranslation());

    // The distanceToEndPoint check prevents removing a segment if the robot is far off path laterally
    // but its projection happens to be past the endpoint.
    // The threshold uses lookaheadDistance as a rough guide for "close enough".
    return projectionFactor > 0.95 && // Slightly before 1.0 can help with not overshooting too much before removal
    distanceToEndPoint < (this.m_lookaheadDistance * m_segmentPassProximityMultiplier + m_waypointCompletionProximity);
  }

  public ChassisSpeeds calculate(Pose2d currentPose, Optional<Rotation2d> robotHeading) {
    // --- 1. Check and Remove Completed Segments ---
    if (!m_remainingSegments.isEmpty()) {
      Path.Segment currentFirstSegment = m_remainingSegments.get(0);
      if (hasRobotPassedSegmentEndpoint(currentPose.getTranslation(), currentFirstSegment)) {
        System.out.println("Robot passed endpoint of segment: " + currentFirstSegment.start.getTranslation() + " -> " + currentFirstSegment.end.getTranslation());
        m_remainingSegments.remove(0);
        m_lastLookaheadFoundOnSegmentIndex = 0; // Reset index as list has shifted

        if (m_remainingSegments.isEmpty()) {
          System.out.println("All segments completed. Targeting final original waypoint.");
          // Path is now empty, drive towards the final original waypoint's position
          double distToFinal = m_finalWaypointOfOriginalPath.getTranslation().getDistance(currentPose.getTranslation());
          if (distToFinal < m_lookaheadDistance) m_lookaheadDistance = 0.1;

          if (distToFinal < m_waypointCompletionProximity) {
            m_previousDesiredSpeed = 0.0;
            return new ChassisSpeeds(); // Stop at final destination
          }
          double speed = (distToFinal > 1e-6) ? profileAcceleration(0.0, m_previousDesiredSpeed, distToFinal) : 0.0;
          return getChassisSpeeds(speed, currentPose, m_finalWaypointOfOriginalPath.getTranslation());
        }
      }
    }
    else { // activeSegments is empty
      System.out.println("No active segments. Targeting final original waypoint.");
      double distToFinal = m_finalWaypointOfOriginalPath.getTranslation().getDistance(currentPose.getTranslation());
      if (distToFinal < m_waypointCompletionProximity) { // Stricter stop
        System.out.println("Very close to final waypoint. Stopping.");
        m_previousDesiredSpeed = 0.0;
        return new ChassisSpeeds();
      }
      double speed = (distToFinal > 1e-6) ? profileAcceleration(0.0, m_previousDesiredSpeed, distToFinal) : 0.0;
      return getChassisSpeeds(speed, currentPose, m_finalWaypointOfOriginalPath.getTranslation());
    }

    // --- 2. Find Lookahead Point on current activeSegments ---
    var lookaheadPoint = findLookaheadPoint(currentPose, this.m_remainingSegments);
    Logger.recordOutput("PurePursuit/LookaheadPoint", lookaheadPoint);

    if (lookaheadPoint == null) { // Should ideally be handled by fallback to target end of current path
      // If activeSegments is not empty, this means fallback failed to find point on current path.
      // This might happen if robot is very far from remaining path. Try to steer towards start of remaining path.
      if (!m_remainingSegments.isEmpty()) {
        lookaheadPoint = m_remainingSegments.get(0).startTranslation; // Simple recovery: aim for start of next segment
      } else { // Should have been caught by logic above
        return new ChassisSpeeds(); // Stop
      }
    }

    // Transform lookahead point to be robot-relative
    var lookaheadPointRelative = lookaheadPoint.minus(currentPose.getTranslation()).rotateBy(currentPose.getRotation().unaryMinus());

    double desiredSpeed = m_previousDesiredSpeed;

    // Step 3b: Calculate speed reduction due to upcoming turns in path geometry
    double pathGeometryTurnReductionFactor = 0.0;
    double actualDesiredSpeed = desiredSpeed;
    int segmentsRemaining = m_remainingSegments.size();
    if (segmentsRemaining <= 1) {
      double distanceToDestination = currentPose.getTranslation().getDistance(m_finalWaypointOfOriginalPath.getTranslation());
      actualDesiredSpeed = profileAcceleration(0.0, m_previousDesiredSpeed, distanceToDestination);
      return getChassisSpeeds(actualDesiredSpeed, currentPose, lookaheadPoint);
    } else if (robotHeading.isPresent() && m_remainingSegments.size() >= 2) {
      Path.Segment nextSegment = m_remainingSegments.get(1); // The segment immediately after, forming the turn
      var vec2 = nextSegment.segmentVector;
      // Ensure segments have length to avoid issues with atan2 or normalization
      if (vec2.getNorm() > 1e-6) {
        double heading2 = Math.atan2(vec2.getY(), vec2.getX()); // Angle of 2nd segment vector
        double angleDifference = MathUtil.angleModulus(heading2 - robotHeading.get().getRadians());
        double turnAngleDegrees = Math.toDegrees(Math.abs(angleDifference)); // Absolute change in heading
        if (turnAngleDegrees > pathAngleThresholdForSlowingDegrees) {
            // Calculate severity of the turn (0.0 to 1.0)
            double effectiveMaxAngle = Math.max(pathMaxAngleForMaxSlowdownDegrees, pathAngleThresholdForSlowingDegrees + 1.0); // Ensure denominator > 0
            double turnSeverity = (turnAngleDegrees - pathAngleThresholdForSlowingDegrees) /
                                  (effectiveMaxAngle - pathAngleThresholdForSlowingDegrees);
            turnSeverity = MathUtil.clamp(turnSeverity, 0.0, 1.0); // Clamp to [0, 1]
            pathGeometryTurnReductionFactor = turnSeverity * pathMaxSpeedReductionForTurns;
        }
      }

      double turnVelocity = desiredSpeed * (1.0 - pathGeometryTurnReductionFactor);

      double distanceToTurn = currentPose.getTranslation().getDistance(nextSegment.startTranslation);

      actualDesiredSpeed = profileAcceleration(turnVelocity, m_previousDesiredSpeed, distanceToTurn);
    }

    return getChassisSpeeds(actualDesiredSpeed, currentPose, lookaheadPoint);
  }

  private ChassisSpeeds getChassisSpeeds(double desiredSpeed, Pose2d currentPose, Translation2d lookaheadPoint) {
    var lookaheadPointRelative = lookaheadPoint.minus(currentPose.getTranslation()).rotateBy(currentPose.getRotation().unaryMinus());
    double targetVx, targetVy;
    double norm = lookaheadPointRelative.getNorm();
    if (norm < 1e-6) {
      targetVx = 0;
      targetVy = 0;
    } else {
      targetVx = desiredSpeed * (lookaheadPointRelative.getX() / norm);
      targetVy = desiredSpeed * (lookaheadPointRelative.getY() / norm);
    }
    m_previousDesiredSpeed = desiredSpeed;
    var targetOmega = getTargetOmega(currentPose);
    return new ChassisSpeeds(targetVx, targetVy, targetOmega.in(Units.RadiansPerSecond));
  }

  private double profileAcceleration(double finalVelocity, double initialVelocity, double distanceToTarget) {
    double decelerationDistance = (finalVelocity * finalVelocity - initialVelocity * initialVelocity) / (2 * -m_acceleration);
    if (distanceToTarget < decelerationDistance)
      return Math.sqrt(finalVelocity * finalVelocity - 2 * -m_acceleration * finalVelocity);
    else return Math.min(m_previousDesiredSpeed + (m_acceleration / GlobalConstants.ROBOT_LOOP_FREQUENCY.in(Units.Hertz)), m_maxVelocity);
  }

  // findLookaheadPoint now takes the current list of active segments
  private Translation2d findLookaheadPoint(Pose2d currentRobotPose_field, List<Path.Segment> currentActiveSegments) {
    Translation2d robotPosition = currentRobotPose_field.getTranslation();
    Translation2d bestLookaheadPoint = null;
    double bestLookaheadScore = -1.0;
    int foundOnRelSegmentIndex = -1; // Index relative to currentActiveSegments

    if (currentActiveSegments.isEmpty()) return null; // No path to follow

    // Iterate through segments, starting near the last known one (relative to current list)
    // lastLookaheadFoundOnSegmentIndex is an index for currentActiveSegments
    for (int i = Math.max(0, m_lastLookaheadFoundOnSegmentIndex - 1); i < currentActiveSegments.size(); i++) {
      Path.Segment segment = currentActiveSegments.get(i);
      List<Translation2d> intersections = segment.findCircleIntersections(robotPosition, m_lookaheadDistance);

      for (Translation2d intersection : intersections) {
        if (segment.isPointOnSegment(intersection)) {
          Translation2d vecStartToIntersection = intersection.minus(segment.startTranslation);
          double distAlongSegment = 0;
          if (segment.segmentVector.getNorm() > 1e-9) {
            distAlongSegment = (vecStartToIntersection.getX() * segment.segmentVector.getX() +
            vecStartToIntersection.getY() * segment.segmentVector.getY())
            / segment.segmentVector.getNorm();
          }
          if (distAlongSegment < -1e-3) continue;

          double currentScore = i + (distAlongSegment / Math.max(1e-6, segment.length));
          if (currentScore > bestLookaheadScore) {
            bestLookaheadScore = currentScore;
            bestLookaheadPoint = intersection;
            foundOnRelSegmentIndex = i;
          }
        }
      }
    }

    if (bestLookaheadPoint != null) {
      m_lastLookaheadFoundOnSegmentIndex = foundOnRelSegmentIndex; // Update with index from currentActiveSegments
      return bestLookaheadPoint;
    }

    // Fallback logic if no direct circle intersection
    return calculateFallbackLookaheadPoint(robotPosition);
  }

  private Translation2d calculateFallbackLookaheadPoint(Translation2d robotPos) {
    if (m_remainingSegments.isEmpty()) return null;

    Translation2d closestPointOnPath = null;
    double minDistanceToPath = Double.MAX_VALUE;
    int closestSegmentIdxForFallback = 0;

    for (int i = 0; i < m_remainingSegments.size(); i++) {
      Path.Segment segment = m_remainingSegments.get(i);
      Translation2d pointOnSegment = segment.findClosestPoint(robotPos);
      double dist = robotPos.getDistance(pointOnSegment);
      if (dist < minDistanceToPath) {
        minDistanceToPath = dist;
        closestPointOnPath = pointOnSegment;
        closestSegmentIdxForFallback = i;
      }
    }

    if (closestPointOnPath == null) {
      return m_remainingSegments.get(m_remainingSegments.size() - 1).endTranslation; // Should not happen if list not empty
    }

    m_lastLookaheadFoundOnSegmentIndex = closestSegmentIdxForFallback;

    double remainingLookahead = m_lookaheadDistance;
    Translation2d currentLookaheadCalcPoint = closestPointOnPath;

    for (int i = closestSegmentIdxForFallback; i < m_remainingSegments.size(); i++) {
      Path.Segment currentSegment = m_remainingSegments.get(i);
      Translation2d vectorToSegmentEnd = currentSegment.endTranslation.minus(currentLookaheadCalcPoint);
      double distanceToSegmentEnd = vectorToSegmentEnd.getNorm();

      if (remainingLookahead <= distanceToSegmentEnd + 1e-6) {
        if (distanceToSegmentEnd < 1e-9) {
          if (remainingLookahead < 1e-6) return currentLookaheadCalcPoint;
        } else {
          Translation2d direction = vectorToSegmentEnd.times(1.0 / distanceToSegmentEnd);
          return currentLookaheadCalcPoint.plus(direction.times(remainingLookahead));
        }
      } else {
        remainingLookahead -= distanceToSegmentEnd;
        currentLookaheadCalcPoint = currentSegment.endTranslation;
      }
    }
    return m_remainingSegments.get(m_remainingSegments.size() - 1).endTranslation; // End of current path
  }

  // getTargetOrientation also uses currentActiveSegments
  private AngularVelocity getTargetOmega(Pose2d currentRobotPose_field) {
    var rotationTarget = currentRobotPose_field.getRotation();

    if (m_remainingSegments.isEmpty()) rotationTarget = m_finalWaypointOfOriginalPath.getRotation(); // Aim for final original orientation

    Path.Segment segmentForOrientation = null;

    // Find which of the *active* segments the lookahead point is on or closest to
    if (this.m_lastLookaheadFoundOnSegmentIndex >= 0 && this.m_lastLookaheadFoundOnSegmentIndex < m_remainingSegments.size()) {
      // If lookahead point was found on a valid segment index
      segmentForOrientation = m_remainingSegments.get(this.m_lastLookaheadFoundOnSegmentIndex);
    } else if (!m_remainingSegments.isEmpty()) {
      // Fallback: use the first available active segment
      segmentForOrientation = m_remainingSegments.get(0);
    }

    if (segmentForOrientation != null) {
      rotationTarget = segmentForOrientation.end.getRotation(); // Target heading of the end of the current lookahead segment
    } else rotationTarget = m_finalWaypointOfOriginalPath.getRotation();

    return Units.RadiansPerSecond.of(
      m_rotationController.calculate(currentRobotPose_field.getRotation().getRadians(), rotationTarget.getRadians())
    );
  }

  // Getter for debugging or external state checks
  public int getRemainingSegmentsCount() {
    return m_remainingSegments.size();
  }

  public Pose2d getFinalWaypointOfOriginalPath() {
    return m_finalWaypointOfOriginalPath;
  }
}