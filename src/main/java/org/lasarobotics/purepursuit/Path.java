// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.purepursuit;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Queue;

import org.lasarobotics.drive.swerve.SwerveDrive;
import org.lasarobotics.purepursuit.actions.TriggeredAction;
import org.lasarobotics.purepursuit.waypoints.EndWaypoint;
import org.lasarobotics.purepursuit.waypoints.GeneralWaypoint;
import org.lasarobotics.purepursuit.waypoints.InterruptWaypoint;
import org.lasarobotics.purepursuit.waypoints.PointTurnWaypoint;
import org.lasarobotics.purepursuit.waypoints.Waypoint;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

/**
* This class represents a pure pursuit path. It is used to store a path's waypoints, and do all the
* calculation required to traverse the path. A path can be implemented in two different way. The first
* way is running the loop/updating motor powers manually. This is a good implementation for those teams
* who wish to just use the pure pursuit section FtcLib and not much else. The other implementation
* is using the automatic feature. This features does all the hard work for you. Teams who use FtcLib's
* features to power most of their code will find this method appealing.
* <p>
* To learn how to implement a pure pursuit path, please see the example implementations and tutorials.
*
* @author Michael Baljet, Team 14470
* @version 1.3
* @see Waypoint
*/
@SuppressWarnings("serial")
public class Path extends ArrayList<Waypoint> {

  public enum Type {
    HEADING_CONTROLLED,
    WAYPOINT_ORDERING_CONTROLLED,
  }

  // This path's type (Heading controlled or waypoint ordering controlled).
  private Type m_type;

  // Timeout fields.
  private long timeoutMiliseconds;
  private long timeSinceStart;
  private Waypoint lastWaypoint;
  private long lastWaypointTimeStamp;
  private boolean timedOut;

  // True if retrace is enabled, this is enabled by default.
  private boolean retraceEnabled;

  // True if the init() method has been run.
  private boolean initComplete;

  // Fields for the retrace feature.
  private boolean retracing;
  private double retraceMovementSpeed;
  private double retraceTurnSpeed;
  private Translation2d lastKnownIntersection;
  private ProfiledPIDController rotatePIDController;

  // Action lists
  private List<TriggeredAction> triggeredActions;
  private Queue<InterruptWaypoint> interruptActionQueue;

  /**
  * Constructs an empty path and sets all settings to their defaults. Use add() to add waypoints.
  */
  public Path() {
    this(new Waypoint[]{});
  }

  /**
  * Constructs a path with the given waypoint. Sets all settings to their defaults.
  *
  * @param waypoints Waypoints in this path.
  */
  public Path(Waypoint... waypoints) {
    addAll(Arrays.asList(waypoints));
    m_type = Path.Type.WAYPOINT_ORDERING_CONTROLLED;
    timeoutMiliseconds = -1;
    timeSinceStart = -1;
    lastWaypointTimeStamp = 0;
    retraceMovementSpeed = 1;
    retraceTurnSpeed = 1;
    retraceEnabled = true;
    initComplete = false;
    timedOut = false;
    triggeredActions = new ArrayList<TriggeredAction>();
    interruptActionQueue = new LinkedList<InterruptWaypoint>();
    lastWaypoint = null;
    rotatePIDController = new ProfiledPIDController(5.0, 0.0, 0.0, new TrapezoidProfile.Constraints(2160.0, 4320.0));
  }

  /**
   * Takes the robot's current position and rotation and calculates the motor powers for the robot to move to the target position.
   *
   * @param currentPose Robot's current pose
   * @param targetPose Robot's target pose
   * @param turnOnly True if the robot should only turn.
   * @return Robot centric chassis speeds
   */
  public static ChassisSpeeds moveToPosition(Pose2d currentPose, Pose2d targetPose, boolean turnOnly) {
    // If turnOnly is true, only return a turn power.
    if (turnOnly) {
      var wrappedAngle = Units.RadiansPerSecond.of(MathUtil.angleModulus(currentPose.getRotation().plus(targetPose.getRotation()).getRadians()));
      return new ChassisSpeeds(Units.MetersPerSecond.zero(), Units.MetersPerSecond.zero(), wrappedAngle);
    }

    var absoluteDistanceX = targetPose.getMeasureX().minus(currentPose.getMeasureX());
    var absoluteDistanceY = targetPose.getMeasureY().minus(currentPose.getMeasureY());

    var absoluteAngle = Units.Radians.of(Math.atan2(absoluteDistanceY.in(Units.Meters), absoluteDistanceX.in(Units.Meters)));
    var distanceToPosition = Units.Meters.of(Math.hypot(absoluteDistanceX.in(Units.Meters), absoluteDistanceY.in(Units.Meters)));

    var relativeAngleToPosition = Units.Radians.of(
      MathUtil.angleModulus(absoluteAngle.plus(currentPose.getRotation().getMeasure()).in(Units.Radians))
    );

    var relativeDistanceX = distanceToPosition.times(Math.cos(relativeAngleToPosition.in(Units.Radians)));
    var relativeDistanceY = distanceToPosition.times(Math.sin(relativeAngleToPosition.in(Units.Radians)));

    var xVelocity = relativeDistanceX.div(relativeDistanceX.abs(Units.Meters) + relativeDistanceY.abs(Units.Meters)).per(Units.Second);
    var yVelocity = relativeDistanceY.div(relativeDistanceX.abs(Units.Meters) + relativeDistanceY.abs(Units.Meters)).per(Units.Second);
    var rotateVelocity = Units.RadiansPerSecond.of(MathUtil.angleModulus(currentPose.getRotation().plus(targetPose.getRotation()).getRadians()));

    return new ChassisSpeeds(xVelocity, yVelocity, rotateVelocity);
  }

  /**
   * Initiates the path. This must be ran before using the path. This checks to make sure everything
   * in the path is valid. Use isLegalPath() to check if a path is valid.
   * <p>
   * In order for a path to be considered legal it must have:
   * - At least 2 waypoints.
   * - Begin with a StartWaypoint
   * - End with an EndWaypoint
   * - Not contain any StartWaypoints or EndWaypoints in it's body.
   *
   * @throws IllegalStateException If the path is not legal.
   */
  public void init() {
    // Verify that the path is valid.
    verifyLegality();
    // Reset the path.
    reset();
    // Configure unconfigured waypoints.
    for (int i = 1; i < size(); i++)
      ((GeneralWaypoint)get(i)).inherit(get(i - 1));
    // Mark the init as complete.
    initComplete = true;
  }

  /**
  * Initiates the automatic path following feature. The robot will follow the path and perform actions as configured.
  *
  * @param drivetrain The robot's drive base. Only mecanum drives are supported currently.
  * @throws IllegalStateException If automatic mode is disabled/not configured or the init has not been ran.
  */
  public Command followPath(SwerveDrive drivetrain) {
    // Make sure arguments are not null.
    if (drivetrain == null)
      throw new IllegalStateException("Path initiation failed. Drivetrain is not set.");

    return new FunctionalCommand(
      this::init,
      () -> {
        var speeds = loop(drivetrain);
        drivetrain.autoDrive(speeds);
        // if (!isFinished()) {
        //   boolean pathAborted = true;
        //   // If the has stopped, then the path has timed out or lost the path.
        //   if (speeds.vxMetersPerSecond != 0.0 && speeds.vyMetersPerSecond != 0.0 && speeds.omegaRadiansPerSecond != 0.0)
        //     pathAborted = false;
        //   if (pathAborted) return false;
        // }
      },
      (interupted) -> {},
      this::isFinished,
      drivetrain).andThen(drivetrain.stopCommand());
  }

  /**
  * This is the principle path method. After everything is configured and initiated, this method can be used.
  * Using the robot's horizontal, y, and rotation, this method calculates the appropriate motor powers for the
  * robot to follow the path. This method calls all triggered/interrupted actions automatically. If this
  * returns zero motor speeds {0, 0, 0} that means the path has either (1) timed out, (2) lost the path and
  * retrace was disabled, or (3) reached the destination. Use isFinished() and timedOut() to troubleshoot.
  *
  * @param drivetrain Robot's swerve drivetrain
  * @return A double array containing the motor powers. a[0] is the x power, a[1] is the y power, and a[2] is the turn power.
  */
  public ChassisSpeeds loop(SwerveDrive drivetrain) {
    var speeds = new ChassisSpeeds();
    // First, make sure the init has been called. While this does not guarantee the program will run without errors, it is better than nothing.
    if (!initComplete)
      throw new IllegalStateException("You must call the init() function before calling loop()");

    if (timedOut) return speeds;

    if (timeoutMiliseconds != -1) {
      // If this path has a timeout.
      if (timeSinceStart == -1) timeSinceStart = System.currentTimeMillis();
      else if (timeSinceStart + timeoutMiliseconds < System.currentTimeMillis()) {
        timedOut = true;
        // If the path has timed out, return no speeds.
        return speeds;
      }
    }

    var currentPose = drivetrain.getPose();

    // Next, loop triggered and perform interrupted actions.
    loopTriggeredActions();
    runQueuedInterruptActions();
    // Get all the intersections on the path.
    ArrayList<TaggedIntersection> intersections = new ArrayList<TaggedIntersection>();
    for (int i = 1; i < size(); i++) {
      // Get the path line segment and circle.
      var linePoint1 = get(i - 1).getPose().getTranslation();
      var linePoint2 = get(i).getPose().getTranslation();
      var radius = get(i).getFollowDistance();
      List<Translation2d> points = PurePursuitUtil.lineCircleIntersection(currentPose.getTranslation(), radius, linePoint1, linePoint2);
      for (Translation2d point : points)
        intersections.add(new TaggedIntersection(point, get(i), i));
      if (get(i) instanceof PointTurnWaypoint) {
        // If the second waypoint is a point turn waypoint, decrease the follow radius so the next point is always found.
        double dx = linePoint2.getX() - currentPose.getX();
        double dy = linePoint2.getY() - currentPose.getY();
        double adjustedRadius = Math.hypot(dx, dy) - 1e-9;
        if (adjustedRadius < radius.in(Units.Meters)) {
          // Add the point to the list.
          intersections.add(new TaggedIntersection(((PointTurnWaypoint) get(i)).getTranslation(), get(i), i));
        }
      }
      // Now all intersections are recorded.
    }

    // If there are no intersections found, the path is lost.
    if (intersections.size() == 0) {
      if (retracing)
        return retrace(currentPose, drivetrain.getCurrentHeading());
      // If retrace is enabled, we can try to re-find the path.
      if (retraceEnabled) {
        if (lastKnownIntersection == null)
        lastKnownIntersection = get(0).getPose().getTranslation();
        retracing = true;
        return retrace(currentPose, drivetrain.getCurrentHeading());
      } else return speeds;
    } else retracing = false;

    // The intersections are handled differently depending on the path type.
    TaggedIntersection bestIntersection = intersections.get(0);
    switch (m_type) {
      case HEADING_CONTROLLED:
        bestIntersection = selectHeadingControlledIntersection(intersections, currentPose);
        break;
      case WAYPOINT_ORDERING_CONTROLLED:
        bestIntersection = selectWaypointOrderingControlledIntersection(intersections);
        break;
    }

    if (retraceEnabled)
      // If retrace is enabled, store the intersection.
      lastKnownIntersection = bestIntersection.intersection;
    if (bestIntersection.taggedPoint != lastWaypoint) {
      // If this is the first intersection of a new waypoint, update timeout values.
      lastWaypoint = bestIntersection.taggedPoint;
      lastWaypointTimeStamp = System.currentTimeMillis();
    }

    if (bestIntersection.taggedPoint.getTimeout() != -1) {
      // If this waypoint has a timeout, make sure it hasn't timed out.
      if (System.currentTimeMillis() > lastWaypointTimeStamp + bestIntersection.taggedPoint.getTimeout()) {
        timedOut = true;
        // If it has, return no motor speeds.
        return speeds;
      }
    }

    // After the best intersection is found, the robot behaves differently depending on the type of waypoint.
    var currentHeading = drivetrain.getCurrentHeading();
    switch (bestIntersection.taggedPoint.getType()) {
      case GENERAL:
        speeds = handleGeneralIntersection(currentPose, currentHeading, bestIntersection);
      break;
      case POINT_TURN:
        speeds = handlePointTurnIntersection(currentPose, currentHeading, bestIntersection);
      break;
      case INTERRUPT:
        speeds = handleInterruptIntersection(currentPose, currentHeading, bestIntersection);
      break;
      case END:
        speeds = handleEndIntersection(currentPose, currentHeading, bestIntersection);
      break;
      case START:
        // This should never happen.
        throw new IllegalStateException("Path has lost integrity.");
    }

    // Return the motor powers.
    return speeds;
  }

  /**
  * Retraces the robot's moves back to the path's last known location.
  *
  * @param xPosition Robot's x position.
  * @param yPosition Robot's y position.
  * @param rotation  Robot's rotation.
  * @return A double array containing the motor powers. a[0] is the x power, a[1] is the y power, and a[2] is the turn power.
  */
  private ChassisSpeeds retrace(Pose2d currentPose, Optional<Rotation2d> currentHeading) {
    // Move towards the last known intersection.
    var speeds = moveToPosition2(
      currentPose,
      new Pose2d(lastKnownIntersection.getX(), lastKnownIntersection.getY(), currentPose.getRotation()),
      currentHeading,
      false
    );
    // speeds.vxMetersPerSecond *= retraceMovementSpeed;
    // speeds.vyMetersPerSecond *= retraceMovementSpeed;
    // speeds.omegaRadiansPerSecond *= retraceTurnSpeed;

    return speeds;
  }

  /**
  * Selects and returns the "best" intersection from the given list using heading
  * control. The intersection is chosen based on the following rules:
  * 1. If the list contains any untraversed waypoints, they are given priority and the best intersection is the point closest to the point turn waypoint.
  * 2. If the list contains no point turn points, then it chooses the intersection the robot is oriented most closely towards.
  *
  * @param intersections Intersection list.
  * @param robotPos      Robot's current position/rotation.
  * @return The best intersection in the form of a TaggedIntersection.
  */
  private TaggedIntersection selectHeadingControlledIntersection(List<TaggedIntersection> intersections, Pose2d robotPos) {
    TaggedIntersection bestIntersection = intersections.get(0);
    boolean pointTurnPriority = false;
    /**
    * In a heading controlled path, the intersection the robot is most closely oriented toward is considered the "best point".
    */
    for (TaggedIntersection intersection : intersections) {
      // Check to see if a point turn waypoint is found.
      if (intersection.taggedPoint instanceof PointTurnWaypoint) {
        PointTurnWaypoint ptwaypoint = (PointTurnWaypoint) intersection.taggedPoint;
        if (!ptwaypoint.hasTraversed()) {
          // If point turn waypoint is found, and it has not already been traversed, then it takes priority.
          pointTurnPriority = true;
          if (!(bestIntersection.taggedPoint instanceof PointTurnWaypoint))
          bestIntersection = intersection;
          else {
            // If two intersections associated with a point turn waypoint are found, choose the one closer to the waypoint.
            if (bestIntersection.waypointIndex < intersection.waypointIndex)
              // If the intersection is obviously behind.
              bestIntersection = intersection;
            else if (bestIntersection.waypointIndex == intersection.waypointIndex)
              // Check to see if it is in front.
              if (PurePursuitUtil.isInFront(get(intersection.waypointIndex - 1).getPose().getTranslation(), intersection.taggedPoint.getPose().getTranslation(), intersection.intersection, bestIntersection.intersection))
                bestIntersection = intersection;
          }
        }
      } else if (pointTurnPriority)
        // If pointTurnPriority is active, ignore non point turn intersections.
        continue;
      else {
        // Normal case.
        // Relative angle to intersection.
        double absoluteAngleToIntersection = Math.atan2(intersection.intersection.getY(), intersection.intersection.getX());
        double relativeAngleToIntersection = absoluteAngleToIntersection - robotPos.getRotation().getRadians();
        // Relative angle to best intersection.
        double absoluteAngleToBestIntersection = Math.atan2(bestIntersection.intersection.getY(), bestIntersection.intersection.getX());
        double relativeAngleToBestIntersection = absoluteAngleToBestIntersection - robotPos.getRotation().getRadians();
        if (relativeAngleToIntersection < relativeAngleToBestIntersection)
          // Update bestIntersection.
          bestIntersection = intersection;
      }
    }
    // Return the best intersection.
    return bestIntersection;
  }

  /**
  * Selects and returns the "best" intersection from the given list by choosing the intersection that is farthest along the path.
  * The intersection is chosen based on the following rules:
  * 1. If the list contains any untraversed waypoints, they are given priority and the best intersection is the point closest to the point turn waypoint.
  * 2. If the list contains no point turn points, then it chooses the intersection that is farthest along the path.
  *
  * @param intersections Intersection list.
  * @return The best intersection in the form of a TaggedIntersection.
  */
  private TaggedIntersection selectWaypointOrderingControlledIntersection(List<TaggedIntersection> intersections) {
    TaggedIntersection bestIntersection = intersections.get(0);
    boolean pointTurnPriority = false;
    /**
    * In a waypoint ordering controlled path, the intersection that is farthest along the path is considered the "best point".
    */
    for (TaggedIntersection intersection : intersections) {
      // Check to see if a point turn waypoint is found.
      if (intersection.taggedPoint instanceof PointTurnWaypoint) {
        PointTurnWaypoint ptwaypoint = (PointTurnWaypoint) intersection.taggedPoint;
        if (!ptwaypoint.hasTraversed()) {
          // If point turn waypoint is found, and it has not already been traversed, then it takes priority.
          pointTurnPriority = true;
          if (!(bestIntersection.taggedPoint instanceof PointTurnWaypoint))
          bestIntersection = intersection;
          else if (((PointTurnWaypoint) bestIntersection.taggedPoint).hasTraversed())
          bestIntersection = intersection;
          else {
            // If two intersections associated with a point turn waypoint are found, choose the one closer to the waypoint.
            if (bestIntersection.waypointIndex > intersection.waypointIndex || ptwaypoint.hasTraversed())
            // If the intersection is obviously behind.
            bestIntersection = intersection;
            else if (bestIntersection.waypointIndex == intersection.waypointIndex)
            // Check to see if it is in front.
            if (PurePursuitUtil.isInFront(get(intersection.waypointIndex - 1).getPose().getTranslation(), intersection.taggedPoint.getPose().getTranslation(), intersection.intersection, bestIntersection.intersection))
            bestIntersection = intersection;
          }
        }
      } else if (pointTurnPriority)
      // If pointTurnPriority is active, ignore non point turn intersections.
      continue;
      else {
        // Normal case.
        if (bestIntersection.waypointIndex < intersection.waypointIndex)
        // If the intersection is obviously ahead.
        bestIntersection = intersection;
        else if (bestIntersection.waypointIndex == intersection.waypointIndex)
        // Check to see if it is in front.
        if (PurePursuitUtil.isInFront(get(intersection.waypointIndex - 1).getPose().getTranslation(), intersection.taggedPoint.getPose().getTranslation(), intersection.intersection, bestIntersection.intersection))
        bestIntersection = intersection;
      }
    }
    // Return the best intersection.
    return bestIntersection;
  }

  /**
  * Returns the motor speeds required to approach the given intersection.
  *
  * @param intersection Intersection to approach
  * @param currentPose     Robot's current pose
  * @return Target pose to drive toward
  */
  private ChassisSpeeds handleGeneralIntersection(Pose2d currentPose, Optional<Rotation2d> currentHeading, TaggedIntersection intersection) {
    /**
     * General intersections are handled like normal pure pursuit intersections. The robot simply moves towards them.
     */
    var waypoint = (GeneralWaypoint)intersection.taggedPoint;
    // Get necessary values.
    Pose2d targetPose;
    // If this waypoint has a preferred angle, use it instead of the calculated angle.
    if (waypoint.usingPreferredAngle()) {
      targetPose = new Pose2d(
        intersection.intersection.getMeasureX(),
        intersection.intersection.getMeasureY(),
        new Rotation2d(waypoint.getPreferredAngle())
      );
    }
    else {
      targetPose = new Pose2d(
        intersection.intersection.getMeasureX(),
        intersection.intersection.getMeasureY(),
        Rotation2d.fromRadians(
          Math.atan2(
            intersection.intersection.getMeasureY().minus(currentPose.getMeasureY()).in(Units.Meters),
            intersection.intersection.getMeasureX().minus(currentPose.getMeasureX()).in(Units.Meters)
          )
        )
      );
    }

    return moveToPosition2(currentPose, targetPose, currentHeading, false);
  }

  /**
  * Returns the motor speeds required to approach the given point turn intersection.
  * This will cause the robot to behave as follows:
  * 1. Approach and decelerate to the waypoint.
  * 2. Perform a point turn.
  * 3. Continue to the next waypoint as normal.
  *
  * @param intersection Intersection to approach.
  * @param currentPose  Robot's current pose
  * @return Target pose to drive towards
  */
  private ChassisSpeeds handlePointTurnIntersection(Pose2d currentPose, Optional<Rotation2d> currentHeading, TaggedIntersection intersection) {
    /**
     * Point turn intersections are handled very differently than general intersections. Instead of "curving" around
     * the point, the robot will decelerate and perform a point turn.
     */
    PointTurnWaypoint waypoint = (PointTurnWaypoint)intersection.taggedPoint;
    Angle targetAngle;
    var speeds = new ChassisSpeeds();
    var targetPose = new Pose2d();

    if (!waypoint.hasTraversed() && Units.Meters.of(currentPose.getTranslation().getDistance(waypoint.getTranslation())).lt(waypoint.getPositionBuffer())) {
      // If the robot has not reached the point.
      if (((GeneralWaypoint)get(intersection.waypointIndex + 1)).usingPreferredAngle()) {
        if (currentPose.getRotation().getMeasure().isNear(((GeneralWaypoint)get(intersection.waypointIndex + 1)).getPreferredAngle(), waypoint.getRotationBuffer()))
        // If the robot has reached the point and is at the preferredAngle, then the point is traversed.
        waypoint.setTraversed();
        // Set the target angle.
        targetAngle = ((GeneralWaypoint)get(intersection.waypointIndex + 1)).getPreferredAngle();
      } else {
        double tempTy = ((GeneralWaypoint)get(intersection.waypointIndex + 1)).getPose().getTranslation().getY();
        double tempTx = ((GeneralWaypoint)get(intersection.waypointIndex + 1)).getPose().getTranslation().getX();
        // Calculate the target angle.
        targetAngle = Units.Radians.of(Math.atan2(tempTy - currentPose.getY(), tempTx - currentPose.getX()));
        if (currentPose.getRotation().getMeasure().isNear(targetAngle, waypoint.getRotationBuffer()))
        // If the robot has reached the point and is at the target angle, then the point is traversed.
        waypoint.setTraversed();
      }
      targetPose = new Pose2d(intersection.intersection.getMeasureX(), intersection.intersection.getMeasureY(), Rotation2d.fromRadians(targetAngle.in(Units.Radians)));
      speeds = moveToPosition2(currentPose, targetPose, currentHeading, true);
    } else {
      // If this waypoint has a preferred angle, use it instead of the calculated angle.
      if (waypoint.usingPreferredAngle())
        targetAngle = (waypoint.getPreferredAngle());
      else
        targetAngle = Units.Radians.of(Math.atan2(intersection.intersection.getY() - currentPose.getY(), intersection.intersection.getX() - currentPose.getX()));

      targetPose = new Pose2d(intersection.intersection.getMeasureX(), intersection.intersection.getMeasureY(), Rotation2d.fromRadians(targetAngle.in(Units.Radians)));
      speeds = moveToPosition2(currentPose, targetPose, currentHeading, false);
    }

    return speeds;
  }

  /**
  * Returns the motor speeds required to approach the given interrupt intersection.
  * This will cause the robot to behave as follows:
  * 1. Approach and decelerate to the waypoint.
  * 2. Perform a point turn / align with the preferred angle.
  * 4. Perform the interrupt action.
  * 3. Continue to the next waypoint as normal.
  *
  * @param intersection Intersection to approach.
  * @param currentPose     Robot's current position/rotation.
  * @return Target pose to drive towards
  */
  private ChassisSpeeds handleInterruptIntersection(Pose2d currentPose, Optional<Rotation2d> currentHeading, TaggedIntersection intersection) {
    /**
     * Interrupt intersections are handled similarly to point turn intersections. Instead of continuing directly
     * after it has turned, the robot will stop and perform the interrupt actions.
     */
    InterruptWaypoint waypoint = (InterruptWaypoint) intersection.taggedPoint;
    Angle targetAngle;
    var speeds = new ChassisSpeeds();
    var targetPose = new Pose2d();

    if (!waypoint.hasTraversed() && Units.Meters.of(currentPose.getTranslation().getDistance(waypoint.getTranslation())).lt(waypoint.getPositionBuffer())) {
      // If the robot has not reached the point.
      if (waypoint.getType() == Waypoint.Type.END) {
        if (waypoint.usingPreferredAngle() && !currentPose.getRotation().getMeasure().isNear(waypoint.getPreferredAngle(), waypoint.getRotationBuffer()))
          targetAngle = waypoint.getPreferredAngle();
        else {
          ((EndWaypoint)waypoint).setTraversed();
          return speeds;
        }
      } else if (((GeneralWaypoint)get(intersection.waypointIndex + 1)).usingPreferredAngle()) {
        if (currentPose.getRotation().getMeasure().isNear(((GeneralWaypoint)get(intersection.waypointIndex + 1)).getPreferredAngle(), waypoint.getRotationBuffer())) {
          // If the robot has reached the point and is at the preferredAngle, then the point is traversed.
          waypoint.setTraversed();
          // Queue the action.
          interruptActionQueue.add(waypoint);
          // Stop the robot while it does the action.
          return speeds;
        }
        // Set the target angle.
        targetAngle = ((GeneralWaypoint)get(intersection.waypointIndex + 1)).getPreferredAngle();
      } else {
        double tempTy = ((GeneralWaypoint)get(intersection.waypointIndex + 1)).getPose().getTranslation().getY();
        double tempTx = ((GeneralWaypoint)get(intersection.waypointIndex + 1)).getPose().getTranslation().getX();
        // Calculate the target angle.
        targetAngle = Units.Radians.of(Math.atan2(tempTy - currentPose.getY(), tempTx - currentPose.getX()));
        if (currentPose.getRotation().getMeasure().isNear(targetAngle, waypoint.getRotationBuffer())) {
          // If the robot has reached the point and is at the target angle, then the point is traversed.
          waypoint.setTraversed();
          // Queue the action.
          interruptActionQueue.add(waypoint);
          // Stop the robot while it does the action.
          return speeds;
        }
      }
      targetPose = new Pose2d(intersection.intersection.getMeasureX(), intersection.intersection.getMeasureY(), Rotation2d.fromRadians(targetAngle.in(Units.Radians)));
      speeds = moveToPosition2(currentPose, targetPose, currentHeading, true);
    } else {
      // If this waypoint has a preferred angle, use it instead of the calculated angle.
      if (waypoint.usingPreferredAngle())
        targetAngle = (waypoint.getPreferredAngle());
      else
        targetAngle = Units.Radians.of(Math.atan2(intersection.intersection.getY() - currentPose.getY(), intersection.intersection.getX() - currentPose.getX()));

      targetPose = new Pose2d(intersection.intersection.getMeasureX(), intersection.intersection.getMeasureY(), Rotation2d.fromRadians(targetAngle.in(Units.Radians)));
      speeds = moveToPosition2(currentPose, targetPose, currentHeading, false);
    }

    return speeds;
  }

  /**
  * Returns the motor speeds required to approach the given end intersection.
  * This will cause the robot to behave as follows:
  * 1. Approach and decelerate to the end point.
  * 2. Turn to face the preferred angle (if provided).
  * 3. Mark the path as complete.
  *
  * @param intersection Intersection to approach.
  * @param robotPos     Robot's current position/rotation.
  * @return Target pose to drive towards
  */
  private ChassisSpeeds handleEndIntersection(Pose2d currentPose, Optional<Rotation2d> currentHeading, TaggedIntersection intersection) {
    /**
    * End intersections are handled the same way as interrupt intersections.
    */
    return handleInterruptIntersection(currentPose, currentHeading, intersection);
  }

  /**
  * Sets a timeout for the entire path. If the path does not complete within the timeout period, it
  * will abort. This is an optional feature.
  *
  * @param timeoutMiliseconds Timeout to be set.
  * @return This path, used for chaining methods.
  */
  public Path setPathTimeout(long timeoutMiliseconds) {
    this.timeoutMiliseconds = timeoutMiliseconds;
    return this;
  }

  /**
  * Sets the path type to the specified type. By default the path type is WAYPOINT_ORDERING_CONTROLLED.
  * This is not recommended, do not change the path type unless you know what you are doing.
  *
  * @param type Path type to be set.
  * @return This path, used for chaining methods.
  */
  public Path setType(Path.Type type) {
    m_type = type;
    return this;
  }

  /**
  * Sets the timeouts of n waypoints where n is the amount of arguments provided.
  * The nth waypoint timeout is set the the nth argument given.
  *
  * @param timeouts Timeouts to be set.
  * @return This path, used for chaining methods.
  */
  public Path setWaypointTimeouts(long... timeouts) {
    for (int i = 0; i < size() && i < timeouts.length; i++)
    if (get(i) instanceof GeneralWaypoint)
    ((GeneralWaypoint) get(i)).setTimeout(timeouts[i]);
    return this;
  }

  /**
  * Sets the timeout for each individual waypoint to be the value provided. This is not recommended.
  *
  * @param timeout Universal timeout to be set.
  * @return This path, used for chaining methods.
  */
  public Path setWaypointTimeouts(long timeout) {
    for (Waypoint waypoint : this)
    if (waypoint instanceof GeneralWaypoint)
    ((GeneralWaypoint)waypoint).setTimeout(timeout);
    return this;
  }

  /**
  * Configures the retrace settings. The default values are as follows:
  * movementSpeed = 1
  * turnSpeed = 1
  *
  * @param movementSpeed Movement speed to be set.
  * @param rotateSpeed Rotate speed to be set.
  * @return This path, used for chaining methods.
  */
  public Path setRetraceSettings(double movementSpeed, double rotateSpeed) {
    retraceMovementSpeed = movementSpeed;
    retraceTurnSpeed = rotateSpeed;
    return this;
  }

  /**
  * Resets all timeouts.
  *
  * @return This path, used for chaining methods.
  */
  public Path resetTimeouts() {
    timedOut = false;
    lastWaypointTimeStamp = System.currentTimeMillis();
    return this;
  }

  /**
  * Enables retrace. If the robot loses the path and this is enabled, the robot will retrace its moves to try
  * to re find the path. This is enabled by default.
  *
  * @return This path, used for chaining methods.
  */
  public Path enableRetrace() {
    retraceEnabled = true;
    return this;
  }

  /**
   * Disables retrace.
   *
   * @return This path, used for chaining methods.
   */
  public Path disableRetrace() {
    retraceEnabled = false;
    return this;
  }

  /**
  * Adds the provided TriggeredActions to the path. These are handled automatically.
  *
  * @param actions TriggeredActions to be added.
  * @return This path, used for chaining methods.
  */
  public Path addTriggeredActions(TriggeredAction... actions) {
    for (TriggeredAction triggeredAction : actions)
    triggeredActions.add(triggeredAction);
    return this;
  }

  /**
  * Removes the first instance of the provided TriggeredAction from the path.
  *
  * @param action TriggeredAction to be removed
  * @return This path, used for chaining methods.
  */
  public Path removeTriggeredAction(TriggeredAction action) {
    triggeredActions.remove(action);
    return this;
  }

  /**
  * Removes all TriggeredActions from the path.
  *
  * @return This path, used for chaining methods.
  */
  public Path clearTriggeredActions() {
    triggeredActions.clear();
    return this;
  }

  /**
  * Returns true if this path is legal, false otherwise.
  * <p>
  * In order for a path to be considered legal it must have:
  * - At least 2 waypoints.
  * - Begin with a StartWaypoint
  * - End with an EndWaypoint
  * - Not contain any StartWaypoints or EndWaypoints in it's body.
  *
  * @return true if this path is legal, false otherwise.
  */
  public boolean isLegalPath() {
    try {
      verifyLegality();
      initComplete = false;
    } catch (IllegalStateException e) {
      return false;
    }
    return true;
  }

  /**
  * Returns true if the path has been completed, false otherwise.
  *
  * @return true if the path has been completed, false otherwise.
  */
  public boolean isFinished() {
    if (size() > 0 && get(size() - 1).getType() == Waypoint.Type.END)
      return ((EndWaypoint)get(size() - 1)).isFinished();
    return false;
  }

  /**
  * Returns true if this path has timed out, false otherwise.
  *
  * @return true if this path has timed out, false otherwise.
  */
  public boolean timedOut() {
    return timedOut;
  }

  /**
  * Resets all the waypoints/timeouts/actions in this path. Called by the init.
  */
  public void reset() {
    resetTimeouts();
    for (Waypoint waypoint : this)
    if (waypoint instanceof GeneralWaypoint)
    ((GeneralWaypoint) waypoint).reset();
    for (TriggeredAction actions : triggeredActions)
    actions.reset();
  }

  /**
  * Checks to make sure the path is legal. If it is not, an exception is thrown.
  * <p>
  * In order for a path to be considered legal it must have:
  * - At least 2 waypoints.
  * - Begin with a StartWaypoint
  * - End with an EndWaypoint
  * - Not contain any StartWaypoints or EndWaypoints in it's body.
  *
  * @throws IllegalStateException If the path is not legal.
  */
  private void verifyLegality() {
    if (size() < 2)
      throw new IllegalStateException("A path must have at least two waypoints.");
    if (get(0).getType() != Waypoint.Type.START)
      throw new IllegalStateException("A path must start with a StartWaypoint.");
    if (get(size() - 1).getType() != Waypoint.Type.END)
      throw new IllegalStateException("A path must end with an EndWaypoint.");
    for (int i = 1; i < size() - 1; i++)
      if (get(i).getType() == Waypoint.Type.END || get(i).getType() == Waypoint.Type.START)
        throw new IllegalStateException("A path must not have end and start waypoints anywhere other than the first and last spot.");
  }

  /**
  * Calls the loop() method on all TriggeredActions in this path.
  */
  private void loopTriggeredActions() {
    for (TriggeredAction action : triggeredActions)
    action.loop();
  }

  /**
  * Performs all queued interrupt actions.
  */
  private void runQueuedInterruptActions() {
    while (!interruptActionQueue.isEmpty())
    interruptActionQueue.remove().performAction();
  }

  public ChassisSpeeds moveToPosition2(Pose2d currentPose, Pose2d targetPose, Optional<Rotation2d> currentHeading, boolean rotateOnly) {
    var MAX_VELOCITY = Units.MetersPerSecond.of(5.0);
    var MAX_ACCELERATION = Units.MetersPerSecondPerSecond.of(2.0);
    var MAX_ANGLE_DIFFERENCE = Units.Degrees.of(90.0);

    // If turnOnly is true, only return a turn power.
    if (rotateOnly) {
      var rotateVelocity = Units.DegreesPerSecond.of(rotatePIDController.calculate(
        currentPose.getRotation().getDegrees(),
        new TrapezoidProfile.State(targetPose.getRotation().getDegrees(), 0.0)
      ));
      return new ChassisSpeeds(Units.MetersPerSecond.zero(), Units.MetersPerSecond.zero(), rotateVelocity);
    }

    var absoluteDistanceX = targetPose.getMeasureX().minus(currentPose.getMeasureX());
    var absoluteDistanceY = targetPose.getMeasureY().minus(currentPose.getMeasureY());

    var absoluteAngle = Units.Radians.of(Math.atan2(absoluteDistanceY.in(Units.Meters), absoluteDistanceX.in(Units.Meters)));
    var distanceToPosition = Units.Meters.of(Math.hypot(absoluteDistanceX.in(Units.Meters), absoluteDistanceY.in(Units.Meters)));

    var relativeAngleToPosition = Units.Radians.of(
      MathUtil.angleModulus(absoluteAngle.plus(currentPose.getRotation().getMeasure()).in(Units.Radians))
    );

    Angle angleError;
    if (currentHeading.isEmpty()) angleError = relativeAngleToPosition.minus(currentPose.getRotation().getMeasure());
    else angleError = relativeAngleToPosition.minus(currentHeading.get().getMeasure());

    angleError = Units.Radians.of(MathUtil.angleModulus(angleError.in(Units.Radians)));

    var desiredVelocity = Units.MetersPerSecond.of(Math.min(
      MAX_VELOCITY.in(Units.MetersPerSecond),
      Math.sqrt(2 * MAX_ACCELERATION.in(Units.MetersPerSecondPerSecond) * distanceToPosition.in(Units.Meters))
    ));

    double curvatureFactor = Math.min(angleError.abs(Units.Radians) / MAX_ANGLE_DIFFERENCE.in(Units.Radians), 0.8);
    //desiredVelocity = desiredVelocity.times(1 - curvatureFactor);

    var relativeDistanceX = distanceToPosition.times(Math.cos(relativeAngleToPosition.in(Units.Radians)));
    var relativeDistanceY = distanceToPosition.times(Math.sin(relativeAngleToPosition.in(Units.Radians)));

    var xVelocity = desiredVelocity.times(relativeDistanceX.in(Units.Meters) / distanceToPosition.in(Units.Meters));
    var yVelocity = desiredVelocity.times(relativeDistanceY.in(Units.Meters) / distanceToPosition.in(Units.Meters));
    var rotateVelocity = Units.DegreesPerSecond.of(rotatePIDController.calculate(
      currentPose.getRotation().getDegrees(),
      new TrapezoidProfile.State(targetPose.getRotation().getDegrees(), 0.0)
    ));

    return new ChassisSpeeds(xVelocity, yVelocity, rotateVelocity);
  }

  /**
  * This private class is used to store additional information associated with an intersection.
  *
  * @version 1.0
  */
  private static class TaggedIntersection {

    // Location of the intersection.
    public Translation2d intersection;

    // Waypoint associated with the intersection.
    public Waypoint taggedPoint;

    // The associated waypoint's index in the path.
    public int waypointIndex;

    /**
    * Constructs a TaggedIntersection object with the given values.
    *
    * @param intersection  Location of the intersection.
    * @param taggedPoint   Waypoint associated with the intersection.
    * @param waypointIndex The associated waypoint's index in the path.
    */
    public TaggedIntersection(Translation2d intersection, Waypoint taggedPoint, int waypointIndex) {
      this.intersection = intersection;
      this.taggedPoint = taggedPoint;
      this.waypointIndex = waypointIndex;
    }
  }
}
