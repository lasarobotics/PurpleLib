// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.purepursuit.waypoints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

/**
* A start waypoint represents the first waypoint in a path. This waypoint is
* very simplified because the robot will never need to traverse it.
*
* @author Michael Baljet, Team 14470
* @version 1.1
* @see Waypoint
*/
public class StartWaypoint extends Pose2d implements Waypoint {

  // If the robot moves towards this waypoint for longer than the timeout period, the path is aborted.
  private long timeoutMiliseconds;

  /**
  * Constructs a StartWaypoint. All values are set to their default.
  */
  public StartWaypoint() {
    timeoutMiliseconds = -1;
  }

  /**
  * Construct a StartWaypoint located at the provided translation.
  *
  * @param translation Position (x, y) of this waypoint.
  */
  public StartWaypoint(Translation2d translation) {
    super(translation, new Rotation2d(0));
  }

  /**
  * Construct a StartWaypoint located at the provided pose.
  *
  * @param pose Position (x, y) of this waypoint.
  */
  public StartWaypoint(Pose2d pose) {
    super(pose.getTranslation(), pose.getRotation());
  }

  /**
  * Construct a StartWaypoint located at the provided coordinate.
  *
  * @param x X Position of this waypoint.
  * @param y Y Position of this waypoint.
  */
  public StartWaypoint(double x, double y) {
    super(x, y, new Rotation2d(0));
  }

  @Override
  public Waypoint.Type getType() {
    return Waypoint.Type.START;
  }

  @Override
  public Pose2d getPose() {
    return this;
  }

  @Override
  public Distance getFollowDistance() {
    return Units.Meters.zero(); // This value will never be used.
  }

  @Override
  public long getTimeout() {
    return timeoutMiliseconds;
  }

  @Override
  public String toString() {
    return String.format("StartWaypoint(%s, %s)", getTranslation().getX(), getTranslation().getY());
  }

}
