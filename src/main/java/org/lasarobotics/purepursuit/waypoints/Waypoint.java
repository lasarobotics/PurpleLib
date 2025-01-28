// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.purepursuit.waypoints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;

/**
 * A pure pursuit Waypoint is a point in which the robot traverses. Using Waypoints
 * one can construct a pure pursuit path for their robot to follow.
 */
public interface Waypoint {

  public enum Type {
    GENERAL,
    POINT_TURN,
    INTERRUPT,
    START,
    END,
  }

  /**
   * Returns this WayPoint's type.
   *
   * @return this WayPoint's type.
   */
  public Waypoint.Type getType();

  /**
   * Returns this Waypoint's position.
   *
   * @return this Waypoint's position.
   */
  public Pose2d getPose();

  /**
   * Returns the follow distance for this waypoint.
   *
   * @return the follow distance for this waypoint.
   */
  public Distance getFollowDistance();

  /**
   * Returns the timeout period of this waypoint.
   *
   * @return the timeout period of this waypoint.
   */
  public long getTimeout();
}
