// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.purepursuit.waypoints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;

/**
* A general waypoint is the most common type of Waypoint. This waypoint acts like a
* conventional pure pursuit waypoint, with the robot simply traversing it. Most other
* types of waypoints are sub classes of this.
*
* @author Michael Baljet, Team 14470
* @version 1.1
* @see Waypoint
*/
public class GeneralWaypoint extends Pose2d implements Waypoint {

  // If the robot moves towards this waypoint for longer than the timeout period, the path is aborted.
  private long m_timeoutMiliseconds;

  // Speed at which the robot moves
  private MutLinearVelocity m_movementVelocity = Units.MetersPerSecond.mutable(0.0);

  // Speed at which the robot rotates
  private MutAngularVelocity m_rotateVelocity = Units.RadiansPerSecond.mutable(0.0);

  // The pure pursuit follow radius.
  private MutDistance m_followRadius = Units.Meters.mutable(0.0);

  // True if this waypoint uses a preferred angle.
  private boolean usePreferredAngle;
  private MutAngle preferredAngle = Units.Radians.mutable(0.0);

  // True if this waypoint is to inherit the previous node's configuration.
  private boolean copyMode;

  /**
  * Constructs a GeneralWaypoint. All values are set to their default.
  */
  public GeneralWaypoint() {
    // Set values to default.
    m_movementVelocity.mut_replace(0.0, Units.MetersPerSecond);
    m_rotateVelocity.mut_replace(0.0, Units.RadiansPerSecond);
    m_followRadius.mut_replace(0.0, Units.Meters);
    m_timeoutMiliseconds = -1;
    usePreferredAngle = false;
    copyMode = false;
  }

  public GeneralWaypoint(Pose2d pose) {
    this(pose, Units.MetersPerSecond.zero(), Units.RadiansPerSecond.zero(), Units.Meters.zero());
    copyMode = true;
  }

  /**
  * Constructs a GeneralWaypoint with the provided values.
  *
  * @param pose          Position and rotation (preferred angle) of this waypoint.
  * @param movementVelocity The speed in which the robot moves at while traversing this waypoint, in the range [0, 1].
  * @param rotateVelocity     The speed in which the robot turns at while traversing this waypoint, in the range [0, 1].
  * @param followRadius  The distance in which the robot traverses this waypoint. Please see guides to learn more about this value.
  */
  public GeneralWaypoint(Pose2d pose, LinearVelocity movementVelocity, AngularVelocity rotateVelocity, Distance followRadius) {
    super(pose.getTranslation(), pose.getRotation());
    this.m_movementVelocity.mut_replace(movementVelocity);
    this.m_rotateVelocity.mut_replace(rotateVelocity);
    this.m_followRadius.mut_replace(followRadius);
    m_timeoutMiliseconds = -1;
    usePreferredAngle = true;
    preferredAngle.mut_replace(pose.getRotation().getMeasure());
    copyMode = false;
  }

  /**
  * Returns the movement speed of this waypoint.
  *
  * @return the movement speed of this waypoint.
  */
  public LinearVelocity getMovementVelocity() {
    return m_movementVelocity;
  }

  /**
  * Returns the turn speed of this waypoint.
  *
  * @return the turn speed of this waypoint.
  */
  public AngularVelocity getRotateVelocity() {
    return m_rotateVelocity;
  }

  /**
  * Returns the follow radius of this waypoint.
  *
  * @return the follow radius of this waypoint.
  */
  public Distance getFollowRadius() {
    return m_followRadius;
  }

  /**
  * Returns this waypoint's preferred angle (in radians).
  *
  * @return this waypoint's preferred angle (in radians).
  * @throws IllegalStateException If this waypoint is not using a preferred angle.
  */
  public Angle getPreferredAngle() {
    if (!usePreferredAngle)
      throw new IllegalStateException("This waypoint is not using a preferredAngle");
    return preferredAngle;
  }

  /**
  * Returns true if this waypoint is using a preferred angle.
  *
  * @return true if this waypoint is using a preferred angle, false otherwise.
  */
  public boolean usingPreferredAngle() {
    return usePreferredAngle;
  }

  /**
  * Sets the movement speed of this waypoint.
  *
  * @param movementVelocity Speed to set.
  * @return This GeneralWaypoint, used for chaining methods.
  */
  public GeneralWaypoint setMovementSpeed(LinearVelocity movementVelocity) {
    m_movementVelocity.mut_replace(movementVelocity);
    return this;
  }

  /**
  * Sets the turn speed of this waypoint.
  *
  * @param rotateVelocity Speed to be set.
  * @return This GeneralWaypoint, used for chaining methods.
  */
  public GeneralWaypoint setRotateSpeed(AngularVelocity rotateVelocity) {
    this.m_rotateVelocity.mut_replace(rotateVelocity);
    return this;
  }

  /**
  * Sets the follow radius of this waypoint.
  *
  * @param followRadius Radius to be set.
  * @return This GeneralWaypoint, used for chaining methods.
  */
  public GeneralWaypoint setFollowRadius(Distance followRadius) {
    this.m_followRadius.mut_replace(followRadius);
    return this;
  }

  /**
  * Sets and enables this waypoint's preferred angle.
  *
  * @param angle Angle to be set.
  * @return This GeneralWaypoint, used for chaining methods.
  */
  public GeneralWaypoint setPreferredAngle(Angle angle) {
    usePreferredAngle = true;
    preferredAngle.mut_replace(angle);
    return this;
  }

  /**
  * Sets the timeout period of this waypoint. This is optional.
  *
  * @param timeoutMiliseconds The timeout period of this waypoint.
  * @return This GeneralWaypoint, used for chaining methods.
  */
  public GeneralWaypoint setTimeout(long timeoutMiliseconds) {
    this.m_timeoutMiliseconds = timeoutMiliseconds;
    return this;
  }

  /**
  * Disables this waypoint's preferredAngle. This is disabled by default.
  *
  * @return This GeneralWaypoint, used for chaining methods.
  */
  public GeneralWaypoint disablePreferredAngle() {
    usePreferredAngle = false;
    preferredAngle.mut_replace(Units.Radians.zero());
    return this;
  }

  /**
  * Resets this waypoint. This is called by Path.
  */
  public void reset() {
    // GeneralWaypoints don't have anything to reset.
  }

  /**
  * Copies configuration from the given waypoint. Does nothing if copy mode is disabled.
  *
  * @param waypoint Waypoint to copy.
  */
  public void inherit(Waypoint waypoint) {
    if (!copyMode) return;
    if (!(waypoint instanceof GeneralWaypoint))
      throw new IllegalArgumentException("A " + getType() + " waypoint cannot inherit the configuration of a " + waypoint.getType() + " waypoint.");

    var w = (GeneralWaypoint)waypoint;
    setMovementSpeed(w.getMovementVelocity());
    setRotateSpeed(w.getRotateVelocity());
    setFollowRadius(w.getFollowRadius());
    setTimeout(w.getTimeout());

    if (w.usingPreferredAngle()) setPreferredAngle(w.getPreferredAngle());
    else usePreferredAngle = false;
  }

  @Override
  public Waypoint.Type getType() {
    return Waypoint.Type.GENERAL;
  }

  @Override
  public Pose2d getPose() {
    return this;
  }

  @Override
  public Distance getFollowDistance() {
    return m_followRadius;
  }

  @Override
  public long getTimeout() {
    return m_timeoutMiliseconds;
  }

  @Override
  public String toString() {
    return String.format("GeneralWaypoint(%s, %s)", getTranslation().getX(), getTranslation().getY());
  }
}
