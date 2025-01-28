// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.purepursuit.waypoints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

/**
* A point turn waypoint is a special type of waypoint where instead of "curving" around it, the
* robot will travel to it, make a complete stop, turn towards the next waypoint, and continue.
*
* @author Michael Baljet, Team 14470
* @version 1.1
*/
public class PointTurnWaypoint extends GeneralWaypoint {

  // The expected level of error, the robot will consider itself at the waypoint when it is within the buffer. The buffers must be > 0.
  private Distance positionBuffer;
  private Angle rotationBuffer;

  // True if the robot has already "passed" this waypoint.
  private boolean traversed;

  /**
  * Constructs a PointTurnWaypoint. All values are set to their default.
  */
  public PointTurnWaypoint() {
    positionBuffer = Units.Meters.zero();
    rotationBuffer = Units.Radians.zero();
    traversed = false;
  }

  /**
  * Constructs a PointTurnWaypoint with the provided values.
  *
  * @param translation    The (x, y) translation of this waypoint.
  * @param rotation       The rotation (preferred angle) of this waypoint.
  * @param movementSpeed  The speed in which the robot moves at while traversing this waypoint, in the range [0, 1].
  * @param turnSpeed      The speed in which the robot turns at while traversing this waypoint, in the range [0, 1].
  * @param followRadius   The distance in which the robot traverses this waypoint. Please see guides to learn more about this value.
  * @param positionBuffer The expected level of error, the robot will consider itself at the waypoint when it is within the buffer. The buffer must be > 0.
  * @param rotationBuffer The expected level of error (in radians), the robot will consider itself at the waypoint when it is within the buffer. The buffer must be > 0.
  */
  public PointTurnWaypoint(Translation2d translation, Rotation2d rotation, LinearVelocity movementSpeed, AngularVelocity turnSpeed, Distance followRadius, Distance positionBuffer, Angle rotationBuffer) {
    super(new Pose2d(translation, rotation), movementSpeed, turnSpeed, followRadius);
    this.positionBuffer = verifyPositionBuffer(positionBuffer);
    this.rotationBuffer = verifyRotationBuffer(rotationBuffer);
    traversed = false;
  }

  /**
  * Constructs a PointTurnWaypoint with the provided values.
  *
  * @param pose           Position and rotation (preferred angle) of this waypoint.
  * @param movementSpeed  The speed in which the robot moves at while traversing this waypoint, in the range [0, 1].
  * @param turnSpeed      The speed in which the robot turns at while traversing this waypoint, in the range [0, 1].
  * @param followRadius   The distance in which the robot traverses this waypoint. Please see guides to learn more about this value.
  * @param positionBuffer The expected level of error, the robot will consider itself at the waypoint when it is within the buffer. The buffer must be > 0.
  * @param rotationBuffer The expected level of error (in radians), the robot will consider itself at the waypoint when it is within the buffer. The buffer must be > 0.
  */
  public PointTurnWaypoint(Pose2d pose, LinearVelocity movementSpeed, AngularVelocity turnSpeed, Distance followRadius, Distance positionBuffer, Angle rotationBuffer) {
    super(pose, movementSpeed, turnSpeed, followRadius);
    this.positionBuffer = verifyPositionBuffer(positionBuffer);
    this.rotationBuffer = verifyRotationBuffer(rotationBuffer);
    traversed = false;
  }

  /**
  * Returns this waypoint's position buffer.
  *
  * @return this waypoint's position buffer.
  */
  public Distance getPositionBuffer() {
    return positionBuffer;
  }

  /**
  * Returns this waypoint's rotation buffer.
  *
  * @return this waypoint's rotation buffer.
  */
  public Angle getRotationBuffer() {
    return rotationBuffer;
  }

  /**
  * Sets this waypoint's position buffer.
  *
  * @param buffer Position buffer to be set.
  * @return This PointTurnWaypoint, used for chaining methods.
  */
  public PointTurnWaypoint setPositionBuffer(Distance buffer) {
    positionBuffer = verifyPositionBuffer(buffer);
    return this;
  }

  /**
  * Sets this waypoint's rotation buffer.
  *
  * @param buffer Rotation buffer to be set.
  * @return This PointTurnWaypoint, used for chaining methods.
  */
  public PointTurnWaypoint setRotationBuffer(Angle buffer) {
    rotationBuffer = verifyRotationBuffer(buffer);
    return this;
  }

  /**
  * Returns true if this waypoint has already been traversed, false otherwise.
  *
  * @return true if this waypoint has already been traversed, false otherwise.
  */
  public boolean hasTraversed() {
    return traversed;
  }

  /**
  * Tells the waypoint that it has been traversed.
  */
  public void setTraversed() {
    traversed = true;
  }

  /**
   * Verified the buffer it valid. The buffer is valid if it is > 1.
   *
   * @param buffer Buffer to be checked.
   * @return True if the buffer is valid, false otherwise.
   */
  private Distance verifyPositionBuffer(Distance buffer) {
    if (buffer.lt(Units.Meters.zero()))
      throw new IllegalArgumentException("The buffer must be > 0");
    return buffer;
  }

  /**
   * Verified the buffer it valid. The buffer is valid if it is > 1.
   *
   * @param buffer Buffer to be checked.
   * @return True if the buffer is valid, false otherwise.
   */
  private Angle verifyRotationBuffer(Angle buffer) {
    if (buffer.lt(Units.Radians.zero()))
      throw new IllegalArgumentException("The buffer must be > 0");
    return buffer;
  }

  @Override
  public void reset() {
    traversed = false;
  }

  @Override
  public Waypoint.Type getType() {
    return Waypoint.Type.POINT_TURN;
  }

  @Override
  public String toString() {
    return String.format("PointTurnWaypoint(%s, %s)", getTranslation().getX(), getTranslation().getY());
  }

}
