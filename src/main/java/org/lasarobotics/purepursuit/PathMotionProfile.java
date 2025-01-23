// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.purepursuit;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

/**
* This class is utility class that is used by Path to adjust the robot speed as it approaches
* or leaves a destination. Users can use this class to create a custom motion profile.
*
* @author Michael Baljet, Team 14470
* @version 1.1
* @see Path
*/
public abstract class PathMotionProfile {

  // Keeps track of previous method calls.
  private Distance lastDistanceToTarget;
  private long lastCallTimeStamp;

  // True if last call was decelerate, false if last call was accelerate
  private boolean lastCallType;

  /**
  * Constructs a PathMotionProfile object. The user just implement the adjustMotorSpeeds() method.
  */
  public PathMotionProfile() {
    lastCallTimeStamp = -1;
    lastDistanceToTarget = Units.Meters.zero();
    lastCallType = true;
  }

  /**
  * Adjusts the motor speeds to decelerate the robot based on this motion profile. This is used to slow the robot when it approaches a target point. This method is called by Path
  * to adjust it's motor speeds while decelerating. Calling this method will relay the given information to the decelerate() method.
  *
  * @param speeds             Raw motor speeds.
  * @param distanceToTarget        Distance the robot is from it's target.
  * @param configuredMovementSpeed Configured movement speed.
  * @param configuredTurnSpeed     Configured turn speed.
  */
  public void processDecelerate(ChassisSpeeds speeds, Distance distanceToTarget, LinearVelocity configuredMovementSpeed, AngularVelocity configuredTurnSpeed) {
    if (lastCallType == true) { // Call decelerate().
      decelerate(
        speeds,
        distanceToTarget,
        Units.MetersPerSecond.of((lastDistanceToTarget.minus(distanceToTarget).in(Units.Meters)) / ((System.nanoTime() - lastCallTimeStamp) * 1e9)),
        configuredMovementSpeed,
        configuredTurnSpeed
      );
    } else { // If the last call was not a decelerate, then skip the first call.
      lastCallType = true;
      // Update fields.
      lastDistanceToTarget = distanceToTarget;
      lastCallTimeStamp = System.nanoTime();
    }
  }

  /**
  * Adjusts the motor speeds to accelerate the robot based on this motion profile. This is used to speed up the robot when it leaves a target point. This method is called by Path
  * to adjust it's motor speeds while accelerating. Calling this method will relay the given information to the accelerate() method.
  *
  * @param speeds             Raw motor speeds.
  * @param distanceFromTarget      Distance the robot is from it's target.
  * @param configuredMovementSpeed Configured movement speed.
  * @param configuredTurnSpeed     Configured turn speed.
  */
  public void processAccelerate(ChassisSpeeds speeds, Distance distanceFromTarget, LinearVelocity configuredMovementSpeed, AngularVelocity configuredTurnSpeed) {
    if (lastCallType == false) {// Call accelerate().
      accelerate(
        speeds,
        distanceFromTarget,
        Units.MetersPerSecond.of((distanceFromTarget.minus(lastDistanceToTarget).in(Units.Meters)) / ((System.nanoTime() - lastCallTimeStamp) * 1e9)),
        configuredMovementSpeed,
        configuredTurnSpeed
      );
    } else {
      // If the last call was not a decelerate, then skip the first call.
      lastCallType = false;
      // Update fields.
      lastDistanceToTarget = distanceFromTarget;
      lastCallTimeStamp = System.nanoTime();
    }
  }

  /**
  * Decelerates the motor speeds. This is used to slow the robot down when it approaches a target point.
  *
  * @param speeds             Raw motor speeds.
  * @param distanceToTarget        Distance the robot is from the target destination.
  * @param speed                   The robot's calculated speed, in units/second.
  * @param configuredMovementSpeed Configured movement speed.
  * @param configuredTurnSpeed     Configured turn speed.
  */
  public abstract void decelerate(ChassisSpeeds speeds, Distance distanceToTarget, LinearVelocity speed, LinearVelocity configuredMovementSpeed, AngularVelocity configuredTurnSpeed);

  /**
  * Accelerates the motor speeds. This is used to speed the robot up when it approaches a target point.
  *
  * @param speeds             Raw motor speeds.
  * @param distanceFromTarget      Distance the robot is from the target.
  * @param speed                   The robot's calculated speed, in units/second.
  * @param configuredMovementSpeed Configured movement speed.
  * @param configuredTurnSpeed     Configured turn speed.
  */
  public abstract void accelerate(ChassisSpeeds speeds, Distance distanceFromTarget, LinearVelocity speed, LinearVelocity configuredMovementSpeed, AngularVelocity configuredTurnSpeed);
}
