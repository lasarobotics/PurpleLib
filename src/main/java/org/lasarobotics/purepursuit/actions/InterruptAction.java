// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.purepursuit.actions;

import org.lasarobotics.purepursuit.waypoints.InterruptWaypoint;

/**
* This interface represents an action that InterruptWaypoint perform when
* they reach their interrupt point.
*
* @author Michael Baljet, Team 14470
* @version 1.0
* @see InterruptWaypoint
*/
public interface InterruptAction {

  /**
   * Performs the action.
   */
  public void doAction();

}
