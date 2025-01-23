// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.purepursuit.actions;

import org.lasarobotics.purepursuit.Path;

/**
* This is an optional feature of paths. A triggered action is an action that called
* when some condition is met. For example, you might have a triggered action that
* triggers when the robot move above some Y value on the field.
*
* @author Michael Baljet, Team 14470
* @version 1.1
* @see Path
*/
public abstract class TriggeredAction {

  // True if doAction() has already been called.
  private boolean m_alreadyPerformed = false;

  /**
   * Called regularly by the path it is apart of. If the trigger condition is met this will call doAction().
   */
  public void loop() {
    if (isTriggered() && !m_alreadyPerformed) {
      doAction();
      m_alreadyPerformed = true;
    }
  }

  /**
   * Resets this actions.
   */
  public void reset() {
    m_alreadyPerformed = false;
  }

  /**
   * Returns true if the trigger condition is met and the action should be performed.
   *
   * @return true if the trigger condition is met and the action should be performed, false otherwise.
   */
  public abstract boolean isTriggered();

  /**
   * Perform the triggered action. Automatically called when the trigger condition is met.
   *
   * @param m_alreadyPerformed True if the action has already been performed, false otherwise.
   */
  public abstract void doAction();

}
