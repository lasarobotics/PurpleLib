// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.fsm;

public interface SystemState {
  /**
   * Initial action of state. Called once when state is initially scheduled
   */
  public default void initialize() {}

  /**
   * Main body of state. Called repeatedly when state is scheduled.
   */
  public default void execute() {}

  /**
   * The action to take when the state ends. Called when either the state finishes normally, or when it interrupted/canceled.
   * @param interrupted Whether the state was interrupted or cancelled
   */
  public default void end(boolean interrupted) {}

  /**
   * Get next state based on variety of inputs. Also used to know if current state is complete.
   * @return Next state
   */
  public SystemState nextState();
}
