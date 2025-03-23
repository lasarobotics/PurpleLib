// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.fsm;

import java.util.Optional;

import org.lasarobotics.hardware.Monitorable;
import org.lasarobotics.hardware.PurpleManager;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** State machine subsystem */
public abstract class StateMachine extends Monitorable implements Subsystem, Sendable {
  private SystemState m_currentState;
  private Optional<SystemState> m_requestedState;

  /**
   * Create a state machine
   * @param initialState Starting state for state machine
   */
  public StateMachine(SystemState initialState) {
    this.m_currentState = initialState;
    m_requestedState = Optional.empty();
    var name = this.getClass().getSimpleName();
    name = name.substring(name.lastIndexOf('.') + 1);
    SendableRegistry.addLW(this, name, name);
    CommandScheduler.getInstance().registerSubsystem(this);
    CommandScheduler.getInstance().setDefaultCommand(this, new StateCommand(this).repeatedly());
    PurpleManager.add(this);
  }

  /**
   * Set system state
   * <p>
   * ONLY TO BE USED BY {@link StateCommand#end(boolean)}
   */
  void setState(SystemState state) {
    m_currentState = state;
  }

  /**
   * Get the state that has been requested of this state machine
   * @return Requested state
   */
  Optional<SystemState> getStateRequest() {
    return m_requestedState;
  }

  void resetStateRequest() {
    m_requestedState = Optional.empty();
  }

  /**
   * Requests a state transition.  The state machine *may* accept or reject this request.
   * @param requestedState The desired state.
   * @return True if the request was accepted, false otherwise.
   */
  public boolean requestState(SystemState requestedState) {
    if (isValidState(requestedState)) {
      m_requestedState = Optional.of(requestedState);
      return true; // Request accepted
    } else return false; // Request rejected
  }

  /**
   * Get current system state
   * @return Current system state
   */
  public SystemState getState() {
    return m_currentState;
  }

  /**
   * Checks if a given state is a valid state for this state machine.
   * @param state The state to check.
   * @return True if the state is valid, false otherwise.
   */
  protected abstract boolean isValidState(SystemState state);

  /**
   * Not valid for state machine
   */
  @Override
  public void setDefaultCommand(Command commmand) {}

  /**
   * Not valid for state machine
   */
  @Override
  public void removeDefaultCommand() {}

  /**
   * Gets the name of this state machine
   * @return name
   */
  @Override
  public String getName() {
    return SendableRegistry.getName(this);
  }

  /**
   * Associates a {@link Sendable} with this state machine. Also update the child's name.
   * @param name name to give child
   * @param child sendable
   */
  public void addChild(String name, Sendable child) {
    SendableRegistry.addLW(child, getName(), name);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Subsystem");
    builder.addStringProperty("State", () -> getState().toString(), (input) -> {});
  }
}
