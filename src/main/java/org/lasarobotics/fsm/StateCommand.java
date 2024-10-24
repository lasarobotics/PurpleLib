// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.fsm;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;

/** StateCommand */
public class StateCommand extends Command {
  private final StateMachine m_machine;
  private SystemState m_selectedState;
  private SystemState m_nextState;

  /**
   * Creates a new StateCommand
   * @param machine State machine that this state command is to be associated with
   */
  public StateCommand(StateMachine machine) {
    m_machine = requireNonNullParam(machine, "machine", "StateCommand");
    addRequirements(machine);
  }

  @Override
  public void initialize() {
    m_selectedState = m_machine.getState();
    m_nextState = m_selectedState;
    m_selectedState.initialize();
  }

  @Override
  public void execute() {
    m_selectedState.execute();
  }

  @Override
  public void end(boolean interrupted) {
    m_selectedState.end(interrupted);
    m_machine.setState(m_nextState);
  }

  @Override
  public boolean isFinished() {
    m_nextState = m_selectedState.nextState();
    return !m_nextState.equals(m_selectedState);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addStringProperty(
        "selected", () -> m_selectedState == null ? "null" : m_selectedState.toString(), null);
  }
}
