// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.lasarobotics.fsm;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import java.util.function.Supplier;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * A command composition that runs one of a selection of commands using a selector and a key to
 * command mapping.
 *
 * <p>The rules for command compositions apply: command instances that are passed to it cannot be
 * added to any other composition or scheduled individually, and the composition requires all
 * subsystems its components require.
 *
 * <p>This class is provided by the NewCommands VendorDep
 *
 * @param <K> The type of key used to select the command
 */
public class StateCommand extends Command {
  private final Supplier<SystemState> m_selector;
  private final StateMachine m_machine;
  private SystemState m_selectedState;
  private SystemState m_nextState;

  /**
   * Creates a new SelectCommand.
   *
   * @param commands the map of commands to choose from
   * @param selector the selector to determine which command to run
   */
  public StateCommand(Supplier<SystemState> selector, StateMachine machine) {
    m_selector = requireNonNullParam(selector, "selector", "StateCommand");
    m_machine = requireNonNullParam(machine, "machine", "StateCommand");

    addRequirements(machine);
  }

  @Override
  public void initialize() {
    m_selectedState = m_selector.get();
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
