package org.lasarobotics.fsm;

import org.lasarobotics.hardware.Monitorable;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

public abstract class StateMachine extends Monitorable implements Subsystem, Sendable {
  private SystemState m_currentState;

  /**
   * Create a state machine
   * @param initialState Starting state for state machine
   */
  public StateMachine(SystemState initialState) {
    this.m_currentState = initialState;
    var name = this.getClass().getSimpleName();
    name = name.substring(name.lastIndexOf('.') + 1);
    SendableRegistry.addLW(this, name, name);
    CommandScheduler.getInstance().registerSubsystem(this);
    CommandScheduler.getInstance().setDefaultCommand(this, new StateCommand(this::getState, this).repeatedly());
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
   * Get current system state
   * @return Current system state
   */
  public SystemState getState() {
    return m_currentState;
  }

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
