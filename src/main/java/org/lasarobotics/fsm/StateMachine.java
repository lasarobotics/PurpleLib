package org.lasarobotics.fsm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class StateMachine extends SubsystemBase {
  private SystemState m_currentState;

  /**
   * Create a state machine
   * @param initialState Starting state for state machine
   */
  public StateMachine(SystemState initialState) {
    this.m_currentState = initialState;
    super.setDefaultCommand(new StateCommand(this::getState, this).repeatedly());
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

  @Override
  public void setDefaultCommand(Command commmand) {
    return;
  }
}
