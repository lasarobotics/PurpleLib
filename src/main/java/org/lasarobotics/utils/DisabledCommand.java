// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.utils;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DisabledCommand extends InstantCommand {
  private final Runnable m_toRun;

  /**
   * Creates a new DisabledCommand that runs the given Runnable with the given requirements, and is allowed to run while disabled
   *
   * @param toRun the Runnable to run
   * @param requirements the subsystems required by this command
   */
  public DisabledCommand(Runnable toRun, Subsystem... requirements) {
    this.m_toRun = toRun;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(requirements);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_toRun.run();
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
