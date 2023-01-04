// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.LauncherCommands.LauncherOutCommand;
import frc.robot.commands.IndexerCommands.IndexerInCommand;
import frc.robot.commands.TowerCommands.TowerUpCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LaunchCommandGroup extends ParallelCommandGroup {
  /** Creates a new LaunchCommandGroup. */
  public LaunchCommandGroup() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new LauncherOutCommand(), new IndexerInCommand(), new TowerUpCommand());
  }
}
