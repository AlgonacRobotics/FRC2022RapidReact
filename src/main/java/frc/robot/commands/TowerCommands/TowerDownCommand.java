// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TowerCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class TowerDownCommand extends CommandBase {
  /** Creates a new TowerDownCommand. */
  public TowerDownCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_towerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_towerSubsystem.TowerOff();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.m_towerSubsystem.TowerDown();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_towerSubsystem.TowerOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
