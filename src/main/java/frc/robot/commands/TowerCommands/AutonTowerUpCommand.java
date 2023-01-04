// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TowerCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class AutonTowerUpCommand extends CommandBase {
  /** Creates a new AutonTowerCommand. */
  double endTime = 0;
  double duration = 0;

  public AutonTowerUpCommand(double ms) {
    // Use addRequirements() here to declare subsystem dependencies.
  addRequirements(RobotContainer.m_towerSubsystem);
  duration = ms;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_towerSubsystem.TowerOff();
    endTime = System.currentTimeMillis() + duration;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.m_towerSubsystem.TowerUp();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_towerSubsystem.TowerOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return System.currentTimeMillis() >= endTime;
  }
}
