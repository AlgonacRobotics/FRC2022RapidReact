// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  
  SendableChooser<String> m_chooser = new SendableChooser<>();
  SendableChooser<Integer> m_waitTime = new SendableChooser<>();

  private RobotContainer m_robotContainer;

  public static String autonomousSelection;

  public static double autonWait;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    CameraServer.startAutomaticCapture();

    m_chooser.addOption("2 Ball", "2Ball");
    m_chooser.addOption("Back off Tarmac ONLY", "tarmacReverse");
    m_chooser.addOption("Shoot Low then Back Off Tarmac", "tarmacReverseShootLow");
    m_chooser.addOption("Back off Tarmac and Shoot High", "tarmacReverseShootHigh");
    //m_chooser.addOption("3 Ball", "3Ball");

    /*m_waitTime.addOption("0 Seconds", 0);
    m_waitTime.addOption("1 Second", 1);
    m_waitTime.addOption("2 Seconds", 2);
    m_waitTime.addOption("3 Seconds", 3);
    m_waitTime.addOption("4 Seconds", 4);
    m_waitTime.addOption("5 Seconds", 5);*/

    SmartDashboard.putData("Autonomous1", m_chooser);
    //SmartDashboard.putData("waitTime", m_waitTime);
    

    //RobotContainer.m_drivetrainSubsystem.speedCounter = 12.0;
    RobotContainer.m_drivetrainSubsystem.driveReduction = 0.6;
    RobotContainer.m_drivetrainSubsystem.turnReduction = 0.3;


    /*if(m_waitTime.getSelected() != null) {
      autonWait = (m_waitTime.getSelected().intValue()) * 1000;
    } else {
      autonWait = 0;
    }*/

    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
  }

    

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autonomousSelection = m_chooser.getSelected().toString();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
