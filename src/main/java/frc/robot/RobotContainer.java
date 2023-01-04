// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IndexerLauncherSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystemEncoder;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.TowerSubsystem;

import java.util.List;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ClimberCommands.ClimbDownCommand;
import frc.robot.commands.ClimberCommands.ClimbUpCommand;
import frc.robot.commands.CommandGroups.AutonShootCommandGroup;
import frc.robot.commands.CommandGroups.BallLoadCommandGroup;
import frc.robot.commands.CommandGroups.BallOutCommandGroup;
import frc.robot.commands.CommandGroups.LaunchCommandGroup;
import frc.robot.commands.CommandGroups.LauncherSpitCommandGroup;
import frc.robot.commands.CommandGroups.MasterLaunchCommandGroup;
import frc.robot.commands.IndexerCommands.AutonIndexerInCommand;
import frc.robot.commands.IndexerCommands.IndexerInCommand;
import frc.robot.commands.IndexerCommands.IndexerOutCommand;

import frc.robot.commands.IntakeCommands.AutonIntakeDownCommand;
import frc.robot.commands.IntakeCommands.AutonIntakeInCommand;
import frc.robot.commands.IntakeCommands.IntakeDownCommand;
import frc.robot.commands.LauncherCommands.AutonLauncherOutCommand;
import frc.robot.commands.LauncherCommands.AutonLauncherSpitCommand;
import frc.robot.commands.TowerCommands.AutonTowerUpCommand;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
import frc.robot.commands.IntakeCommands.IntakeInCommand;
import frc.robot.commands.IntakeCommands.IntakeOutCommand;
import frc.robot.commands.IntakeCommands.IntakeUpAutonCommand;
import frc.robot.commands.IntakeCommands.IntakeUpCommand;
import frc.robot.commands.LauncherCommands.LauncherOutCommand;
import frc.robot.commands.LauncherCommands.LauncherSpitCommand;
import frc.robot.commands.TowerCommands.TowerDownCommand;
import frc.robot.commands.TowerCommands.TowerUpCommand;
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  
  private final IntakeInCommand m_intakeInCommand = new IntakeInCommand();
  private final IntakeOutCommand m_intakeOutCommand = new IntakeOutCommand();
  private final IntakeUpCommand m_intakeUpCommand = new IntakeUpCommand();
  private final IntakeDownCommand m_intakeDownCommand = new IntakeDownCommand();
  private final ClimbDownCommand m_climbDownCommand = new ClimbDownCommand();
  private final ClimbUpCommand m_climbUpCommand = new ClimbUpCommand();
  private final IndexerInCommand m_indexerInCommand = new IndexerInCommand();
  private final IndexerOutCommand m_indexerOutCommand = new IndexerOutCommand();
  private final LauncherOutCommand m_launcherOutCommand = new LauncherOutCommand();
  private final MasterLaunchCommandGroup m_masterLaunchCommandGroup = new MasterLaunchCommandGroup();
  private final TowerUpCommand m_towerUpCommand = new TowerUpCommand();
  private final TowerDownCommand m_towerDownCommand = new TowerDownCommand();
  private final AutonShootCommandGroup m_autonShootCommandGroup = new AutonShootCommandGroup();
  private final LauncherSpitCommandGroup m_launcherSpitCommand = new LauncherSpitCommandGroup();
  private final BallOutCommandGroup m_ballOutCommand = new BallOutCommandGroup();
  //private final MasterLaunchSpitCommandGroup m_masterLaunchSpitCommand = new MasterLaunchSpitCommandGroup();

  public static final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();

  private final XboxController m_driverController = new XboxController(0);
  private final XboxController m_operatorController = new XboxController(1);

  public static final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  public static final IndexerLauncherSubsystem m_indexerSubsystem = new IndexerLauncherSubsystem();
  public static final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  public static final LauncherSubsystem m_launcherSubsystem = new LauncherSubsystem();
  public static final TowerSubsystem m_towerSubsystem = new TowerSubsystem();
  public static final IntakeSubsystemEncoder m_intakeAutonSubsystem = new IntakeSubsystemEncoder();
 

  public static final BallLoadCommandGroup m_loadCommandGroup = new BallLoadCommandGroup();
  public static final LaunchCommandGroup m_launchCommandGroup = new LaunchCommandGroup();

  //public ChassisSpeeds m_autochassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    


  // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> -modifyAxis(m_driverController.getLeftY()) * m_drivetrainSubsystem.driveReduction * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_driverController.getLeftX()) * m_drivetrainSubsystem.driveReduction * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_driverController.getRightX()) * m_drivetrainSubsystem.turnReduction * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
     // Back but****ton zeros the gyroscope
     new Button(m_driverController::getBackButton)
     // No requirements because we don't need to interrupt anything
     .whenPressed(m_drivetrainSubsystem::zeroGyroscope);

     //Operator Controller
     new Button(m_operatorController::getLeftBumper).whileHeld(m_intakeUpCommand);
     new Button(m_operatorController::getRightBumper).whileHeld(m_intakeDownCommand);
     new Button(m_operatorController::getXButton).whileHeld(m_launchCommandGroup);
     new Button(m_operatorController::getYButton).whileHeld(m_loadCommandGroup);
     new Button(m_operatorController::getBButton).whileHeld(m_launcherSpitCommand);
     new Button(m_operatorController::getAButton).whileHeld(m_launcherOutCommand); 
     new Button(m_operatorController::getStartButton).whileHeld(m_ballOutCommand);
     new Button(m_operatorController::getBackButton).whileHeld(m_towerDownCommand);
     //new Button(m_operatorController::getLeftStickButton).whileHeld(m_towerDownCommand);

     //Driver Controller
     //new Button(m_driverController::getXButton).whileHeld(m_towerUpCommand);
     //new Button(m_driverController::getYButton).whileHeld(m_towerDownCommand);
     //new Button(m_driverController::getAButton).whileHeld(m_loadCommandGroup);
     //new Button(m_driverController::getBButton).whileHeld(m_launchCommandGroup);
     //new Button(m_driverController::getStartButton).whileHeld(m_launcherOutCommand);
     new Button(m_driverController::getLeftBumper).whileHeld(new InstantCommand(()-> m_drivetrainSubsystem.enableCrawlSpeed()));
     new Button(m_driverController::getLeftBumper).whenReleased(new InstantCommand(()-> m_drivetrainSubsystem.disableCrawlSpeed()));
     new Button(m_driverController::getXButton).whileHeld(m_climbDownCommand);
     new Button(m_driverController::getYButton).whileHeld(m_climbUpCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    //Create Trajectory Configs
    TrajectoryConfig trajectoryConfigStandardSpeed = new TrajectoryConfig((6380.0 / 60.0 *
    SdsModuleConfigurations.MK4_L1.getDriveReduction() *
    SdsModuleConfigurations.MK4_L1.getWheelDiameter() * Math.PI)/2, 1)
    .setKinematics(m_drivetrainSubsystem.m_kinematics);

    TrajectoryConfig trajectoryConfigSlowSpeed = new TrajectoryConfig((6380.0 / 60.0 *
    SdsModuleConfigurations.MK4_L1.getDriveReduction() *
    SdsModuleConfigurations.MK4_L1.getWheelDiameter() * Math.PI)/3, 1)
    .setKinematics(m_drivetrainSubsystem.m_kinematics);

    TrajectoryConfig trajectoryConfigFastSpeed = new TrajectoryConfig((6380.0 / 60.0 *
    SdsModuleConfigurations.MK4_L1.getDriveReduction() *
    SdsModuleConfigurations.MK4_L1.getWheelDiameter() * Math.PI), 3)
    .setKinematics(m_drivetrainSubsystem.m_kinematics);

    TrajectoryConfig trajectoryConfigStandardSpeedReverse = new TrajectoryConfig((6380.0 / 60.0 *
    SdsModuleConfigurations.MK4_L1.getDriveReduction() *
    SdsModuleConfigurations.MK4_L1.getWheelDiameter() * Math.PI)/2, 1)
    .setKinematics(m_drivetrainSubsystem.m_kinematics);
    trajectoryConfigStandardSpeedReverse.setReversed(true);

    //Generate Trajectory
  
    Trajectory ballPickup = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, Rotation2d.fromDegrees(0)), 
      List.of(
        new Translation2d(0.5, 0),
        new Translation2d(1, 0),
        new Translation2d(1.5, 0)
      ), 
      new Pose2d(2, 0, Rotation2d.fromDegrees(0)), 
      trajectoryConfigStandardSpeed);

      Trajectory backUp = TrajectoryGenerator.generateTrajectory(
        new Pose2d(2, 0, Rotation2d.fromDegrees(0)), 
        List.of(
          new Translation2d(1.5, -0.25)
        ), 
        new Pose2d(1, -1, Rotation2d.fromDegrees(190)), 
        trajectoryConfigStandardSpeedReverse);

        Trajectory shootBall = TrajectoryGenerator.generateTrajectory(
        new Pose2d(1, -1, Rotation2d.fromDegrees(190)), 
        List.of(
          new Translation2d(1.5, -0.25)
        ), 
        new Pose2d(1.25, 0.2, Rotation2d.fromDegrees(190)), 
        trajectoryConfigStandardSpeedReverse);

        Trajectory tarmacReverse = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, Rotation2d.fromDegrees(180)), 
        List.of(
          new Translation2d(1, 0)
        ), 
        new Pose2d(2.25, 0, Rotation2d.fromDegrees(180)), 
        trajectoryConfigStandardSpeedReverse);

        Trajectory tarmacReverseToShoot = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, Rotation2d.fromDegrees(180)), 
        List.of(
          new Translation2d(1, 0)
        ), 
        new Pose2d(1.25, 0, Rotation2d.fromDegrees(180)), 
        trajectoryConfigStandardSpeedReverse);

        Trajectory shootToOut = TrajectoryGenerator.generateTrajectory(
        new Pose2d(1.25, 0, Rotation2d.fromDegrees(180)), 
        List.of(
          new Translation2d(1.75, 0)
        ), 
        new Pose2d(2.25, 0, Rotation2d.fromDegrees(180)), 
        trajectoryConfigStandardSpeedReverse);

        Trajectory ThreeballPickup = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, Rotation2d.fromDegrees(175)), 
        List.of(
          new Translation2d(1, 0.5)
        ), 
        new Pose2d(2, 1, Rotation2d.fromDegrees(0)), 
        trajectoryConfigStandardSpeedReverse);


      
    //Define PID Controllers for tracking trajectory
    PIDController xController = new PIDController(1.5, 0, 0);
    PIDController yController = new PIDController(1.5, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(3, 0, 0, Constants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);


    //return m_autoCommand;

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
      ballPickup, 
      m_drivetrainSubsystem::getPose, 
      m_drivetrainSubsystem.m_kinematics, 
      xController, 
      yController, 
      thetaController, 
      m_drivetrainSubsystem::getModuleState, 
      m_drivetrainSubsystem);

      SwerveControllerCommand swerveControllerCommand2 = new SwerveControllerCommand(
      backUp, 
      m_drivetrainSubsystem::getPose, 
      m_drivetrainSubsystem.m_kinematics, 
      xController, 
      yController, 
      thetaController, 
      m_drivetrainSubsystem::getModuleState, 
      m_drivetrainSubsystem);

      SwerveControllerCommand swerveControllerCommand3 = new SwerveControllerCommand(
      shootBall, 
      m_drivetrainSubsystem::getPose, 
      m_drivetrainSubsystem.m_kinematics, 
      xController, 
      yController, 
      thetaController, 
      m_drivetrainSubsystem::getModuleState, 
      m_drivetrainSubsystem);

      SwerveControllerCommand tarmacReverseController = new SwerveControllerCommand(
        tarmacReverse, 
        m_drivetrainSubsystem::getPose, 
        m_drivetrainSubsystem.m_kinematics, 
        xController, 
        yController, 
        thetaController, 
        m_drivetrainSubsystem::getModuleState, 
        m_drivetrainSubsystem);
      
      SwerveControllerCommand tarmacReverseToShootController = new SwerveControllerCommand(
        tarmacReverseToShoot, 
        m_drivetrainSubsystem::getPose, 
        m_drivetrainSubsystem.m_kinematics, 
        xController, 
        yController, 
        thetaController, 
        m_drivetrainSubsystem::getModuleState, 
        m_drivetrainSubsystem);

      SwerveControllerCommand shootToOutController = new SwerveControllerCommand(
        shootToOut, 
        m_drivetrainSubsystem::getPose, 
        m_drivetrainSubsystem.m_kinematics, 
        xController, 
        yController, 
        thetaController, 
        m_drivetrainSubsystem::getModuleState, 
        m_drivetrainSubsystem);

        SwerveControllerCommand ThreeBallPickupController = new SwerveControllerCommand(
        ThreeballPickup, 
        m_drivetrainSubsystem::getPose, 
        m_drivetrainSubsystem.m_kinematics, 
        xController, 
        yController, 
        thetaController, 
        m_drivetrainSubsystem::getModuleState, 
        m_drivetrainSubsystem);

    //Auton base
    /*return new SequentialCommandGroup(
      new InstantCommand(()-> m_drivetrainSubsystem.resetOdometry(.getInitialPose())),
      new InstantCommand(() -> m_drivetrainSubsystem.stopModules()));*/

     //Add some init and wrap-up and return everything
     if(Robot.autonomousSelection.equals("2Ball")){
     return new SequentialCommandGroup(
      new InstantCommand(()-> m_drivetrainSubsystem.resetOdometry(ballPickup.getInitialPose())),
      new SequentialCommandGroup(new AutonIntakeDownCommand(600)),
      new ParallelCommandGroup(swerveControllerCommand, new AutonIntakeInCommand(3000), new AutonIndexerInCommand(3000)),
      swerveControllerCommand2,
      new ParallelCommandGroup(swerveControllerCommand3, new AutonLauncherOutCommand(3500)),
      new ParallelCommandGroup(new AutonIndexerInCommand(5000), new AutonTowerUpCommand(5000), new AutonLauncherOutCommand(5000)),
      new InstantCommand(() -> m_drivetrainSubsystem.stopModules()));

     } else if(Robot.autonomousSelection.equals("tarmacReverse")){
    //Tarmac Reverse
      return new SequentialCommandGroup(
      new InstantCommand(()-> m_drivetrainSubsystem.resetOdometry(tarmacReverse.getInitialPose())),
      tarmacReverseController,
      new InstantCommand(() -> m_drivetrainSubsystem.stopModules()));

     } else if(Robot.autonomousSelection.equals("tarmacReverseShootLow")){
    //Low Goal Tarmac Reverse
      return new SequentialCommandGroup(
      new InstantCommand(()-> m_drivetrainSubsystem.resetOdometry(tarmacReverse.getInitialPose())),
      new ParallelCommandGroup(new AutonLauncherSpitCommand(3000), new AutonTowerUpCommand(3000), new AutonIndexerInCommand(3000)),
      tarmacReverseController,
      new InstantCommand(() -> m_drivetrainSubsystem.stopModules()));

     } else if(Robot.autonomousSelection.equals("tarmacReverseShootHigh")){
    //Shoot High Tarmac Reverse
     return new SequentialCommandGroup(
      new InstantCommand(()-> m_drivetrainSubsystem.resetOdometry(tarmacReverseToShoot.getInitialPose())),
      new ParallelCommandGroup(tarmacReverseToShootController, new AutonLauncherOutCommand(3500)),
      new ParallelCommandGroup(new AutonLauncherOutCommand(3500), new AutonTowerUpCommand(3500), new AutonIndexerInCommand(3500)),
      shootToOutController,
      new InstantCommand(() -> m_drivetrainSubsystem.stopModules()));
      //3 Ball
     /*} else if(Robot.autonomousSelection.equals("3Ball")){
      return new SequentialCommandGroup(
      new InstantCommand(()-> m_drivetrainSubsystem.resetOdometry(ThreeballPickup.getInitialPose())),
      new ParallelCommandGroup(new AutonTowerUpCommand(2500), new AutonIndexerInCommand(2500), new AutonLauncherOutCommand(2500)),
      ThreeBallPickupController,
      new InstantCommand(() -> m_drivetrainSubsystem.stopModules()));*/
      

     } else {
       return null;
     }
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
