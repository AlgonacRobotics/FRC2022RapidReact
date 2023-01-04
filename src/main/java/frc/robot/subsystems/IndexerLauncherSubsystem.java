// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerLauncherSubsystem extends SubsystemBase {
  /** Creates a new IndexerLauncherSubsystem. */

  private double Launcherspeed = (0.5); //SPEED OF LAUNCHER

  /**
   * Creates a new IndexLauncherSubsystem.
   */
  WPI_TalonSRX indexerLauncher1 = new WPI_TalonSRX(Constants.indexerMotor1);

  

  public IndexerLauncherSubsystem() {
    indexerLauncher1.configFactoryDefault();
  };

  public void IndexerIn() {
    indexerLauncher1.set(-Launcherspeed);
  }

  public void IndexerOff() {
    indexerLauncher1.set(0);
  }

  public void IndexerOut() {
    indexerLauncher1.set(Launcherspeed);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
