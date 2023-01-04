/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LauncherSubsystem extends SubsystemBase {
  /**
   * Creates a new LauncherSubsystem.
   */

   WPI_TalonFX launcherMotor = new WPI_TalonFX(Constants.launcherMotor1);
   WPI_TalonFX launcherMotor2 = new WPI_TalonFX(Constants.launcherMotor2);

   Faults faults = new Faults();

  public LauncherSubsystem() {
    launcherMotor.configFactoryDefault();
    launcherMotor2.configFactoryDefault();

    launcherMotor2.follow(launcherMotor);

    launcherMotor.setInverted(false);
    launcherMotor2.setInverted(true);
  }

  public void LauncherOut(){
    launcherMotor.set(.35);
  } 

  public void LauncherSpit(){
    launcherMotor.set(.22);
  }

  public void LauncherOff(){
    launcherMotor.set(0);
  } 
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

