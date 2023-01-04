// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class TowerSubsystem extends SubsystemBase {
  /**
   * Creates a new TurretSubsystem.
   */

  WPI_TalonSRX towerIndexMotor = new WPI_TalonSRX(Constants.towerIndexMotor);

  Faults faults = new Faults();
  
    public TowerSubsystem() {
      
      towerIndexMotor.configFactoryDefault();
      
      towerIndexMotor.setNeutralMode(NeutralMode.Brake);
    }
  
    public void TowerUp() {
      towerIndexMotor.set(0.6);
    }
  
    public void TowerDown() {
      towerIndexMotor.set(-0.75);
    }
  
    public void TowerOff() {
      towerIndexMotor.set(0);
    }
  
    @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
