// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
   WPI_TalonFX climberMotor1 = new WPI_TalonFX(Constants.climberMotor1);
   WPI_TalonFX climberMotor2 = new WPI_TalonFX(Constants.climberMotor2);

   Faults faults = new Faults();

  public ClimberSubsystem() {
    climberMotor1.configFactoryDefault();
    climberMotor2.configFactoryDefault();

    climberMotor1.setNeutralMode(NeutralMode.Brake);
    climberMotor2.setNeutralMode(NeutralMode.Brake);

    //climberMotor2.follow(climberMotor1);
    climberMotor2.setInverted(true);
    climberMotor1.setInverted(false);
  }

  public void ClimberUp() {
    climberMotor1.set(1);
    climberMotor2.set(1);
  }

  public void ClimberDown() {
    climberMotor1.set(-1);
    climberMotor2.set(-1);
  }

  public void ClimberStop() {
    climberMotor1.set(0);
    climberMotor2.set(0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
