// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  WPI_TalonSRX intakeUpDownMotor = new WPI_TalonSRX(Constants.intakeUpDownMotor);
  WPI_TalonSRX intakeInOutMotor = new WPI_TalonSRX(Constants.intakeInOutMotor);
  
  public IntakeSubsystem() {
    intakeInOutMotor.configFactoryDefault();
    intakeUpDownMotor.configFactoryDefault();

    //intakeUpDownMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void IntakeIn() {
    intakeInOutMotor.set(-0.5);
  }

  public void IntakeOut() {
    intakeInOutMotor.set(0.5);
  }

  public void IntakeOff() {
    intakeInOutMotor.set(0);
  }

  public void IntakeUp() {
    intakeUpDownMotor.set(-0.4);
  }

  public void IntakeDown() {
    intakeUpDownMotor.set(0.4);
  }

  public void IntakeStop() {
    intakeUpDownMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
