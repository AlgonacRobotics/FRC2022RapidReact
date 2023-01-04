// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystemEncoder extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  WPI_TalonSRX intakeUpDownMotor = new WPI_TalonSRX(Constants.intakeUpDownMotor);
  WPI_TalonSRX intakeInOutMotor = new WPI_TalonSRX(Constants.intakeInOutMotor);

  public int maxLoopNumber = 0;
  public int onTargetCounter = 0;
  public int allowedErrorRange = 0;

  Faults faults = new Faults();
  
  public IntakeSubsystemEncoder() {
    intakeInOutMotor.configFactoryDefault();
    intakeUpDownMotor.configFactoryDefault();

    intakeUpDownMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,0,0);
    intakeUpDownMotor.setSensorPhase(true);
  }

  public void IntakeIn() {
    intakeInOutMotor.set(0.5);
  }

  public void IntakeOut() {
    intakeInOutMotor.set(-0.5);
  }

  public void IntakeOff() {
    intakeInOutMotor.set(0);
  }

  public void IntakeUp() {
    intakeUpDownMotor.set(0.1);
  }

  public void IntakeDown() {
    intakeUpDownMotor.set(-0.1);
  }

  public void IntakeStop() {
    intakeUpDownMotor.set(0);
  }

  public void zeroEncoders(){
    intakeUpDownMotor.setSelectedSensorPosition(0, 0, 0);
    
    intakeUpDownMotor.getSensorCollection().setQuadraturePosition(0, 0);
  }

  public void autonIntakeUpDown(double targetTics){
    intakeUpDownMotor.set(ControlMode.Position, targetTics);
  }

  public boolean onTarget(){
    if (Math.abs(intakeUpDownMotor.getClosedLoopError(0)) <= allowedErrorRange){
      onTargetCounter++;
    }
    else{
      onTargetCounter = 0;
    }
    if (maxLoopNumber <= onTargetCounter){
      return true;
    }
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
