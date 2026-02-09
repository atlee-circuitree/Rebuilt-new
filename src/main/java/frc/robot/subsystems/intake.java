// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Speed;


public class Intake extends SubsystemBase {
  /** Creates a new intake. */
 
 private TalonFX motorLeft;
 private TalonFX motorRight;
 private TalonFX deployMotor;
 
  public Intake() {
    motorLeft = new TalonFX(Constants.CAN_IDS.intakeMotorLeft, "1599-B");
    motorRight = new TalonFX(Constants.CAN_IDS.intakeMotorRight, "1599-B");
    deployMotor = new TalonFX(Constants.CAN_IDS.deployMotor, "1599-B");
   

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void deploy() {
    deployMotor.set(1.0);
  }

  public void intake() {
    motorLeft.set(1.0);
    motorRight.set(1.0);
  }

  public void outtake() {
    motorLeft.set(-1.0);
    motorRight.set(-1.0);
  }

  public void stopWheels() {
    motorLeft.set(0);
    motorRight.set(0);
  }

  public void retract() {
    deployMotor.set(-1.0);
  }

  public void stop() {
    deployMotor.set(0);
  }

  
}
