// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Speed;


public class intake extends SubsystemBase {
  /** Creates a new intake. */
 
 private TalonFX intakeMotor;
 private TalonFX deployMotor;
 
  public intake() {
    intakeMotor = new TalonFX(Constants.CAN_IDS.intakeMotor, "1599-B");
    deployMotor = new TalonFX(Constants.CAN_IDS.deployMotor, "1599-B");
   

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void deploy() {
    deployMotor.set(1.0);
  }

  public void spin(double velocity) {
    intakeMotor.set(velocity);
  }

  public void stop() {
    intakeMotor.set(0);
  }

  
}
