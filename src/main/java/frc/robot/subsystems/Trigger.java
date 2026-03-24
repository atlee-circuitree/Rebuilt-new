// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Trigger extends SubsystemBase {

  private final TalonFX kickUpMotor;
  private final TalonFX indexMotor;

  public Trigger() {
    kickUpMotor = new TalonFX(Constants.CAN_IDS.kickUpMotor, "FRC 1599B");
    indexMotor  = new TalonFX(Constants.CAN_IDS.indexMotor,  "FRC 1599B");
  }

  public void shoot() {
    kickUpMotor.set(-Constants.Trigger.SHOOT_SPEED);
    indexMotor.set(Constants.Trigger.SHOOT_SPEED);
  }

  public void reverseShoot() {
    kickUpMotor.set(-Constants.Trigger.REVERSE_SPEED);
    indexMotor.set(Constants.Trigger.REVERSE_SPEED);
  }

  public void stop() {
    kickUpMotor.set(0);
    indexMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
