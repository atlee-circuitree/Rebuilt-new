// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {

  private TalonFX motorLeft;
  private TalonFX motorRight;
  private TalonFX motorRotator;
  /** Creates a new Turret. */
  public Turret() {
    motorLeft = new TalonFX(Constants.CAN_IDS.turretMotorLeft, "1599-B");
    motorRight = new TalonFX(Constants.CAN_IDS.turretMotorRight, "1599-B");
    motorRotator = new TalonFX(Constants.CAN_IDS.turretMotorRotator, "1599-B");
    //add servos, motorHood1 and motorHood2
  }

  public void rotate(double speed) {
    motorRotator.set(speed);
  }

  public void spin(double Speed) {
    motorLeft.set(Speed);
    motorRight.set(Speed);
    //add pid controls later for spin
  }

  public void stop() {
    motorLeft.set(0);
    motorRight.set(0);
  }

  //add change hood position when servos are added.

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
