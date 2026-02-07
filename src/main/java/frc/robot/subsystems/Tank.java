// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Tank extends SubsystemBase {

  private TalonFX motorLeft;
  private TalonFX motorRight;

  /** Creates a new Tank. */
  public Tank() {

    motorLeft = new TalonFX(Constants.CAN_IDS.tankMotorLeft, "1599-B");
    motorLeft = new TalonFX(Constants.CAN_IDS.tankMotorRight, "1599-B");

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
