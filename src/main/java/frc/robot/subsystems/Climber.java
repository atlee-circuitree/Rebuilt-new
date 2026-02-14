// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private TalonFX motorLeft;
  private TalonFX motorRight;

  public Climber() {
    motorLeft = new TalonFX(Constants.CAN_IDS.climberMotorLeft);
    motorRight = new TalonFX(Constants.CAN_IDS.climberMotorRight);

    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = 2.4; // An error of 1 rotation results in 2.4 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.1; // A velocity of 1 rps results in 0.1 V output

    motorLeft.getConfigurator().apply(slot0Configs);
    motorRight.getConfigurator().apply(slot0Configs);
  }

  public void setPosition(double position) {
    position = position; // TODO replace with math
    final PositionVoltage m_request = new PositionVoltage(position).withSlot(0);
    motorLeft.setControl(m_request);
    motorRight.setControl(m_request);
  }

  public double getPosition() {
    return motorLeft.getPosition().getValueAsDouble(); // TODO replace with math
  }
}
