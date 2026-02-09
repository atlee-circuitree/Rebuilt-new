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
  /** Creates a new Climber. */
  public Climber() {

    motorLeft = new TalonFX(Constants.CAN_IDS.climberMotorLeft, "1599-B");
    motorRight = new TalonFX(Constants.CAN_IDS.climberMotorRight, "1599-B");

    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = 2.4; // An error of 1 rotation results in 2.4 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.1; // A velocity of 1 rps results in 0.1 V output

    motorLeft.getConfigurator().apply(slot0Configs);
    motorRight.getConfigurator().apply(slot0Configs);

  }

  public void setPosition(double position) {
    final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
    position = position; // TODO replace with math
    motorLeft.setControl(m_request.withPosition(position));
    motorRight.setControl(m_request.withPosition(position));
  }

  public double getPosition() {
    return 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
