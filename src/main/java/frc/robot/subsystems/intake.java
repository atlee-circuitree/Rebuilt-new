// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private TalonFX motorLeft;
  private TalonFX motorRight;
  private TalonFX deployMotor;

  public Intake() {
    motorLeft = new TalonFX(Constants.CAN_IDS.intakeMotorLeft);
    motorRight = new TalonFX(Constants.CAN_IDS.intakeMotorRight);
    deployMotor = new TalonFX(Constants.CAN_IDS.deployMotor);

    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = 2.4; // An error of 1 rotation results in 2.4 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.1; // A velocity of 1 rps results in 0.1 V output

    deployMotor.getConfigurator().apply(slot0Configs);
  }

  public void deploy() {
    final PositionVoltage m_request = new PositionVoltage(Constants.Intake.deployPosition).withSlot(0);
    deployMotor.setControl(m_request);
  }

  public void retract() {
    final PositionVoltage m_request = new PositionVoltage(Constants.Intake.homePosition).withSlot(0);
    deployMotor.setControl(m_request);
  }

  public void intake() {
    motorLeft.set(Constants.Intake.intakeSpeed);
    motorRight.set(Constants.Intake.intakeSpeed);
  }

  public void outtake() {
    motorLeft.set(-Constants.Intake.intakeSpeed);
    motorRight.set(-Constants.Intake.intakeSpeed);
  }

  public void stopWheels() {
    motorLeft.set(0);
    motorRight.set(0);
  }
}
