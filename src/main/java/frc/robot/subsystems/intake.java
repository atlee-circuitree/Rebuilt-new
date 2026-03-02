// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private TalonFX feedMotor;
  private TalonFX deployMotor;
  private final double gearRatio;
  private DutyCycleEncoder pivotEncoder;

  public Intake() {
    feedMotor = new TalonFX(Constants.CAN_IDS.feedIntakeMotor, "FRC 1599B");
    deployMotor = new TalonFX(Constants.CAN_IDS.deployMotor, "FRC 1599B");
    deployEncoder = new DutyCycleEncoder(Constants.Channels.deployEncoder);


    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = 0.8; // An error of 1 rotation results in 2.4 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.1; // A velocity of 1 rps results in 0.1 V output

    deployMotor.getConfigurator().apply(slot0Configs);
    feedMotor.getConfigurator().apply(slot0Configs);
    gearRatio = 9;
  }

  public void deploy() {
    // position unit is rotations of the motor
    final PositionVoltage m_request = new PositionVoltage(Constants.Intake.deployPosition).withSlot(0);
    deployMotor.setControl(m_request);
  }

  public void retract() {
    // no math needed because it is zero
    final PositionVoltage m_request = new PositionVoltage(Constants.Intake.homePosition).withSlot(0);
    deployMotor.setControl(m_request);
  }

  public void intake() {

    feedMotor.set(Constants.Intake.intakeSpeed);
  }

  public double getAngleEncoder() {
    return (pivotEncoder.get() - Constants.) * 360;
  }

  public void outtake() {
    feedMotor.set(-Constants.Intake.intakeSpeed);
  }

  public void stopWheels() {
    VoltageOut v = new VoltageOut(0);
    feedMotor.setControl(v);
    //feedMotor.set(0);
  }

  public void periodic() {
    SmartDashboard.putNumber("deployMotor", deployMotor.getPosition().getValueAsDouble());
  }
}
