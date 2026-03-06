// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.DeployIntake;

public class Intake extends SubsystemBase {
  private TalonFX feedMotor;
  private TalonFX deployMotor;
  private CANcoder deployEncoder;
  private final double gearRatio;
  private final PIDController pid = new PIDController(0.01, 0, 0);

  public Intake() {
    feedMotor = new TalonFX(Constants.CAN_IDS.feedIntakeMotor, "FRC 1599B");
    deployMotor = new TalonFX(Constants.CAN_IDS.deployMotor, "FRC 1599B");
    deployEncoder = new CANcoder(33, "FRC 1599B");


    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = 9.5; // An error of 1 rotation results in 2.4 V output
    slot0Configs.kI = 0.0; // no output for integrated error
    slot0Configs.kD = 0.0; // A velocity of 1 rps results in 0.1 V output

    deployMotor.getConfigurator().apply(slot0Configs);
    gearRatio = 45;
  }

  public void deploy() {
      runToPosition(.268);
  }

  public void retract() {
      runToPosition(0);
  }

  public void intake() {
    feedMotor.set(Constants.Intake.intakeSpeed);
  }

  public void runToPosition(double deg)
  {
    final PositionVoltage m_request = new PositionVoltage(deg).withFeedForward(0).withSlot(0);
    deployMotor.setControl(m_request);
  }

  public double getAngleEncoder() {
    return (deployEncoder.getAbsolutePosition().getValueAsDouble());
  }

  public void outtake() {
    feedMotor.set(-Constants.Intake.intakeSpeed);
  }

  public void stopWheels() {
    VoltageOut v = new VoltageOut(0);
    feedMotor.setControl(v);
    //feedMotor.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("CANCODER Angle", deployEncoder.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("MOTOR Angle", deployMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("MOTOR votlage", deployMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("MOTOR current", deployMotor.getSupplyCurrent().getValueAsDouble());
  }
}
