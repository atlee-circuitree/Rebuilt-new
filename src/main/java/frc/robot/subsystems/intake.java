// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
  private boolean runningToPosition;
  private PositionVoltage m_request;
  private double setPoint;
  private final PIDController pid = new PIDController(0.01, 0, 0);

  public Intake() {
    feedMotor = new TalonFX(Constants.CAN_IDS.feedIntakeMotor, "FRC 1599B");
    deployMotor = new TalonFX(Constants.CAN_IDS.deployMotor, "FRC 1599B");
    deployEncoder = new CANcoder(33, "FRC 1599B");


    Slot0Configs slot0ConfigsUp = new Slot0Configs();
    slot0ConfigsUp.kP = 5;//30; // An error of 1 rotation results in 2.4 V output
    slot0ConfigsUp.kI = 0.0;//1.5; // no output for integrated error
    slot0ConfigsUp.kD = 0.0; // A velocity of 1 rps results in 0.1 V output

    Slot1Configs slot1ConfigsDown = new Slot1Configs();
    slot1ConfigsDown.kP = 1.5;//35; // An error of 1 rotation results in 2.4 V output
    slot1ConfigsDown.kI = 0.0;//0.05; // no output for integrated error
    slot1ConfigsDown.kD = 0.0; // A velocity of 1 rps results in 0.1 V output

    MotorOutputConfigs motorOutput = new MotorOutputConfigs();
    motorOutput.NeutralMode = NeutralModeValue.Coast;
    deployMotor.getConfigurator().apply(motorOutput);

    deployMotor.getConfigurator().apply(slot0ConfigsUp);
    deployMotor.getConfigurator().apply(slot1ConfigsDown);
    runningToPosition = false;
  }

  public void deploy() {
      runToPosition(-.7);
      
  }

  public void retract() {
      runToPosition(-.9);
    MotorOutputConfigs motorOutput = new MotorOutputConfigs();
    motorOutput.NeutralMode = NeutralModeValue.Brake;
    deployMotor.getConfigurator().apply(motorOutput);
  }

  public void intake() {
    feedMotor.set(Constants.Intake.intakeSpeed);
  }

  public void deployManual(double speed) {
    deployMotor.set(speed);
  }

  public void stopDeploy() {
    deployMotor.set(0);
  }

  public double getDeployCurrent() {
    return deployMotor.getSupplyCurrent().getValueAsDouble();
  }

  public void deployBrake() {
    MotorOutputConfigs motorOutput = new MotorOutputConfigs();
    motorOutput.NeutralMode = NeutralModeValue.Brake;
    deployMotor.getConfigurator().apply(motorOutput);
  }

  public void runToPosition(double deg)
  {
    if (Math.abs(deg) < 0.015) // down position
      m_request = new PositionVoltage(deg).withFeedForward(0).withSlot(0);
    else
      m_request = new PositionVoltage(deg).withFeedForward(0).withSlot(1);

    deployMotor.setControl(m_request);
    setPoint = deg;
    runningToPosition = true;
  }

  public double getAngleEncoder() {
    return (deployMotor.getPosition().getValueAsDouble() + 1);
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
    SmartDashboard.putNumber("CANCODER Angle", getAngleEncoder());
    SmartDashboard.putNumber("MOTOR Angle", deployMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("MOTOR votlage", deployMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("MOTOR current", deployMotor.getSupplyCurrent().getValueAsDouble());

    if (runningToPosition)
    {
      SmartDashboard.putNumber("run request", m_request.getPositionMeasure().baseUnitMagnitude());
      SmartDashboard.putNumber("run motor", deployMotor.getPosition().getValueAsDouble());
      if (Math.abs(setPoint - getAngleEncoder()) < 0.035)
      {
        runningToPosition = false;
        deployMotor.set(0);
      }
    }
  }
}
