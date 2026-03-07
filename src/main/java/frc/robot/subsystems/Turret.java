// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.HiTecServo;
import frc.robot.util.LinearServo;

public class Turret extends SubsystemBase {

  private TalonFX motorLeft;
  private TalonFX motorRight;
  private TalonFX motorRotator;
  private LinearServo motorHoodLeft;
  private boolean hoodUp;
  private CANcoder turretEncoder;
  private double targetVelocity;
  private final double gearRatio;

  public Turret() {
    motorLeft = new TalonFX(Constants.CAN_IDS.turretMotorLeft, "FRC 1599B");
    motorRight = new TalonFX(Constants.CAN_IDS.turretMotorRight, "FRC 1599B");
    motorRotator = new TalonFX(Constants.CAN_IDS.turretMotorRotator, "FRC 1599B");

    motorHoodLeft = new HiTecServo(Constants.Channels.motorHoodLeft);
    hoodUp = false;

    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = 0.1; // An error of 1 rotation results in 2.4 V output
    slot0Configs.kI = 0.0; // no output for integrated error
    slot0Configs.kD = 0.0; // A velocity of 1 rps results in 0.1 V output

    turretEncoder = new CANcoder(Constants.CAN_IDS.turretEncoder, "FRC 1599B");

    motorRotator.getConfigurator().apply(slot0Configs);
    TalonFXConfiguration conf = new TalonFXConfiguration();
    conf.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    conf.Feedback.FeedbackRemoteSensorID = turretEncoder.getDeviceID();
    motorRotator.getConfigurator().apply(conf);

    Slot0Configs spinMotorConfigs = new Slot0Configs();
    spinMotorConfigs.kP = 0.64; // An error of 1 rotation results in 2.4 V output
    spinMotorConfigs.kI = 0.0; // no output for integrated error
    spinMotorConfigs.kD = 0.0; // A velocity of 1 rps results in 0.1 V output

    motorLeft.getConfigurator().apply(spinMotorConfigs);
    motorRight.getConfigurator().apply(spinMotorConfigs);

    targetVelocity = 0;
    gearRatio = 50;
    setHoodPosition(true);
  }

  public void rotate(double speed) {
    if (isSafe(speed))
      motorRotator.set(speed * 0.05);
    else
      stopRotator();
  }

  public void rotateTo(double degree) {
    double position = (degree / 360) * gearRatio;
    final PositionVoltage m_request = new PositionVoltage(position).withSlot(0);
    motorRotator.setControl(m_request);
  }

  public boolean getHoodPosition()
  {
    return hoodUp;
  }

  public void setHoodPosition(boolean up) {
    hoodUp = up;
    if (hoodUp)
      motorHoodLeft.setPosition(0.8);
    else
      motorHoodLeft.setPosition(0.0);
  }

  public double getVelocity()
  {
    return motorLeft.getVelocity().getValueAsDouble();
  }

  public void spin(double speed) {
    // velocity unit is rev per sec of the motor
    targetVelocity = speed; // gear ratio is 1:1 so no math needed
    final VelocityVoltage m_request = new VelocityVoltage(targetVelocity).withFeedForward(0).withSlot(0);
    motorLeft.setControl(m_request);
    motorRight.setControl(m_request);
  }

  public boolean isAtSpeed()
  {
    return Math.abs(getSpeed() - targetVelocity) < Constants.Turret.shooterThreshold;
  }

  public void stopShooter() {
    motorLeft.set(0);
    motorRight.set(0);
  }

  public void stopRotator() {
    motorRotator.set(0);
  }

  public double getAngle() {
    return (motorRotator.getPosition().getValueAsDouble() / gearRatio) * 360;
  }

  public double getSpeed() {
    return motorLeft.getVelocity().getValueAsDouble();
  }

  public void runAtPower(double power)
  {
    motorLeft.set(power);
    motorRight.set(power);
  }

  public boolean isSafe(double output)
  {
    if (output < 0 && getAngle() > Constants.Turret.minAngle)
      return true;
    else if (output > 0 && getAngle() < Constants.Turret.maxAngle)
      return true;
    else
      return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("turret angle", motorRotator.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("turret position", getAngle());

    /*
    if (!isSafe(motorRotator.getMotorOutputStatus().getValueAsDouble())) // ALWAYS check for safety
      stopRotator();
    */
  }
}
