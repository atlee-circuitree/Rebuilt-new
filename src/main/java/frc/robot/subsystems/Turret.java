// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {

  private TalonFX motorLeft;
  private TalonFX motorRight;
  private TalonFX motorRotator;
  private Servo motorHoodLeft;
  private Servo motorHoodRight;
  private double targetVelocity;

  public Turret() {
    motorLeft = new TalonFX(Constants.CAN_IDS.turretMotorLeft, "1599-B");
    motorRight = new TalonFX(Constants.CAN_IDS.turretMotorRight, "1599-B");
    motorRotator = new TalonFX(Constants.CAN_IDS.turretMotorRotator, "1599-B");

    motorHoodLeft = new Servo(Constants.Channels.motorHoodLeft);
    motorHoodRight = new Servo(Constants.Channels.motorHoodRight);

    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = 2.4; // An error of 1 rotation results in 2.4 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.1; // A velocity of 1 rps results in 0.1 V output

    motorRotator.getConfigurator().apply(slot0Configs);

    Slot0Configs spinMotorConfigs = new Slot0Configs();
    spinMotorConfigs.kP = 2.4; // An error of 1 rotation results in 2.4 V output
    spinMotorConfigs.kI = 0; // no output for integrated error
    spinMotorConfigs.kD = 0.1; // A velocity of 1 rps results in 0.1 V output

    motorLeft.getConfigurator().apply(spinMotorConfigs);
    motorRight.getConfigurator().apply(spinMotorConfigs);

    targetVelocity = 0;
  }

  public void rotate(double speed) {
    if (speed < 0 && getAngle() > Constants.Turret.minAngle)
      motorRotator.set(speed);
    else if (speed > 0 && getAngle() < Constants.Turret.maxAngle)
      motorRotator.set(speed);
    else
      stopRotator();
  }

  public void rotateTo(double degree) {
    double position = degree; // TODO replace with math
    final PositionVoltage m_request = new PositionVoltage(position).withSlot(0);
    motorRotator.setControl(m_request);
  }

  public void setHoodPosition(double position) {
    motorHoodLeft.setPosition(position);
    motorHoodRight.setPosition(position);
  }

  public void spin(double speed) {
    double velocity = speed; //TODO replace with math
    targetVelocity = velocity;
    final VelocityVoltage m_request = new VelocityVoltage(velocity).withSlot(0);
    motorLeft.setControl(m_request.withVelocity(velocity).withFeedForward(0.5));
    motorRight.setControl(m_request.withVelocity(velocity).withFeedForward(0.5));
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
    return motorRotator.getPosition().getValueAsDouble(); // TODO replace with math
  }

  public double getSpeed() {
    return motorLeft.getVelocity().getValueAsDouble(); // TODO replace with math
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Turret Motor Right Speed", motorRight.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Turret Motor Left Speed", motorLeft.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Turret Motor Rotator Speed", motorRotator.getVelocity().getValueAsDouble());
  }
}
