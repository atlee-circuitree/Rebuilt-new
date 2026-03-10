// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.controller.PIDController;
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
  private ArrayList<Double> currents;
  private double realZero;
  private boolean zeroing;
  private PIDController pid;

  public Turret() {
    motorLeft = new TalonFX(Constants.CAN_IDS.turretMotorLeft, "FRC 1599B");
    motorRight = new TalonFX(Constants.CAN_IDS.turretMotorRight, "FRC 1599B");
    motorRotator = new TalonFX(Constants.CAN_IDS.turretMotorRotator, "FRC 1599B");

    motorHoodLeft = new HiTecServo(Constants.Channels.motorHoodLeft);
    hoodUp = false;
    currents = new ArrayList<>(7);
    realZero = 0;
    zeroing = false;
    pid = new PIDController(0.014, 0, 0);

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
    gearRatio = 10;

  }

  public void rotate(double speed) {
    if (isSafe(speed))
      motorRotator.set(speed * 0.15);
    else
      stopRotator();
  }

  public void setSetpoint(double degree)
  {
    pid.setSetpoint(degree);
  }

  public void rotateTo() {
    double out = pid.calculate(getAngle());
    if (out > 1.0)
      out = 1.0;
    else if (out < -1.0)
      out = -1.0;
    motorRotator.set(out);
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

  public void findZero() {
    zeroing = true;
    motorRotator.set(-0.08);
  }

  public void setZero()
  {
    realZero = motorRotator.getPosition().getValueAsDouble();
  }

  public boolean zeroPeriodic() {
    currents.add(Math.abs(motorRotator.getSupplyCurrent().getValueAsDouble()));
    
    int count = 0;
    for (int i = 1; i < currents.size(); i++) {
        double delta = Math.abs(currents.get(i) - currents.get(i - 1));
        if (delta >= 0.1)
            count++;
    }

    if (currents.size() > 3)
      return currents.get(currents.size() - 1) > 1.25 && currents.get(currents.size() - 2) > 1.25 && currents.get(currents.size() - 3) > 1.25;
    return count > 3;
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
    return ((motorRotator.getPosition().getValueAsDouble() - realZero) / gearRatio) * 360;
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
    else if (output == 0)
      return true;
    else
      return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("turret pos", motorRotator.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("turret angle", getAngle());
    SmartDashboard.putNumber("given output", motorRotator.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("turret curent output", motorRotator.getSupplyCurrent().getValueAsDouble());

    if (!zeroing)
    {
      if (!isSafe(motorRotator.getMotorVoltage().getValueAsDouble())) // ALWAYS check for safety
        stopRotator();
    }
  }
}
