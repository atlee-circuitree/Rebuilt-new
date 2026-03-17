// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private final TalonFX feedMotor;
  private final TalonFX deployMotor;

  // Reusable CTRE control request objects — never allocate in hot path
  private final PositionVoltage m_positionRequest = new PositionVoltage(0).withFeedForward(0);
  private final VoltageOut m_voltageOut = new VoltageOut(0);

  // Cached status signals — refreshed once per periodic
  private final StatusSignal<edu.wpi.first.units.measure.Angle> m_deployPosition;
  private final StatusSignal<edu.wpi.first.units.measure.Voltage> m_deployVoltage;
  private final StatusSignal<edu.wpi.first.units.measure.Current> m_deployCurrent;

  private boolean runningToPosition = false;
  private double setPoint = 0;
  // 4-cycle (80 ms) debounce — prevents a single noisy sample from stopping the deploy motor
  private final Debouncer m_atPositionDebouncer = new Debouncer(4 * 0.02, Debouncer.DebounceType.kRising);

  public Intake() {
    feedMotor   = new TalonFX(Constants.CAN_IDS.feedIntakeMotor, "FRC 1599B");
    deployMotor = new TalonFX(Constants.CAN_IDS.deployMotor,     "FRC 1599B");

    Slot0Configs slot0ConfigsUp = new Slot0Configs();
    slot0ConfigsUp.kP = Constants.Intake.DEPLOY_SLOT0_KP;
    slot0ConfigsUp.kI = 0.0;
    slot0ConfigsUp.kD = 0.0;

    Slot1Configs slot1ConfigsDown = new Slot1Configs();
    slot1ConfigsDown.kP = Constants.Intake.DEPLOY_SLOT1_KP;
    slot1ConfigsDown.kI = 0.0;
    slot1ConfigsDown.kD = 0.0;

    MotorOutputConfigs motorOutput = new MotorOutputConfigs();
    motorOutput.NeutralMode = NeutralModeValue.Coast;
    deployMotor.getConfigurator().apply(motorOutput);
    deployMotor.getConfigurator().apply(slot0ConfigsUp);
    deployMotor.getConfigurator().apply(slot1ConfigsDown);

    m_deployPosition = deployMotor.getPosition();
    m_deployVoltage  = deployMotor.getMotorVoltage();
    m_deployCurrent  = deployMotor.getSupplyCurrent();
  }

  public void deploy() {
    runToPosition(Constants.Intake.DEPLOY_POS_ROT);
  }

  /**
   * Commands the deploy motor to the retract position.
   * Call deployBrake() from the command's initialize() before calling this
   * if brake mode is needed during retract.
   */
  public void retract() {
    runToPosition(Constants.Intake.RETRACT_POS_ROT);
  }

  public void intake() {
    feedMotor.set(Constants.Intake.INTAKE_SPEED);
  }

  public void deployManual(double speed) {
    deployMotor.set(speed);
  }

  public void stopDeploy() {
    deployMotor.set(0);
  }

  /** Switches the deploy motor to brake neutral mode. Call once from command initialize(). */
  public void deployBrake() {
    MotorOutputConfigs motorOutput = new MotorOutputConfigs();
    motorOutput.NeutralMode = NeutralModeValue.Brake;
    deployMotor.getConfigurator().apply(motorOutput, 0.0); // 0 s timeout = non-blocking
  }

  public void runToPosition(double positionRotations) {
    int slot = (Math.abs(positionRotations) < Constants.Intake.SLOT_SELECT_THRESHOLD) ? 0 : 1;
    deployMotor.setControl(m_positionRequest.withPosition(positionRotations).withSlot(slot));
    setPoint = positionRotations; // raw motor position for completion check
    runningToPosition = true;
  }

  //need to fix deploy and retract.
  public double getAngleEncoder() {
    return m_deployPosition.getValueAsDouble() + 1;
  }

  public void outtake() {
    feedMotor.set(-Constants.Intake.INTAKE_SPEED);
  }

  public void stopWheels() {
    feedMotor.setControl(m_voltageOut.withOutput(0));
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(m_deployPosition, m_deployVoltage, m_deployCurrent);

    SmartDashboard.putNumber("Intake/encoder_offset", getAngleEncoder());
    SmartDashboard.putNumber("Intake/position_rot",   m_deployPosition.getValueAsDouble());
    SmartDashboard.putNumber("Intake/voltage",        m_deployVoltage.getValueAsDouble());
    SmartDashboard.putNumber("Intake/current_amps",   m_deployCurrent.getValueAsDouble());

    if (runningToPosition) {
      SmartDashboard.putNumber("Intake/setpoint_rot", setPoint);
      // Compare setPoint directly against raw motor position (no +1 offset)
      boolean atPosition = Math.abs(setPoint - m_deployPosition.getValueAsDouble()) < Constants.Intake.POSITION_TOLERANCE_ROT;
      if (m_atPositionDebouncer.calculate(atPosition)) {
        runningToPosition = false;
        deployMotor.set(0);
      }
    }
  }
}
