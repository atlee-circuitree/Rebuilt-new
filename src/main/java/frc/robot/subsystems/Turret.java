// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.HiTecServo;
import frc.robot.util.Limelight;
import frc.robot.util.LinearServo;

public class Turret extends SubsystemBase {

  private final TalonFX motorLeft;
  private final TalonFX motorRight;
  private final TalonFX motorRotator;
  private final LinearServo motorHoodLeft;
  private final CANcoder turretEncoder;
  private final InterpolatingDoubleTreeMap shooterSpeedMap = new InterpolatingDoubleTreeMap();

  // Reusable CTRE control request objects (never allocate in hot path)
  private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0).withFeedForward(0).withSlot(0);
  private final PositionVoltage m_rotatorPositionRequest = new PositionVoltage(0).withSlot(0);

  // Cached status signals — refresh once per periodic, then read .getValue()
  private final StatusSignal<edu.wpi.first.units.measure.AngularVelocity> m_leftVelocity;
  private final StatusSignal<edu.wpi.first.units.measure.Angle> m_rotatorPosition;
  private final StatusSignal<edu.wpi.first.units.measure.Current> m_rotatorCurrent;

  private boolean hoodUp = false;
  private double targetVelocity = 0;
  private double targetAngleDeg = 0;   // setpoint for CTRE closed-loop rotation
  private double realZero = 0;
  private boolean zeroing = false;
  /** Last commanded rotator direction — used for software safety backstop. */
  private double commandedDirection = 0;
  private final ArrayList<Double> currents = new ArrayList<>(7);

  public Turret() {
    motorLeft    = new TalonFX(Constants.CAN_IDS.turretMotorLeft,     "FRC 1599B");
    motorRight   = new TalonFX(Constants.CAN_IDS.turretMotorRight,    "FRC 1599B");
    motorRotator = new TalonFX(Constants.CAN_IDS.turretMotorRotator,  "FRC 1599B");
    turretEncoder = new CANcoder(Constants.CAN_IDS.turretEncoder,     "FRC 1599B");

    // Flywheel motors
    MotorOutputConfigs coastConfig = new MotorOutputConfigs();
    coastConfig.NeutralMode = NeutralModeValue.Coast;
    motorLeft.getConfigurator().apply(coastConfig);

    MotorOutputConfigs invertConfig = new MotorOutputConfigs();
    invertConfig.NeutralMode = NeutralModeValue.Coast;
    invertConfig.Inverted = InvertedValue.Clockwise_Positive;
    motorRight.getConfigurator().apply(invertConfig);

    Slot0Configs spinMotorConfigs = new Slot0Configs();
    spinMotorConfigs.kP = Constants.Turret.FLYWHEEL_KP;
    spinMotorConfigs.kI = Constants.Turret.FLYWHEEL_KI;
    spinMotorConfigs.kD = Constants.Turret.FLYWHEEL_KD;
    motorLeft.getConfigurator().apply(spinMotorConfigs);
    motorRight.getConfigurator().apply(spinMotorConfigs);

    // Rotator motor — CTRE closed-loop with remote CANcoder feedback
    TalonFXConfiguration conf = new TalonFXConfiguration();
    conf.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    conf.Feedback.FeedbackRemoteSensorID = turretEncoder.getDeviceID();
    conf.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    conf.Slot0.kP = Constants.Turret.ROTATOR_KP;
    conf.Slot0.kI = Constants.Turret.ROTATOR_KI;
    conf.Slot0.kD = Constants.Turret.ROTATOR_KD;
    motorRotator.getConfigurator().apply(conf);

    // Cache signal references — avoids repeated object allocation in hot path
    m_leftVelocity   = motorLeft.getVelocity();
    m_rotatorPosition = motorRotator.getPosition();
    m_rotatorCurrent  = motorRotator.getSupplyCurrent();

    // Distance-to-speed interpolation table
    shooterSpeedMap.put(Constants.Turret.distClose, Constants.Turret.speedClose);
    shooterSpeedMap.put(Constants.Turret.distMid,   Constants.Turret.speedMid);
    shooterSpeedMap.put(Constants.Turret.distFar,   Constants.Turret.speedFar);

    SmartDashboard.putNumber("Shooter/distClose",  Constants.Turret.distClose);
    SmartDashboard.putNumber("Shooter/speedClose", Constants.Turret.speedClose);
    SmartDashboard.putNumber("Shooter/distMid",    Constants.Turret.distMid);
    SmartDashboard.putNumber("Shooter/speedMid",   Constants.Turret.speedMid);
    SmartDashboard.putNumber("Shooter/distFar",    Constants.Turret.distFar);
    SmartDashboard.putNumber("Shooter/speedFar",   Constants.Turret.speedFar);

    motorHoodLeft = new HiTecServo(Constants.Turret.HOOD_SERVO_CHANNEL);
  }

  public void rotate(double speed) {
    if (isSafe(speed)) {
      commandedDirection = speed;
      motorRotator.set(speed * Constants.Turret.ROTATE_OUTPUT_SCALE);
    } else {
      stopRotator();
    }
  }

  /** Set the target angle (degrees) for CTRE closed-loop position control via rotateTo(). */
  public void setSetpoint(double degree) {
    targetAngleDeg = degree;
  }

  /** Drive the rotator to the last angle set by setSetpoint() using CTRE onboard PID. */
  public void rotateTo() {
    double targetRotations = realZero + (targetAngleDeg / 360.0) * Constants.Turret.GEAR_RATIO;
    commandedDirection = Math.signum(targetAngleDeg - getAngle());
    motorRotator.setControl(m_rotatorPositionRequest.withPosition(targetRotations));
  }

  public void autoRotate() {
    double position = Limelight.getTurretSpeed();
    if (isSafe(position)) {
      commandedDirection = position;
      motorRotator.set(position);
    }
  }

  public boolean getHoodPosition() {
    return hoodUp;
  }

  public void setHoodPosition(boolean up) {
    hoodUp = up;
    motorHoodLeft.setPosition(hoodUp ? Constants.Turret.HOOD_UP_POS : Constants.Turret.HOOD_DOWN_POS);
  }

  /** Returns flywheel velocity in RPS (left motor). Uses cached, refreshed signal. */
  public double getVelocity() {
    return m_leftVelocity.getValueAsDouble();
  }

  public void spinAtDistance() {
    double distance = Limelight.getDistance();
    if (distance <= 0)
      spin(Constants.Turret.speedMid); // no valid target — use mid-range default
    else
      spin(shooterSpeedMap.get(distance));
  }

  public void spin(double speed) {
    targetVelocity = speed;
    motorLeft.setControl(m_velocityRequest.withVelocity(targetVelocity));
    motorRight.setControl(m_velocityRequest.withVelocity(targetVelocity));
  }

  public void clearZeroCurrents() {
    currents.clear();
  }

  public void findZero() {
    zeroing = true;
    commandedDirection = -1;
    motorRotator.set(Constants.Turret.ZERO_POWER);
  }

  public void setZero() {
    realZero = m_rotatorPosition.getValueAsDouble();
    // Apply CTRE soft limits now that we know the calibrated zero
    SoftwareLimitSwitchConfigs limitConf = new SoftwareLimitSwitchConfigs();
    limitConf.ForwardSoftLimitEnable = true;
    limitConf.ForwardSoftLimitThreshold =
        realZero + (Constants.Turret.maxAngle / 360.0) * Constants.Turret.GEAR_RATIO;
    limitConf.ReverseSoftLimitEnable = true;
    limitConf.ReverseSoftLimitThreshold =
        realZero + (Constants.Turret.minAngle / 360.0) * Constants.Turret.GEAR_RATIO;
    motorRotator.getConfigurator().apply(limitConf);
    zeroing = false;
    commandedDirection = 0;
  }

  public boolean zeroPeriodic() {
    currents.add(Math.abs(m_rotatorCurrent.getValueAsDouble()));

    int count = 0;
    for (int i = 1; i < currents.size(); i++) {
      double delta = Math.abs(currents.get(i) - currents.get(i - 1));
      if (delta >= Constants.Turret.ZERO_CURRENT_DELTA_AMPS)
        count++;
    }

    if (currents.size() > 3) {
      int n = currents.size();
      return currents.get(n - 1) > Constants.Turret.ZERO_STALL_CURRENT_AMPS
          && currents.get(n - 2) > Constants.Turret.ZERO_STALL_CURRENT_AMPS
          && currents.get(n - 3) > Constants.Turret.ZERO_STALL_CURRENT_AMPS;
    }
    return count > 3;
  }

  public boolean isAtSpeed() {
    return Math.abs(getVelocity() - targetVelocity) < Constants.Turret.shooterThreshold;
  }

  public void stopShooter() {
    motorLeft.set(0);
    motorRight.set(0);
  }

  public void stopRotator() {
    commandedDirection = 0;
    motorRotator.set(0);
  }

  /** Returns turret angle in degrees relative to the homed zero position. */
  public double getAngle() {
    return ((m_rotatorPosition.getValueAsDouble() - realZero) / Constants.Turret.GEAR_RATIO) * 360.0;
  }

  public void runAtPower(double power) {
    motorLeft.set(power);
    motorRight.set(power);
  }

  /**
   * Returns true if the given output direction is safe given the current angle.
   * Negative output = moving toward minAngle; positive = toward maxAngle.
   */
  public boolean isSafe(double output) {
    if (output < 0 && getAngle() > Constants.Turret.minAngle) return true;
    if (output > 0 && getAngle() < Constants.Turret.maxAngle) return true;
    if (output == 0) return true;
    return false;
  }

  @Override
  public void periodic() {
    // Refresh all cached signals at the top of periodic — never read stale values
    BaseStatusSignal.refreshAll(m_leftVelocity, m_rotatorPosition, m_rotatorCurrent);

    SmartDashboard.putNumber("Turret/position_rot", m_rotatorPosition.getValueAsDouble());
    SmartDashboard.putNumber("Turret/angle_deg",    getAngle());
    SmartDashboard.putNumber("Turret/current_amps", m_rotatorCurrent.getValueAsDouble());
    SmartDashboard.putNumber("Turret/velocity_rps", getVelocity());

    // Software safety backstop: stop if commanded direction would exceed limits
    if (!zeroing && !isSafe(commandedDirection)) {
      stopRotator();
    }
  }
}
