// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix6.BaseStatusSignal;
import edu.wpi.first.math.filter.LinearFilter;
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

  private final TalonFX flywheelLeft;
  private final TalonFX flywheelRight;
  private final TalonFX motorRotator;
  private final LinearServo motorHoodLeft;
  private final CANcoder turretEncoder;
  private final InterpolatingDoubleTreeMap shooterSpeedMap = new InterpolatingDoubleTreeMap();

  // Reusable CTRE control request objects (never allocate in hot path)
  private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0).withFeedForward(0).withSlot(0);
  private final PositionVoltage m_rotatorPositionRequest = new PositionVoltage(0).withSlot(0);

  // Cached status signals — refresh once per periodic, then read .getValue()
  private final StatusSignal<edu.wpi.first.units.measure.AngularVelocity> m_flywheelLeftVelocity;
  private final StatusSignal<edu.wpi.first.units.measure.AngularVelocity> m_flywheelRightVelocity;
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

  // 100 ms moving-average filter on flywheel velocity (5 samples × 20 ms loop)
  private final LinearFilter m_velocityFilter = LinearFilter.movingAverage(5);
  private double m_filteredVelocity = 0;

  // 4-cycle (80 ms) debounce — prevents a single noisy CANcoder reading from triggering the safety stop
  private final edu.wpi.first.math.filter.Debouncer m_unsafeDebouncer =
      new edu.wpi.first.math.filter.Debouncer(4 * 0.02, edu.wpi.first.math.filter.Debouncer.DebounceType.kRising);

  public Turret() {
    flywheelLeft    = new TalonFX(Constants.CAN_IDS.flywheelMotorLeft,   "FRC 1599B");
    flywheelRight   = new TalonFX(Constants.CAN_IDS.flywheelMotorRight,  "FRC 1599B");
    motorRotator = new TalonFX(Constants.CAN_IDS.turretMotorRotator,  "FRC 1599B");
    turretEncoder = new CANcoder(Constants.CAN_IDS.turretEncoder,     "FRC 1599B");

    // Flywheel motors
    MotorOutputConfigs coastConfig = new MotorOutputConfigs();
    coastConfig.NeutralMode = NeutralModeValue.Coast;
    flywheelLeft.getConfigurator().apply(coastConfig);

    MotorOutputConfigs invertConfig = new MotorOutputConfigs();
    invertConfig.NeutralMode = NeutralModeValue.Coast;
    invertConfig.Inverted = InvertedValue.Clockwise_Positive;
    flywheelRight.getConfigurator().apply(invertConfig);

    Slot0Configs spinMotorConfigs = new Slot0Configs();
    spinMotorConfigs.kP = Constants.Turret.FLYWHEEL_KP;
    spinMotorConfigs.kI = Constants.Turret.FLYWHEEL_KI;
    spinMotorConfigs.kD = Constants.Turret.FLYWHEEL_KD;
    flywheelLeft.getConfigurator().apply(spinMotorConfigs);
    flywheelRight.getConfigurator().apply(spinMotorConfigs);

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
    m_flywheelLeftVelocity  = flywheelLeft.getVelocity();
    m_flywheelRightVelocity = flywheelRight.getVelocity();
    m_rotatorPosition = motorRotator.getPosition();
    m_rotatorCurrent  = motorRotator.getSupplyCurrent();

    // Distance-to-speed interpolation table
    shooterSpeedMap.put(Constants.Turret.DIST_CLOSE_FT, Constants.Turret.SPEED_CLOSE_RPS);
    shooterSpeedMap.put(Constants.Turret.DIST_MID_FT,   Constants.Turret.SPEED_MID_RPS);
    shooterSpeedMap.put(Constants.Turret.DIST_FAR_FT,   Constants.Turret.SPEED_FAR_RPS);

    SmartDashboard.putNumber("Shooter/distClose",  Constants.Turret.DIST_CLOSE_FT);
    SmartDashboard.putNumber("Shooter/speedClose", Constants.Turret.SPEED_CLOSE_RPS);
    SmartDashboard.putNumber("Shooter/distMid",    Constants.Turret.DIST_MID_FT);
    SmartDashboard.putNumber("Shooter/speedMid",   Constants.Turret.SPEED_MID_RPS);
    SmartDashboard.putNumber("Shooter/distFar",    Constants.Turret.DIST_FAR_FT);
    SmartDashboard.putNumber("Shooter/speedFar",   Constants.Turret.SPEED_FAR_RPS);

    motorHoodLeft = new HiTecServo(Constants.Turret.HOOD_SERVO_CHANNEL);
  }
  //hardcoded 0.15
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

  /** Returns the 100 ms moving-average flywheel velocity in RPS. Updated once per periodic. */
  public double getVelocity() {
    return m_filteredVelocity;
  }

  public void spinAtDistance() {
    double distance = Limelight.getDistance();
    if (distance >= 0)
      spin(Constants.Turret.SPEED_CLOSE_RPS); // no valid target — use mid-range default
    else if (distance >= Constants.Turret.DIST_MID_FT)
      spin(Constants.Turret.SPEED_MID_RPS);
    else if (distance >= Constants.Turret.DIST_FAR_FT)
      spin(Constants.Turret.SPEED_FAR_RPS);
    else
      spin(shooterSpeedMap.get(distance));
  }

  public void spin(double speed) {
    targetVelocity = speed;
    flywheelLeft.setControl(m_velocityRequest.withVelocity(targetVelocity));
    flywheelRight.setControl(m_velocityRequest.withVelocity(targetVelocity));
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
        realZero + (Constants.Turret.MAX_ANGLE_DEG / 360.0) * Constants.Turret.GEAR_RATIO;
    limitConf.ReverseSoftLimitEnable = true;
    limitConf.ReverseSoftLimitThreshold =
        realZero + (Constants.Turret.MIN_ANGLE_DEG / 360.0) * Constants.Turret.GEAR_RATIO;
    motorRotator.getConfigurator().apply(limitConf, 0.0); // 0 s timeout = non-blocking
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
    return Math.abs(getVelocity() - targetVelocity) < Constants.Turret.SHOOTER_THRESHOLD_RPS;
  }

  public void stopShooter() {
    flywheelLeft.set(0);
    flywheelRight.set(0);
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
    flywheelLeft.set(power);
    flywheelRight.set(power);
  }

  /**
   * Returns true if the given output direction is safe given the current angle.
   * Negative output = moving toward minAngle; positive = toward maxAngle.
   */
  public boolean isSafe(double output) {
    if (output < 0 && getAngle() > Constants.Turret.MIN_ANGLE_DEG) return true;
    if (output > 0 && getAngle() < Constants.Turret.MAX_ANGLE_DEG) return true;
    if (output == 0) return true;
    return false;
  }

  @Override
  public void periodic() {
    // Refresh all cached signals at the top of periodic — never read stale values
    BaseStatusSignal.refreshAll(m_flywheelLeftVelocity, m_flywheelRightVelocity, m_rotatorPosition, m_rotatorCurrent);
    double rawVelocity = (m_flywheelLeftVelocity.getValueAsDouble() + m_flywheelRightVelocity.getValueAsDouble()) / 2.0;
    m_filteredVelocity = m_velocityFilter.calculate(rawVelocity);

    SmartDashboard.putNumber("Turret/position_rot", m_rotatorPosition.getValueAsDouble());
    SmartDashboard.putNumber("Turret/angle_deg",    getAngle());
    SmartDashboard.putNumber("Turret/current_amps", m_rotatorCurrent.getValueAsDouble());
    SmartDashboard.putNumber("Turret/velocity_rps", getVelocity());

    // Software safety backstop: stop if commanded direction has been unsafe for 4 consecutive cycles
    if (!zeroing && m_unsafeDebouncer.calculate(!isSafe(commandedDirection))) {
      stopRotator();
    }
  }
}
