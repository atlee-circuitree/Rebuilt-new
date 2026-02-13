package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WristSystem extends SubsystemBase {

  TalonFX wristMotor;

  DutyCycleEncoder wristEncoder;

  PIDController pid;

  /** Creates a new WristSystem. */
  public WristSystem() {

    //wristMotor = new TalonFX(Constants.CAN_IDS.wristMotor, "1599-B");
    //wristEncoder = new DutyCycleEncoder(Constants.CAN_IDS.wristEncoder);

    pid = new PIDController(Constants.Wrist.P, Constants.Wrist.I, Constants.Wrist.D);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Wrist Encoder", wristEncoder.get());
    SmartDashboard.putNumber("Wrist Angle", getAngle());

  }

  public double getAngle() {
    return ((wristEncoder.get() - Constants.Wrist.wristEncoderOffset) * 360 + 360) % 360;
  }

  public void runWrist(double velocity) {
    if (velocity == 0) {
      wristMotor.set(0);
    }
    if (getAngle() >= Constants.Wrist.wristLimit[0] && velocity > 0) {
      stop();
    } else if (getAngle() <= Constants.Wrist.wristLimit[1] && velocity < 0) {
      stop();
    } else {
      wristMotor.set(velocity);
    }
  }

  public void clearPID() {
    pid.reset();
  }

  public double getPID(double angle) {
    pid.setSetpoint(angle);
    double out = pid.calculate(getAngle());
    return out;
  }

  public void runToPosition(double angle) {
    double out = getPID(angle);
    if (getAngle() >= Constants.Wrist.wristLimit[0] && out > 0) {
      stop();
    } else if (getAngle() <= Constants.Wrist.wristLimit[1] && out < 0) {
      stop();
    } else {
      runWrist(out);
    }
  }

  public void stop() {
    runWrist(0);
  }

}