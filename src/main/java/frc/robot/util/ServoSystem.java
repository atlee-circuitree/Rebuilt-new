// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ServoSystem extends SubsystemBase {
  /** Creates a new Servo. */

  // Create a Servo object for port 0
  private final Servo m_exampleServo = new Servo(0);
  
   // Define constants for specific positions (optional, but good practice)
  public static final double k_openPosition = 0.0; // 0 degrees
  public static final double k_closedPosition = 1.0; // 180 degrees
  public static final double k_midPosition = 0.5; // 90 degrees

  public ServoSystem() {}

  public void setPosition(double position) {
    m_exampleServo.set(position);
  }

   public void setAngle(double angle) {
    m_exampleServo.setAngle(angle);
  }

  public double getPosition() {
    return m_exampleServo.get();
  }


  @Override
  public void periodic() {
  }
}
