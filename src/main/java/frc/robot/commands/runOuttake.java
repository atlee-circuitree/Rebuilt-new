// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class RunOuttake extends Command {
  Intake intake;
  
  public RunOuttake(Intake in) {
   intake = in;
   addRequirements(in);
  }

  @Override
  public void initialize() {
    intake.outtake();
  }

  @Override
  public void end(boolean interrupted) {
    intake.stopWheels();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
