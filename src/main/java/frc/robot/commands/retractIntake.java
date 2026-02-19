// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake;

public class retractIntake extends Command {
  private intake intake;

  public retractIntake(intake in) {
    intake = in;
    addRequirements(in);
  }

  @Override
  public void initialize() {
    intake.retract();
  }

 @Override
  public boolean isFinished() {
    return true;
  }
}
