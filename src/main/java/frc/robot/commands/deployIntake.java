// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class DeployIntake extends Command {
  private Intake intake;

  public DeployIntake(Intake in) {
    intake = in;
    addRequirements(in);
  }

  @Override
  public void initialize() {
    intake.deploy();
  }

 @Override
  public boolean isFinished() {
    return true;
  }
}
