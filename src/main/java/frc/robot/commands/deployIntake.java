// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class DeployIntake extends Command {
  private final Intake intake;
  private final Timer timer = new Timer();

  public DeployIntake(Intake in) {
    intake = in;
    addRequirements(in);
  }

  @Override
  public void initialize() {
    timer.restart();
    intake.deployManual(Constants.Intake.DEPLOY_MANUAL_SPEED);
  }

  @Override
  public void end(boolean interrupted) {
    intake.stopDeploy();
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(Constants.Intake.DEPLOY_WAIT_SECS);
  }
}
