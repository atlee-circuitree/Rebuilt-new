// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class DeployJumpCommand extends Command {
  private static final double upDuration = 0.62;
  private static final double downDuration = 0.45;
  private static final double speed = 0.15;

  private final Intake intake;
  private final Timer intervalTimer = new Timer();
  private boolean pulsing = false;

  /** Creates a new DeployJumpCommand. */
  public DeployJumpCommand(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intervalTimer.restart();
    pulsing = false;
    intake.stopDeploy();
  }

  @Override
  public void execute() {
    if (pulsing)
    {
      if (intervalTimer.hasElapsed(upDuration))
      {
        intake.deployManual(speed);
        pulsing = !pulsing;
        intervalTimer.restart();
      }
    }
    else
    {
      if (intervalTimer.hasElapsed(downDuration))
      {
        intake.deployManual(-speed);
        pulsing = !pulsing;
        intervalTimer.restart();
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    intake.stopDeploy();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
