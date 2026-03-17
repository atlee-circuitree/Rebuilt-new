// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class DeployJumpCommand extends Command {

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
      if (intervalTimer.hasElapsed(Constants.Intake.JUMP_UP_DURATION_SECS))
      {
        intake.deployManual(Constants.Intake.JUMP_SPEED);
        pulsing = !pulsing;
        intervalTimer.restart();
      }
    }
    else
    {
      if (intervalTimer.hasElapsed(Constants.Intake.JUMP_DOWN_DURATION_SECS))
      {
        intake.deployManual(-Constants.Intake.JUMP_SPEED);
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
