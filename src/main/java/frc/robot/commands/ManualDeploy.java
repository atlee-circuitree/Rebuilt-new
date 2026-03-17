// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class ManualDeploy extends Command {
  private final Intake intake;
  private final double speed;
  private final boolean brake;

  public ManualDeploy(Intake intake, double speed) {
    this.intake = intake;
    this.speed = speed;
    this.brake = (speed < 0); // brake when moving up (negative speed)
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    if (brake) {
      intake.deployBrake(); // configure brake mode once, not every cycle
    }
  }

  @Override
  public void execute() {
    intake.deployManual(speed);
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
