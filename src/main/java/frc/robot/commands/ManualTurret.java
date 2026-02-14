// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

public class ManualTurret extends Command {
  private Turret turret;
  private double speed;

  public ManualTurret(Turret turret, double speed) {
    this.turret = turret;
    this.speed = speed;
    addRequirements(turret);
  }

  @Override
  public void execute()
  {
    turret.rotate(speed);
  }

  @Override
  public void end(boolean interrupted) {
    turret.stopRotator();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
