// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;

public class TurretTo extends Command {
  private Turret turret;
  private double ang;

  public TurretTo(Turret turret, double ang) {
    this.turret = turret;
    this.ang = ang;
    addRequirements(turret);
  }

  @Override
  public void execute()
  {
    turret.rotateTo(ang);
  }

  @Override
  public void end(boolean interrupted) {
    turret.stopRotator();
  }

  @Override
  public boolean isFinished() {
    return Math.abs(turret.getAngle() - ang) < Constants.Turret.angleThreshold;
  }
}
