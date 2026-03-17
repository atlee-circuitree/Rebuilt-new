// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurnTurret extends Command {
      private Turret turret;

  /** Creates a new MoveTurret. */
  public TurnTurret(Turret tur) {
    // Use addRequirements() here to declare subsystem dependencies.
    turret = tur;
    addRequirements(tur);
  }

  // Called when the command is initially scheduled.
  @Override
  public void execute() {
    turret.autoRotate();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
