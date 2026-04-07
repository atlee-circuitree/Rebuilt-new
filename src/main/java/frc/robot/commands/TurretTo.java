package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;

public class TurretTo extends Command 
{
    private Turret turret;
    private double angle;
    /** Creates a new StopIntake. */
    public TurretTo(Turret tur, double ang) {
        turret = tur;
        angle = ang;
        addRequirements(turret);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        turret.rotateTo(angle);
    }

    public void end(boolean interrupted)
    {
        turret.stopRotator();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return turret.isAtAngle();
    }
}
