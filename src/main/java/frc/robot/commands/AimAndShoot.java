package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Trigger;
import frc.robot.subsystems.Turret;

/**
 * Aims the turret at an alliance AprilTag, then spins the flywheel and fires
 * once up to speed.
 *
 * Step 1 — TurnTurret: sweeps and locks onto an alliance tag (finishes within
 *           5° of center, or times out after 7 s).
 * Step 2 — Parallel: spins the flywheel at distance-appropriate speed while
 *           AutoShoot waits for velocity ≥ 30 rps, then delays, then runs
 *           both the kicker and indexer until the driver releases the trigger.
 */
public class AimAndShoot extends SequentialCommandGroup {

    public AimAndShoot(Turret turret, Trigger trigger) {
        addCommands(
            new TurnTurret(turret),
            new ParallelCommandGroup(
                turret.startEnd(turret::spinAtDistance, turret::stopShooter),
                new AutoShoot(turret, trigger)
            )
        );
    }
}
