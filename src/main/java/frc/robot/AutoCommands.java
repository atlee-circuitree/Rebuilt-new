package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class AutoCommands {

    public static SendableChooser<Command> buildAutoChooser(
            CommandSwerveDrivetrain drivetrain,
            SwerveRequest.FieldCentric drive,
            double maxSpeed,
            Turret turret,
            Trigger trigger,
            Intake intake) 
            {

        SendableChooser<Command> chooser = new SendableChooser<>();

        //double maxSpeed = 1;
        double maxAngularRate = 1;

        chooser.addOption("Just Shoot", new SequentialCommandGroup(
            new SpinToSpeed(turret, 64), //mid rps but + 1, close rps was missing by hitting the foot of the hub
            new Shoot(turret, trigger).withTimeout(12),
            new StopTurretWheels(turret)
        ));

        chooser.addOption("Center Shoot (RED)", new SequentialCommandGroup(
            drivetrain.applyRequest(() -> drive.withVelocityX(0.4 * maxSpeed)
                .withVelocityY(0 * maxSpeed)
                .withRotationalRate(0 * maxAngularRate)
            ).withTimeout(1.2), //3.5
            drivetrain.applyRequest(() -> drive.withVelocityX(0 * maxSpeed)
                .withVelocityY(0 * maxSpeed)
                .withRotationalRate(0 * maxAngularRate)
            ).withTimeout(0.2),
            new SpinToSpeed(turret, Constants.Turret.SPEED_MID_RPS),
            new Shoot(turret, trigger).withTimeout(12),
            new StopTurretWheels(turret)
        ));

        chooser.addOption("Center Shoot (BLUE)", new SequentialCommandGroup(
            drivetrain.applyRequest(() -> drive.withVelocityX(-0.4 * maxSpeed)
                .withVelocityY(0 * maxSpeed)
                .withRotationalRate(0 * maxAngularRate)
            ).withTimeout(1.2),
            drivetrain.applyRequest(() -> drive.withVelocityX(0 * maxSpeed)
                .withVelocityY(0 * maxSpeed)
                .withRotationalRate(0 * maxAngularRate)
            ).withTimeout(0.2),
            new SpinToSpeed(turret, Constants.Turret.SPEED_MID_RPS),
            new Shoot(turret, trigger).withTimeout(12),
            new StopTurretWheels(turret)
        ));

        chooser.addOption("Human Player Shoot (Blue)", new SequentialCommandGroup(
            new ParallelCommandGroup(
                new ManualDeploy(intake, 0.15).withTimeout(0.5),
                new SequentialCommandGroup(
                    drivetrain.applyRequest(() -> drive.withVelocityX(0)
                        .withVelocityY(0.6 * maxSpeed)
                        .withRotationalRate(0)
                    ).withTimeout(3.75),
                    drivetrain.applyRequest(() -> drive.withVelocityX(0 * maxSpeed)
                        .withVelocityY(0 * maxSpeed)
                        .withRotationalRate(0)
                    ).withTimeout(0.2)
                )),
            new WaitCommand(2),
            new SequentialCommandGroup(
                drivetrain.applyRequest(() -> drive.withVelocityX(0 * maxSpeed)
                    .withVelocityY(-0.6 * maxSpeed)
                    .withRotationalRate(0)
                ).withTimeout(1.5),
                drivetrain.applyRequest(() -> drive.withVelocityX(0 * maxSpeed)
                    .withVelocityY(0 * maxSpeed)
                    .withRotationalRate(0)
                ).withTimeout(0.2)
            ),
            new SpinToDistanceSpeed(turret),
            new TurnTurret(turret).withTimeout(1.5),
            new ParallelCommandGroup(
                new DeployJumpCommand(intake).withTimeout(12),
                new Shoot(turret, trigger).withTimeout(12)
            ).withTimeout(12),
            new StopTurretWheels(turret)
        ));

        chooser.addOption("Human Player Shoot (RED)", new SequentialCommandGroup(
            new ParallelCommandGroup(
                new ManualDeploy(intake, 0.15).withTimeout(0.5),
                new SequentialCommandGroup(
                    drivetrain.applyRequest(() -> drive.withVelocityX(0)
                        .withVelocityY(-0.6 * maxSpeed)
                        .withRotationalRate(0)
                    ).withTimeout(3.75),
                    drivetrain.applyRequest(() -> drive.withVelocityX(0 * maxSpeed)
                        .withVelocityY(0 * maxSpeed)
                        .withRotationalRate(0)
                    ).withTimeout(0.2)
                )),
            new WaitCommand(2),
            new SequentialCommandGroup(
                drivetrain.applyRequest(() -> drive.withVelocityX(0 * maxSpeed)
                    .withVelocityY(0.6 * maxSpeed)
                    .withRotationalRate(0)
                ).withTimeout(1.5),
                drivetrain.applyRequest(() -> drive.withVelocityX(0 * maxSpeed)
                    .withVelocityY(0 * maxSpeed)
                    .withRotationalRate(0)
                ).withTimeout(0.2)
            ),
            new SpinToDistanceSpeed(turret),
            new TurnTurret(turret).withTimeout(1.5),
            new ParallelCommandGroup(
                new DeployJumpCommand(intake).withTimeout(12),
                new Shoot(turret, trigger).withTimeout(12)
            ).withTimeout(12),
            new StopTurretWheels(turret)
        ));

        chooser.addOption("Feeder Shoot", new SequentialCommandGroup(
            new ManualDeploy(intake, 0.15).withTimeout(0.5),
            new SequentialCommandGroup(
                drivetrain.applyRequest(() -> drive.withVelocityX(-0.6 * maxSpeed)
                    .withVelocityY(0 * maxSpeed)
                    .withRotationalRate(0 * maxAngularRate)
                ).withTimeout(3.5),
                drivetrain.applyRequest(() -> drive.withVelocityX(0 * maxSpeed)
                    .withVelocityY(0 * maxSpeed)
                    .withRotationalRate(0 * maxAngularRate)
                ).withTimeout(0.2)
            ).deadlineFor(new RunIntake(intake)),
            new SpinToSpeed(turret, Constants.Turret.SPEED_MID_RPS),
            new TurnTurret(turret).withTimeout(1.5),
            new Shoot(turret, trigger).withTimeout(12),
            new StopTurretWheels(turret)
        ));

        return chooser;
    }
}
