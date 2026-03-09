// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.*;
import frc.robot.generated.TunerConstants;
import frc.robot.util.LinearServo;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

import frc.robot.Telemetry;

public class RobotContainer {

    // subsystems
    private Elevator climber;
    private CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private Intake intake;
    private Trigger trigger;
    private Turret turret;
    private Trigger index;

    // drivetrain
    private final Field2d field;

    private final SendableChooser<Command> autoChooser;


    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) / 4; // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final Telemetry logger = new Telemetry(MaxSpeed);

    // IO devices
    private final CommandXboxController Player1 = new CommandXboxController(0);
    private final CommandXboxController Player2 = new CommandXboxController(1);
    
    public RobotContainer() {
        field = new Field2d();
        climber = new Elevator();
        intake = new Intake();
        trigger = new Trigger();
        turret = new Turret();
        autoChooser = AutoBuilder.buildAutoChooser();

        boolean isCompetition = true;
        // Do all other initialization
        configureBindings();
        SmartDashboard.putNumber("velocity", 1.0);
        SmartDashboard.putData("Auto Chooser", autoChooser);

        mapEventsToCommands();
    }

    public void mapEventsToCommands()
    {
        // replace null with command instance
        // do this for all commands
        NamedCommands.registerCommand("Test Event", new RunIntake(intake));
    }

    public Command getAutonomousCommand() {
    // This method loads the auto when it is called, however, it is recommended
    // to first load your paths/autos when code starts, then return the
    // pre-loaded auto/path
        return autoChooser.getSelected();
    }

    private void configureDrivetrain() {
        // drivetrain
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-Player1.getLeftY() * MaxSpeed) // Drive forward with
                                                                                                   // negative Y
                                                                                                   // (forward)
                        .withVelocityY(-Player1.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-Player1.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                    // negative X (left)
        ));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
        
        //().whileTrue(drivetrain.applyRequest(() -> brake));
        Player1.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-Player1.getLeftY(), -Player1.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
       
        // reset the field-centric heading on left bumper press
        //joystick.leftBumper().onTrue(new runIntake(intake, 0.5));

        drivetrain.registerTelemetry(logger::telemeterize);
    }



    private void configureBindings() {
        configureDrivetrain();
 drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> 
                drive.withVelocityX(-(Player1.getLeftY()) * ((Constants.Drive.MaxSpeed) / Constants.Drive.Speed)) // Drive forward with negative Y (forward)
                    .withVelocityY(-(Player1.getLeftX()) * ((Constants.Drive.MaxSpeed) / Constants.Drive.Speed)) // Drive left with negative X (left)
                    .withRotationalRate(-Player1.getRightX() * Constants.Drive.MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        drivetrain.seedFieldCentric();
        Player1.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        //Player1.rightBumper().onTrue(climber.goToSetpoint(() -> {return Elevator.Setpoint.Top;}));
        //Player1.leftBumper().onTrue(climber.goToSetpoint(() -> {return Elevator.Setpoint.Starting;}));
        
        //Player1.y().onTrue(new DeployIntake(intake));
        //Player1.x().onTrue(new RetractIntake(intake));
        Player1.a().whileTrue(new RunIntake(intake));
        Player1.b().whileTrue(new Shoot(turret, trigger));

        Player1.povDown().onTrue(new StopTurretWheels(turret));
        Player1.povRight().onTrue(new SpinToSpeed(turret, 40));
        Player1.povUp().onTrue(new ToggleHood(turret));

        turret.setDefaultCommand(new ManualTurret(turret, () -> { return Player2.getLeftX(); }));

        Player2.y().onTrue(new DeployIntake(intake));
        Player2.x().onTrue(new RetractIntake(intake));
        //Player2.a().whileTrue(new RunIntake(intake));
       // Player2.b().whileTrue(new Shoot(turret, trigger));

      //  Player2.povDown().onTrue(new StopTurretWheels(turret));
       // Player2.povRight().onTrue(new SpinToSpeed(turret, 40));
       // Player2.povUp().onTrue(new ToggleHood(turret));

       Player2.rightBumper().onTrue(climber.goToSetpoint(() -> {return Elevator.Setpoint.Top;}));
        Player2.leftBumper().onTrue(climber.goToSetpoint(() -> {return Elevator.Setpoint.Starting;}));
        Player2.povUp().onTrue(climber.goToSetpoint(() -> {return Elevator.Setpoint.Middle;}));

    }

}
