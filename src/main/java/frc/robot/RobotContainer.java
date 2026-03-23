// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.FuelSim;
import frc.robot.commands.*;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final SendableChooser<Command> autoChooser;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake xLockWheels = new SwerveRequest.SwerveDriveBrake();
    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandXboxController driverController = new CommandXboxController(0);
    public final CommandXboxController operatorController = new CommandXboxController(1);

    public final FuelSim fuelSim = new FuelSim();

    public Pose2d simulatedPosition = new Pose2d();
    public Pose2d lobTarget = new Pose2d();

    public final DataLog log = DataLogManager.getLog();;

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Limelight limelightFront = new Limelight(this, "front", drivetrain);
    public final Limelight limelightSide = new Limelight(this, "side", drivetrain);
    public final IntakePivot intakePivot = new IntakePivot(this);
    public final Intake intake = new Intake();
    public final Spindexer spindexer = new Spindexer();
    public final ShooterHood shooterHood = new ShooterHood(this);
    public final Flywheels flywheels = new Flywheels(this);
    public final Turret turret = new Turret(this, Constants.Turret.offsetFromCenter);
    public final Feeder feeder = new Feeder(this);
    // public final LEDs leds = new LEDs();

    public final Field2d fieldMap = new Field2d();

    public RobotContainer() {
        addCommandsToPathplanner();
        CameraServer.startAutomaticCapture();
        DataLogManager.start();
        DriverStation.startDataLog(log);

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
        configureFuelSim();

        SmartDashboard.putData(fieldMap);
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                double cappedMaxSpeed = MaxSpeed;
                double cappedMaxAngularVelocity = MaxAngularRate;

                if (driverController.rightTrigger(0.3).getAsBoolean()) {
                    cappedMaxSpeed = 0.75;
                    cappedMaxAngularVelocity *= 0.5;
                }

            
                // double xVel = -driverController.getLeftY() * cappedMaxSpeed;
                // double yVel = -driverController.getLeftX() * cappedMaxSpeed;

                double xVel = Math.max(-cappedMaxSpeed, Math.min(cappedMaxSpeed,-driverController.getLeftY() * MaxSpeed));
                double yVel = Math.max(-cappedMaxSpeed, Math.min(cappedMaxSpeed,-driverController.getLeftX() * MaxSpeed));



                return drive.withVelocityX(xVel) // Drive forward with negative Y (forward)
                    .withVelocityY(yVel) // Drive left with negative X (left)
                    .withRotationalRate( Math.max(-cappedMaxAngularVelocity, Math.min(cappedMaxAngularVelocity, -driverController.getRightX()* MaxAngularRate)));
                    //.withRotationalRate(-driverController.getRightX() * cappedMaxAngularVelocity); // Drive counterclockwise with negative X (left)
            })
        );

        drivetrain.registerTelemetry(logger::telemeterize);

        /* instant commands */
        // driverController.povDown().onTrue(new InstantCommand(shooterHood::zeroHood)); // zero hood
        // driverController.povUp().onTrue(new InstantCommand (() -> shooterHood.changeDegrees(1)));
        // driverController.povDown().onTrue(new InstantCommand (() -> shooterHood.changeDegrees(-1)));
        driverController.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric)); // zero gyro
        driverController.x().onTrue(new InstantCommand(() -> intakePivot.setPosition(-0.5)));
        driverController.a().onTrue(new InstantCommand(() -> intakePivot.setPosition(15.3)));

        /* longer commands */
        operatorController.rightTrigger().whileTrue(new FlywheelShoot(this));
        operatorController.rightBumper().whileTrue(new RunSpindexer(this));
        operatorController.leftBumper().whileTrue(new IntakeUnjam(this));
        operatorController.b().whileTrue(new Unjam(this));
        operatorController.povDown().onTrue(new InstantCommand(intakePivot::zeroIntake));
        operatorController.povUp().onTrue(new InstantCommand(shooterHood::zeroHood));
        operatorController.povRight().onTrue(new InstantCommand(() -> turret.manualDistanceOffset += 0.05));
        operatorController.povLeft().onTrue(new InstantCommand(() -> turret.manualDistanceOffset -= 0.05));

        driverController.leftTrigger(0.3).whileTrue(new RunIntake(this, 3600));
        driverController.povUp().whileTrue(new Snap(this, 5, 179)); // TODO: when limelight works, and MAYBE HAS TO DO WITH ALLIANCE AS WELL RED BLUE
        driverController.povLeft().whileTrue(new Snap(this, 5, 90));
        driverController.povDown().whileTrue(new Snap(this, 5, 0)); 
        driverController.povRight().whileTrue(new Snap(this, 5, -90));
        driverController.b().whileTrue(new RunIntake(this, -3200));
        driverController.leftBumper().whileTrue(drivetrain.applyRequest(() -> xLockWheels)); // remove later if necessary
    
        //operator controller, zia can i have my spinning gear thing back plz

        /*
        operatorController.povRight().onTrue(new InstantCommand(() -> turret.setManualOffset(turret.getManualOffset()+5)));
        operatorController.povLeft().onTrue(new InstantCommand(() -> turret.setManualOffset(turret.getManualOffset()-5)));

        operatorController.povUp().onTrue(new InstantCommand(() -> shooterHood.setManualOffset(shooterHood.getManualOffset()+2)));
        operatorController.povDown().onTrue(new InstantCommand(() -> shooterHood.setManualOffset(shooterHood.getManualOffset()-2)));
        */
    }

    private void configureFuelSim() {
        SmartDashboard.putData(Commands.runOnce(() -> {
                fuelSim.clearFuel();
                fuelSim.spawnStartingFuel();
            })
            .withName("Reset Fuel")
            .ignoringDisable(true));

        fuelSim.spawnStartingFuel();
        fuelSim.start();

        fuelSim.registerRobot(
            Constants.Dimensions.robotWidth,
            Constants.Dimensions.robotLength,
            0,
            drivetrain::getPoseMeters,
            () -> drivetrain.getState().Speeds
        );

        fuelSim.registerIntake(
            // TODO: is 0 middle of bot or side?
            0, 0.762,
            0, Units.inchesToMeters(4),
            () -> true
        );

        fuelSim.setSubticks(1);
        fuelSim.enableAirResistance();
    }

    public boolean isRed() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == Alliance.Red;
    }
    

    private void addCommandsToPathplanner() {
        // NamedCommands.registerCommand("Shoot", 
        //     new SequentialCommandGroup(
        //         new InstantCommand(() -> new AutoFlywheelShoot(this)),
        //         new InstantCommand(() -> new AutoRunSpindexer(this))
        //     )
        // );  
 
        NamedCommands.registerCommand("Flywheels", 
            new InstantCommand(() -> {
                // double future[] = turret.aLookIntoTheFuture(drivetrain.getPoseMeters(), turret.getDistanceToTarget(), isRed() ? Constants.targetPositionRed : Constants.targetPositionBlue); 
                // flywheels.setVelocity(flywheels.rpmToHub(future[0]));   
                // shooterHood.setDegrees();
                turret.autoShoot = true;
            })
        );

        // TODO: auto decap on rightside auto
        NamedCommands.registerCommand("Auto Decap True",
            new InstantCommand(shooterHood::enableAutoDecap)
        );

        NamedCommands.registerCommand("Auto Decap False",
            new InstantCommand(shooterHood::disableAutoDecap)
        );

        NamedCommands.registerCommand("Spindexer", 
            new RunSpindexer(this)
        );
        
        NamedCommands.registerCommand("IntakeDown",    
            new InstantCommand(() -> intakePivot.setPosition(15.9))
        );

        NamedCommands.registerCommand("Intake",
            new InstantCommand(() -> intake.setVelocity(5000)) // was 4750, this was a change at comp
        );

        NamedCommands.registerCommand("Unjam Intake",
            new AutoIntakeUnjam(this)
        );
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
