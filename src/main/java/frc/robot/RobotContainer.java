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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.ImmutableLinearVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.FuelSim;
import frc.robot.commands.*;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    // private final SendableChooser<Command> autoChooser;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandXboxController driverController = new CommandXboxController(0);
    //public final CommandXboxController operatorController = new CommandXboxController(1);

    public final FuelSim fuelSim = new FuelSim();

    public Pose2d simulatedPosition = new Pose2d();
    public Pose2d lobTarget = new Pose2d();

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Limelight limelightFront = new Limelight("front", drivetrain);
    public final Limelight limelightSide = new Limelight("side", drivetrain);
    public final Flywheels flywheels = new Flywheels(this);
    public final IntakePivot intakePivot = new IntakePivot(this);
    public final Intake intake = new Intake();
    public final Spindexer spindexer = new Spindexer();
    public final ShooterHood shooterHood = new ShooterHood(this);
    public final MotionMagicTurret mmturret = new MotionMagicTurret(this, Constants.Turret.offsetFromCenter);
    public final Feeder feeder = new Feeder(this);

    // public final LEDs leds = new LEDs();

    public final Field2d fieldMap = new Field2d();

    public RobotContainer() {
        // addCommandsToPathplanner();
        // autoChooser = AutoBuilder.buildAutoChooser();
        CameraServer.startAutomaticCapture();
        configureBindings();
        configureFuelSim();
        configurePathPlanner();

        SmartDashboard.putData(fieldMap);
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        drivetrain.registerTelemetry(logger::telemeterize);

        /* instant commands */
        driverController.povDown().onTrue(new InstantCommand(shooterHood::zeroHood)); // zero hood
        driverController.y().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric)); // zero gyro

        /* longer commands */
        driverController.rightTrigger().whileTrue(new FlywheelShoot(this));
        driverController.rightBumper().whileTrue(new RunSpindexer(this));
        driverController.b().whileTrue(new Unjam(this));
        driverController.leftTrigger().whileTrue(new RunIntake(this));
        driverController.leftBumper().whileTrue(new IntakeUnjam(this));
        
        driverController.x()
            .onTrue(new InstantCommand(() -> intakePivot.setPosition(-0.5)));
        driverController.a()
            .onTrue(new InstantCommand(() -> intakePivot.setPosition(15.5)));

        // driverController.leftBumper()
        //     .whileTrue(new InstantCommand(() -> {
        //         fuelSim.launchFuel(
        //             LinearVelocity.ofBaseUnits(7, MetersPerSecond),
        //             Angle.ofBaseUnits(90 - shooterHood.getDegrees(), Degree),
        //             Angle.ofBaseUnits(mmturret.getDegrees(), Degree),
        //             Distance.ofBaseUnits(0.75, Meters)
        //         );
        //     }));

        //operator controller, zia can i have my spinning gear thing back plz
        // operatorController.povDown()
        //     .onTrue(new InstantCommand(() -> shooterHood.zeroHood()));
        /*        operatorController.leftTrigger().onTrue(new InstantCommand(() ->
            lobTarget = isRed() ? Constants.leftCornerRed : Constants.leftCornerBlue
        ));
        operatorController.rightTrigger().onTrue(new InstantCommand(() ->
            lobTarget = isRed() ? Constants.rightCornerRed : Constants.rightCornerBlue
        ));

        operatorController.povRight().onTrue(new InstantCommand(() -> mmturret.setManualOffset(mmturret.getManualOffset()+5)));
        operatorController.povLeft().onTrue(new InstantCommand(() -> mmturret.setManualOffset(mmturret.getManualOffset()-5)));

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

    private void configurePathPlanner() {
        
    }

    public boolean isRed() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == Alliance.Red;
    }

    private void addCommandsToPathplanner() {
        NamedCommands.registerCommand("Shoot", 
            new SequentialCommandGroup(
                new InstantCommand(() -> new FlywheelShoot(this)),
                new InstantCommand(() -> new RunSpindexer(this))
            )
        );
        NamedCommands.registerCommand("IntakeDown",    
            new InstantCommand(() -> intakePivot.setPosition(15.5))
        );

        NamedCommands.registerCommand("Intake",
            new InstantCommand(() -> new RunIntake(this))
        );  
        
    }
    

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
