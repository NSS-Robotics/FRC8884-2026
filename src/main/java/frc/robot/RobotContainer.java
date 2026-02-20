// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
// import frc.robot.commands.*;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;

public class RobotContainer {
    /* START CTRE SWERVE GEN */
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    /* END CTRE SWERVE GEN */

    public final CommandXboxController driverController = new CommandXboxController(0);
    public final CommandXboxController operatorController = new CommandXboxController(1);

    public Pose2d simulatedPosition = new Pose2d();
    public Pose2d lobTarget = new Pose2d();

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Limelight limelightFront = new Limelight("front", drivetrain);
    public final Limelight limelightSide = new Limelight("side", drivetrain);
    public final Flywheels flywheels = new Flywheels(this);
    public final IntakePivot intakePivot = new IntakePivot(this);
    public final Intake intake = new Intake();
    public final MotionMagicTurret mmturret = new MotionMagicTurret(this, Constants.Turret.offsetFromCenter);
    public final Spindexer spindexer = new Spindexer();
    public final Feeder feeder = new Feeder(this);
    public final ShooterHood shooterHood = new ShooterHood(this);
    // public final LEDs leds = new LEDs();

    public final Field2d fieldMap = new Field2d();

    public RobotContainer() {
        CameraServer.startAutomaticCapture();
        configureBindings();

        SmartDashboard.putData(fieldMap);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Reset the field-centric heading on left bumper press.
        driverController.y().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        // shoot
        driverController.leftTrigger()
            .whileTrue(new InstantCommand(() -> {
                flywheels.setVelocity();
                feeder.setSpeed(-3000);
                shooterHood.setDegrees();
            }))
            .onFalse(new InstantCommand(() -> {
                flywheels.stop();
                feeder.stop();
                shooterHood.stop();
            }));

        driverController.povDown()
            .onTrue(new InstantCommand(shooterHood::zeroHood));
        
        // unjam
        // driverController.x()
        //     .whileTrue(new InstantCommand(() -> {
        //         flywheels.setVelocity(1000);
        //         feeder.setSpeed(-2000);
        //         shooterHood.setDegrees();
        //     }))
        //     .onFalse(new InstantCommand(() -> {
        //         flywheels.stop();
        //         feeder.stop();
        //         shooterHood.stop();
        //     }));     
        
        driverController.rightTrigger()
            .whileTrue(new InstantCommand(() -> spindexer.setSpeed(-2500))) // increase when get new spindexer
            .onFalse(new InstantCommand(() -> spindexer.stop()));

        driverController.x()
            .onTrue(new InstantCommand(() -> intakePivot.setPosition(-0.5)));
        driverController.a()
            .onTrue(new InstantCommand(() -> intakePivot.setPosition(15.5)));

        driverController.b()
            .whileTrue(new InstantCommand(() -> intake.setVelocity(4500)))
            .onFalse(new InstantCommand(intake::stopIntake));

        //operator controller, zia can i have my spinning gear thing back plz
        // operatorController.povDown()
        //     .onTrue(new InstantCommand(() -> shooterHood.zeroHood()));
        operatorController.leftTrigger().onTrue(new InstantCommand(() ->
            lobTarget = isRed() ? Constants.leftCornerRed : Constants.leftCornerBlue
        ));
        operatorController.rightTrigger().onTrue(new InstantCommand(() ->
            lobTarget = isRed() ? Constants.rightCornerRed : Constants.rightCornerBlue
        ));

        operatorController.povRight().onTrue(new InstantCommand(() -> mmturret.setManualOffset(mmturret.getManualOffset()+5)));
        operatorController.povLeft().onTrue(new InstantCommand(() -> mmturret.setManualOffset(mmturret.getManualOffset()-5)));

        operatorController.povUp().onTrue(new InstantCommand(() -> shooterHood.setManualOffset(shooterHood.getManualOffset()+2)));
        operatorController.povDown().onTrue(new InstantCommand(() -> shooterHood.setManualOffset(shooterHood.getManualOffset()-2)));
    }

    public boolean isRed() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == Alliance.Red;
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
