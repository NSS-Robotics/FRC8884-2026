// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    // Setting up bindings for necessary control of the swerve drive platform
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandXboxController driverController = new CommandXboxController(0);
    /* END CTRE SWERVE GEN */

    public Pose2d simulatedPosition = new Pose2d();

    // public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    // public final Limelight limelight = new Limelight("tom", drivetrain);
    public final Flywheels flywheels = new Flywheels();
    public final Feeder feeder = new Feeder();
    // public final Turret turret = new Turret(this);
    public final Spindexer spindexer = new Spindexer();
    public final ShooterHood shooterHood = new ShooterHood();

    public final Field2d fieldMap = new Field2d();

    public RobotContainer() {
        CameraServer.startAutomaticCapture();
        configureBindings();

        SmartDashboard.putData(fieldMap);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        /*drivetrain.setDefaultCommand(
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

        // Zero gyro
        driverController.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);*/

        // driverController.a().whileTrue(new Align(
        //     this,
        //     new Pose2d(6.097173043336289, 4.081071090110617, Rotation2d.fromDegrees(0)),
        //     0.05)
        // );

        // driverController.rightTrigger().whileTrue(new FlywheelShoot(this));
        driverController.leftTrigger()
            .whileTrue(new InstantCommand(() -> {
                flywheels.setVelocity(-4600);
                feeder.setSpeed(5350);
            }))
            .onFalse(new InstantCommand(() -> {
                flywheels.stop();
                feeder.stop();
            }));

        driverController.povDown()
            .onTrue(new InstantCommand(() -> shooterHood.setRotations(shooterHood.getRotations() - 0.05)));
        driverController.povUp()
            .onTrue(new InstantCommand(() -> shooterHood.setRotations(shooterHood.getRotations() + 0.05)));
        
        driverController.rightTrigger()
            .onTrue(new InstantCommand(() -> spindexer.setSpeed(2000)));
        driverController.rightBumper()
            .onFalse(new InstantCommand(() -> spindexer.stop()));

        // driverController.a()
        //     .whileTrue(new InstantCommand(() -> {
        //         flywheels.setVelocity(5200);
        //         feeder.setSpeed(-5350);
        //         spindexer.setSpeed(100);
        //     }))
        //     .onFalse(new InstantCommand(() -> {
        //         flywheels.stop();
        //         feeder.stop();
        //         spindexer.stop();
        //     }));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
