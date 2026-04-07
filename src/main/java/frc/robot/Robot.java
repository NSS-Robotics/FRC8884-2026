// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.LimelightHelpers;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer rob;

  private final Timer timer = new Timer();
  private Shift shift = Shift.TRANSITION;
  private XboxController driverController;
  private XboxController operatorController;

  public Robot() {
    rob = new RobotContainer();
    driverController = rob.driverController.getHID();
    operatorController = rob.operatorController.getHID();

    LimelightHelpers.SetIMUMode("limelight-front", 1);
    LimelightHelpers.SetIMUMode("limelight-side", 1);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    rob.fieldMap.setRobotPose(rob.drivetrain.getPoseMeters());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    LimelightHelpers.SetIMUMode("limelight-front", 1);
    LimelightHelpers.SetIMUMode("limelight-side", 1);
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    LimelightHelpers.SetIMUMode("limelight-front", 3);
    LimelightHelpers.SetIMUMode("limelight-side", 3);

    rob.isShooting = true;
    rob.flywheels.rpmToRest();
    m_autonomousCommand = rob.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    timer.start();
    //rob.shooterHood.zeroHood();
    rob.isShooting = false;
    rob.flywheels.stop();
    rob.spindexer.stop();
    rob.feeder.stop();
    rob.intake.stopIntake();
    rob.turret.autoShoot = false;
    rob.shooterHood.disableAutoDecap();

    LimelightHelpers.SetIMUMode("limelight-front", 0);
    LimelightHelpers.SetIMUMode("limelight-side", 0);
  }

  @Override
  public void teleopPeriodic() {
    // driver controller rumble
    double switchNotification = 5;
    int time = (int)timer.get();

    SmartDashboard.putNumber("Timer", timer.get());
    SmartDashboard.putNumber("Time (Seconds)", time);
    SmartDashboard.putNumber("Time Left in Shift", rob.timeLeft(DriverStation.getMatchTime()));

    // SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    int minutes = (int)DriverStation.getMatchTime() / 60;
    int seconds = (int)DriverStation.getMatchTime() % 60;
    SmartDashboard.putString("Match Time", minutes+":"+seconds);

    SmartDashboard.putBoolean("HUB ACTIVE", rob.isHubActive());

    if (timer.get() < 10)       shift = Shift.TRANSITION;
    // else if (timer.get() < 35)  shift = rob.isHubActive() ? Shift.ALLIANCEA : Shift.ALLIANCEB;
    // else if (timer.get() < 60)  shift = rob.isHubActive() ? Shift.ALLIANCEA : Shift.ALLIANCEB;
    // else if (timer.get() < 85)  shift = rob.isHubActive() ? Shift.ALLIANCEA : Shift.ALLIANCEB;
    else if (timer.get() < 110) shift = rob.isHubActive() ? Shift.ALLIANCEA : Shift.ALLIANCEB;
    else                        shift = Shift.ENDGAME;
    double prefire = rob.turret.distanceToTime(rob.turret.getDistanceToTarget());
    // NOTE: Rumble does not work in simulation
    if (
      ((time >= (10 - prefire)) && (time < 10)) ||
      ((time >= (35 - prefire)) && (time < 35)) ||
      ((time >= (60 - prefire)) && (time < 60)) ||
      ((time >= (85 - prefire)) && (time < 85)) ||
      ((time >= (110 - prefire)) && (time < 110))) {
        driverController.setRumble(RumbleType.kBothRumble, 1);
    } else if (
      ((time >= (10 - switchNotification)) && (time < 10)) ||
      ((time >= (35 - switchNotification)) && (time < 35)) ||
      ((time >= (60 - switchNotification)) && (time < 60)) ||
      ((time >= (85 - switchNotification)) && (time < 85)) ||
      ((time >= (110 - switchNotification)) && (time < 110))) {
      driverController.setRumble(RumbleType.kBothRumble, 0.5);
    } else {
      driverController.setRumble(RumbleType.kBothRumble, 0);
    }

    SmartDashboard.putString("Shift", shift.toString());

    // operator controller rumble
    if (rob.isShooting && rob.flywheels.isAtTargetVelocity(2)) {
      operatorController.setRumble(RumbleType.kBothRumble, 0.7);
    }
  }

  @Override
  public void teleopExit() {
    timer.stop();
    driverController.setRumble(RumbleType.kBothRumble, 0);
    operatorController.setRumble(RumbleType.kBothRumble, 0);
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {
    rob.fuelSim.updateSim();
  }
}