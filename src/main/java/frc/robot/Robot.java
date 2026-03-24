// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private final Timer timer = new Timer();
  private Shift shift = Shift.TRANSITION;
  private XboxController driverController;
  private XboxController operatorController;

  public Robot() {
    m_robotContainer = new RobotContainer();
    driverController = m_robotContainer.driverController.getHID();
    operatorController = m_robotContainer.operatorController.getHID();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    m_robotContainer.fieldMap.setRobotPose(m_robotContainer.drivetrain.getPoseMeters());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    // m_robotContainer.shooterHood.zeroHood();
    m_robotContainer.isShooting = true;
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

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
    m_robotContainer.shooterHood.zeroHood();
    m_robotContainer.isShooting = false;
    m_robotContainer.flywheels.stop();
    m_robotContainer.spindexer.stop();
    m_robotContainer.feeder.stop();
    m_robotContainer.intake.stopIntake();
    m_robotContainer.turret.autoShoot = false;
    m_robotContainer.shooterHood.disableAutoDecap();
  }

  @Override
  public void teleopPeriodic() {
    double switchNotification = 5;
    int time = (int)timer.get();

    SmartDashboard.putNumber("Timer", timer.get());
    SmartDashboard.putNumber("Time (Seconds)", time);
    

    if (timer.get() < 10)       shift = Shift.TRANSITION;
    else if (timer.get() < 35)  shift = Shift.ALLIANCEA;
    else if (timer.get() < 60)  shift = Shift.ALLIANCEB;
    else if (timer.get() < 85)  shift = Shift.ALLIANCEA;
    else if (timer.get() < 110) shift = Shift.ALLIANCEB;
    else                        shift = Shift.ENDGAME;
    double prefire = m_robotContainer.turret.distanceToTime(m_robotContainer.turret.getDegrees());
    // NOTE: Rumble does not work in simulation
    if (
      ((time >= (10 - prefire)) && (time < 10)) ||
      ((time >= (35 - prefire)) && (time < 35)) ||
      ((time >= (60 - prefire)) && (time < 60)) ||
      ((time >= (85 - prefire)) && (time < 85)) ||
      ((time >= (110 - prefire)) && (time < 110)))
    {
        driverController.setRumble(RumbleType.kBothRumble, 1);
        // operatorController.setRumble(RumbleType.kBothRumble, 1);
    } else if (
      ((time >= (10 - switchNotification)) && (time < 10)) ||
      ((time >= (35 - switchNotification)) && (time < 35)) ||
      ((time >= (60 - switchNotification)) && (time < 60)) ||
      ((time >= (85 - switchNotification)) && (time < 85)) ||
      ((time >= (110 - switchNotification)) && (time < 110))) {
      driverController.setRumble(RumbleType.kBothRumble, 0.5);
      // operatorController.setRumble(RumbleType.kBothRumble, 0.5);
    } else {
      driverController.setRumble(RumbleType.kBothRumble, 0);
      // operatorController.setRumble(RumbleType.kBothRumble, 0);
    }

    SmartDashboard.putString("Shift", shift.toString());
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
    m_robotContainer.fuelSim.updateSim();
  }
}