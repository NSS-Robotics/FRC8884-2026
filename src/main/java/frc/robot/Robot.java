// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private final Timer timer = new Timer();
  private Shift shift = Shift.TRANSITION;
  private XboxController xbox;

  public Robot() {
    m_robotContainer = new RobotContainer();
    xbox = m_robotContainer.driverController.getHID();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // m_robotContainer.fieldMap.setRobotPose(m_robotContainer.drivetrain.getPoseMeters());
    // m_robotContainer.fieldMap.setRobotPose(m_robotContainer.mmturret.turretLocation);
    // m_robotContainer.fieldMap.setRobotPose(new Pose2d(
    //   m_robotContainer.simulatedPosition.getX(),
    //   m_robotContainer.simulatedPosition.getY(),
    //   Rotation2d.fromDegrees(m_robotContainer.mmturret.getDegrees())
    // ));
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
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
    
    // NOTE: Rumble does not work in simulation
    /*if (
      ((time >= (10 - switchNotification)) && (time < 10)) ||
      ((time >= (35 - switchNotification)) && (time < 35)) ||
      ((time >= (60 - switchNotification)) && (time < 60)) ||
      ((time >= (85 - switchNotification)) && (time < 85)) ||
      ((time >= (110 - switchNotification)) && (time < 110))) {
      xbox.setRumble(RumbleType.kBothRumble, 1);
    } else {
      xbox.setRumble(RumbleType.kBothRumble, 0);
    }*/

    SmartDashboard.putString("Shift", shift.toString());
    SmartDashboard.putNumber("Distance to Target", m_robotContainer.mmturret.getDistanceToTarget());
  }

  @Override
  public void teleopExit() {
    timer.stop();
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
  public void simulationPeriodic() {}
}
