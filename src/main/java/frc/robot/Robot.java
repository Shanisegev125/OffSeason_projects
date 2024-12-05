// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.Constructors.CanDeviceID;
import frc.robot.Utils.Phoenix6Motor.Phoenix6MotorIOTester;
import frc.robot.autos.AutoChooser;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.DriveCommand;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {

    Logger.addDataReceiver(new NT4Publisher());
    Logger.start();

    m_robotContainer = new RobotContainer();
    AutoChooser.getInstance();

    var pose = m_robotContainer.getVision().getFrontCamPose();
    if (pose != null)
      m_robotContainer.getRobotState().resetPose(pose);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = AutoChooser.getInstance().getSelectedCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.drivebase.setDefaultCommand(new DriveCommand(m_robotContainer.drivebase, m_robotContainer.driver));
    m_robotContainer.climb.setDefaultCommand(new ClimbCommand(m_robotContainer.climb, m_robotContainer.operator));
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    Phoenix6MotorIOTester.createInstance(new CanDeviceID(1));
    Phoenix6MotorIOTester.createInstance(new CanDeviceID(2));
    Phoenix6MotorIOTester.createInstance(new CanDeviceID(3));
    Phoenix6MotorIOTester.createInstance(new CanDeviceID(4));
  }

  @Override
  public void testPeriodic() {
    Logger.recordOutput("Drive/Cancoders", m_robotContainer.getDrivebase().getCancoderAngles());
    for (var motor : Phoenix6MotorIOTester.getInstances())
      motor.update();
  }
}
