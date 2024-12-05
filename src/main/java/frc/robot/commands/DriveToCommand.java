// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Utils.tools;
import frc.robot.RobotState;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.subsystems.Drivebase.Drivebase;

public class DriveToCommand extends Command {

  private final RobotState robotState;
  private final Drivebase drivebase;

  Pose2d targetPose;

  ChassisSpeeds controlSpeeds = new ChassisSpeeds();

  ProfiledPIDController pidX = new ProfiledPIDController(1, 0.01, 0.0,
      new Constraints(DrivebaseConstants.maxRobotVelocity, DrivebaseConstants.maxAcc));
  ProfiledPIDController pidY = new ProfiledPIDController(1, 0.01, 0.0,
      new Constraints(DrivebaseConstants.maxRobotVelocity, DrivebaseConstants.maxAcc));
  ProfiledPIDController pidTheata = new ProfiledPIDController(5, 0.1, 0.0,
      new Constraints(DrivebaseConstants.maxRobotOmega, DrivebaseConstants.maxAngularAcc));

  double poseTolerance;
  double angleTolerance;

  public DriveToCommand(Drivebase drivebase, RobotState robotState, Pose2d pose, double poseTolerance,
      double angleTolerance) {

    this.drivebase = drivebase;
    this.robotState = robotState;
    this.targetPose = pose;

    this.poseTolerance = poseTolerance;
    this.angleTolerance = angleTolerance;

    pidTheata.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    Logger.recordOutput("DriveToCommand", true);

    pidX.setGoal(targetPose.getX());
    pidY.setGoal(targetPose.getY());
    pidTheata.setGoal(targetPose.getRotation().getRadians());

    pidTheata.reset(this.robotState.getAngle().getRadians());
  }

  @Override
  public void execute() {
    Pose2d currentPose = this.robotState.getPose();

    controlSpeeds.vxMetersPerSecond = pidX.calculate(currentPose.getX());
    controlSpeeds.vyMetersPerSecond = pidY.calculate(currentPose.getY());
    controlSpeeds.omegaRadiansPerSecond = -pidTheata.calculate(currentPose.getRotation().getRadians());

    drivebase.setGlobalDrive(controlSpeeds);
  }

  @Override
  public void end(boolean interrupted) {
    Logger.recordOutput("DriveToCommand", false);
    drivebase.setGlobalDrive(new ChassisSpeeds());
  }

  @Override
  public boolean isFinished() {

    Pose2d currentPose = robotState.getPose();

    double dist = Math.hypot(targetPose.getX() - currentPose.getX(), targetPose.getY() - currentPose.getY());
    double angleError = tools.getAngleError(targetPose.getRotation().getRadians(),
        currentPose.getRotation().getRadians());

    double omega = Units.radiansToDegrees(drivebase.getGlobalChassisSpeeds().omegaRadiansPerSecond);

    return dist < poseTolerance
        && Math.abs(angleError) < angleTolerance
        && Math.abs(omega) < 0.2;
  }
}
