package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Utils.tools;
import frc.robot.RobotState;
import frc.robot.subsystems.Drivebase.Drivebase;

public class RotateToCommand extends Command {

    private final RobotState robotState;
    private final Drivebase drivebase;

    Rotation2d targetRot;

    ChassisSpeeds controlSpeeds = new ChassisSpeeds();

    ProfiledPIDController pid = new ProfiledPIDController(5, 0.1, 0.0,
            new Constraints(DrivebaseConstants.maxRobotOmega, DrivebaseConstants.maxAngularAcc));

    double angleTolerance;

    public RotateToCommand(Drivebase drivebase, RobotState robotState, Rotation2d rot, double angleTolerance) {

        this.drivebase = drivebase;
        this.robotState = robotState;
        this.targetRot = rot;

        this.angleTolerance = angleTolerance;

        pid.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(drivebase);
    }

    @Override
    public void initialize() {
        Logger.recordOutput("RotateToCommand", true);

        pid.setGoal(targetRot.getRadians());
        pid.reset(this.robotState.getAngle().getRadians());
    }

    @Override
    public void execute() {
        Rotation2d currentRot = this.robotState.getAngle();
        controlSpeeds.omegaRadiansPerSecond = -pid.calculate(currentRot.getRadians());

        drivebase.setLocalDrive(controlSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("RotateToCommand", false);
        drivebase.setLocalDrive(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {

        Pose2d currentPose = robotState.getPose();

        double angleError = tools.getAngleError(targetRot.getRadians(),
                currentPose.getRotation().getRadians());

        double omega = Units.radiansToDegrees(drivebase.getGlobalChassisSpeeds().omegaRadiansPerSecond);

        return Math.abs(angleError) < angleTolerance
                && Math.abs(omega) < 0.2;
    }

}
